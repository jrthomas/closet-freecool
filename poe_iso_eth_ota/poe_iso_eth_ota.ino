/*
 * ESP32-POE-ISO -- closet controller
 * Board target: "OLIMEX ESP32-PoE-ISO"   (esp32 core 3.x)
 *
 * Validated on the rev-A carrier (after patching the missing GND
 * between the Olimex header and the carrier ground pour):
 *   PoE + Ethernet, OTA, telnet console, BME280 @0x76,
 *   DS18B20 1-Wire, both SSR drives, LEDC fan PWM stage.
 *
 * Site settings and credentials live in secrets.h (not committed).
 * Copy secrets.example.h to secrets.h and fill it in before building.
 *
 * Flash with ./flash.sh (compile + espota). Notes at the bottom.
 */

#if __has_include("secrets.h")
  #include "secrets.h"
#else
  #error "secrets.h not found -- copy secrets.example.h to secrets.h and fill it in"
#endif

#include <ETH.h>
#include <ArduinoOTA.h>
#include <TelnetStream.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PubSubClient.h>

// ---------------------------------------------------------------
// Olimex ESP32-POE-ISO hardware definition
// ---------------------------------------------------------------
#define ETH_PHY_TYPE   ETH_PHY_LAN8720
#define ETH_PHY_ADDR   0
#define ETH_PHY_MDC    23
#define ETH_PHY_MDIO   18
#define ETH_PHY_POWER  12      // also MTDI strapping pin -- do not load
#define ETH_CLK_MODE   ETH_CLOCK_GPIO17_OUT

// ---------------------------------------------------------------
// Carrier board I/O -- GPIO numbers, NOT schematic pin numbers
// ---------------------------------------------------------------
#define PIN_FAN_PWM    4    // BSS138 gate, inverted logic (open drain)
#define PIN_I2C_SDA    13   // UEXT SDA, 2k2 pull-up on the Olimex board
#define PIN_I2C_SCL    16   // UEXT SCL, 2k2 pull-up on the Olimex board
#define PIN_ONEWIRE    33   // DS18B20 bus, R2 4k7 to 3V3 on the carrier
#define PIN_DAMPER     32   // AQH2223 SSR -> AZ2280 Relay -> damper actuator
#define PIN_DEHU_FAN   5    // 2nd SSR, dehumidifier build only
#define PIN_BUT2       36   // input only, 2k2 pull-up on board
#define PIN_BUTTON     34   // BUT1, 10k pull-up on board

// ---------------------------------------------------------------
// Identity
//   OTA_PASSWORD, MQTT_HOST/PORT/USER/PASS, NTP_SERVER, TZ_LOCAL,
//   TOPIC_BASE and the DS18B20 ROMs come from secrets.h.
// ---------------------------------------------------------------
#define OTA_HOSTNAME   "closet-ctrl"

// ---------------------------------------------------------------
// MQTT
// ---------------------------------------------------------------
#define MQTT_CLIENT_ID OTA_HOSTNAME

#define TOPIC_STATE    TOPIC_BASE "/state"
#define TOPIC_STATUS   TOPIC_BASE "/status"    // retained, LWT
#define TOPIC_CMD      TOPIC_BASE "/cmd/+"

#define MQTT_STATE_MS  30000    // idle heartbeat; changes publish at once
#define MQTT_RETRY_MS  5000     // reconnect backoff floor

// ---------------------------------------------------------------
// Display: U8g2 / SSD1309 2.42" (128x64), I2C -- same part as
// PQMonitor. Set DISPLAY_ENABLE false for headless builds.
// ---------------------------------------------------------------
#define DISPLAY_ENABLE true

#if DISPLAY_ENABLE
  #include <U8g2lib.h>
  U8G2_SSD1309_128X64_NONAME0_F_HW_I2C display(
      U8G2_R0,
      /* reset=*/ U8X8_PIN_NONE,
      /* clock=*/ PIN_I2C_SCL,
      /* data=*/  PIN_I2C_SDA);

  #define FONT_ROW            u8g2_font_7x13B_tf
  #define DISPLAY_INTERVAL_MS 3000    // page dwell
  #define DISPLAY_REDRAW_MS   200     // frame rate cap

#endif

// ---------------------------------------------------------------
// Sensors
// ---------------------------------------------------------------
#define BME_ADDR 0x76

Adafruit_BME280 bme;
static bool     bme_ok   = false;
static float    closetT  = NAN;   // degF
static float    closetRH = NAN;   // %
static uint32_t bme_bad  = 0;

OneWire  oneWire(PIN_ONEWIRE);
DallasTemperature ds(&oneWire);

#define DS_RESOLUTION  11        // 0.125C, ~375ms conversion
#define DS_MAX         4
static int      ds_count = 0;
static float    dsTemp[DS_MAX] = { NAN, NAN, NAN };  // degF, by index for now
static uint32_t ds_bad   = 0;

// Probe ROM addresses are site-specific -- set in secrets.h.
DeviceAddress ADDR_EXHAUST = ROM_EXHAUST;
DeviceAddress ADDR_INTAKE  = ROM_INTAKE;
DeviceAddress ADDR_OUTDOOR = ROM_OUTDOOR;

#define IDX_OUTDOOR     0
#define IDX_INTAKE   1
#define IDX_EXHAUST  2

// ---------------------------------------------------------------
// Control mode
//   Declared before the first function definition: Arduino inserts
//   auto-generated prototypes there, and they must not reference a
//   type the compiler has not seen yet.
// ---------------------------------------------------------------
enum CtrlMode { MODE_SEALED, MODE_FREECOOL, MODE_MANUAL, MODE_FAULT };
static CtrlMode mode = MODE_SEALED;

static const char *modeName(CtrlMode m) {
  switch (m) {
    case MODE_FREECOOL:  return "Freecool";
    case MODE_MANUAL:    return "Manual";
    case MODE_FAULT:     return "Fault";
    default:             return "Sealed";
  }
}


// ---------------------------------------------------------------
// Fan -- LEDC PWM into the open-drain level shifter
// ---------------------------------------------------------------
#define FAN_PWM_FREQ  200      // Hz
#define FAN_PWM_BITS  12       // 0..4095
#define FAN_MAX_DUTY  ((1 << FAN_PWM_BITS) - 1)

// Hardware stall guard. Measured on the bench: runs at 10, stalls
// at 5. This is a property of the motor and the drive stage, not of
// the air path -- the installed floor, where the fan finally makes
// enough static pressure to crack the backdraft damper, is a higher
// number that belongs in NVS because it drifts as the filter loads.
#define FAN_MIN_PCT   10

// Speed commanded at boot. 100 is the correct value in service: it
// matches what the fan does with no controller at all, so every path
// that is not the control loop running normally ends with air moving.
// Drop it to 50 while benching if the noise is tiresome, but it must
// be 100 before this goes in the wall.
#define FAN_BOOT_PCT  100

static uint8_t fanPct = 0;     // EFFECTIVE speed after clamping
static uint8_t fanReq = 0;     // what the caller actually asked for

// pct is FAN SPEED. The output stage inverts: duty 0 leaves the
// MOSFET off, the 0-10V line floats to 9.7V, and the fan runs full
// speed. That is the intended failsafe on reset or loss of the
// controller.
//
// A request between 1 and FAN_MIN_PCT is raised to FAN_MIN_PCT
// rather than dropped to zero: it means "a little cooling", and the
// least the fan can actually deliver is a better answer than a
// spinning impeller moving no air. 0 remains an explicit off.
void fanSet(uint8_t pct) {
  if (pct > 100) pct = 100;
  fanReq = pct;
  if (pct > 0 && pct < FAN_MIN_PCT) pct = FAN_MIN_PCT;
  fanPct = pct;
  uint32_t duty = (uint32_t)(100 - pct) * FAN_MAX_DUTY / 100;
  ledcWrite(PIN_FAN_PWM, duty);
}

void fanInit() {
  ledcAttach(PIN_FAN_PWM, FAN_PWM_FREQ, FAN_PWM_BITS);
  fanSet(FAN_BOOT_PCT);
}

// ---------------------------------------------------------------
// State
// ---------------------------------------------------------------

static float setpointF = 77.0f;       // closet target

static bool eth_connected  = false;
static bool ota_started    = false;
static bool mqtt_connected = false;
static bool lastButton     = true;    // BUT1 idles high

NetworkClient  netClient;
PubSubClient   mqtt(netClient);

// ---------------------------------------------------------------

#define LOG(...) do { Serial.printf(__VA_ARGS__); \
                      TelnetStream.printf(__VA_ARGS__); } while(0)

// ---------------------------------------------------------------
// BME280
// ---------------------------------------------------------------

static float lastT = NAN, lastRH = NAN;
static uint32_t lastChange = 0;
static uint32_t bme_stale = 0;


bool bmeInit() {
  if (!bme.begin(BME_ADDR, &Wire)) return false;
  // MODE_NORMAL: the part free-runs at 1Hz and we read the latest
  // result. At X1 oversampling on a 1s standby the die stays close
  // enough to ambient that self-heating is not worth chasing.
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1,      // temperature
                  Adafruit_BME280::SAMPLING_X1,      // pressure
                  Adafruit_BME280::SAMPLING_X1,      // humidity
                  Adafruit_BME280::FILTER_OFF);
                  //Adafruit_BME280::STANDBY_MS_1000);
  return true;
}

#define BME_T_OFFSET  -1.0f   // die runs above ambient; measured against
                              // two ice-bath-verified DS18B20s, 2026-10-19

// Saturation vapour pressure, Magnus. Only the ratio is used, so the
// units cancel.
static float satVP(float tF) {
  float tc = (tF - 32.0f) * 5.0f / 9.0f;
  return expf(17.625f * tc / (tc + 243.04f));
}

void readBME() {
  if (!bme_ok) return;
  // ~9ms at X1 oversampling. The library polls the status bit and
  // gives up after 2s, so a wedged part costs one slow pass rather
  // than hanging the loop.
  if (!bme.takeForcedMeasurement()) { bme_bad++; return; }
  float t_raw  = bme.readTemperature() * 9.0f / 5.0f + 32.0f;
  float rh_raw = bme.readHumidity();

  // The part measures RH at its own die temperature. Same absolute
  // moisture at the cooler true air temperature is a higher RH, by
  // the ratio of saturation pressures.
  float t  = t_raw + BME_T_OFFSET;
  float rh = rh_raw * satVP(t_raw) / satVP(t);
  if (rh > 100.0f) rh = 100.0f;

  // A failed I2C read returns all-ones and compensates to ~358F.
  // Reject it and keep the last good value rather than acting on
  // garbage.
  if (t > 20.0f && t < 130.0f && rh >= 0.0f && rh <= 100.0f) {
    closetT  = t;
    closetRH = rh;
  } else {
    bme_bad++;
  }

  if (t != lastT || rh != lastRH) { lastT = t; lastRH = rh; lastChange = millis(); }
  else if (millis() - lastChange > 300000) {
    bme_stale++;
    LOG("[BME] stale %lus -- reinit\n", (millis() - lastChange) / 1000UL);
    bme_ok = bmeInit();
    lastChange = millis();
  }

}

// ---------------------------------------------------------------
// DS18B20
// ---------------------------------------------------------------
void dsInit() {
  ds.begin();
  ds.setResolution(DS_RESOLUTION);
  ds.setWaitForConversion(false);     // never block the loop
  ds_count = ds.getDeviceCount();
  Serial.printf("[1WIRE] %d device(s)\n", ds_count);

  DeviceAddress a;
  for (int i = 0; i < DS_MAX; i++) dsTemp[i] = NAN;
  for (int i = 0; i < ds_count && i < DS_MAX; i++) {
    if (ds.getAddress(a, i)) {
      Serial.printf("[1WIRE]  %d: ", i);
      for (uint8_t b = 0; b < 8; b++) Serial.printf("%02X", a[b]);
      Serial.println();
    }
  }
}


static const uint8_t *ADDR_ROLE[3] = { ADDR_OUTDOOR, ADDR_INTAKE, ADDR_EXHAUST };
// A probe that stops answering must not hold its last reading
// forever. controlService() already seals up on NAN -- it just
// never gets one, because a failed read leaves dsTemp[] alone and
// only ds_bad climbs. 120 s is ~24 missed reads at the 5 s cadence:
// long enough to ride out a one-off CRC error, short enough that a
// dead outdoor probe can't hold the damper open for an afternoon.
#define DS_STALE_MS  120000UL

static const char *ROLE_NAME[3] = { "outdoor", "intake", "exhaust" };
static uint32_t dsGoodAt[3] = { 0, 0, 0 };

// An all-zero address means that role is not fitted. A real DS18B20
// always starts with family code 0x28, so byte 0 is a safe test.
static bool addrSet(const uint8_t *a) { return a[0] != 0; }


void dsService() {
  static uint32_t t0 = 0;
  static bool pending = false;

  if (!pending && millis() - t0 >= 5000) {
    ds.requestTemperatures();
    t0 = millis();
    pending = true;
  }
  if (pending && millis() - t0 >= 500) {
    pending = false;
    // Read by ROM address, not bus index. Enumeration order is by ROM
    // value, so adding the third probe would otherwise reshuffle the
    // roles of the two already fitted.
    for (int r = 0; r < 3; r++) {
      if (!addrSet(ADDR_ROLE[r])) { dsTemp[r] = NAN; continue; }
      float f = ds.getTempF(ADDR_ROLE[r]);
      if (f != DEVICE_DISCONNECTED_F && f > -40.0f && f < 200.0f) {
        dsTemp[r]   = f;
        dsGoodAt[r] = millis();
        if (!dsGoodAt[r]) dsGoodAt[r] = 1;   // 0 means "never good"
      } else {
        ds_bad++;
        if (dsGoodAt[r] && (millis() - dsGoodAt[r]) > DS_STALE_MS) {
          dsTemp[r]   = NAN;
          dsGoodAt[r] = 0;
          LOG("[1WIRE] %s stale -- dropping to NAN\n", ROLE_NAME[r]);
        }
      }
    }
  }
}


// ---------------------------------------------------------------
// I2C scan -- on demand via BUT1
// ---------------------------------------------------------------
void i2cScan() {
  int found = 0;
  LOG("[I2C] scanning...\n");
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      const char *guess = "";
      if (a == 0x76 || a == 0x77) guess = "  <- BME280";
      if (a == 0x3C || a == 0x3D) guess = "  <- SSD1309";
      LOG("[I2C]   0x%02X%s\n", a, guess);
      found++;
    }
  }
  if (!found) LOG("[I2C]   nothing responded\n");
}

#include "closet_control.h"

// ---------------------------------------------------------------
// MQTT
// ---------------------------------------------------------------

// Append a float field, or null when the reading is not valid.
// A missing sensor must not publish as 0.0 -- downstream cannot
// tell that apart from a real reading.
static int jsonF(char *p, size_t n, const char *key, float v, int dp) {
  if (isnan(v)) return snprintf(p, n, "\"%s\":null,", key);
  return snprintf(p, n, "\"%s\":%.*f,", key, dp, v);
}

size_t buildStateJson(char *buf, size_t len) {
  size_t o = 0;
  buf[o++] = '{';

  time_t now = time(nullptr);
  if (now > 1700000000) {              // NTP has landed
    struct tm tmv;
    localtime_r(&now, &tmv);

    char ts[40];
    strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%S%z", &tmv);
    // %z gives -0500; ISO 8601 extended wants -05:00
    size_t l = strlen(ts);
    if (l >= 5) { ts[l + 1] = '\0'; ts[l] = ts[l - 1];
                  ts[l - 1] = ts[l - 2]; ts[l - 2] = ':'; }
    o += snprintf(buf + o, len - o, "\"ts\":\"%s\",", ts);

  }

  o += jsonF(buf + o, len - o, "closet_t",  closetT,  1);
  o += jsonF(buf + o, len - o, "closet_rh", closetRH, 1);
  o += jsonF(buf + o, len - o, "outdoor_t",    ds_count > IDX_OUTDOOR    ? dsTemp[IDX_OUTDOOR]    : NAN, 1);
  o += jsonF(buf + o, len - o, "intake_t",  ds_count > IDX_INTAKE  ? dsTemp[IDX_INTAKE]  : NAN, 1);
  o += jsonF(buf + o, len - o, "exhaust_t", ds_count > IDX_EXHAUST ? dsTemp[IDX_EXHAUST] : NAN, 1);

  float duct_dt = NAN;
  float cdt = NAN;
  if (ds_count > IDX_EXHAUST &&
      !isnan(dsTemp[IDX_EXHAUST]) && !isnan(dsTemp[IDX_INTAKE]))
    duct_dt = dsTemp[IDX_EXHAUST] - dsTemp[IDX_INTAKE];
  
  o += jsonF(buf + o, len - o, "duct_dt", duct_dt, 1);
  
  if (ds_count > IDX_OUTDOOR &&
      !isnan(dsTemp[IDX_OUTDOOR]) && !isnan(closetT))
    cdt = closetT - dsTemp[IDX_OUTDOOR];
  o += jsonF(buf + o, len - o, "cdt", cdt, 2);
  
  //o += jsonF(buf + o, len - o, "lockout_s", lockoutLeftS(), 1);

  o += jsonF(buf + o, len - o, "sat_outdoor", satOutdoor, 1);

  o += snprintf(buf + o, len - o,
                "\"fan_pct\":%u,\"damper\":\"%s\","
                "\"mode\":\"%s\",\"mode_reason\":\"%s\","
                "\"setpoint\":%.1f,\"lockout_s\":%lu,",
                fanPct,
                digitalRead(PIN_DAMPER) ? "open" : "closed",
                modeName(mode), modeReason, setpointF,
                (unsigned long)lockoutLeftS());

  o += snprintf(buf + o, len - o,
                "\"bme_ok\":%s,\"bme_bad\":%lu,\"ds_count\":%d,"
                "\"ds_bad\":%lu,\"uptime\":%lu,\"ip\":\"%s\"}",
                bme_ok ? "true" : "false",
                (unsigned long)bme_bad, ds_count, (unsigned long)ds_bad,
                millis() / 1000UL,
                ETH.localIP().toString().c_str());
 
  return o;
}


void dsDump() {
  DeviceAddress a;
  LOG("[1WIRE] %d device(s)\n", ds_count);
  for (int i = 0; i < ds_count && i < DS_MAX; i++) {
    if (!ds.getAddress(a, i)) continue;
    LOG("[1WIRE]  %d: {", i);
    for (uint8_t b = 0; b < 8; b++) LOG("0x%02X%s", a[b], b < 7 ? ", " : "");
    //LOG("}  %.1fF\n", dsTemp[i]);
    // Read BY ADDRESS, not dsTemp[i]. dsTemp is indexed by ROLE and
    // bus order has nothing to do with role order, so dsTemp[i]
    // pairs each ROM with a different probe's reading -- precisely
    // backwards for the one job this function exists to do.
    LOG("}  %.1fF\n", ds.getTempF(a));
  }
}

void mqttPublishState() {
  if (!mqtt.connected()) return;
  char buf[512];
  buildStateJson(buf, sizeof(buf));
  mqtt.publish(TOPIC_STATE, buf, false);     // NOT retained -- see notes
}

// Publish immediately when something the operator cares about moves,
// rather than waiting out the heartbeat interval.
void mqttPublishIfChanged() {
  static uint8_t  lastFan    = 255;
  static int      lastDamper = -1;
  static CtrlMode lastMode   = MODE_FAULT;
  static bool     first      = true;

  int damper = digitalRead(PIN_DAMPER);
  if (first || fanPct != lastFan || damper != lastDamper || mode != lastMode) {
    first = false;
    lastFan = fanPct; lastDamper = damper; lastMode = mode;
    mqttPublishState();
  }
}


void mqttCallback(char *topic, byte *payload, unsigned int len) {
  char val[32];
  unsigned int n = len < sizeof(val) - 1 ? len : sizeof(val) - 1;
  memcpy(val, payload, n);
  val[n] = '\0';

  const char *leaf = strrchr(topic, '/');
  leaf = leaf ? leaf + 1 : topic;
  LOG("[MQTT] cmd %s = %s\n", leaf, val);

  if (!strcmp(leaf, "setpoint")) {
    // atof() yields 0.0 on garbage, which this range check rejects.
    float f = atof(val);
    if (f > 50.0f && f < 100.0f) setpointF = f;
    else LOG("[MQTT] bad setpoint '%s' -- ignored\n", val);

  } else if (!strcmp(leaf, "fan")) {
    if (!strcmp(val, "auto")) {
      setMode(MODE_SEALED, "auto");
    } else {
      // strtol, not atoi: atoi() returns 0 for unparseable input, so
      // a typo would stop the fan AND park the controller in manual
      // until the timeout. The broker is anonymous, so a stray
      // publish is a real path in here -- validate before acting.
      char *end;
      long pct = strtol(val, &end, 10);
      if (end == val || *end != '\0' || pct < 0 || pct > 100) {
        LOG("[MQTT] bad fan value '%s' -- ignored\n", val);
      } else {
        setMode(MODE_MANUAL, "mqtt fan");
        fanSet((uint8_t)pct);
      }
    }

  } else if (!strcmp(leaf, "damper")) {
    // Require an explicit word either way. Treating "anything that
    // is not 'open'" as close means a typo silently closes the
    // damper and enters manual, which is a bad thing to learn by
    // accident.
    bool open   = (!strcmp(val, "open")  || !strcmp(val, "1"));
    bool closed = (!strcmp(val, "close") || !strcmp(val, "closed") ||
                   !strcmp(val, "0"));
    if (!open && !closed) {
      LOG("[MQTT] bad damper value '%s' -- ignored\n", val);
    } else {
      setMode(MODE_MANUAL, "mqtt damper");
      damperCommand(open);
    }

  } else if (!strcmp(leaf, "reboot")) {
    // Require a deliberate payload. An empty or retained message on
    // this topic should not be able to restart the controller.
    if (!strcmp(val, "1") || !strcmp(val, "yes")) {
      LOG("[MQTT] reboot requested\n");
      delay(200);
      ESP.restart();
    } else {
      LOG("[MQTT] reboot ignored (send '1' or 'yes')\n");
    }
  }

  mqttPublishIfChanged();
}



// Non-blocking connect with backoff. Telemetry is never allowed to
// stall the control loop.
void mqttService() {
  static uint32_t lastTry = 0;
  static uint32_t lastPub = 0;

  if (!eth_connected) { mqtt_connected = false; return; }

  if (!mqtt.connected()) {
    mqtt_connected = false;
    if (millis() - lastTry < MQTT_RETRY_MS) return;
    lastTry = millis();

    bool ok =
#if defined(MQTT_USER)
      mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS,
                   TOPIC_STATUS, 0, true, "offline");
#else
      mqtt.connect(MQTT_CLIENT_ID, nullptr, nullptr,
                   TOPIC_STATUS, 0, true, "offline");
#endif
    if (ok) {
      mqtt_connected = true;
      mqtt.publish(TOPIC_STATUS, "online", true);   // retained
      mqtt.subscribe(TOPIC_CMD);
      LOG("[MQTT] connected to %s\n", MQTT_HOST);
      mqttPublishState();
    } else {
      LOG("[MQTT] connect failed, rc=%d\n", mqtt.state());
    }
    return;
  }

  mqtt.loop();
  mqtt_connected = true;

  if (millis() - lastPub >= MQTT_STATE_MS) {
    lastPub = millis();
    mqttPublishState();
  }
  mqttPublishIfChanged();
}

// ---------------------------------------------------------------
// Display drawing
// ---------------------------------------------------------------
#if DISPLAY_ENABLE

  #define DISPLAY_BLANK_MS    600000UL   // 5 min, then dark

  static unsigned long lastDisplayChange = 0;
  static unsigned long lastDisplayDraw   = 0;
  static unsigned long lastWakeAt        = 0;
  static bool          displayAwake      = true;
  static uint8_t       displayPage       = 0;
  static const uint8_t NUM_DISPLAY_PAGES = 4;

void drawEthIcon(int x, int y, bool up) {
  if (up) display.drawBox(x, y, 9, 7);
  else    display.drawFrame(x, y, 9, 7);
  display.drawBox(x + 3, y + 7, 3, 2);
}

void drawMqttIcon(int x, int y, bool up) {
  if (up) display.drawBox(x, y, 9, 9);
  else    display.drawFrame(x, y, 9, 9);
}

void drawHeader() {
  drawEthIcon(2, 1, eth_connected);
  drawMqttIcon(16, 0, mqtt_connected);

  int dotX = 128 - 4 - (NUM_DISPLAY_PAGES * 8);
  for (uint8_t i = 0; i < NUM_DISPLAY_PAGES; i++) {
    int cx = dotX + i * 8;
    if (i == displayPage) display.drawDisc(cx, 6, 2);
    else                  display.drawCircle(cx, 6, 2);
  }

  display.drawHLine(0, 13, 128);
}

// Page content starts at y=28 or below: drawStr's y is the text
// BASELINE, and the header rule sits at y=13.

void drawPageClimate() {
  display.setCursor(0, 30);
  if (bme_ok) display.printf("Room Temp: %.1f F", closetT);
  else        display.print("BME Fault");
  display.setCursor(0, 46);
  if (bme_ok) display.printf("Room RH: %.0f%%", closetRH);
  display.setCursor(0, 62);
  if (bme_ok) display.printf("Cdt: %.2f F", (closetT - dsTemp[IDX_OUTDOOR]));
}

void drawPageDuct() {
  if (ds_count == 0) { display.drawStr(0, 30, "no 1-wire"); return; }

  // dsTemp[] is indexed by ROLE, not bus position. ds_count says how
  // many devices answered on the wire and nothing about which roles
  // are populated -- an unfitted or stale role is NAN. Testing
  // ds_count here is what kept the exhaust line from ever drawing.
  display.setCursor(0, 30);
  if (isnan(dsTemp[IDX_OUTDOOR])) display.print("Outside: --");
  else                           display.printf("Outside: %.1f F", dsTemp[IDX_OUTDOOR]);
  
  display.setCursor(0, 46);
  if (isnan(dsTemp[IDX_INTAKE])) display.print("Intake: --");
  else                           display.printf("Intake: %.1f F", dsTemp[IDX_INTAKE]);

  display.setCursor(0, 62);
  if (isnan(dsTemp[IDX_EXHAUST])) {
    display.print("Exhaust: --");
  } else {
    display.printf("Exhaust: %.1f d%.1f F",
                   dsTemp[IDX_EXHAUST],
                   dsTemp[IDX_EXHAUST] - dsTemp[IDX_INTAKE]);
  }
}

static bool displayWake() {
  lastWakeAt = millis();
  if (displayAwake) return false;
  displayAwake = true;
  display.setPowerSave(0);
  return true;
}

void drawPageControl() {
  display.setCursor(0, 30);
  display.printf("Mode: %s", modeName(mode));
  display.setCursor(0, 46);
  display.printf("Fan Speed: %u%%", fanPct);
  display.setCursor(0, 62);
  display.printf("Damper: %s %s", digitalRead(PIN_DAMPER) ? "OPEN" : "CLOSED",
  freecoolLockUntil ? "LK" : "OK");
}

// millis() wraps at 49.7 days. Track the wraps so a box that is
// supposed to live in a wall for years can still tell you how long it
// has been up. Safe as long as this is called more often than every
// 49.7 days, which the display loop guarantees.

static uint64_t upMs() {
  static uint32_t last = 0;
  static uint64_t base = 0;
  uint32_t now = millis();
  if (now < last) base += 0x100000000ULL;
  last = now;
  return base + now;
}

void drawPageNet() {
  display.setCursor(0, 30);
  if (eth_connected) display.printf("IP: %s", ETH.localIP().toString().c_str());
  else               display.print("IP No Link");

  uint64_t s = upMs() / 1000ULL;
  display.setCursor(0, 62);
  display.printf("Up: %02u:%02u:%02u:%02u",
                 (unsigned)( s / 86400ULL),
                 (unsigned)((s /  3600ULL) % 24ULL),
                 (unsigned)((s /    60ULL) % 60ULL),
                 (unsigned)( s %    60ULL));
}



void renderDisplay() {
  if (millis() - lastDisplayChange >= DISPLAY_INTERVAL_MS) {
    lastDisplayChange = millis();
    displayPage = (displayPage + 1) % NUM_DISPLAY_PAGES;
  }

  // Never blank in fault. A lit panel is the only local indication
  // that something is wrong -- in a wall, with nobody reading MQTT.
  if (mode == MODE_FAULT) lastWakeAt = millis();

  if (displayAwake && (millis() - lastWakeAt) >= DISPLAY_BLANK_MS) {
    displayAwake = false;
    display.setPowerSave(1);
  }
  if (!displayAwake) return;

  display.clearBuffer();
  display.setFont(FONT_ROW);
  drawHeader();
  switch (displayPage) {
    case 0: drawPageClimate(); break;
    case 1: drawPageDuct();    break;
    case 2: drawPageControl(); break;
    case 3: drawPageNet();     break;
  }
  display.sendBuffer();
}


#else   // DISPLAY_ENABLE
// No panel to wake, so every press goes straight to its real action.
static bool displayWake() { return false; }
#endif  // DISPLAY_ENABLE

// ---------------------------------------------------------------
// OTA
// ---------------------------------------------------------------
void setupOTA() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA
    .onStart([]() {
      // The fan runs from a hardware LEDC timer and holds its duty
      // cycle through the flash. The SSRs do not -- park them in the
      // safe (de-energized) state before the image is replaced.
      digitalWrite(PIN_DAMPER,   LOW);
      digitalWrite(PIN_DEHU_FAN, LOW);
      if (mqtt.connected()) mqtt.publish(TOPIC_STATUS, "updating", true);
      Serial.println("[OTA] start");
    })
    .onProgress([](unsigned int p, unsigned int t) {
      Serial.printf("[OTA] %u%%\r", (p * 100) / t);
    })
    .onEnd([]() {
      Serial.println("\n[OTA] done -- rebooting");
    })
    .onError([](ota_error_t e) {
      Serial.printf("[OTA] error %u: ", e);
      switch (e) {
        case OTA_AUTH_ERROR:    Serial.println("auth failed");    break;
        case OTA_BEGIN_ERROR:   Serial.println("begin failed");   break;
        case OTA_CONNECT_ERROR: Serial.println("connect failed"); break;
        case OTA_RECEIVE_ERROR: Serial.println("receive failed"); break;
        case OTA_END_ERROR:     Serial.println("end failed");     break;
        default:                Serial.println("unknown");        break;
      }
    });

  ArduinoOTA.begin();
  Serial.printf("[OTA] listening on %s:3232\n",
                ETH.localIP().toString().c_str());

  TelnetStream.begin();
  Serial.println("[TELNET] listening on port 23");
}

// ---------------------------------------------------------------
// Ethernet events
// ---------------------------------------------------------------
void onEvent(arduino_event_id_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("[ETH] started");
      ETH.setHostname(OTA_HOSTNAME);
      break;

    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("[ETH] link up");
      break;

    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.println("---------------------------------");
      Serial.printf("  MAC     : %s\n", ETH.macAddress().c_str());
      Serial.printf("  IP      : %s\n", ETH.localIP().toString().c_str());
      Serial.printf("  Netmask : %s\n", ETH.subnetMask().toString().c_str());
      Serial.printf("  Gateway : %s\n", ETH.gatewayIP().toString().c_str());
      Serial.printf("  DNS     : %s\n", ETH.dnsIP().toString().c_str());
      Serial.printf("  Speed   : %d Mbps %s\n",
                    ETH.linkSpeed(),
                    ETH.fullDuplex() ? "full duplex" : "half duplex");
      Serial.println("---------------------------------");
      eth_connected = true;
      // Fire-and-forget: nothing below ever waits on valid time.
      configTzTime(TZ_LOCAL, NTP_SERVER);
      break;

    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("[ETH] link down");
      eth_connected = false;
      break;

    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("[ETH] stopped");
      eth_connected = false;
      break;

    default:
      break;
  }
}

// ---------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(2000);          // long enough to catch the 1-Wire ROM dump
  Serial.println("\n\nESP32-POE-ISO -- closet controller");

  // Outputs to the safe state FIRST: damper closed, dehu fan off.
  pinMode(PIN_DAMPER,   OUTPUT);  digitalWrite(PIN_DAMPER,   LOW);
  pinMode(PIN_DEHU_FAN, OUTPUT);  digitalWrite(PIN_DEHU_FAN, LOW);

  pinMode(PIN_BUT2,  INPUT);     // 2k2 pull-up on the Olimex board
  pinMode(PIN_BUTTON, INPUT);     // BUT1, 10k pull-up on the board

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(100000);

  bme_ok = bmeInit();
  Serial.println(bme_ok ? "[BME] ok" : "[BME] NOT FOUND");

  dsInit();
  fanInit();
  controlInit();

#if DISPLAY_ENABLE
  display.begin();
  display.setBusClock(100000);
  display.setFont(FONT_ROW);
  display.setContrast(255);
  display.setPowerSave(0);

  display.clearBuffer();
  display.drawStr(0, 30, "FAN CONTROLLER 1.0");
  display.drawStr(0, 46, "Booting...");
  display.sendBuffer();
  delay(2000);

#endif

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(768);        // default 256 is too small for the state doc
  mqtt.setKeepAlive(30);
  mqtt.setSocketTimeout(5);       // never block the loop for long

  Network.onEvent(onEvent);

  // (type, phy_addr, mdc, mdio, power, clk_mode)
  ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC,
            ETH_PHY_MDIO, ETH_PHY_POWER, ETH_CLK_MODE);
}

// ---------------------------------------------------------------

void loop() {
  // Must run every pass. Without it OTA silently never services.
  ArduinoOTA.handle();

  if (eth_connected && !ota_started) {
    setupOTA();
    ota_started = true;
  }

  dsService();
  mqttService();
  controlService();

#if DISPLAY_ENABLE
  // Frame-rate cap: a full 1KB framebuffer push at 100kHz is ~90ms,
  // so redrawing every pass would starve the OTA handler.
  if (millis() - lastDisplayDraw >= DISPLAY_REDRAW_MS) {
    lastDisplayDraw = millis();
    renderDisplay();
  }
#endif

  char lock[20] = "";
  if (freecoolLockUntil)
    snprintf(lock, sizeof(lock), "  lock=%lus", (unsigned long)lockoutLeftS());

  static bool lastBut1 = true, lastBut2 = true;

  bool b1 = digitalRead(PIN_BUTTON);
  if (lastBut1 && !b1 && !displayWake()) { i2cScan(); dsDump(); }
  lastBut1 = b1;

  bool b2 = digitalRead(PIN_BUT2);
  if (lastBut2 && !b2 && !displayWake()) {
    displayPage = (displayPage + 1) % NUM_DISPLAY_PAGES;
    lastDisplayChange = millis();          // restart the auto-cycle timer
  }
  lastBut2 = b2;

  // Heartbeat. No blocking network calls here, so an OTA push is
  // never left waiting on a socket timeout.
  static uint32_t last = 0;
  if (millis() - last > 10000) {
    last = millis();

    if (!bme_ok && bmeInit()) { bme_ok = true; LOG("[BME] recovered\n"); }
    readBME();

    if (eth_connected) {
      // Specifiers and arguments must stay paired. An argument added
      // without its specifier shifts everything after it and produces
      // plausible-looking nonsense.
      LOG("[UP] %lus  ip=%s  mqtt=%d  bme=%d  RoomT=%.1fF  RoomRH=%.0f%%  "
          "fan=%u%%  mode=%s  damper=%s",
          millis() / 1000UL,
          ETH.localIP().toString().c_str(),
          mqtt_connected,
          bme_ok,
          closetT,
          closetRH,
          fanPct,
          modeName(mode),
          digitalRead(PIN_DAMPER) ? "open" : "closed");

      LOG("  OutdoorT=%.1f  IntakeT=%.1f  ExhaustT=%.1f",
          dsTemp[IDX_OUTDOOR], dsTemp[IDX_INTAKE], dsTemp[IDX_EXHAUST]);

      //for (int i = 0; i < ds_count && i < DS_MAX; i++)
      //  LOG("  ds%d=%.1fF", i, dsTemp[i]);
      LOG("\n");
    } else {
      LOG("[ETH] waiting for link / DHCP ...\n");
    }
  }
}

/*
 * ---------------------------------------------------------------
 * MQTT TOPICS  (prefix is TOPIC_BASE from secrets.h)
 *
 *   <base>/state     one JSON object. NOT retained: retained
 *                    state goes stale and a new subscriber
 *                    cannot tell a live reading from a
 *                    two-week-old one. Liveness lives in
 *                    status instead.
 *   <base>/status    retained. "online" on connect, "updating"
 *                    during an OTA push, and the broker
 *                    publishes the "offline" LWT when the TCP
 *                    session drops. This is what makes a dead
 *                    controller visible without polling.
 *   <base>/cmd/+     setpoint | fan (0-100 or "auto") |
 *                    damper (open/closed) | reboot
 *
 * Missing sensors publish as null, never 0.0 -- downstream cannot
 * tell a zero from an absent probe otherwise.
 *
 * Test:
 *   mosquitto_sub -h <broker-ip> -t '<base>/#' -v
 *   mosquitto_pub -h <broker-ip> -t <base>/cmd/fan -m 40
 *   mosquitto_pub -h <broker-ip> -t <base>/cmd/fan -m auto
 *
 * ---------------------------------------------------------------
 * REV-A CARRIER ERRATA
 *
 * The GND net between the Olimex header and the carrier ground
 * pour was missing from the schematic entirely. Patched with a
 * wire on the rev-A board. Symptoms before the patch: intermittent
 * I2C reads (0xFFFFFF compensating to ~358F), a 2.89V reading on
 * the 3V3 rail at the sensor, and I2C corruption whenever an opto
 * LED was driven. Fix the schematic and tie BOTH Olimex GND pins
 * on the next spin.
 *
 * ---------------------------------------------------------------
 * esp32 Arduino core 2.x uses a different argument ORDER:
 *
 *   ETH.begin(ETH_PHY_ADDR, ETH_PHY_POWER, ETH_PHY_MDC,
 *             ETH_PHY_MDIO, ETH_PHY_TYPE, ETH_CLK_MODE);
 *
 * and the event handler takes WiFiEvent_t with SYSTEM_EVENT_ETH_*
 * constants, registered via WiFi.onEvent().
 *
 * ---------------------------------------------------------------
 * OTA from the command line (mDNS does not cross VLANs, so the
 * board will NOT appear under Tools > Port from another subnet):
 *
 *   ./flash.sh [ip]
 *
 * or directly:
 *
 *   python3 espota.py -i <board-ip> -p 3232 -P 40170 \
 *           -a <OTA_PASSWORD> -f build/poe_iso_eth_ota.ino.bin -r -d
 *
 * espota.py lives under the esp32 core's tools/ directory inside
 * ~/.arduino15/packages/esp32/hardware/esp32/<ver>/tools/.
 * ---------------------------------------------------------------
 */
