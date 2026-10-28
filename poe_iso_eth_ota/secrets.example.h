// ---------------------------------------------------------------
// secrets.example.h -- TEMPLATE
//
// Copy to secrets.h (same directory) and fill in your values.
// secrets.h is git-ignored; this file is the committed template.
//
// flash.sh reads OTA_PASSWORD and OTA_DEFAULT_IP from secrets.h,
// so keep those two lines in the simple form:
//   #define NAME   "value"
// ---------------------------------------------------------------
#pragma once

// ---- OTA ------------------------------------------------------
#define OTA_PASSWORD     "CHANGE-ME"
#define OTA_DEFAULT_IP   "192.168.1.50"     // flash.sh default target

// ---- MQTT -----------------------------------------------------
// Broker by IP, never by hostname: resolving through DNS would make
// a DNS outage take telemetry down with it.
#define MQTT_HOST        "192.168.1.10"
#define MQTT_PORT        1883
// Uncomment both if the broker requires authentication:
// #define MQTT_USER     "closet"
// #define MQTT_PASS     "CHANGE-ME"

#define TOPIC_BASE       "home/closet"

// ---- Time -----------------------------------------------------
#define NTP_SERVER       "pool.ntp.org"     // or a local NTP server IP
#define TZ_LOCAL         "CST6CDT,M3.2.0,M11.1.0"   // POSIX TZ string

// ---- Static addressing (optional) -----------------------------
// Call ETH.config() BEFORE ETH.begin(). Pick an address below the
// DHCP pool start.
//
//   IPAddress ip   (192, 168, 1, 50);
//   IPAddress gw   (192, 168, 1,  1);
//   IPAddress mask (255, 255, 255, 0);
//   IPAddress dns1 (192, 168, 1,  1);
//   ETH.config(ip, gw, mask, dns1);

// ---- DS18B20 probe ROM addresses ------------------------------
// Fit one probe at a time, press BUT1 to run dsDump(), and copy the
// address it prints into the matching role. An all-zero address
// marks a role as not fitted -- the loop treats it as NAN.
#define ROM_EXHAUST  {0, 0, 0, 0, 0, 0, 0, 0}
#define ROM_INTAKE   {0, 0, 0, 0, 0, 0, 0, 0}
#define ROM_OUTDOOR  {0, 0, 0, 0, 0, 0, 0, 0}
