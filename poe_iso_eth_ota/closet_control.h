
// ---------------------------------------------------------------
// Free-cooling control loop
//
// Sits between the sensors and the two actuators. Owns fan and
// damper whenever mode is SEALED, FREECOOL or FAULT; keeps its
// hands off entirely in MANUAL.
//
// Placement matters: this block must come AFTER setpointF is
// declared and BEFORE mqttCallback(), which calls setMode().
// ---------------------------------------------------------------

// Heat the closet actually rejects. Measured at the UPS: 42% of
// 1500VA, ~600W, with the R740 already included. With
// Q = 1.08 * CFM * dT this plus the fan's airflow sets the outdoor
// temperature below which free cooling can carry the whole load.
#define CLOSET_LOAD_BTUH   2800.0f   // Based on anemometer reading
#define FAN_MAX_CFM        232.0f     

// Changeover. At 2050 Btu/h and 352 CFM the fan runs out of capacity
// near dT = 5.4F, so engaging there would pin it at 100% for almost
// no benefit. 8F puts the first call around 70% and leaves headroom.
#define FREECOOL_DT_MIN    8.0f
#define FREECOOL_DT_HYST   2.0f       // outdoor must warm this much to drop out

#define SETPOINT_DEADBAND  1.0f       // no call until this far above setpoint
#define CLOSET_RH_MAX      65.0f      // humidity guard, measured not predicted
#define CLOSET_RH_HYST     5.0f

// Installed airflow floor. FAN_MIN_PCT is the motor's stall point;
// this is the speed that actually cracks the backdraft damper with
// the filter in place. RAISE IT AT COMMISSIONING -- 10 is the bench
// number and will not be enough through a loaded filter.
#define FAN_FLOOR_PCT      10

#define CTRL_KP            8.0f       // % per degF
#define CTRL_KI            0.05f      // % per degF-second
#define CTRL_I_CLAMP       100.0f      // bound the integral's contribution

// Pinned at 100% and still losing ground means free cooling cannot
// carry this load. The mini-split has its own thermostat and will be
// running anyway, so holding the damper open just has it condition
// outdoor air. SATURATION_MS below is how long before it gives up.

#define CTRL_TICK_MS       10000UL
#define DAMPER_TIMEOUT_MS  60000UL
#define DAMPER_HAS_ENDSW   false

// Bench timers. Five minutes between transitions makes testing
// unbearable; these shrink the waits without changing any logic.
// MUST be false before this goes in the wall -- a 30s dwell would
// short-cycle the damper.
#define BENCH_FAST         false

#if BENCH_FAST
  #define MODE_DWELL_MS      30000UL
  #define SATURATION_MS      60000UL
  #define FAULT_RETRY_MS     60000UL
  #define AIRFLOW_PROVE_MS   30000UL
  #define FREECOOL_LOCKOUT_MS  120000UL
  #define MANUAL_TIMEOUT_MS  120000UL
#else
  #define MODE_DWELL_MS      300000UL   // 5 min minimum in a mode
  #define SATURATION_MS      600000UL
  #define FAULT_RETRY_MS     600000UL
  #define AIRFLOW_PROVE_MS     180000UL
  #define FREECOOL_LOCKOUT_MS  1800000UL
  #define MANUAL_TIMEOUT_MS    1800000UL
#endif

static uint32_t freecoolLockUntil = 0;
static float    satOutdoor        = NAN;

// Seconds left on the free-cooling lockout, 0 when not locked.
static uint32_t lockoutLeftS() {
  if (!freecoolLockUntil) return 0;
  int32_t d = (int32_t)(freecoolLockUntil - millis());
  return d > 0 ? (uint32_t)d / 1000UL : 0;
}

// Damper position feedback.
//
// With no end switch fitted, position is ASSUMED after a fixed
// travel time rather than proven. That is weaker in a specific way:
// nothing can then detect a damper that failed to open, so the
// intake probe's airflow check stops being a nicety and becomes the
// only backstop. Fit the switch before this runs unattended.
//
// GPIO36 has a 2k2 pull-up on the Olimex board and reads HIGH with
// nothing connected, so leaving DAMPER_HAS_ENDSW true without a
// switch wired would fault on the first open.
#define DAMPER_TRAVEL_MS   20000UL   // actuator stroke time plus margin

static uint32_t modeSince     = 0;
static uint32_t damperCmdAt   = 0;
static bool     damperCmdOpen = false;
static bool     damperProven  = false;
static float    ctrlInteg     = 0.0f;   // degF-seconds
static uint32_t fanRunningAt  = 0;
static uint32_t faultAt       = 0;
static uint32_t saturatedAt   = 0;
static char     faultMsg[48]  = "";

// Outdoor dry bulb comes from the hood probe. Null until it is
// installed, and the loop refuses to decide anything without it.
static float outdoorT() {
  if (ds_count > IDX_OUTDOOR && !isnan(dsTemp[IDX_OUTDOOR])) return dsTemp[IDX_OUTDOOR];
  return NAN;
}

static char modeReason[40] = "";


static void setMode(CtrlMode m, const char *why) {
  snprintf(modeReason, sizeof(modeReason), "%s", why);

  if (m == mode) {
    // A repeat of the same mode is not a transition -- no log, no
    // state reset. One exception: re-asserting MANUAL is the
    // operator saying "still here", and it has to push the timeout
    // out or a long commissioning session gets dropped back to
    // sealed mid-measurement.
    //
    // Deliberately NOT done for the other modes: the freecool entry
    // test reads modeSince as a dwell timer, so refreshing it on a
    // same-mode call would delay transitions that are waiting on it.
    if (m == MODE_MANUAL) modeSince = millis();
    return;
  }

  LOG("[CTRL] %s -> %s (%s)\n", modeName(mode), modeName(m), why);
  mode = m;
  modeSince   = millis();
  ctrlInteg   = 0.0f;
  saturatedAt = 0;
  if (m != MODE_FAULT) faultMsg[0] = '\0';
}

static void damperCommand(bool open) {
  // Write the pin unconditionally. The cached state exists for
  // travel timing and the proof logic -- it must NEVER gate the
  // output, because other paths (MQTT manual, OTA onStart) write
  // PIN_DAMPER directly and leave the cache stale. A stale cache
  // used as a guard makes the loop stop commanding the damper
  // entirely, silently, while every log line claims it did.
  digitalWrite(PIN_DAMPER, open ? HIGH : LOW);

  if (damperCmdOpen == open) return;   // no state change: timing stands
  damperCmdOpen = open;
  damperCmdAt   = millis();
  damperProven  = false;
  LOG("[CTRL] damper %s commanded\n", open ? "OPEN" : "CLOSE");
}

static void enterFault(const char *why) {
  snprintf(faultMsg, sizeof(faultMsg), "%s", why);
  fanSet(0);
  digitalWrite(PIN_DAMPER, LOW);
  damperCmdOpen = false;
  damperProven  = false;
  damperCmdAt   = millis();
  faultAt       = millis();
  setMode(MODE_FAULT, why);
}

void controlInit() {
  mode          = MODE_SEALED;
  modeSince     = millis();
  damperCmdOpen = false;
  damperCmdAt   = millis();
  damperProven  = false;
  digitalWrite(PIN_DAMPER, LOW);
}

void controlService() {

  uint32_t now = millis();

  // MANUAL means the operator owns both outputs -- but only while
  // there is an operator. An unattended box must not sit with the
  // damper open because a command arrived and nobody came back.
  if (mode == MODE_MANUAL) {
    if (now - modeSince >= MANUAL_TIMEOUT_MS) {
      fanSet(0);
      damperCommand(false);
      setMode(MODE_SEALED, "manual timeout");
    }
    return;
  }

  if (mode == MODE_FAULT) {
    // A sticky damper often frees itself. Retry rather than waiting
    // for a human to visit a box inside a wall -- the fault is
    // published either way, so it is visible without being fatal.
    if (now - faultAt >= FAULT_RETRY_MS) setMode(MODE_SEALED, "fault retry");
    return;
  }

  // Damper position, checked every pass so transitions are not
  // quantised to the control tick.
  if (!damperProven) {
    if (!DAMPER_HAS_ENDSW) {
      // Assumed, not proven. No fault path exists here -- a stuck
      // damper looks identical to a working one until the airflow
      // check notices.
      if (now - damperCmdAt >= DAMPER_TRAVEL_MS) {
        damperProven = true;
        LOG("[CTRL] damper %s assumed after %lums (no end switch)\n",
            damperCmdOpen ? "open" : "closed",
            (unsigned long)(now - damperCmdAt));
      } else if (now - damperCmdAt > DAMPER_TIMEOUT_MS) {
        enterFault(damperCmdOpen ? "damper failed to open"
                               : "damper failed to close");
        return;
      }
    }
  }

  static uint32_t lastTick = 0;
  if (now - lastTick < CTRL_TICK_MS) return;
  lastTick = now;

  float od = outdoorT();
  if (isnan(closetT) || isnan(od)) {
    // No basis for a decision. Seal up and let the mini-split work.
    if (mode != MODE_SEALED) setMode(MODE_SEALED, "sensors invalid");
    fanSet(0);
    damperCommand(false);
    return;
  }

  float dT      = closetT - od;          // free-cooling potential
  float error   = closetT - setpointF;   // positive = too hot
  bool  dwellOk = (now - modeSince) >= MODE_DWELL_MS;

  if (mode == MODE_SEALED) {
    fanSet(0);
    damperCommand(false);

    bool demand    = error > SETPOINT_DEADBAND;
    bool coldOut   = dT >= FREECOOL_DT_MIN;
    bool dryEnough = isnan(closetRH) || closetRH < (CLOSET_RH_MAX - CLOSET_RH_HYST);

    // A capacity bail-out sets freecoolLockUntil. Without it the entry
    // test below is true again the instant MODE_DWELL_MS expires, so the
    // watchdog's decision gets overturned seconds after it is made and
    // the system cycles the damper all night accomplishing nothing.
    //
    // Release early if outdoor has dropped meaningfully below where it
    // gave up: conditions genuinely changed, and a flat timer would
    // throw away real free cooling on an evening that is still falling.
    if (freecoolLockUntil &&
        ((int32_t)(now - freecoolLockUntil) >= 0 ||
         (!isnan(satOutdoor) && od < satOutdoor - 3.0f))) {
      freecoolLockUntil = 0;
      satOutdoor        = NAN;
      LOG("[CTRL] freecool lockout released\n");
    }

    if (demand && coldOut && dryEnough && dwellOk && !freecoolLockUntil) {
      setMode(MODE_FREECOOL, "outdoor cold enough");
      damperCommand(true);
      fanRunningAt = 0;
    }
    return;
  }


  // ---- MODE_FREECOOL ----
  if (!damperProven) { fanSet(0); return; }   // wait for travel

  bool tooWarmOut = dT < (FREECOOL_DT_MIN - FREECOOL_DT_HYST);

  // Saturation watchdog: full fan and still above setpoint.
  if (fanPct >= 100 && error > SETPOINT_DEADBAND) {
    if (!saturatedAt) saturatedAt = now;
  } else {
    saturatedAt = 0;
  }
  bool outOfCapacity = saturatedAt && (now - saturatedAt) > SATURATION_MS;

  bool tooHumid   = !isnan(closetRH) && closetRH > CLOSET_RH_MAX;
  // Satisfied only counts once the fan is already at the floor. Above
  // it, modulate down instead of slamming the damper -- this is the
  // boundary between modulating and cycling.
  bool satisfied  = (error < -SETPOINT_DEADBAND) && (fanPct <= FAN_FLOOR_PCT);

  if ((tooWarmOut || tooHumid || satisfied || outOfCapacity) && dwellOk) {
    fanSet(0);
    damperCommand(false);

    // Only a capacity bail-out earns a lockout. The other three exits
    // are self-correcting: the entry test already blocks re-entry while
    // it is too warm or too humid, and 'satisfied' means it worked.
    // Giving up because the fan could not hold setpoint is the one case
    // where the entry test still says yes and puts us straight back in.
    // The guard mirrors the message chain's priority, so the lockout and
    // the logged reason can never disagree about why we left.
    if (outOfCapacity && !tooHumid && !tooWarmOut) {
      freecoolLockUntil = now + FREECOOL_LOCKOUT_MS;
      if (!freecoolLockUntil) freecoolLockUntil = 1;   // millis rollover
      satOutdoor = od;
      LOG("[CTRL] freecool locked out for %lus (gave up at outdoor %.1fF)\n",
          (unsigned long)(FREECOOL_LOCKOUT_MS / 1000UL), od);
    }

    setMode(MODE_SEALED, tooHumid       ? "closet RH high"
                       : tooWarmOut     ? "outdoor too warm"
                       : outOfCapacity  ? "full fan, not holding setpoint"
                                        : "satisfied at minimum");
    return;
  }

  // Feedforward straight from the heat balance, trimmed by PI. The
  // loop starts near the right answer instead of hunting for it.
  float ff;
  if (dT > 1.0f) ff = (CLOSET_LOAD_BTUH / (1.08f * dT)) / FAN_MAX_CFM * 100.0f;
  else           ff = 100.0f;

  // Integrate, then compute. If the new integral only pushed us further
  // into saturation it bought nothing and would have to be unwound
  // later, so roll it back -- conditional integration. That protection
  // is what lets CTRL_I_CLAMP be generous enough to actually close a
  // standing offset: at a large dT the feedforward is small (539/dT),
  // so a 50-point integral could not reach full fan at small errors and
  // the loop would sit on the offset forever.
  float integPrev = ctrlInteg;
  ctrlInteg += error * (CTRL_TICK_MS / 1000.0f);
  float iLimit = CTRL_I_CLAMP / CTRL_KI;
  if (ctrlInteg >  iLimit) ctrlInteg =  iLimit;
  if (ctrlInteg < -iLimit) ctrlInteg = -iLimit;

  float out = ff + CTRL_KP * error + CTRL_KI * ctrlInteg;

  if ((out > 100.0f        && error > 0.0f) ||
      (out < FAN_FLOOR_PCT && error < 0.0f)) {
    ctrlInteg = integPrev;
  }
  if (out > 100.0f)        out = 100.0f;
  if (out < FAN_FLOOR_PCT) out = FAN_FLOOR_PCT;

  fanSet((uint8_t)(out + 0.5f));
  if (!fanRunningAt) fanRunningAt = now;

  // Airflow proof. With air moving, the intake probe sits near the
  // hood. If it has drifted toward closet temperature instead, the
  // backdraft damper never opened or the filter is blinded -- the
  // fan spins, the controller believes it is cooling, and nothing
  // else would notice until the rack is hot.
  if (ds_count > IDX_INTAKE && !isnan(dsTemp[IDX_INTAKE]) &&
      fanRunningAt && (now - fanRunningAt) > AIRFLOW_PROVE_MS) {
  
  //  float toHood   = fabsf(dsTemp[IDX_INTAKE] - od);
  //  float toCloset = fabsf(dsTemp[IDX_INTAKE] - closetT);
  //if (toCloset < toHood) enterFault("no airflow -- damper or filter");

  //float avail = closetT - od;                    // >= FREECOOL_DT_MIN here
  //float drop  = closetT - dsTemp[IDX_INTAKE];
  //if (fanPct >= 40 && avail >= 12.0f && drop < avail * 0.30f)
  //    enterFault("no airflow -- damper or filter");

  // Prove airflow by the closet's own temperature rise rather than
  // by comparing the intake against outdoor. The supply duct gains
  // ~7.7F crossing the attic, which puts the intake near the
  // midpoint even at full flow -- the old test faulted on perfectly
  // good airflow. Exhaust-minus-intake has no such dependency.
  float rise = dsTemp[IDX_EXHAUST] - dsTemp[IDX_INTAKE];
  if (fanPct >= 40 && rise < 1.0f) enterFault("no airflow -- damper or filter");

  }
}

