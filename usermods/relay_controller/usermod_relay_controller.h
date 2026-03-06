#pragma once

/*
 * ============================================================
 *  Relay Controller Usermod for WLED
 *  File   : usermod_relay_controller.h
 *  Place  : wled00/usermods/relay_controller/
 *
 *  Features
 *  ─────────
 *  • GPIO / PCF8574 / MCP23017 relay control (up to 32 relays)
 *  • Pulse, timer, toggle commands
 *  • Pattern sequencer (bitmask steps)
 *  • NTP scheduling (per-relay on/off time + days mask)
 *  • Digital sensor inputs w/ debounce + relay automation
 *  • WLED MQTT topic integration (sub + pub)
 *  • Custom REST API  (/api/relays, /api/relay, ...)
 *  • WLED /json/state  seg[] ↔ relay mapping
 *  • WLED /json/info   relay count visible
 *  • Config persisted via WLED cfg.json (addToConfig)
 *  • Relay & sensor defs persisted in LittleFS
 *    (/rc_relays.json, /rc_sensors.json)
 *
 *  Compatibility
 *  ─────────────
 *  • WLED 0.14.x / 0.15.x  (SepTGut fork included)
 *  • ArduinoJson v6  (shipped with WLED)
 *  • ESP32 only (Wire / GPIO / LittleFS)
 *
 *  MQTT topics  (prefix = cfg.mqttPrefix, default "home/relay")
 *  ─────────────────────────────────────────────────────────────
 *  Sub  {prefix}            ON / OFF / T / JSON
 *  Sub  {prefix}/api        JSON state or HTTP-string
 *  Sub  {prefix}/relay/N    ON / OFF / T / PULSE:ms / TIMER:sec
 *  Sub  {prefix}/relay/N/api  {"on":…} {"pulse":…} {"timer":…}
 *  Sub  {prefix}/ping       → push full state
 *  Sub  {prefix}/col        ignored
 *  Sub  {group_topic}/#     same as above
 *
 *  Pub  {prefix}/v          full JSON state (retained)
 *  Pub  {prefix}/relay/N/v  "on" | "off"   (retained)
 *  Pub  {prefix}/g          0 | 255         (retained, WLED compat)
 *  Pub  {prefix}/c          #FFFFFF         (retained, WLED compat)
 *  Pub  {prefix}/status     online/offline  (LWT)
 *  Pub  {prefix}/sensor/N/v active | idle   (retained)
 * ============================================================
 */

#include "wled.h"

// ── Tune these to fit your memory budget ─────────────────────
#define RM_MAX_RELAYS        32
#define RM_MAX_SENSORS       16
#define RM_MAX_PATTERN_STEPS 16

#define RM_RELAYS_FILE  "/rc_relays.json"
#define RM_SENSORS_FILE "/rc_sensors.json"

// ── Unique usermod ID (not used by any official WLED usermod) ─
#ifndef USERMOD_ID_RELAY_CONTROLLER
  #define USERMOD_ID_RELAY_CONTROLLER 0xACDC
#endif

// ============================================================
//  Enums & structs
// ============================================================

enum RmExpanderType : uint8_t { RM_GPIO = 0, RM_PCF8574, RM_MCP23017 };

struct RmSchedule {
  bool    enabled  = false;
  uint8_t onHour   = 8,   onMin  = 0;
  uint8_t offHour  = 22,  offMin = 0;
  uint8_t days     = 0x7F; // bitmask: bit0=Sun … bit6=Sat
};

struct RmRelayDef {
  char           name[24]   = "Relay";
  RmExpanderType expType    = RM_GPIO;
  uint8_t        addr       = 0x20;   // I2C addr for expanders
  uint8_t        pin        = 0;      // GPIO or expander pin
  bool           activeLow  = false;
};

struct RmRelayRuntime {
  bool          state    = false;
  unsigned long timerEnd = 0;       // 0 = no active timer
  RmSchedule    schedule;
};

struct RmPatternStep {
  uint8_t  mask        = 0;    // bitmask of relays (bit0 = relay 0)
  uint16_t durationMs  = 500;
};

struct RmPatternEngine {
  bool          active       = false;
  RmPatternStep steps[RM_MAX_PATTERN_STEPS];
  int           numSteps     = 0;
  int           currentStep  = 0;
  int           repeat       = -1;   // -1=infinite, 0=once, N=N more
  unsigned long stepEnd      = 0;
};

enum RmTriggerMode : uint8_t {
  RM_TRIG_NONE = 0,
  RM_TRIG_ON,
  RM_TRIG_OFF,
  RM_TRIG_TOGGLE,
  RM_TRIG_MIRROR
};

struct RmSensorDef {
  char          name[24]       = "Sensor";
  uint8_t       pin            = 0;
  bool          activeLow      = false;
  bool          pullup         = true;
  uint16_t      debounceMs     = 50;
  int8_t        triggerRelay   = -1;   // -1 = no automation
  RmTriggerMode triggerMode    = RM_TRIG_NONE;
};

struct RmSensorRuntime {
  bool          state         = false;
  bool          rawLast       = false;
  unsigned long debounceEnd   = 0;
  unsigned long lastChangeMs  = 0;
};

// ============================================================
//  Usermod class
// ============================================================

class RelayControllerUsermod : public Usermod {

// ── Private members ─────────────────────────────────────────
private:

  // Relay data
  RmRelayDef     relayDefs[RM_MAX_RELAYS];
  RmRelayRuntime relayRt[RM_MAX_RELAYS];
  int            numRelays   = 0;

  // Sensor data
  RmSensorDef     sensorDefs[RM_MAX_SENSORS];
  RmSensorRuntime sensorRt[RM_MAX_SENSORS];
  int             numSensors = 0;

  // Pattern engine
  RmPatternEngine pattern;

  // I2C expander output caches (index = addr - 0x20)
  uint8_t pcf8574State[8]   = {};
  uint8_t mcp23017StateA[8] = {};
  uint8_t mcp23017StateB[8] = {};
  bool    pcf8574Used[8]    = {};
  bool    mcp23017Used[8]   = {};

  // Config (saved via addToConfig / readFromConfig)
  bool     enabled          = true;
  char     mqttPrefix[48]   = "home/relay";
  char     mqttGroupTopic[80] = "";
  uint16_t mqttStateInterval  = 0;   // seconds, 0=off

  // Loop timers
  unsigned long lastScheduleMs  = 0;
  unsigned long lastMqttStateMs = 0;
  int           lastCheckedMin  = -1;

  bool initDone = false;

  // ============================================================
  //  I2C expander helpers
  // ============================================================

  void initPCF8574(uint8_t addr) {
    int ai = addr - 0x20;
    if (pcf8574Used[ai]) return;
    pcf8574Used[ai] = true;
    Wire.beginTransmission(addr);
    Wire.write(pcf8574State[ai]);
    Wire.endTransmission();
  }

  void writePCF8574(uint8_t addr) {
    Wire.beginTransmission(addr);
    Wire.write(pcf8574State[addr - 0x20]);
    Wire.endTransmission();
  }

  void initMCP23017(uint8_t addr) {
    int ai = addr - 0x20;
    if (mcp23017Used[ai]) return;
    mcp23017Used[ai] = true;
    // IODIRA / IODIRB = all outputs
    Wire.beginTransmission(addr); Wire.write(0x00); Wire.write(0x00); Wire.endTransmission();
    Wire.beginTransmission(addr); Wire.write(0x01); Wire.write(0x00); Wire.endTransmission();
  }

  void writeMCP23017(uint8_t addr) {
    int ai = addr - 0x20;
    Wire.beginTransmission(addr); Wire.write(0x14); Wire.write(mcp23017StateA[ai]); Wire.endTransmission();
    Wire.beginTransmission(addr); Wire.write(0x15); Wire.write(mcp23017StateB[ai]); Wire.endTransmission();
  }

  // ── Raw hardware write (no state, no notifications) ─────────
  void writeRelayHW(int idx, bool on) {
    if (idx < 0 || idx >= numRelays) return;
    RmRelayDef& d = relayDefs[idx];
    bool level = d.activeLow ? !on : on;
    int  ai    = d.addr - 0x20;
    switch (d.expType) {
      case RM_GPIO:
        if (d.pin < 40) digitalWrite(d.pin, level ? HIGH : LOW);
        break;
      case RM_PCF8574:
        if (level) pcf8574State[ai] |=  (1 << d.pin);
        else       pcf8574State[ai] &= ~(1 << d.pin);
        writePCF8574(d.addr);
        break;
      case RM_MCP23017:
        if (d.pin < 8) {
          if (level) mcp23017StateA[ai] |=  (1 << d.pin);
          else       mcp23017StateA[ai] &= ~(1 << d.pin);
        } else {
          uint8_t p = d.pin - 8;
          if (level) mcp23017StateB[ai] |=  (1 << p);
          else       mcp23017StateB[ai] &= ~(1 << p);
        }
        writeMCP23017(d.addr);
        break;
    }
  }

  void initRelayHW(int idx) {
    RmRelayDef& d = relayDefs[idx];
    switch (d.expType) {
      case RM_GPIO:
        if (d.pin < 40) {
          pinMode(d.pin, OUTPUT);
          digitalWrite(d.pin, d.activeLow ? HIGH : LOW);
        }
        break;
      case RM_PCF8574:  initPCF8574(d.addr);  break;
      case RM_MCP23017: initMCP23017(d.addr); break;
    }
  }

  // ============================================================
  //  Relay control
  // ============================================================

  void setRelayState(int idx, bool on) {
    if (idx < 0 || idx >= numRelays) return;
    relayRt[idx].state   = on;
    relayRt[idx].timerEnd = 0;
    writeRelayHW(idx, on);
    mqttPublishRelay(idx);
  }

  void pulseRelay(int idx, uint32_t ms) {
    if (idx < 0 || idx >= numRelays) return;
    relayRt[idx].state    = true;
    relayRt[idx].timerEnd = millis() + ms;
    writeRelayHW(idx, true);
    mqttPublishRelay(idx);
  }

  void timerRelay(int idx, uint32_t sec) {
    pulseRelay(idx, sec * 1000UL);
  }

  void allOff() {
    for (int i = 0; i < numRelays; i++) {
      relayRt[i].state    = false;
      relayRt[i].timerEnd = 0;
      writeRelayHW(i, false);
    }
    pattern.active = false;
    mqttPublishFullState();
  }

  bool addRelayDef(const char* name, RmExpanderType expType,
                   uint8_t addr, uint8_t pin, bool activeLow) {
    if (numRelays >= RM_MAX_RELAYS) return false;
    int i = numRelays;
    strlcpy(relayDefs[i].name, name, sizeof(relayDefs[i].name));
    relayDefs[i].expType   = expType;
    relayDefs[i].addr      = addr;
    relayDefs[i].pin       = pin;
    relayDefs[i].activeLow = activeLow;
    relayRt[i] = RmRelayRuntime();
    initRelayHW(i);
    numRelays++;
    return true;
  }

  bool removeRelayDef(int id) {
    if (id < 0 || id >= numRelays) return false;
    writeRelayHW(id, false);
    for (int i = id; i < numRelays - 1; i++) {
      relayDefs[i] = relayDefs[i + 1];
      relayRt[i]   = relayRt[i + 1];
    }
    numRelays--;
    return true;
  }

  // ============================================================
  //  Pattern sequencer
  // ============================================================

  void startPattern(RmPatternStep* steps, int n, int repeat) {
    if (n <= 0 || n > RM_MAX_PATTERN_STEPS) return;
    memcpy(pattern.steps, steps, n * sizeof(RmPatternStep));
    pattern.numSteps    = n;
    pattern.currentStep = 0;
    pattern.repeat      = repeat;
    pattern.stepEnd     = millis() + steps[0].durationMs;
    pattern.active      = true;
    for (int i = 0; i < numRelays; i++)
      writeRelayHW(i, (steps[0].mask >> i) & 1);
  }

  void tickPattern() {
    if (!pattern.active) return;
    if (millis() < pattern.stepEnd) return;
    pattern.currentStep++;
    if (pattern.currentStep >= pattern.numSteps) {
      if (pattern.repeat == 0) { allOff(); pattern.active = false; return; }
      if (pattern.repeat  > 0) pattern.repeat--;
      pattern.currentStep = 0;
    }
    RmPatternStep& s = pattern.steps[pattern.currentStep];
    pattern.stepEnd = millis() + s.durationMs;
    for (int i = 0; i < numRelays; i++)
      writeRelayHW(i, (s.mask >> i) & 1);
  }

  // ============================================================
  //  NTP scheduling
  // ============================================================

  void checkSchedules() {
    struct tm t;
    if (!getLocalTime(&t)) return;
    int     key    = t.tm_hour * 60 + t.tm_min;
    if (key == lastCheckedMin) return;
    lastCheckedMin = key;
    uint8_t dayBit = 1 << t.tm_wday;
    for (int i = 0; i < numRelays; i++) {
      RmSchedule& s = relayRt[i].schedule;
      if (!s.enabled || !(s.days & dayBit)) continue;
      if (t.tm_hour == s.onHour  && t.tm_min == s.onMin)  setRelayState(i, true);
      if (t.tm_hour == s.offHour && t.tm_min == s.offMin) setRelayState(i, false);
    }
  }

  // ============================================================
  //  Digital sensors
  // ============================================================

  void initSensors() {
    for (int i = 0; i < numSensors; i++) {
      RmSensorDef& d = sensorDefs[i];
      if (d.pin >= 40) continue;
      pinMode(d.pin, d.pullup ? INPUT_PULLUP : INPUT);
      bool raw = digitalRead(d.pin);
      sensorRt[i].rawLast      = raw;
      sensorRt[i].state        = d.activeLow ? !raw : raw;
      sensorRt[i].lastChangeMs = millis();
      sensorRt[i].debounceEnd  = 0;
    }
  }

  void tickSensors() {
    unsigned long now = millis();
    for (int i = 0; i < numSensors; i++) {
      RmSensorDef&     d = sensorDefs[i];
      RmSensorRuntime& r = sensorRt[i];
      if (d.pin >= 40) continue;

      bool raw = digitalRead(d.pin);
      if (raw != r.rawLast) {
        r.rawLast     = raw;
        r.debounceEnd = now + d.debounceMs;
      }
      if (r.debounceEnd && now >= r.debounceEnd) {
        r.debounceEnd = 0;
        bool newState = d.activeLow ? !raw : raw;
        if (newState != r.state) {
          r.state        = newState;
          r.lastChangeMs = now;
          mqttPublishSensor(i);
          applySensorTrigger(i, newState);
        }
      }
    }
  }

  void applySensorTrigger(int idx, bool active) {
    RmSensorDef& d = sensorDefs[idx];
    if (d.triggerRelay < 0 || d.triggerRelay >= numRelays) return;
    switch (d.triggerMode) {
      case RM_TRIG_ON:     if (active)  setRelayState(d.triggerRelay, true);  break;
      case RM_TRIG_OFF:    if (active)  setRelayState(d.triggerRelay, false); break;
      case RM_TRIG_TOGGLE: if (active)  setRelayState(d.triggerRelay, !relayRt[d.triggerRelay].state); break;
      case RM_TRIG_MIRROR: setRelayState(d.triggerRelay, active); break;
      default: break;
    }
  }

  // ============================================================
  //  JSON helpers
  // ============================================================

  static const char* expanderTypeName(RmExpanderType t) {
    switch (t) {
      case RM_PCF8574:  return "pcf8574";
      case RM_MCP23017: return "mcp23017";
      default:          return "gpio";
    }
  }

  static RmExpanderType expanderTypeFromStr(const char* s) {
    if (strcmp(s, "pcf8574")  == 0) return RM_PCF8574;
    if (strcmp(s, "mcp23017") == 0) return RM_MCP23017;
    return RM_GPIO;
  }

  void relayToJson(JsonObject obj, int i) {
    obj[F("id")]          = i;
    obj[F("name")]        = relayDefs[i].name;
    obj[F("state")]       = relayRt[i].state;
    obj[F("pin")]         = relayDefs[i].pin;
    obj[F("activeLow")]   = relayDefs[i].activeLow;
    obj[F("type")]        = expanderTypeName(relayDefs[i].expType);
    obj[F("addr")]        = relayDefs[i].addr;
    long rem = relayRt[i].timerEnd > 0
               ? (long)((relayRt[i].timerEnd - millis()) / 1000) : 0;
    obj[F("timer_remaining")] = max(0L, rem);
    JsonObject sch = obj.createNestedObject(F("schedule"));
    sch[F("enabled")] = relayRt[i].schedule.enabled;
    sch[F("days")]    = relayRt[i].schedule.days;
    char buf[6];
    snprintf(buf, sizeof(buf), "%02d:%02d",
             relayRt[i].schedule.onHour, relayRt[i].schedule.onMin);
    sch[F("on")]  = buf;
    snprintf(buf, sizeof(buf), "%02d:%02d",
             relayRt[i].schedule.offHour, relayRt[i].schedule.offMin);
    sch[F("off")] = buf;
  }

  void sensorToJson(JsonObject obj, int i) {
    obj[F("id")]            = i;
    obj[F("name")]          = sensorDefs[i].name;
    obj[F("pin")]           = sensorDefs[i].pin;
    obj[F("activeLow")]     = sensorDefs[i].activeLow;
    obj[F("pullup")]        = sensorDefs[i].pullup;
    obj[F("debounce_ms")]   = sensorDefs[i].debounceMs;
    obj[F("state")]         = sensorRt[i].state;
    obj[F("trigger_relay")] = sensorDefs[i].triggerRelay;
    obj[F("trigger_mode")]  = (uint8_t)sensorDefs[i].triggerMode;
    obj[F("last_change_ms")] = sensorRt[i].lastChangeMs;
  }

  // ── Apply a JSON command object (all-relay or per-relay) ────
  void applyJsonCmd(JsonObject b) {
    // all on/off/toggle
    if (b.containsKey(F("on"))) {
      JsonVariant v = b[F("on")];
      bool target = false;
      if (v.is<bool>())        target = v.as<bool>();
      else if (v.is<int>())    target = v.as<int>() != 0;
      else {
        String s = v.as<String>(); s.toLowerCase();
        target = (s == "on" || s == "1" || s == "true");
        if (s == "t" || s == "toggle") {
          for (int i = 0; i < numRelays; i++)
            setRelayState(i, !relayRt[i].state);
          mqttPublishFullState();
          return;
        }
      }
      for (int i = 0; i < numRelays; i++) setRelayState(i, target);
    }

    // bri (WLED compat)
    if (b.containsKey(F("bri"))) {
      bool on = b[F("bri")].as<int>() > 0;
      for (int i = 0; i < numRelays; i++) setRelayState(i, on);
    }

    // seg[] → per-relay (WLED compat: seg id = relay index)
    if (b.containsKey(F("seg"))) {
      JsonArray segs = b[F("seg")].as<JsonArray>();
      for (JsonVariant sv : segs) {
        JsonObject seg = sv.as<JsonObject>();
        int id = seg[F("id")] | -1;
        if (id < 0 || id >= numRelays) continue;
        if (seg.containsKey(F("on")))  setRelayState(id, seg[F("on")].as<bool>());
        if (seg.containsKey(F("bri"))) setRelayState(id, seg[F("bri")].as<int>() > 0);
      }
    }

    // relays[] → multi-relay batch
    if (b.containsKey(F("relays"))) {
      JsonArray arr = b[F("relays")].as<JsonArray>();
      for (JsonVariant rv : arr) {
        JsonObject r = rv.as<JsonObject>();
        int id = r[F("id")] | -1;
        if (id < 0 || id >= numRelays) continue;
        if (r.containsKey(F("on")))    setRelayState(id, r[F("on")].as<bool>());
        if (r.containsKey(F("pulse"))) pulseRelay(id, r[F("pulse")].as<uint32_t>());
        if (r.containsKey(F("timer"))) timerRelay(id, r[F("timer")].as<uint32_t>());
      }
    }

    // single relay shortcut  {"relay":0,"on":true}
    if (b.containsKey(F("relay"))) {
      int id = b[F("relay")].as<int>();
      if (b.containsKey(F("on")))    setRelayState(id, b[F("on")].as<bool>());
      if (b.containsKey(F("pulse"))) pulseRelay(id, b[F("pulse")].as<uint32_t>());
      if (b.containsKey(F("timer"))) timerRelay(id, b[F("timer")].as<uint32_t>());
    }

    // pattern
    if (b.containsKey(F("pattern"))) {
      JsonVariant pv = b[F("pattern")];
      if (pv.is<const char*>() && String(pv.as<const char*>()) == "stop") {
        pattern.active = false; allOff();
      } else if (pv.is<JsonObject>()) {
        JsonObject po = pv.as<JsonObject>();
        JsonArray  sa = po[F("steps")].as<JsonArray>();
        int        rp = po[F("repeat")] | -1;
        RmPatternStep steps[RM_MAX_PATTERN_STEPS]; int n = 0;
        for (JsonVariant sv : sa) {
          if (n >= RM_MAX_PATTERN_STEPS) break;
          steps[n].mask       = sv[F("mask")]        | 0;
          steps[n].durationMs = sv[F("duration_ms")] | 500;
          n++;
        }
        if (n > 0) startPattern(steps, n, rp);
      }
    }

    // alloff shortcut
    if (b[F("alloff")] | false) allOff();
  }

  // ── Apply a string command to a single relay ────────────────
  void applyRelayStr(int id, const char* raw) {
    if (id < 0 || id >= numRelays) return;
    String s = raw; s.trim(); s.toUpperCase();
    if (s == "ON"  || s == "1")          { setRelayState(id, true);                           return; }
    if (s == "OFF" || s == "0")          { setRelayState(id, false);                          return; }
    if (s == "T"   || s == "TOGGLE")     { setRelayState(id, !relayRt[id].state);             return; }
    if (s.startsWith("PULSE:"))          { pulseRelay(id, s.substring(6).toInt());             return; }
    if (s.startsWith("TIMER:"))          { timerRelay(id, s.substring(6).toInt());             return; }
  }

  // ── Apply basic HTTP-string command ("T=1&SS=0&SV=1") ───────
  void applyHttpStr(const String& pl) {
    if (pl.indexOf("T=0") >= 0) { allOff(); return; }
    if (pl.indexOf("T=1") >= 0) { for (int i = 0; i < numRelays; i++) setRelayState(i, true); mqttPublishFullState(); return; }
    if (pl.indexOf("T=2") >= 0) { for (int i = 0; i < numRelays; i++) setRelayState(i, !relayRt[i].state); mqttPublishFullState(); return; }
    int ssPos = pl.indexOf("SS=");
    if (ssPos >= 0) {
      int id = pl.substring(ssPos + 3).toInt();
      bool sv = pl.indexOf("SV=1") >= 0;
      setRelayState(id, sv);
      mqttPublishFullState();
    }
  }

  // ============================================================
  //  Filesystem  (WLED_FS = LittleFS on ESP32)
  // ============================================================

  void loadRelays() {
    File f = WLED_FS.open(RM_RELAYS_FILE, "r");
    if (!f) { DEBUG_PRINTLN(F("[RelayCtrl] No relays file")); return; }
    DynamicJsonDocument doc(4096);
    if (deserializeJson(doc, f)) { f.close(); return; }
    f.close();
    numRelays = 0;
    JsonArray arr = doc[F("relays")].as<JsonArray>();
    for (JsonVariant rv : arr) {
      JsonObject r = rv.as<JsonObject>();
      bool ok = addRelayDef(
        r[F("name")]      | "Relay",
        expanderTypeFromStr(r[F("type")] | "gpio"),
        r[F("addr")]      | 0x20,
        r[F("pin")]       | 0,
        r[F("activeLow")] | false
      );
      if (!ok) break;
      int i = numRelays - 1;
      if (r.containsKey(F("schedule"))) {
        JsonObject sch              = r[F("schedule")];
        relayRt[i].schedule.enabled = sch[F("enabled")] | false;
        relayRt[i].schedule.days    = sch[F("days")]    | 0x7F;
        relayRt[i].schedule.onHour  = sch[F("onHour")]  | 8;
        relayRt[i].schedule.onMin   = sch[F("onMin")]   | 0;
        relayRt[i].schedule.offHour = sch[F("offHour")] | 22;
        relayRt[i].schedule.offMin  = sch[F("offMin")]  | 0;
      }
    }
    DEBUG_PRINTF("[RelayCtrl] Loaded %d relays\n", numRelays);
  }

  void saveRelays() {
    DynamicJsonDocument doc(4096);
    JsonArray arr = doc.createNestedArray(F("relays"));
    for (int i = 0; i < numRelays; i++) {
      JsonObject r = arr.createNestedObject();
      r[F("name")]      = relayDefs[i].name;
      r[F("type")]      = expanderTypeName(relayDefs[i].expType);
      r[F("pin")]       = relayDefs[i].pin;
      r[F("addr")]      = relayDefs[i].addr;
      r[F("activeLow")] = relayDefs[i].activeLow;
      JsonObject sch  = r.createNestedObject(F("schedule"));
      sch[F("enabled")] = relayRt[i].schedule.enabled;
      sch[F("days")]    = relayRt[i].schedule.days;
      sch[F("onHour")]  = relayRt[i].schedule.onHour;
      sch[F("onMin")]   = relayRt[i].schedule.onMin;
      sch[F("offHour")] = relayRt[i].schedule.offHour;
      sch[F("offMin")]  = relayRt[i].schedule.offMin;
    }
    File f = WLED_FS.open(RM_RELAYS_FILE, "w");
    if (!f) return;
    serializeJson(doc, f);
    f.close();
  }

  void loadSensors() {
    File f = WLED_FS.open(RM_SENSORS_FILE, "r");
    if (!f) return;
    DynamicJsonDocument doc(2048);
    if (deserializeJson(doc, f)) { f.close(); return; }
    f.close();
    numSensors = 0;
    JsonArray arr = doc[F("sensors")].as<JsonArray>();
    for (JsonVariant sv : arr) {
      if (numSensors >= RM_MAX_SENSORS) break;
      JsonObject s = sv.as<JsonObject>();
      int i = numSensors;
      strlcpy(sensorDefs[i].name, s[F("name")] | "Sensor", sizeof(sensorDefs[i].name));
      sensorDefs[i].pin          = s[F("pin")]           | 0;
      sensorDefs[i].activeLow    = s[F("activeLow")]     | false;
      sensorDefs[i].pullup       = s[F("pullup")]        | true;
      sensorDefs[i].debounceMs   = s[F("debounce_ms")]   | 50;
      sensorDefs[i].triggerRelay = s[F("trigger_relay")] | -1;
      sensorDefs[i].triggerMode  = (RmTriggerMode)(s[F("trigger_mode")] | 0);
      numSensors++;
    }
    DEBUG_PRINTF("[RelayCtrl] Loaded %d sensors\n", numSensors);
  }

  void saveSensors() {
    DynamicJsonDocument doc(2048);
    JsonArray arr = doc.createNestedArray(F("sensors"));
    for (int i = 0; i < numSensors; i++) {
      JsonObject s = arr.createNestedObject();
      s[F("name")]          = sensorDefs[i].name;
      s[F("pin")]           = sensorDefs[i].pin;
      s[F("activeLow")]     = sensorDefs[i].activeLow;
      s[F("pullup")]        = sensorDefs[i].pullup;
      s[F("debounce_ms")]   = sensorDefs[i].debounceMs;
      s[F("trigger_relay")] = sensorDefs[i].triggerRelay;
      s[F("trigger_mode")]  = (uint8_t)sensorDefs[i].triggerMode;
    }
    File f = WLED_FS.open(RM_SENSORS_FILE, "w");
    if (!f) return;
    serializeJson(doc, f);
    f.close();
  }

  // ============================================================
  //  MQTT publish helpers
  // ============================================================

  void mqttPublishRelay(int idx) {
    if (!WLED_MQTT_CONNECTED || idx < 0 || idx >= numRelays) return;
    char topic[80];
    snprintf(topic, sizeof(topic), "%s/relay/%d/v", mqttPrefix, idx);
    mqtt->publish(topic, relayRt[idx].state ? "on" : "off", true);
  }

  void mqttPublishFullState() {
    if (!WLED_MQTT_CONNECTED) return;
    DynamicJsonDocument doc(2048);
    bool anyOn = false;
    JsonArray arr = doc.createNestedArray(F("relays"));
    for (int i = 0; i < numRelays; i++) {
      JsonObject r = arr.createNestedObject();
      r[F("id")]   = i;
      r[F("name")] = relayDefs[i].name;
      r[F("on")]   = relayRt[i].state;
      long rem = relayRt[i].timerEnd > 0
                 ? (long)((relayRt[i].timerEnd - millis()) / 1000) : 0;
      r[F("timer")] = max(0L, rem);
      if (relayRt[i].state) anyOn = true;
    }
    doc[F("on")]  = anyOn;
    doc[F("num")] = numRelays;
    char buf[1800];
    serializeJson(doc, buf, sizeof(buf));
    char topic[80];
    snprintf(topic, sizeof(topic), "%s/v", mqttPrefix);
    mqtt->publish(topic, buf, true);
    // WLED compat brightness + colour topics
    snprintf(topic, sizeof(topic), "%s/g", mqttPrefix);
    mqtt->publish(topic, anyOn ? "255" : "0", true);
    snprintf(topic, sizeof(topic), "%s/c", mqttPrefix);
    mqtt->publish(topic, "#FFFFFF", true);
    lastMqttStateMs = millis();
  }

  void mqttPublishSensor(int idx) {
    if (!WLED_MQTT_CONNECTED || idx < 0 || idx >= numSensors) return;
    char topic[80];
    snprintf(topic, sizeof(topic), "%s/sensor/%d/v", mqttPrefix, idx);
    mqtt->publish(topic, sensorRt[idx].state ? "active" : "idle", true);
  }

  // ── Route an MQTT message against a given prefix ─────────────
  bool handleForPrefix(const char* prefix, const char* topic, const char* payload) {
    size_t plen = strlen(prefix);
    if (strncmp(topic, prefix, plen) != 0) return false;
    const char* sub = topic + plen;   // everything after the prefix

    // {prefix}  (bare topic)
    if (sub[0] == '\0') {
      String pl = payload; pl.trim(); String plu = pl; plu.toUpperCase();
      if (plu == "ON")  { for (int i=0;i<numRelays;i++) setRelayState(i,true);  mqttPublishFullState(); return true; }
      if (plu == "OFF") { for (int i=0;i<numRelays;i++) setRelayState(i,false); mqttPublishFullState(); return true; }
      if (plu == "T")   { for (int i=0;i<numRelays;i++) setRelayState(i,!relayRt[i].state); mqttPublishFullState(); return true; }
      if (pl[0] == '{') {
        DynamicJsonDocument doc(1024);
        if (!deserializeJson(doc, pl)) { applyJsonCmd(doc.as<JsonObject>()); mqttPublishFullState(); }
      }
      return true;
    }

    // {prefix}/api
    if (strcmp(sub, "/api") == 0) {
      String pl = payload; pl.trim();
      if (pl[0] == '{') {
        DynamicJsonDocument doc(1024);
        if (!deserializeJson(doc, pl)) { applyJsonCmd(doc.as<JsonObject>()); mqttPublishFullState(); }
      } else {
        applyHttpStr(pl);
      }
      return true;
    }

    // {prefix}/ping
    if (strcmp(sub, "/ping") == 0) { mqttPublishFullState(); return true; }

    // {prefix}/col  (ignored — relays have no colour)
    if (strcmp(sub, "/col") == 0) return true;

    // {prefix}/relay/N  or  {prefix}/relay/N/api
    if (strncmp(sub, "/relay/", 7) == 0) {
      const char* rest  = sub + 7;
      int         id    = atoi(rest);
      const char* slash = strchr(rest, '/');
      if (!slash) {
        applyRelayStr(id, payload);
        mqttPublishRelay(id);
        return true;
      }
      if (strcmp(slash, "/api") == 0) {
        DynamicJsonDocument doc(256);
        if (!deserializeJson(doc, payload)) {
          JsonObject r = doc.as<JsonObject>();
          if (r.containsKey(F("on")))    setRelayState(id, r[F("on")].as<bool>());
          if (r.containsKey(F("pulse"))) pulseRelay(id, r[F("pulse")].as<uint32_t>());
          if (r.containsKey(F("timer"))) timerRelay(id, r[F("timer")].as<uint32_t>());
          mqttPublishRelay(id);
        }
        return true;
      }
    }

    return false;
  }

  // ============================================================
  //  HTTP REST API  (/api/relays, /api/relay, ...)
  // ============================================================

  void registerEndpoints() {

    // GET /api/relays
    server.on("/api/relays", HTTP_GET, [this](AsyncWebServerRequest* req) {
      AsyncResponseStream* res = req->beginResponseStream("application/json");
      DynamicJsonDocument doc(4096);
      JsonArray arr = doc.createNestedArray(F("relays"));
      for (int i = 0; i < numRelays; i++) relayToJson(arr.createNestedObject(), i);
      serializeJson(doc, *res);
      req->send(res);
    });

    // GET /api/relay?id=N
    server.on("/api/relay", HTTP_GET, [this](AsyncWebServerRequest* req) {
      int id = req->hasParam("id") ? req->getParam("id")->value().toInt() : -1;
      if (id < 0 || id >= numRelays) { req->send(404, "application/json", F("{\"error\":\"not found\"}")); return; }
      AsyncResponseStream* res = req->beginResponseStream("application/json");
      DynamicJsonDocument doc(512);
      relayToJson(doc.as<JsonObject>(), id);
      serializeJson(doc, *res);
      req->send(res);
    });

    // POST /api/relay?id=N
    auto* relayPost = new AsyncCallbackJsonWebHandler("/api/relay",
      [this](AsyncWebServerRequest* req, JsonVariant& body) {
        int id = req->hasParam("id") ? req->getParam("id")->value().toInt() : -1;
        if (id < 0 || id >= numRelays) { req->send(404, "application/json", F("{\"error\":\"not found\"}")); return; }
        JsonObject b = body.as<JsonObject>();
        if (b.containsKey(F("name")))  strlcpy(relayDefs[id].name, b[F("name")].as<const char*>(), 24);
        if (b.containsKey(F("state"))) setRelayState(id, b[F("state")].as<bool>());
        if (b.containsKey(F("pulse"))) pulseRelay(id, b[F("pulse")].as<uint32_t>());
        if (b.containsKey(F("timer"))) timerRelay(id, b[F("timer")].as<uint32_t>());
        if (b.containsKey(F("schedule"))) {
          JsonObject sch = b[F("schedule")].as<JsonObject>();
          if (sch.containsKey(F("enabled"))) relayRt[id].schedule.enabled = sch[F("enabled")];
          if (sch.containsKey(F("days")))    relayRt[id].schedule.days    = sch[F("days")];
          if (sch.containsKey(F("on"))) {
            String t = sch[F("on")].as<String>();
            relayRt[id].schedule.onHour = t.substring(0, 2).toInt();
            relayRt[id].schedule.onMin  = t.substring(3, 5).toInt();
          }
          if (sch.containsKey(F("off"))) {
            String t = sch[F("off")].as<String>();
            relayRt[id].schedule.offHour = t.substring(0, 2).toInt();
            relayRt[id].schedule.offMin  = t.substring(3, 5).toInt();
          }
        }
        saveRelays();
        AsyncResponseStream* res = req->beginResponseStream("application/json");
        DynamicJsonDocument doc(512);
        relayToJson(doc.as<JsonObject>(), id);
        serializeJson(doc, *res);
        req->send(res);
      }
    );
    server.addHandler(relayPost);

    // POST /api/relays/add
    auto* addRelay = new AsyncCallbackJsonWebHandler("/api/relays/add",
      [this](AsyncWebServerRequest* req, JsonVariant& body) {
        JsonObject b = body.as<JsonObject>();
        bool ok = addRelayDef(
          b[F("name")]      | "Relay",
          expanderTypeFromStr(b[F("type")] | "gpio"),
          b[F("addr")]      | 0x20,
          b[F("pin")]       | 0,
          b[F("activeLow")] | false
        );
        if (!ok) { req->send(507, "application/json", F("{\"error\":\"max relays reached\"}")); return; }
        saveRelays();
        AsyncResponseStream* res = req->beginResponseStream("application/json");
        res->setCode(201);
        DynamicJsonDocument doc(512);
        relayToJson(doc.as<JsonObject>(), numRelays - 1);
        serializeJson(doc, *res);
        req->send(res);
      }
    );
    server.addHandler(addRelay);

    // POST /api/relay/remove?id=N
    server.on("/api/relay/remove", HTTP_POST, [this](AsyncWebServerRequest* req) {
      int id = req->hasParam("id") ? req->getParam("id")->value().toInt() : -1;
      if (!removeRelayDef(id)) { req->send(404, "application/json", F("{\"error\":\"not found\"}")); return; }
      saveRelays();
      req->send(200, "application/json", F("{\"ok\":true}"));
    });

    // POST /api/alloff
    server.on("/api/alloff", HTTP_POST, [this](AsyncWebServerRequest* req) {
      allOff();
      req->send(200, "application/json", F("{\"ok\":true}"));
    });

    // POST /api/pattern
    auto* patternStart = new AsyncCallbackJsonWebHandler("/api/pattern",
      [this](AsyncWebServerRequest* req, JsonVariant& body) {
        JsonObject b = body.as<JsonObject>();
        JsonArray sa = b[F("steps")].as<JsonArray>();
        int rp = b[F("repeat")] | -1;
        RmPatternStep steps[RM_MAX_PATTERN_STEPS]; int n = 0;
        for (JsonVariant sv : sa) {
          if (n >= RM_MAX_PATTERN_STEPS) break;
          steps[n].mask       = sv[F("mask")]        | 0;
          steps[n].durationMs = sv[F("duration_ms")] | 500;
          n++;
        }
        if (n > 0) startPattern(steps, n, rp);
        req->send(200, "application/json", F("{\"ok\":true}"));
      }
    );
    server.addHandler(patternStart);

    // POST /api/pattern/stop
    server.on("/api/pattern/stop", HTTP_POST, [this](AsyncWebServerRequest* req) {
      pattern.active = false; allOff();
      req->send(200, "application/json", F("{\"ok\":true}"));
    });

    // GET /api/sensors
    server.on("/api/sensors", HTTP_GET, [this](AsyncWebServerRequest* req) {
      AsyncResponseStream* res = req->beginResponseStream("application/json");
      DynamicJsonDocument doc(2048);
      JsonArray arr = doc.createNestedArray(F("sensors"));
      for (int i = 0; i < numSensors; i++) sensorToJson(arr.createNestedObject(), i);
      serializeJson(doc, *res);
      req->send(res);
    });

    // POST /api/sensors/add
    auto* addSensor = new AsyncCallbackJsonWebHandler("/api/sensors/add",
      [this](AsyncWebServerRequest* req, JsonVariant& body) {
        if (numSensors >= RM_MAX_SENSORS) { req->send(507, "application/json", F("{\"error\":\"max sensors\"}")); return; }
        JsonObject b = body.as<JsonObject>();
        int i = numSensors;
        strlcpy(sensorDefs[i].name, b[F("name")] | "Sensor", sizeof(sensorDefs[i].name));
        sensorDefs[i].pin          = b[F("pin")]           | 0;
        sensorDefs[i].activeLow    = b[F("activeLow")]     | false;
        sensorDefs[i].pullup       = b[F("pullup")]        | true;
        sensorDefs[i].debounceMs   = b[F("debounce_ms")]   | 50;
        sensorDefs[i].triggerRelay = b[F("trigger_relay")] | -1;
        sensorDefs[i].triggerMode  = (RmTriggerMode)(b[F("trigger_mode")] | 0);
        numSensors++;
        initSensors();
        saveSensors();
        AsyncResponseStream* res = req->beginResponseStream("application/json");
        res->setCode(201);
        DynamicJsonDocument doc(512);
        sensorToJson(doc.as<JsonObject>(), i);
        serializeJson(doc, *res);
        req->send(res);
      }
    );
    server.addHandler(addSensor);

    // POST /api/sensor/remove?id=N
    server.on("/api/sensor/remove", HTTP_POST, [this](AsyncWebServerRequest* req) {
      int id = req->hasParam("id") ? req->getParam("id")->value().toInt() : -1;
      if (id < 0 || id >= numSensors) { req->send(404, "application/json", F("{\"error\":\"not found\"}")); return; }
      for (int i = id; i < numSensors - 1; i++) {
        sensorDefs[i] = sensorDefs[i + 1];
        sensorRt[i]   = sensorRt[i + 1];
      }
      numSensors--;
      saveSensors();
      req->send(200, "application/json", F("{\"ok\":true}"));
    });

    // GET /api/i2c/scan
    server.on("/api/i2c/scan", HTTP_GET, [](AsyncWebServerRequest* req) {
      AsyncResponseStream* res = req->beginResponseStream("application/json");
      DynamicJsonDocument doc(512);
      JsonArray arr = doc.createNestedArray(F("found"));
      for (uint8_t a = 1; a < 127; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) arr.add(a);
      }
      serializeJson(doc, *res);
      req->send(res);
    });
  }

// ── Public WLED usermod interface ───────────────────────────
public:

  // ── setup() — called once after WLED finishes its own setup ──
  void setup() override {
    if (!enabled) return;
    loadRelays();
    loadSensors();
    initSensors();
    registerEndpoints();
    initDone = true;
    DEBUG_PRINTLN(F("[RelayCtrl] setup complete"));
  }

  // ── loop() — called every WLED loop iteration ──────────────
  void loop() override {
    if (!enabled || !initDone) return;
    unsigned long now = millis();

    // Relay auto-off timers
    for (int i = 0; i < numRelays; i++) {
      if (relayRt[i].timerEnd && now >= relayRt[i].timerEnd) {
        setRelayState(i, false);
      }
    }

    // Pattern sequencer
    tickPattern();

    // NTP schedules (1-second resolution)
    if (now - lastScheduleMs >= 1000UL) {
      lastScheduleMs = now;
      checkSchedules();
    }

    // Digital sensors
    tickSensors();

    // Periodic MQTT state broadcast
    if (mqttStateInterval > 0 && WLED_MQTT_CONNECTED) {
      if (now - lastMqttStateMs >= (unsigned long)mqttStateInterval * 1000UL) {
        mqttPublishFullState();
      }
    }
  }

  // ── addToJsonState() — add relay data to GET /json/state ────
  void addToJsonState(JsonObject& root) override {
    JsonObject rc = root.createNestedObject(F("rc"));
    bool anyOn = false;
    JsonArray arr = rc.createNestedArray(F("relays"));
    for (int i = 0; i < numRelays; i++) {
      JsonObject r = arr.createNestedObject();
      r[F("id")]   = i;
      r[F("name")] = relayDefs[i].name;
      r[F("on")]   = relayRt[i].state;
      long rem = relayRt[i].timerEnd > 0
                 ? (long)((relayRt[i].timerEnd - millis()) / 1000) : 0;
      r[F("timer")] = max(0L, rem);
      if (relayRt[i].state) anyOn = true;
    }
    rc[F("anyOn")]   = anyOn;
    rc[F("pattern")] = pattern.active;
    rc[F("num")]     = numRelays;
    // Sensor summary
    JsonArray sarr = rc.createNestedArray(F("sensors"));
    for (int i = 0; i < numSensors; i++) {
      JsonObject s = sarr.createNestedObject();
      s[F("id")]    = i;
      s[F("name")]  = sensorDefs[i].name;
      s[F("state")] = sensorRt[i].state;
    }
  }

  // ── readFromJsonState() — apply state from POST /json/state ──
  void readFromJsonState(JsonObject& root) override {
    if (!initDone) return;
    // Native WLED seg[] → relay mapping
    if (root.containsKey(F("seg"))) applyJsonCmd(root);
    // Our own namespace  {"rc":{"on":true}} or {"rc":{"relays":[...]}}
    if (root.containsKey(F("rc"))) {
      JsonObject rc = root[F("rc")].as<JsonObject>();
      applyJsonCmd(rc);
    }
  }

  // ── addToJsonInfo() — append relay info to GET /json/info ───
  void addToJsonInfo(JsonObject& root) override {
    JsonObject relay = root.createNestedObject(F("relay"));
    relay[F("count")]   = numRelays;
    relay[F("sensors")] = numSensors;
    relay[F("pattern")] = pattern.active;
    bool anyOn = false;
    for (int i = 0; i < numRelays; i++) if (relayRt[i].state) { anyOn = true; break; }
    relay[F("anyOn")] = anyOn;
  }

  // ── addToConfig() — save usermod settings to cfg.json ───────
  void addToConfig(JsonObject& root) override {
    JsonObject rc = root.createNestedObject(F("RelayCtrl"));
    rc[F("enabled")]      = enabled;
    rc[F("mqttPrefix")]   = mqttPrefix;
    rc[F("mqttGroup")]    = mqttGroupTopic;
    rc[F("mqttInterval")] = mqttStateInterval;
  }

  // ── readFromConfig() — load settings from cfg.json ──────────
  bool readFromConfig(JsonObject& root) override {
    JsonObject rc = root[F("RelayCtrl")];
    if (rc.isNull()) return false;
    bool prevEnabled = enabled;
    enabled           = rc[F("enabled")]      | true;
    strlcpy(mqttPrefix,     rc[F("mqttPrefix")]   | "home/relay", sizeof(mqttPrefix));
    strlcpy(mqttGroupTopic, rc[F("mqttGroup")]    | "",           sizeof(mqttGroupTopic));
    mqttStateInterval = rc[F("mqttInterval")] | 0;
    // If usermod was re-enabled after config load, re-init
    if (!prevEnabled && enabled && initDone) {
      loadRelays(); loadSensors(); initSensors();
    }
    return true;
  }

  // ── onMqttConnect() — subscribe to our topics ───────────────
  void onMqttConnect(bool sessionPresent) override {
    if (!enabled) return;
    char topic[96];
    // Subscribe to {prefix}/#
    snprintf(topic, sizeof(topic), "%s/#", mqttPrefix);
    mqtt->subscribe(topic);
    // Subscribe to group topic if configured
    if (mqttGroupTopic[0]) {
      snprintf(topic, sizeof(topic), "%s/#", mqttGroupTopic);
      mqtt->subscribe(topic);
    }
    // Publish LWT online
    snprintf(topic, sizeof(topic), "%s/status", mqttPrefix);
    mqtt->publish(topic, "online", true);
    // Push current state immediately
    mqttPublishFullState();
    for (int i = 0; i < numRelays;  i++) mqttPublishRelay(i);
    for (int i = 0; i < numSensors; i++) mqttPublishSensor(i);
    DEBUG_PRINTLN(F("[RelayCtrl] MQTT connected, topics subscribed"));
  }

  // ── onMqttMessage() — handle incoming MQTT ──────────────────
  bool onMqttMessage(char* topic, char* payload) override {
    if (!enabled || !initDone) return false;
    if (handleForPrefix(mqttPrefix, topic, payload)) return true;
    if (mqttGroupTopic[0] && handleForPrefix(mqttGroupTopic, topic, payload)) return true;
    return false;
  }

  // ── Usermod metadata ─────────────────────────────────────────
  uint16_t getId()    override { return USERMOD_ID_RELAY_CONTROLLER; }
  const char* getName() const  { return "RelayController"; }
};
