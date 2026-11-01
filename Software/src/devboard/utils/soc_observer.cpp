#include "soc_observer.h"

#include <Arduino.h>
#include <math.h>
#include "../../communication/nvm/comm_nvm.h"
#include "../../datalayer/datalayer.h"
#include "logging.h"

bool soc_observer_enabled = false;
uint32_t soc_observer_capacity_Wh = 0;
uint16_t soc_observer_full_mV = 3360;
uint16_t soc_observer_empty_mV = 3150;
uint16_t soc_observer_full_pptt = 10000;
uint16_t soc_observer_empty_pptt = 0;
uint16_t soc_observer_anchor_max_W = 500;
uint16_t soc_observer_timeout_s = 60;

/* An anchor has to hold this long before it counts, so a spike or a sag cannot reset the counter */
static constexpr float ANCHOR_DEBOUNCE_S = 20.0f;
/* Longest step integrated in one go. A stalled loop must not turn into a huge energy jump */
static constexpr float MAX_STEP_S = 5.0f;
/* Flash is only written when SOC moved this much (0.01% units) and at most this often */
static constexpr float SAVE_STEP_PPTT = 100.0f;
static constexpr uint32_t SAVE_MIN_INTERVAL_MS = 60000;

// Written from the MQTT task, read from the main loop
static volatile float rx_power_W = 0.0f;
static volatile uint32_t rx_power_ms = 0;
static volatile bool rx_power_valid = false;
static volatile int32_t pending_soc_pptt = -1;

void soc_observer_set_power(float power_W) {
  if (isnan(power_W) || isinf(power_W)) {
    return;
  }
  rx_power_W = power_W;
  rx_power_ms = millis();
  rx_power_valid = true;
}

void soc_observer_set_soc_pct(float soc_pct) {
  if (isnan(soc_pct) || soc_pct < 0.0f || soc_pct > 100.0f) {
    return;
  }
  pending_soc_pptt = (int32_t)lroundf(soc_pct * 100.0f);
}

static void save_soc(float soc_pptt) {
  BatteryEmulatorSettingsStore settings;
  settings.saveUInt("SOCOBSPPTT", (uint32_t)lroundf(soc_pptt));
}

void soc_observer_update() {
  if (!soc_observer_enabled) {
    return;
  }

  static bool initialised = false;
  static float soc_pptt = 0.0f;
  static float last_saved_pptt = 0.0f;
  static uint32_t last_ms = 0;
  static uint32_t last_save_ms = 0;
  static bool at_full = false;   // counter sits exactly on the full anchor
  static bool at_empty = false;  // counter sits exactly on the empty anchor
  static float full_s = 0.0f;    // how long the full condition has held
  static float empty_s = 0.0f;
  static bool full_fired = false;   // an anchor fires once per time the condition becomes true
  static bool empty_fired = false;
  static bool was_fresh = false;
  static bool have_empty_ref = false;  // an empty anchor was seen, with a gap-free count since
  static float net_since_empty_Wh = 0.0f;

  DATALAYER_BATTERY_STATUS_TYPE& st = datalayer.battery.status;
  const uint32_t now = millis();

  const uint32_t cap_Wh =
      soc_observer_capacity_Wh > 0 ? soc_observer_capacity_Wh : datalayer.battery.info.total_capacity_Wh;
  if (cap_Wh == 0) {
    return;
  }

  /* Forced balancing makes the Tesla driver pin its own SOC, leave that alone */
  if (datalayer.battery_settings.user_requests_balancing) {
    return;
  }

  if (!initialised) {
    BatteryEmulatorSettingsStore settings(true);
    const uint32_t saved = settings.getUInt("SOCOBSPPTT", 0xFFFFFFFF);
    if (saved <= 10000) {
      soc_pptt = (float)saved;
    } else if (st.real_soc > 0 && st.real_soc <= 10000) {
      soc_pptt = (float)st.real_soc;  // first ever start: take the BMS figure, then correct it with SET_SOC
    } else {
      return;  // nothing to start from yet, leave the BMS value in place
    }
    last_saved_pptt = soc_pptt;
    last_ms = now;
    last_save_ms = now;
    initialised = true;
    logging.printf("SOC observer started at %.2f%%\n", soc_pptt / 100.0f);
  }

  float dt_s = (float)(now - last_ms) / 1000.0f;
  last_ms = now;
  if (dt_s > MAX_STEP_S) {
    dt_s = MAX_STEP_S;
  }

  bool save_now = false;
  const int32_t pending = pending_soc_pptt;
  if (pending >= 0) {
    pending_soc_pptt = -1;
    soc_pptt = (float)pending;
    at_full = false;
    at_empty = false;
    have_empty_ref = false;
    save_now = true;
    logging.printf("SOC observer set to %.2f%%\n", soc_pptt / 100.0f);
  }

  /* Power input: only counts while it keeps arriving */
  const bool fresh = rx_power_valid && ((uint32_t)(now - rx_power_ms) <= (uint32_t)soc_observer_timeout_s * 1000u);
  const float power_W = fresh ? rx_power_W : 0.0f;
  if (fresh != was_fresh) {
    logging.printf(fresh ? "SOC observer: power input is back\n"
                         : "SOC observer: power input is missing, holding SOC (no integration, no full anchor)\n");
    if (!fresh) {
      have_empty_ref = false;  // an energy gap makes the capacity measurement useless
    }
    was_fresh = fresh;
  }

  if (fresh) {
    const float dWh = power_W * dt_s / 3600.0f;
    soc_pptt += dWh / (float)cap_Wh * 10000.0f;
    net_since_empty_Wh += dWh;
  }

  /* Anchors */
  const bool cells_valid =
      st.voltage_dV > 500 && st.cell_min_voltage_mV > 1000 && st.cell_max_voltage_mV >= st.cell_min_voltage_mV;
  if (cells_valid && fresh && st.cell_max_voltage_mV >= soc_observer_full_mV &&
      fabsf(power_W) <= (float)soc_observer_anchor_max_W) {
    full_s += dt_s;
  } else {
    full_s = 0.0f;
  }
  /* No power gate on empty: the BMS and BE stop discharge near empty, so the load may never be light */
  if (cells_valid && st.cell_min_voltage_mV <= soc_observer_empty_mV) {
    empty_s += dt_s;
  } else {
    empty_s = 0.0f;
  }

  if (full_s == 0.0f) {
    full_fired = false;
  }
  if (empty_s == 0.0f) {
    empty_fired = false;
  }
  if (!full_fired && full_s >= ANCHOR_DEBOUNCE_S && soc_pptt < (float)soc_observer_full_pptt) {
    full_fired = true;
    logging.printf("SOC observer: full anchor (max cell %u mV), SOC %.2f%% -> %.2f%%\n", st.cell_max_voltage_mV,
                   soc_pptt / 100.0f, soc_observer_full_pptt / 100.0f);
    if (have_empty_ref && net_since_empty_Wh > 0.0f && soc_observer_full_pptt > soc_observer_empty_pptt) {
      const float span = (float)(soc_observer_full_pptt - soc_observer_empty_pptt) / 10000.0f;
      logging.printf("SOC observer: %.0f Wh in since the empty anchor, implies %.0f Wh between the anchors\n",
                     net_since_empty_Wh, net_since_empty_Wh / span);
    }
    soc_pptt = (float)soc_observer_full_pptt;
    at_full = true;
    save_now = true;
  }
  if (!empty_fired && empty_s >= ANCHOR_DEBOUNCE_S && soc_pptt > (float)soc_observer_empty_pptt) {
    empty_fired = true;
    logging.printf("SOC observer: empty anchor (min cell %u mV), SOC %.2f%% -> %.2f%%\n", st.cell_min_voltage_mV,
                   soc_pptt / 100.0f, soc_observer_empty_pptt / 100.0f);
    soc_pptt = (float)soc_observer_empty_pptt;
    at_empty = true;
    have_empty_ref = fresh;
    net_since_empty_Wh = 0.0f;
    save_now = true;
  }

  /* The counter alone never reaches the anchor values: it stops one step short and waits for the
     cell voltage to confirm. Only the anchor itself puts it exactly on full or empty, which is
     also what lets the safety layer's battery full / empty handling act on a confirmed state. */
  const float hi = at_full ? (float)soc_observer_full_pptt : (float)soc_observer_full_pptt - 1.0f;
  const float lo = at_empty ? (float)soc_observer_empty_pptt : (float)soc_observer_empty_pptt + 1.0f;
  if (soc_pptt > hi) {
    soc_pptt = hi;
  }
  if (soc_pptt < lo) {
    soc_pptt = lo;
  }
  if (soc_pptt < (float)soc_observer_full_pptt) {
    at_full = false;
  }
  if (soc_pptt > (float)soc_observer_empty_pptt) {
    at_empty = false;
  }

  /* Hand the result to the rest of BE */
  st.real_soc = (uint16_t)lroundf(soc_pptt);
  st.remaining_capacity_Wh = (uint32_t)((double)cap_Wh * (double)soc_pptt / 10000.0);
  if (fresh && st.voltage_dV > 10) {
    int32_t dA = (int32_t)lroundf(power_W * 100.0f / (float)st.voltage_dV);
    if (dA > INT16_MAX) {
      dA = INT16_MAX;
    } else if (dA < INT16_MIN) {
      dA = INT16_MIN;
    }
    st.current_dA = (int16_t)dA;  // active_power_W is derived from this later in the cycle
  }

  if (!save_now && fabsf(soc_pptt - last_saved_pptt) >= SAVE_STEP_PPTT &&
      (uint32_t)(now - last_save_ms) >= SAVE_MIN_INTERVAL_MS) {
    save_now = true;
  }
  if (save_now) {
    save_soc(soc_pptt);
    last_saved_pptt = soc_pptt;
    last_save_ms = now;
  }
}
