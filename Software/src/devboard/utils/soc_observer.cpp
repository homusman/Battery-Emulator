#include "soc_observer.h"
#include "soc_observer_metrics.h"

#include <Arduino.h>
#include <math.h>
#include "../../communication/nvm/comm_nvm.h"
#include "../../datalayer/datalayer.h"
#include "../../datalayer/datalayer_extended.h"
#include "logging.h"

bool soc_observer_enabled = false;
uint32_t soc_observer_nominal_capacity_Ah = 0;
bool soc_observer_auto_capacity = false;
uint16_t soc_observer_learn_max_delta_pct = 10;
uint16_t soc_observer_learn_rate_pct = 50;
uint16_t soc_observer_full_mV = 3360;
uint16_t soc_observer_empty_mV = 3150;
uint16_t soc_observer_full_pptt = 10000;
uint16_t soc_observer_empty_pptt = 0;
uint16_t soc_observer_anchor_max_W = 500;
uint16_t soc_observer_timeout_s = 60;
bool soc_observer_count_energy = false;
uint32_t soc_observer_energy_save_Wh = 1000;
uint16_t soc_observer_bms_soh_pptt = 0;
bool soc_observer_bms_soh_valid = false;
uint16_t soc_observer_bms_pptt = 0;
bool soc_observer_bms_valid = false;
SocObserverStats soc_observer_stats;

/* An anchor has to hold this long before it counts, so a spike or a sag cannot reset the counter */
static constexpr float ANCHOR_DEBOUNCE_S = 20.0f;
/* Longest step integrated in one go. A stalled loop must not turn into a huge jump */
static constexpr float MAX_STEP_S = 5.0f;
/* Flash is only written when SOC moved this much (0.01% units) and at most this often */
static constexpr float SAVE_STEP_PPTT = 100.0f;
static constexpr uint32_t SAVE_MIN_INTERVAL_MS = 60000;
/* A leg shorter than this cannot be a real charge or discharge between the anchors */
static constexpr float MIN_LEG_S = 300.0f;

// Written from the MQTT task, read from the main loop
static volatile float rx_power_W = 0.0f;
static volatile uint32_t rx_power_ms = 0;
static volatile bool rx_power_valid = false;
static volatile int32_t pending_soc_pptt = -1;

/* Capacity learned from clean legs, in mAh, 0 = nothing learned. Kept in NVM together with the nominal
   capacity it grew from, so changing the nominal capacity in settings starts the learning over. */
static uint32_t learned_mAh = 0;
/* Estimated once from the pack's own figures when no capacity has been set */
static uint32_t estimated_mAh = 0;

/* Lifetime energy through the pack terminals, counted from the power input while the totals are enabled */
static double energy_charged_Wh = 0.0;
static double energy_discharged_Wh = 0.0;
static double energy_unsaved_Wh = 0.0;  // moved since the totals were last written to flash
static bool energy_ready = false;

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

static void save_learned() {
  BatteryEmulatorSettingsStore settings;
  settings.saveUInt("SOCOBSLMAH", learned_mAh);
}

static void save_energy() {
  BatteryEmulatorSettingsStore settings;
  settings.saveUInt("SOCOBSECHG", (uint32_t)energy_charged_Wh);
  settings.saveUInt("SOCOBSEDIS", (uint32_t)energy_discharged_Wh);
  energy_unsaved_Wh = 0.0;
}

/* Working capacity in mAh: learned (when learning is on), configured nominal, then the estimate. */
static uint32_t eff_cap_mAh() {
  if (soc_observer_auto_capacity && learned_mAh > 0) {
    return learned_mAh;
  }
  if (soc_observer_nominal_capacity_Ah > 0) {
    return soc_observer_nominal_capacity_Ah * 1000u;
  }
  return estimated_mAh;
}

void soc_observer_update() {
  if (!soc_observer_enabled) {
    return;
  }

  static bool initialised = false;
  static bool learned_loaded = false;
  static float soc_pptt = 0.0f;
  static float last_saved_pptt = 0.0f;
  static uint32_t last_ms = 0;
  static uint32_t last_save_ms = 0;
  static bool at_full = false;   // counter sits exactly on the full anchor
  static bool at_empty = false;  // counter sits exactly on the empty anchor
  static float full_s = 0.0f;    // how long the full condition has held
  static float empty_s = 0.0f;
  static bool full_fired = false;  // an anchor fires once per time the condition becomes true
  static bool empty_fired = false;
  static bool was_fresh = false;
  // The leg in progress: what went through the pack since the last anchor
  static int leg_start = 0;  // 0 = no leg being followed, 1 = started at the full anchor, 2 = at the empty anchor
  static bool leg_dirty = false;
  static const char* leg_why = nullptr;  // first reason the leg stopped being clean
  static float leg_Wh = 0.0f;
  static float leg_Ah = 0.0f;
  static float leg_s = 0.0f;
  static bool estimate_logged = false;
  static uint32_t energy_wait_start_ms = 0;

  DATALAYER_BATTERY_STATUS_TYPE& st = datalayer.battery.status;
  const uint32_t now = millis();

  auto mark_dirty = [&](const char* why) {
    if (!leg_dirty) {
      leg_dirty = true;
      leg_why = why;
    }
  };

  /* The driver has just written its own SOC: keep it for display before anything below replaces it */
  soc_observer_bms_pptt = st.real_soc;
  soc_observer_bms_valid = true;

  if (!learned_loaded) {
    BatteryEmulatorSettingsStore settings(true);
    const uint32_t base = settings.getUInt("SOCOBSLBASE", 0xFFFFFFFF);
    const uint32_t nominal_mAh = soc_observer_nominal_capacity_Ah * 1000u;
    if (base == nominal_mAh) {
      learned_mAh = settings.getUInt("SOCOBSLMAH", 0);
    } else {
      // A changed nominal reference starts learning from the new figure.
      learned_mAh = 0;
      BatteryEmulatorSettingsStore writer;
      writer.saveUInt("SOCOBSLBASE", nominal_mAh);
      writer.saveUInt("SOCOBSLMAH", 0);
    }
    learned_loaded = true;
  }

  if (eff_cap_mAh() == 0) {
    /* Nothing set: estimate Ah from the pack's energy figure at its present voltage, once, so the counter
       does not wobble with the voltage. Wrong by some percent at most, and learning can fix it. */
    if (st.voltage_dV > 500 && datalayer.battery.info.total_capacity_Wh > 0) {
      estimated_mAh = (uint32_t)((double)datalayer.battery.info.total_capacity_Wh * 10000.0 / (double)st.voltage_dV);
    }
  }
  soc_observer_stats.cap_mAh = eff_cap_mAh();
  if (soc_observer_stats.cap_mAh == 0) {
    return;  // no capacity and nothing to estimate from yet, leave the BMS values in place
  }
  if (soc_observer_nominal_capacity_Ah == 0 && learned_mAh == 0 && !estimate_logged) {
    logging.printf(
        "SOC observer: no capacity set, estimating %.1f Ah from the battery's own figure. Set it in settings.\n",
        estimated_mAh / 1000.0f);
    estimate_logged = true;
  }

  /* Forced balancing makes the Tesla driver pin its own SOC, leave that alone */
  if (datalayer.battery_settings.user_requests_balancing) {
    mark_dirty("forced balancing");
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

  /* Lifetime energy totals: continue from what the BMS shows the first time, or from what was saved.
     The BMS needs a moment to send its totals, so wait for a non-zero figure (a pack that really is at zero
     is seeded after two minutes). */
  if (soc_observer_count_energy && !energy_ready) {
    if (energy_wait_start_ms == 0) {
      energy_wait_start_ms = now;
    }
    BatteryEmulatorSettingsStore settings(true);
    const uint32_t saved_chg = settings.getUInt("SOCOBSECHG", 0xFFFFFFFF);
    const uint32_t saved_dis = settings.getUInt("SOCOBSEDIS", 0xFFFFFFFF);
    if (saved_chg != 0xFFFFFFFF && saved_dis != 0xFFFFFFFF) {
      energy_charged_Wh = (double)saved_chg;
      energy_discharged_Wh = (double)saved_dis;
      energy_ready = true;
    } else if (st.total_charged_battery_Wh > 0 || st.total_discharged_battery_Wh > 0 ||
               (uint32_t)(now - energy_wait_start_ms) > 120000u) {
      energy_charged_Wh = (double)st.total_charged_battery_Wh;
      energy_discharged_Wh = (double)st.total_discharged_battery_Wh;
      energy_ready = true;
      save_energy();
      logging.printf("SOC observer: energy totals start from the BMS figures, %.0f Wh in / %.0f Wh out\n",
                     energy_charged_Wh, energy_discharged_Wh);
    }
  }

  float dt_s = (float)(now - last_ms) / 1000.0f;
  last_ms = now;
  if (dt_s > MAX_STEP_S) {
    dt_s = MAX_STEP_S;
    mark_dirty("loop stalled");  // charge was missed
  }

  bool save_now = false;
  const int32_t pending = pending_soc_pptt;
  if (pending >= 0) {
    pending_soc_pptt = -1;
    soc_pptt = (float)pending;
    at_full = false;
    at_empty = false;
    leg_start = 0;  // a manual SOC makes the leg in progress meaningless
    save_now = true;
    logging.printf("SOC observer set to %.2f%%\n", soc_pptt / 100.0f);
  }

  /* Power input: only counts while it keeps arriving */
  const bool fresh = rx_power_valid && ((uint32_t)(now - rx_power_ms) <= (uint32_t)soc_observer_timeout_s * 1000u);
  const float power_W = fresh ? rx_power_W : 0.0f;
  if (fresh != was_fresh) {
    logging.printf(fresh ? "SOC observer: power input is back\n"
                         : "SOC observer: power input is missing, holding SOC (no counting, no full anchor)\n");
    was_fresh = fresh;
  }
  if (!fresh) {
    mark_dirty("power input missing");
  }

  const bool voltage_ok = st.voltage_dV > 10;
  if (fresh) {
    const float dWh = power_W * dt_s / 3600.0f;
    if (soc_observer_count_energy && energy_ready) {
      if (dWh >= 0.0f) {
        energy_charged_Wh += dWh;
      } else {
        energy_discharged_Wh -= dWh;
      }
      energy_unsaved_Wh += fabsf(dWh);
    }
    if (voltage_ok) {
      const float dAh = (power_W / ((float)st.voltage_dV / 10.0f)) * dt_s / 3600.0f;
      soc_pptt += dAh * 1000.0f / (float)soc_observer_stats.cap_mAh * 10000.0f;
      leg_Wh += dWh;
      leg_Ah += dAh;
      leg_s += dt_s;
    } else {
      mark_dirty("no pack voltage");
    }
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

  /* A leg ends at the opposite anchor and the next one starts there. Reaching the same anchor again
     (discharged a little, then back to full) is not a leg, it just restarts the count. */
  auto finish_leg = [&](bool at_full_anchor) {
    const float span = (float)(soc_observer_full_pptt - soc_observer_empty_pptt) / 10000.0f;
    const bool expected = at_full_anchor ? (leg_start == 2) : (leg_start == 1);
    const float Wh = at_full_anchor ? leg_Wh : -leg_Wh;
    const float Ah = at_full_anchor ? leg_Ah : -leg_Ah;
    const char* name = at_full_anchor ? "charge" : "discharge";
    if (expected && span > 0.05f && Wh > 0.0f && Ah > 0.0f) {
      SocObserverLeg leg;
      leg.valid = true;
      leg.Wh = Wh;
      leg.Ah = Ah;
      leg.minutes = leg_s / 60.0f;
      (at_full_anchor ? soc_observer_stats.all_charge : soc_observer_stats.all_discharge) = leg;

      const bool clean = !leg_dirty && leg_s >= MIN_LEG_S;
      if (clean) {
        (at_full_anchor ? soc_observer_stats.clean_charge : soc_observer_stats.clean_discharge) = leg;
        logging.printf("SOC observer: clean %s leg, %.0f Wh and %.1f Ah between the anchors in %.0f min\n", name, Wh,
                       Ah, leg.minutes);
      } else {
        logging.printf("SOC observer: unclean %s leg (%s), %.0f Wh and %.1f Ah in %.0f min, not used for learning\n",
                       name, leg_dirty ? leg_why : "too short", Wh, Ah, leg.minutes);
      }

      if (clean && soc_observer_auto_capacity) {
        const float measured_mAh = Ah * 1000.0f / span;
        const float cur_mAh = (float)eff_cap_mAh();
        const float dev_pct = 100.0f * fabsf(measured_mAh - cur_mAh) / cur_mAh;
        if (soc_observer_learn_rate_pct == 0) {
          // never update: nothing to do
        } else if (dev_pct <= (float)soc_observer_learn_max_delta_pct) {
          const float rate = (float)soc_observer_learn_rate_pct / 100.0f;
          const float next_mAh = (1.0f - rate) * cur_mAh + rate * measured_mAh;
          learned_mAh = (uint32_t)lroundf(next_mAh);
          save_learned();
          logging.printf("SOC observer: capacity %.1f Ah -> %.1f Ah (leg measured %.1f Ah, %.1f%% off, rate %u%%)\n",
                         cur_mAh / 1000.0f, next_mAh / 1000.0f, measured_mAh / 1000.0f, dev_pct,
                         (unsigned)soc_observer_learn_rate_pct);
        } else {
          logging.printf(
              "SOC observer: leg measured %.1f Ah, %.1f%% from the %.1f Ah in use (limit %u%%), not applied\n",
              measured_mAh / 1000.0f, dev_pct, cur_mAh / 1000.0f, (unsigned)soc_observer_learn_max_delta_pct);
        }
      }
    }
    leg_start = at_full_anchor ? 1 : 2;
    leg_Wh = 0.0f;
    leg_Ah = 0.0f;
    leg_s = 0.0f;
    leg_dirty = false;
    leg_why = nullptr;
  };

  if (!full_fired && full_s >= ANCHOR_DEBOUNCE_S && soc_pptt < (float)soc_observer_full_pptt) {
    full_fired = true;
    logging.printf("SOC observer: full anchor (max cell %u mV), SOC %.2f%% -> %.2f%%\n", st.cell_max_voltage_mV,
                   soc_pptt / 100.0f, soc_observer_full_pptt / 100.0f);
    soc_pptt = (float)soc_observer_full_pptt;
    at_full = true;
    save_now = true;
    finish_leg(true);
  }
  if (!empty_fired && empty_s >= ANCHOR_DEBOUNCE_S && soc_pptt > (float)soc_observer_empty_pptt) {
    empty_fired = true;
    logging.printf("SOC observer: empty anchor (min cell %u mV), SOC %.2f%% -> %.2f%%\n", st.cell_min_voltage_mV,
                   soc_pptt / 100.0f, soc_observer_empty_pptt / 100.0f);
    soc_pptt = (float)soc_observer_empty_pptt;
    at_empty = true;
    save_now = true;
    finish_leg(false);
  }
  soc_observer_stats.cap_mAh = eff_cap_mAh();  // a leg may just have changed it

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

  // The configured energy and nominal Ah stay fixed; only the observer's working Ah can learn.
  st.real_soc = (uint16_t)lroundf(soc_pptt);
  const SocObserverMetrics metrics =
      calculate_soc_observer_metrics(soc_observer_stats.cap_mAh, soc_observer_nominal_capacity_Ah,
                                     datalayer.battery.info.total_capacity_Wh, st.real_soc, st.voltage_dV);
  st.remaining_capacity_Wh = metrics.remaining_Wh;
  st.soh_pptt = metrics.soh_pptt;
  st.soh_available = metrics.soh_valid;
  // Diagnostic energy at present voltage, with no separate energy-capacity learning or feedback.
  soc_observer_stats.calculated_remaining_Wh = metrics.calculated_remaining_Wh;

  // Keep the BMS's own energy-based health estimate as a separate MQTT diagnostic.
  const double bol_Wh = (double)datalayer_extended.tesla.battery_beginning_of_life * 1000.0;
  double bms_full_Wh = (double)datalayer_extended.tesla.battery_nominal_full_pack_energy * 100.0;  // 0.1 kWh units
  if (bms_full_Wh <= 0.0) {
    bms_full_Wh = (double)datalayer_extended.tesla.battery_nominal_full_pack_energy_m0 * 20.0;  // 0.02 kWh units
  }
  soc_observer_bms_soh_valid = bol_Wh > 0.0 && bms_full_Wh > 0.0;
  if (soc_observer_bms_soh_valid) {
    const double ratio = bms_full_Wh / bol_Wh;
    soc_observer_bms_soh_pptt = (uint16_t)lround(ratio > 1.0 ? 10000.0 : ratio * 10000.0);
  }
  if (soc_observer_count_energy && energy_ready) {
    st.total_charged_battery_Wh = (uint32_t)energy_charged_Wh;
    st.total_discharged_battery_Wh = (uint32_t)energy_discharged_Wh;
  }

  if (fresh && voltage_ok) {
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
  const double energy_step = soc_observer_energy_save_Wh < 100u ? 100.0 : (double)soc_observer_energy_save_Wh;
  if (soc_observer_count_energy && energy_ready && energy_unsaved_Wh >= energy_step) {
    save_energy();
  }
}
