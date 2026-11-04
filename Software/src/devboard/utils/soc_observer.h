#ifndef SOC_OBSERVER_H
#define SOC_OBSERVER_H

#include <stdint.h>

/* SOC observer: BE's own coulomb counter for packs whose BMS cannot see the pack current (for
   example a Tesla pack running without its PCS). The power comes from outside over MQTT, so
   Home Assistant can hand over the inverter's battery power. BE divides it by the pack voltage to
   get the current and counts amp-hours. It replaces datalayer.battery real_soc /
   remaining_capacity_Wh / current_dA while enabled.

   Settings (loaded from NVM in comm_nvm.cpp) */
extern bool soc_observer_enabled;
extern uint32_t soc_observer_capacity_Ah;  // usable Ah between the anchors, 0 = estimate from the pack's own capacity
extern bool soc_observer_auto_capacity;    // learn the capacity from clean anchor-to-anchor legs
extern uint16_t soc_observer_learn_max_delta_pct;  // a leg only teaches if it is within this % of the capacity in use
extern uint16_t soc_observer_learn_rate_pct;       // 100 = take the measured value, 0 = never change
extern uint16_t soc_observer_full_mV;      // max cell at/above this (and idle) re-anchors SOC to the full value
extern uint16_t soc_observer_empty_mV;     // min cell at/below this re-anchors SOC to the empty value
extern uint16_t soc_observer_full_pptt;    // SOC set at the full anchor, 0.01% units (10000 = 100%)
extern uint16_t soc_observer_empty_pptt;   // SOC set at the empty anchor, 0.01% units
extern uint16_t soc_observer_anchor_max_W;  // |power| must be at or below this for the full anchor to count
extern uint16_t soc_observer_timeout_s;     // power input older than this is treated as missing
extern bool soc_observer_count_energy;      // keep BE's own lifetime charged/discharged energy totals
extern uint32_t soc_observer_energy_save_Wh;  // write the energy totals to flash after this much more energy has moved

/** What the BMS itself reported as SOC this cycle (0.01% units), kept before the observer overwrites real_soc.
   Valid once the observer has run at least once. */
extern uint16_t soc_observer_bms_pptt;
extern bool soc_observer_bms_valid;

/** State of health as the BMS itself implies it (nominal full pack energy / beginning of life energy, 0.01% units),
    for MQTT. The observer replaces the main SOH with its own figure: the capacity in use compared with the BMS's
    beginning of life energy. */
extern uint16_t soc_observer_bms_soh_pptt;
extern bool soc_observer_bms_soh_valid;

/** One completed trip from one anchor to the other: what went through the pack terminals between the two
    anchor states (inverter losses excluded). */
struct SocObserverLeg {
  bool valid = false;
  float Wh = 0.0f;  // energy, always positive (in for a charge leg, out for a discharge leg)
  float Ah = 0.0f;  // charge, power / pack voltage integrated, always positive
  float minutes = 0.0f;
};

/** Results for MQTT. A leg is clean when it ran from one anchor to the other with the power input present the
    whole time, no manual SOC change and no loop stall in between, and lasted a few minutes at least. Only clean
    legs can change the capacity. The "all" figures are the latest completed leg in each direction, clean or not. */
struct SocObserverStats {
  SocObserverLeg clean_charge;     // empty anchor -> full anchor
  SocObserverLeg clean_discharge;  // full anchor -> empty anchor
  SocObserverLeg all_charge;
  SocObserverLeg all_discharge;
  uint32_t cap_mAh = 0;  // capacity in use right now
};
extern SocObserverStats soc_observer_stats;

/** Battery power from outside, in W at the pack terminals. Positive = charging. Called from MQTT. */
void soc_observer_set_power(float power_W);

/** Force the counter to this SOC, percent 0-100. Called from MQTT (SET_SOC). */
void soc_observer_set_soc_pct(float soc_pct);

/** Run once per second, after battery->update_values() and before update_calculated_values(). */
void soc_observer_update();

#endif
