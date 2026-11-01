#ifndef SOC_OBSERVER_H
#define SOC_OBSERVER_H

#include <stdint.h>

/* SOC observer: BE's own energy counter for packs whose BMS cannot see the pack current (for
   example a Tesla pack running without its PCS). The current comes from outside over MQTT, so
   Home Assistant can hand over the inverter's battery power. It replaces datalayer.battery
   real_soc / remaining_capacity_Wh / current_dA while enabled.

   Settings (loaded from NVM in comm_nvm.cpp) */
extern bool soc_observer_enabled;
extern uint32_t soc_observer_capacity_Wh;  // 0 = use the pack's own total_capacity_Wh
extern uint16_t soc_observer_full_mV;      // max cell at/above this (and idle) re-anchors SOC to the full value
extern uint16_t soc_observer_empty_mV;     // min cell at/below this re-anchors SOC to the empty value
extern uint16_t soc_observer_full_pptt;    // SOC set at the full anchor, 0.01% units (10000 = 100%)
extern uint16_t soc_observer_empty_pptt;   // SOC set at the empty anchor, 0.01% units
extern uint16_t soc_observer_anchor_max_W;  // |power| must be at or below this for the full anchor to count
extern uint16_t soc_observer_timeout_s;     // power input older than this is treated as missing

/** Battery power from outside, in W at the pack terminals. Positive = charging. Called from MQTT. */
void soc_observer_set_power(float power_W);

/** Force the counter to this SOC, percent 0-100. Called from MQTT (SET_SOC). */
void soc_observer_set_soc_pct(float soc_pct);

/** Run once per second, after battery->update_values() and before update_calculated_values(). */
void soc_observer_update();

#endif
