#ifndef SOC_OBSERVER_METRICS_H
#define SOC_OBSERVER_METRICS_H

#include <stdint.h>

struct SocObserverMetrics {
  uint32_t remaining_Wh;
  double calculated_remaining_Wh;
  uint16_t soh_pptt;
  bool soh_valid;
};

// Nominal Ah is a fixed beginning-of-life reference, independent of the learned capacity.
inline SocObserverMetrics calculate_soc_observer_metrics(uint32_t capacity_mAh, uint32_t nominal_Ah,
                                                         uint32_t configured_Wh, uint16_t soc_pptt,
                                                         uint16_t voltage_dV) {
  const uint16_t soc = soc_pptt > 10000u ? 10000u : soc_pptt;
  const uint64_t nominal_mAh = (uint64_t)nominal_Ah * 1000u;
  const uint64_t health = nominal_mAh ? (uint64_t)capacity_mAh * 10000u / nominal_mAh : 0u;
  return {
      (uint32_t)((uint64_t)configured_Wh * soc / 10000u),
      (double)capacity_mAh / 1000.0 * soc / 10000.0 * voltage_dV / 10.0,
      (uint16_t)(health > 10000u ? 10000u : health),
      nominal_Ah > 0 && capacity_mAh > 0,
  };
}

#endif
