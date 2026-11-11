#include <gtest/gtest.h>

#include "../Software/src/devboard/utils/soc_observer_metrics.h"

TEST(SocObserverMetrics, FullAnchorCorrectionUsesConfiguredEnergy) {
  const auto before = calculate_soc_observer_metrics(150000, 150, 50000, 8200, 3600);
  const auto full = calculate_soc_observer_metrics(150000, 150, 50000, 10000, 3600);
  EXPECT_EQ(before.remaining_Wh, 41000u);
  EXPECT_EQ(full.remaining_Wh, 50000u);
  EXPECT_DOUBLE_EQ(full.calculated_remaining_Wh, 54000.0);
  EXPECT_EQ(full.soh_pptt, 10000u);
}

TEST(SocObserverMetrics, HealthUsesWorkingAhAgainstFixedNominalAh) {
  const uint32_t nominal_Ah = 150;
  const auto initial = calculate_soc_observer_metrics(150000, nominal_Ah, 50000, 10000, 3600);
  const auto learned = calculate_soc_observer_metrics(135000, nominal_Ah, 50000, 10000, 3600);
  EXPECT_EQ(initial.soh_pptt, 10000u);
  EXPECT_EQ(learned.soh_pptt, 9000u);
  EXPECT_TRUE(learned.soh_valid);
  EXPECT_EQ(learned.remaining_Wh, 50000u);
  EXPECT_DOUBLE_EQ(learned.calculated_remaining_Wh, 48600.0);
}

TEST(SocObserverMetrics, VoltageAffectsOnlyDiagnosticEnergy) {
  const auto low = calculate_soc_observer_metrics(135000, 150, 50000, 5000, 3000);
  const auto high = calculate_soc_observer_metrics(135000, 150, 50000, 5000, 3600);
  EXPECT_EQ(low.remaining_Wh, high.remaining_Wh);
  EXPECT_EQ(low.soh_pptt, high.soh_pptt);
  EXPECT_GT(high.calculated_remaining_Wh, low.calculated_remaining_Wh);
}

TEST(SocObserverMetrics, MissingNominalAhLeavesHealthUnknown) {
  const auto estimated = calculate_soc_observer_metrics(150000, 0, 50000, 10000, 3600);
  EXPECT_FALSE(estimated.soh_valid);
  EXPECT_EQ(estimated.remaining_Wh, 50000u);
  EXPECT_FALSE(calculate_soc_observer_metrics(0, 150, 50000, 10000, 3600).soh_valid);
}

TEST(SocObserverMetrics, EmptyAndOutOfRangeSocStayWithinConfiguredEnergy) {
  EXPECT_EQ(calculate_soc_observer_metrics(150000, 150, 50000, 0, 3600).remaining_Wh, 0u);
  EXPECT_EQ(calculate_soc_observer_metrics(150000, 150, 50000, 10800, 3600).remaining_Wh, 50000u);
  EXPECT_EQ(calculate_soc_observer_metrics(150000, 150, 0, 10000, 3600).remaining_Wh, 0u);
}

TEST(SocObserverMetrics, LargePacksDoNotOverflowAndHealthIsCapped) {
  const auto large = calculate_soc_observer_metrics(2000000, 2000, 1000000, 10000, 5000);
  EXPECT_EQ(large.remaining_Wh, 1000000u);
  EXPECT_EQ(large.soh_pptt, 10000u);
  EXPECT_EQ(calculate_soc_observer_metrics(160000, 150, 50000, 10000, 3600).soh_pptt, 10000u);
}
