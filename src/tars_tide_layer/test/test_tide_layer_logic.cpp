#include "tars_tide_layer/tide_layer_logic.hpp"

#include <gtest/gtest.h>

TEST(TideLayerLogic, MoveClearsOldMarksNew)
{
  EXPECT_TRUE(tars_tide_layer::should_clear_tide_cell(true, false));
  EXPECT_FALSE(tars_tide_layer::should_clear_tide_cell(true, true));
  EXPECT_FALSE(tars_tide_layer::should_clear_tide_cell(false, false));
  EXPECT_FALSE(tars_tide_layer::should_clear_tide_cell(false, true));
  EXPECT_TRUE(tars_tide_layer::should_mark_tide_cell(true));
  EXPECT_FALSE(tars_tide_layer::should_mark_tide_cell(false));
}

TEST(TideLayerLogic, EquivalentPolygonsSkipClear)
{
  const std::vector<std::pair<double, double>> box_a{
    {0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}};
  const std::vector<std::pair<double, double>> box_b{
    {20.0, 0.0}, {30.0, 0.0}, {30.0, 10.0}, {20.0, 10.0}};
  EXPECT_TRUE(tars_tide_layer::polygons_equivalent(box_a, box_a));
  EXPECT_FALSE(tars_tide_layer::polygons_equivalent(box_a, box_b));
  const std::vector<std::vector<std::pair<double, double>>> list_a{box_a, box_b};
  const std::vector<std::vector<std::pair<double, double>>> list_b{box_a};
  EXPECT_TRUE(tars_tide_layer::polygon_lists_equivalent(list_a, list_a));
  EXPECT_FALSE(tars_tide_layer::polygon_lists_equivalent(list_a, list_b));
}

TEST(TideLayerLogic, OpenWaterlineIsNotClosed)
{
  const std::vector<std::pair<double, double>> waterline_c{
    {0.0, 0.0}, {0.0, 100.0}, {80.0, 100.0}};
  const std::vector<std::pair<double, double>> keepout_loop{
    {0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}, {0.05, 0.05}};
  EXPECT_FALSE(tars_tide_layer::ring_is_closed(waterline_c));
  EXPECT_TRUE(tars_tide_layer::ring_is_closed(keepout_loop));
}

TEST(TideLayerLogic, AabbRejectsFarWindow)
{
  const std::vector<std::pair<double, double>> ring{
    {1000.0, 1000.0}, {1100.0, 1000.0}, {1100.0, 1100.0}, {1000.0, 1100.0}};
  const auto box = tars_tide_layer::ring_aabb(ring);
  EXPECT_TRUE(box.valid);
  EXPECT_FALSE(
    tars_tide_layer::aabb_intersects_window(
      box, -250.0, -250.0, 250.0, 250.0, 4.0));
  EXPECT_TRUE(
    tars_tide_layer::aabb_intersects_window(
      box, 900.0, 900.0, 1200.0, 1200.0, 4.0));
}

TEST(TideLayerLogic, ClipKeepsOnWindowSegment)
{
  double x0 = -10.0, y0 = 5.0, x1 = 30.0, y1 = 5.0;
  EXPECT_TRUE(
    tars_tide_layer::clip_segment_to_aabb(
      x0, y0, x1, y1, 0.0, 0.0, 10.0, 10.0));
  EXPECT_NEAR(x0, 0.0, 1e-9);
  EXPECT_NEAR(x1, 10.0, 1e-9);
  EXPECT_NEAR(y0, 5.0, 1e-9);
}

TEST(TideLayerLogic, ClipDropsFarSegment)
{
  double x0 = 1000.0, y0 = 1000.0, x1 = 1100.0, y1 = 1000.0;
  EXPECT_FALSE(
    tars_tide_layer::clip_segment_to_aabb(
      x0, y0, x1, y1, 0.0, 0.0, 10.0, 10.0));
}

TEST(TideLayerLogic, TwoPointWaterlinePaintsNearbyCell)
{
  const std::vector<std::pair<double, double>> ring{{0.0, 5.0}, {20.0, 5.0}};
  int marked = 0;
  int far = 0;
  tars_tide_layer::visit_corridor_cells(
    ring, 0.0, 0.0, 1.0, 0, 0, 20, 20, 2.0,
    [&](int i, int j) {
      ++marked;
      if (j >= 10) {
        ++far;
      }
      EXPECT_GE(i, 0);
      EXPECT_LT(i, 20);
    });
  EXPECT_GT(marked, 10);
  EXPECT_EQ(far, 0);
}

TEST(TideLayerLogic, FarKmRingDoesNotStampLocalWindow)
{
  const std::vector<std::pair<double, double>> ring{
    {8000.0, 8000.0}, {8012.0, 8000.0}, {8024.0, 8012.0}};
  int marked = 0;
  tars_tide_layer::visit_corridor_cells(
    ring, 0.0, 0.0, 0.1, 0, 0, 5000, 5000, 12.0,
    [&](int, int) { ++marked; });
  EXPECT_EQ(marked, 0);
  double min_x = 0.0, min_y = 0.0, max_x = 1.0, max_y = 1.0;
  EXPECT_FALSE(
    tars_tide_layer::touch_ring_in_window(
      ring, 0.0, 0.0, 500.0, 500.0, 12.0,
      &min_x, &min_y, &max_x, &max_y));
  EXPECT_DOUBLE_EQ(min_x, 0.0);
  EXPECT_DOUBLE_EQ(max_x, 1.0);
}

TEST(TideLayerLogic, ShouldRestampOnlyWhenNeeded)
{
  EXPECT_TRUE(tars_tide_layer::should_restamp_tide(false, false, false, false));
  EXPECT_FALSE(tars_tide_layer::should_restamp_tide(false, false, false, true));
  EXPECT_TRUE(tars_tide_layer::should_restamp_tide(true, false, false, true));
  EXPECT_TRUE(tars_tide_layer::should_restamp_tide(false, true, false, true));
  EXPECT_TRUE(tars_tide_layer::should_restamp_tide(false, false, true, true));
}

TEST(TideLayerLogic, LongDiagonalDoesNotVisitWholeGrid)
{
  const std::vector<std::pair<double, double>> ring{
    {0.0, 0.0}, {400.0, 400.0}};
  int marked = 0;
  tars_tide_layer::visit_corridor_cells(
    ring, 0.0, 0.0, 0.2, 0, 0, 2000, 2000, 12.0,
    [&](int, int) { ++marked; });
  EXPECT_GT(marked, 100);
  EXPECT_LT(marked, 2000000);
  EXPECT_LT(marked, 2000 * 2000);
}

TEST(TideLayerLogic, ClosedBoxFillsInterior)
{
  const std::vector<std::pair<double, double>> box{
    {0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}, {0.0, 0.0}};
  int filled = 0;
  tars_tide_layer::visit_fill_cells(
    box, 0.0, 0.0, 1.0, 0, 0, 20, 20,
    [&](int, int) { ++filled; });
  EXPECT_GT(filled, 20);
  EXPECT_LT(filled, 200);
}
