// TARS recovery — SLIP: wheels spin, GNSS position barely moves.

#include "tars_recovery_behaviors/slip_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tars_recovery_behaviors
{

namespace
{

// Wheel odometry published by eclipse_test_controller (encoder-based).
constexpr const char * kWheelOdomTopic = "/odom";
// Raw GNSS from GPS_node (not /gps/fix_gated — gate can drop fixes pre-calib).
constexpr const char * kGpsFixTopic = "/gps/fix";

constexpr double kEarthRadiusM = 6371000.0;

bool isFiniteLatLon(double lat, double lon)
{
  return std::isfinite(lat) && std::isfinite(lon) &&
         lat >= -90.0 && lat <= 90.0 &&
         lon >= -180.0 && lon <= 180.0;
}

}  // namespace

BT::PortsList SlipDetector::providedPorts()
{
  // 아래 기본값이 SLIP 임계. BT XML 속성이 있으면 그쪽이 이김.
  return {
    BT::InputPort<double>(
      "window_s", 10.0,
      "SLIP 임계: 휠·GPS 를 모아 보는 시간 창(초)."),
    BT::InputPort<double>(
      "min_wheel_speed_mps", 0.05,
      "SLIP 임계: 창 안 |/odom| 평균 ≥ 이 값이면 바퀴가 돈다고 봄 (m/s)."),
    BT::InputPort<int>(
      "min_fix_status", 0,
      "SLIP 임계: 이 status 미만 GPS 는 샘플에서 제외. "
      "-1=NO_FIX, 0=FIX, 1=Float/SBAS, 2=RTK Fixed."),
    BT::InputPort<double>(
      "max_disp_fix_m", 0.50,
      "SLIP 임계: status=FIX 일 때 GPS 수평 이동이 이(m) 미만이면 몸 안 감."),
    BT::InputPort<double>(
      "max_disp_sbas_m", 0.35,
      "SLIP 임계: status=SBAS/Float 일 때 몸 안 감 상한(m)."),
    BT::InputPort<double>(
      "max_disp_gbas_m", 0.15,
      "SLIP 임계: status=RTK Fixed 일 때 몸 안 감 상한(m)."),
  };
}

SlipDetector::SlipDetector(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  getInput("window_s", window_s_);
  getInput("min_wheel_speed_mps", min_wheel_speed_mps_);
  getInput("min_fix_status", min_fix_status_);
  getInput("max_disp_fix_m", max_disp_fix_m_);
  getInput("max_disp_sbas_m", max_disp_sbas_m_);
  getInput("max_disp_gbas_m", max_disp_gbas_m_);

  const rclcpp::QoS reliable_qos = rclcpp::QoS(rclcpp::KeepLast(5));
  // GPS_node publishes RELIABLE; BEST_EFFORT sub still receives.
  const rclcpp::QoS gps_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());
  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;

  wheel_odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    kWheelOdomTopic, reliable_qos,
    std::bind(&SlipDetector::wheelOdomCallback, this, std::placeholders::_1),
    sub_option);
  gps_fix_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
    kGpsFixTopic, gps_qos,
    std::bind(&SlipDetector::gpsFixCallback, this, std::placeholders::_1),
    sub_option);

  RCLCPP_INFO(
    node_->get_logger(),
    "SlipDetector ready (SLIP/헛돌기, GNSS body): window_s=%.1f "
    "min_wheel_speed=%.3f min_fix_status=%d "
    "max_disp fix/sbas/gbas=%.2f/%.2f/%.2f m — topic %s",
    window_s_, min_wheel_speed_mps_, min_fix_status_,
    max_disp_fix_m_, max_disp_sbas_m_, max_disp_gbas_m_, kGpsFixTopic);
}

void SlipDetector::wheelOdomCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const double stamp_s = node_->now().seconds();
  const double speed = std::fabs(msg->twist.twist.linear.x);
  std::lock_guard<std::mutex> lock(windows_mutex_);
  wheel_speed_window_.emplace_back(stamp_s, speed);
  const double cutoff = stamp_s - window_s_;
  while (!wheel_speed_window_.empty() &&
    wheel_speed_window_.front().first < cutoff)
  {
    wheel_speed_window_.pop_front();
  }
}

void SlipDetector::gpsFixCallback(
  const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  const double lat = msg->latitude;
  const double lon = msg->longitude;
  const int8_t status = msg->status.status;

  // 임계 미달 샘플 폐기: status < min_fix_status, 또는 lat/lon NaN/비정상.
  if (status < static_cast<int8_t>(min_fix_status_)) {
    return;
  }
  if (!isFiniteLatLon(lat, lon)) {
    return;
  }

  const double stamp_s = node_->now().seconds();
  std::lock_guard<std::mutex> lock(windows_mutex_);
  gps_fix_window_.push_back(GpsFixSample{stamp_s, lat, lon, status});
  const double cutoff = stamp_s - window_s_;
  while (!gps_fix_window_.empty() &&
    gps_fix_window_.front().stamp_s < cutoff)
  {
    gps_fix_window_.pop_front();
  }
}

double SlipDetector::haversineMeters(
  double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg)
{
  const double lat1 = lat1_deg * M_PI / 180.0;
  const double lat2 = lat2_deg * M_PI / 180.0;
  const double dlat = (lat2_deg - lat1_deg) * M_PI / 180.0;
  const double dlon = (lon2_deg - lon1_deg) * M_PI / 180.0;
  const double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
    std::cos(lat1) * std::cos(lat2) *
    std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
  const double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
  return kEarthRadiusM * c;
}

double SlipDetector::maxDispForStatus(int8_t status) const
{
  // status → GPS 몸 이동 상한(m). 숫자 의미는 멤버 주석·providedPorts 참고.
  // sensor_msgs/NavSatStatus: -1 NO_FIX, 0 FIX, 1 SBAS, 2 GBAS.
  if (status >= 2) {
    return max_disp_gbas_m_;  // RTK Fixed 임계
  }
  if (status >= 1) {
    return max_disp_sbas_m_;  // Float/DGNSS 임계
  }
  return max_disp_fix_m_;     // 단독 FIX 임계
}

BT::NodeStatus SlipDetector::tick()
{
  // 구독 콜백은 여기서만 돈다 — 헤더의 callback_group_ 주석 참고.
  callback_group_executor_.spin_some();

  std::deque<std::pair<double, double>> wheel_snap;
  std::deque<GpsFixSample> gps_snap;
  {
    const double now_s = node_->now().seconds();
    const double cutoff = now_s - window_s_;
    std::lock_guard<std::mutex> lock(windows_mutex_);
    while (!wheel_speed_window_.empty() &&
      wheel_speed_window_.front().first < cutoff)
    {
      wheel_speed_window_.pop_front();
    }
    while (!gps_fix_window_.empty() &&
      gps_fix_window_.front().stamp_s < cutoff)
    {
      gps_fix_window_.pop_front();
    }
    wheel_snap = wheel_speed_window_;
    gps_snap = gps_fix_window_;
  }

  // Fail-open: 데이터 부족이면 SLIP 아님 → COSTMAP/UNKNOWN.
  //   휠 샘플 0개, 또는 유효 GPS 2점 미만(변위 계산 불가).
  if (wheel_snap.empty() || gps_snap.size() < 2) {
    RCLCPP_WARN(
      node_->get_logger(),
      "SlipDetector: incomplete window (wheel=%zu gps_valid=%zu) -> FAILURE "
      "(try costmap/unknown)",
      wheel_snap.size(), gps_snap.size());
    return BT::NodeStatus::FAILURE;
  }

  double speed_sum = 0.0;
  for (const auto & sample : wheel_snap) {
    speed_sum += sample.second;
  }
  // 임계 min_wheel_speed_mps: 창 평균 휠 속도.
  const double wheel_speed_avg =
    speed_sum / static_cast<double>(wheel_snap.size());

  const GpsFixSample & oldest = gps_snap.front();
  const GpsFixSample & newest = gps_snap.back();
  // 임계 max_disp_*: 창 맨 앞·맨 뒤 GPS 수평 거리(m).
  const double body_displacement = haversineMeters(
    oldest.lat_deg, oldest.lon_deg, newest.lat_deg, newest.lon_deg);

  // 양 끝 중 더 낮은 status → 더 느슨한/빡센 이동 상한 선택.
  const int8_t status_for_thresh =
    std::min(oldest.status, newest.status);
  const double max_disp_m = maxDispForStatus(status_for_thresh);

  // SLIP SUCCESS 조건: 바퀴 평균 ≥ min_wheel  AND  GPS 이동 < max_disp.
  if (wheel_speed_avg >= min_wheel_speed_mps_ &&
    body_displacement < max_disp_m)
  {
    RCLCPP_INFO(
      node_->get_logger(),
      "SlipDetector: wheel_speed_avg=%.3f >= %.3f and gnss_disp=%.3f < %.3f "
      "(status min=%d) -> SUCCESS (SLIP branch)",
      wheel_speed_avg, min_wheel_speed_mps_, body_displacement, max_disp_m,
      static_cast<int>(status_for_thresh));
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "SlipDetector: no slip evidence (wheel_speed_avg=%.3f gnss_disp=%.3f "
    "max_disp=%.3f status_min=%d) -> FAILURE (obstacle branch)",
    wheel_speed_avg, body_displacement, max_disp_m,
    static_cast<int>(status_for_thresh));
  return BT::NodeStatus::FAILURE;
}

}  // namespace tars_recovery_behaviors
