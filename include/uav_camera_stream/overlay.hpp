#pragma once
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/UInt32.h>
#include <array>
#include <ctime>
#include <string>
#include <vector>

namespace uav_camera_stream {
enum class Metric { HorizontalSpeed, VerticalSpeed, Pitch, Yaw, Roll, Battery, Satellites, Count };
constexpr size_t kMetricCount = static_cast<size_t>(Metric::Count);
const char* metricName(Metric id);
struct Sample { double value{0}, received_at{0}; bool received{false}, valid{false}; };
class Telemetry {
  std::array<Sample, kMetricCount> values_{};
  void set(Metric id, double value, bool valid, double now);
 public:
  void velocity(double x, double y, double z, double now);
  void attitude(double x, double y, double z, double w, bool available, double now);
  void battery(double fraction, bool present, double now);
  void satellites(uint32_t count, double now);
  const Sample& get(Metric id) const { return values_.at(static_cast<size_t>(id)); }
  std::string state(Metric id, double now, double timeout) const;
};
struct Field {
  Metric id; bool enabled; int line; std::string label; int precision; std::string unit;
};
struct Layout {
  double update_hz{5}, telemetry_timeout_sec{3};
  int time_line{1}, stats_line{2}, fps_precision{0}, mbps_precision{2}, xpad{16}, ypad{16};
  bool stats_enabled{true};
  std::string font_desc{"Sans 20"}, missing_text{"--"}, stale_text{"STALE"}, invalid_text{"N/A"}, separator{"  "};
  std::string velocity_topic{"/mavros/local_position/velocity_local"}, attitude_topic{"/mavros/imu/data"};
  std::string battery_topic{"/mavros/battery"}, satellites_topic{"/mavros/global_position/raw/satellites"};
  std::vector<Field> fields;
  Layout();
  static Layout parse(const XmlRpc::XmlRpcValue& config);
  void validate() const;
};
std::string wallTime(time_t now, const std::string& format);
std::string number(double value, int precision);
std::string render(const Layout& layout, const Telemetry& data, time_t wall, double steady,
                   const std::string& time_format, bool show_time, bool rates_ready, double fps, double mbps);
class Overlay {
  Layout layout_;
  Telemetry telemetry_;
  std::vector<ros::Subscriber> subscribers_;
 public:
  explicit Overlay(ros::NodeHandle& private_nh);
  const Layout& layout() const { return layout_; }
  const Telemetry& telemetry() const { return telemetry_; }
};
}  // namespace uav_camera_stream
