// IMU drosseln: leitet von imu_in höchstens `rate` Nachrichten pro Sekunde (nach Zeitstempel) an imu_out weiter.
//
// Wozu: mount_tf (Python) braucht die IMU nur für die Schwerkraft. 200 Hz je IMU wecken den Python-Executor
// 400×/s und kosten ~45 % eines Kerns; in C++ kostet das Drosseln fast nichts.
// Als Komponente gebaut (slop_sensors::ImuThrottle), läuft in einem Container (sensors.launch.py).
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace slop_sensors
{

class ImuThrottle : public rclcpp::Node
{
public:
  explicit ImuThrottle(const rclcpp::NodeOptions & options)
  : Node("imu_throttle", options)
  {
    const double rate = declare_parameter<double>("rate", 25.0);   // Hz
    min_period_ns_ = rate > 0.0 ? static_cast<int64_t>(1e9 / rate) : 0;
    pub_ = create_publisher<sensor_msgs::msg::Imu>("imu_out", rclcpp::SensorDataQoS());
    sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu_in", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::UniquePtr msg) {
        const int64_t t = rclcpp::Time(msg->header.stamp).nanoseconds();
        // Zeitsprung rückwärts (z. B. Treiberneustart) → neu anfangen
        if (t - last_ns_ < min_period_ns_ && t >= last_ns_) {
          return;
        }
        last_ns_ = t;
        pub_->publish(std::move(msg));
      });
    RCLCPP_INFO(get_logger(), "%s → %s mit höchstens %.0f Hz", sub_->get_topic_name(), pub_->get_topic_name(), rate);
  }

private:
  int64_t min_period_ns_{0};
  int64_t last_ns_{0};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
};

}  // namespace slop_sensors

RCLCPP_COMPONENTS_REGISTER_NODE(slop_sensors::ImuThrottle)
