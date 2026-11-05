// Livox-CustomMsg → sensor_msgs/PointCloud2 (x, y, z, intensity; Frame bleibt livox_frame).
//
//   /livox/lidar  livox_ros_driver2/CustomMsg   Sub  (xfer_format 1, wie FAST-LIO es braucht)
//   /livox/points sensor_msgs/PointCloud2       Pub  ohne Nullpunkte, ohne Punkte näher als `blind`, ohne Punkte in den
//                                                    Quadern `exclude_boxes` (Roboter selbst), wahlweise nur jeder n-te Punkt
//
// Wozu: Der Livox-Treiber kann nur ein Format senden. FAST-LIO braucht das CustomMsg, pointcloud_to_laserscan und die
// Nav2-Costmaps brauchen PointCloud2. Die Filter sind dieselben wie in FAST-LIO (Tag-Prüfung, blind, exclude_boxes), damit
// Scan und LIO denselben Roboter "nicht sehen".
#include <cmath>
#include <cstring>
#include <memory>
#include <vector>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

class LivoxToCloud : public rclcpp::Node
{
public:
  LivoxToCloud() : Node("livox_to_cloud")
  {
    blind_ = declare_parameter<double>("blind", 0.3);              // m, näher wird verworfen
    keep_every_ = std::max<int64_t>(1, declare_parameter<int64_t>("keep_every", 1));
    boxes_ = declare_parameter<std::vector<double>>("exclude_boxes", std::vector<double>());
    if (boxes_.size() % 6 != 0) {
      RCLCPP_ERROR(get_logger(), "exclude_boxes: Länge %zu ist kein Vielfaches von 6 – ignoriert", boxes_.size());
      boxes_.clear();
    }
    const auto in = declare_parameter<std::string>("input_topic", "/livox/lidar");
    const auto out = declare_parameter<std::string>("output_topic", "/livox/points");
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(out, rclcpp::SensorDataQoS());
    sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      in, rclcpp::SensorDataQoS(), [this](livox_ros_driver2::msg::CustomMsg::UniquePtr msg) { convert(*msg); });
    RCLCPP_INFO(get_logger(), "%s → %s (blind %.2f m, %zu Quader ausgeschnitten, jeder %ld. Punkt)", in.c_str(),
                out.c_str(), blind_, boxes_.size() / 6, static_cast<long>(keep_every_));
  }

private:
  bool excluded(float x, float y, float z) const
  {
    for (size_t k = 0; k + 5 < boxes_.size(); k += 6)
      if (x >= boxes_[k] && x <= boxes_[k + 1] && y >= boxes_[k + 2] && y <= boxes_[k + 3] && z >= boxes_[k + 4] &&
          z <= boxes_[k + 5])
        return true;
    return false;
  }

  void convert(const livox_ros_driver2::msg::CustomMsg & msg)
  {
    if (pub_->get_subscription_count() == 0) return;
    auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    cloud->header = msg.header;
    cloud->height = 1;
    cloud->is_bigendian = false;
    cloud->is_dense = true;
    cloud->point_step = 16;
    const char * names[] = {"x", "y", "z", "intensity"};
    for (uint32_t i = 0; i < 4; ++i) {
      sensor_msgs::msg::PointField f;
      f.name = names[i];
      f.offset = 4 * i;
      f.datatype = sensor_msgs::msg::PointField::FLOAT32;
      f.count = 1;
      cloud->fields.push_back(f);
    }
    cloud->data.resize(static_cast<size_t>(msg.points.size()) * cloud->point_step);
    const double blind2 = blind_ * blind_;
    size_t n = 0;
    int64_t valid = 0;
    for (const auto & p : msg.points) {
      // wie FAST-LIO (preprocess.cpp, avia_handler): nur Punkte mit gültigem Tag
      if ((p.tag & 0x30) != 0x10 && (p.tag & 0x30) != 0x00) continue;
      if (static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y + static_cast<double>(p.z) * p.z <= blind2)
        continue;                                                // auch die Nullpunkte (kein Echo)
      if (excluded(p.x, p.y, p.z)) continue;
      if (valid++ % keep_every_ != 0) continue;
      const float v[4] = {p.x, p.y, p.z, static_cast<float>(p.reflectivity)};
      std::memcpy(&cloud->data[n * cloud->point_step], v, sizeof(v));
      ++n;
    }
    cloud->data.resize(n * cloud->point_step);
    cloud->width = static_cast<uint32_t>(n);
    cloud->row_step = cloud->width * cloud->point_step;
    pub_->publish(std::move(cloud));
  }

  double blind_;
  int64_t keep_every_;
  std::vector<double> boxes_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxToCloud>());
  rclcpp::shutdown();
  return 0;
}
