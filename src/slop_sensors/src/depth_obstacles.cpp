// Hinderniserkennung aus dem Tiefenbild der RealSense (ohne Abgründe).
//
//   /camera/depth/image_rect_raw  sensor_msgs/Image (16UC1 in mm oder 32FC1 in m)   Sub
//   /camera/depth/camera_info     sensor_msgs/CameraInfo                             Sub
//   /camera/obstacles             sensor_msgs/PointCloud2 (target_frame)             Pub  Hindernisse: min_obstacle_height
//                                                                                         bis max_obstacle_height über dem
//                                                                                         Boden, je Rasterzelle ein Punkt
//   /camera/floor                 sensor_msgs/PointCloud2 (target_frame)             Pub  freier Boden (|z| < floor_band),
//                                                                                         zum Freiräumen der Costmap
//
// Ablauf je Bild: jeden `stride`-ten Pixel in 3D umrechnen (Kameramodell aus camera_info), über TF in target_frame
// (base_footprint, z = 0 ist der Boden) bringen und nach Höhe einteilen. Hindernispunkte werden in ein Raster
// (`voxel` m) einsortiert; eine Zelle zählt erst ab `min_points` Punkten (gegen Tiefenrauschen und Ausreißer).
// Zwischen floor_band und min_obstacle_height liegt eine Lücke ohne Entscheidung (Bodenrauschen, Teppichkanten).
// Punkte unter dem Boden (Abgründe) werden noch nicht ausgewertet.
//
// Prüfung der Kameraneigung: Alle stats_period s steht im Log, auf welcher Höhe der Boden gesehen wird (Median der
// Punkte mit |z| < 0,15 m, getrennt nah/fern). Weicht er mehr als ein paar Zentimeter von 0 ab, stimmt die Neigung im
// URDF nicht: ros2 run slop_calibration calibrate mount.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{
struct Cell
{
  float x = 0, y = 0, z = 0;
  int n = 0;
};

inline int64_t cell_key(float x, float y, float z, float size)
{
  // 21 Bit je Achse (±1 Mio. Zellen) reichen für einige Kilometer bei 5 cm
  const auto q = [size](float v) { return static_cast<int64_t>(std::floor(v / size)) & 0x1FFFFF; };
  return (q(x) << 42) | (q(y) << 21) | q(z);
}

std::unique_ptr<sensor_msgs::msg::PointCloud2> make_cloud(const std_msgs::msg::Header & header,
                                                          const std::vector<float> & xyz)
{
  auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
  cloud->header = header;
  cloud->height = 1;
  cloud->width = static_cast<uint32_t>(xyz.size() / 3);
  cloud->is_bigendian = false;
  cloud->is_dense = true;
  cloud->point_step = 12;
  cloud->row_step = cloud->width * cloud->point_step;
  const char * names[] = {"x", "y", "z"};
  for (uint32_t i = 0; i < 3; ++i) {
    sensor_msgs::msg::PointField f;
    f.name = names[i];
    f.offset = 4 * i;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
    cloud->fields.push_back(f);
  }
  cloud->data.resize(xyz.size() * sizeof(float));
  if (!xyz.empty()) std::memcpy(cloud->data.data(), xyz.data(), cloud->data.size());
  return cloud;
}

double median(std::vector<float> & v)
{
  if (v.empty()) return std::nan("");
  auto mid = v.begin() + static_cast<long>(v.size() / 2);
  std::nth_element(v.begin(), mid, v.end());
  return *mid;
}
}  // namespace

class DepthObstacles : public rclcpp::Node
{
public:
  DepthObstacles() : Node("depth_obstacles")
  {
    target_frame_ = declare_parameter<std::string>("target_frame", "base_footprint");
    stride_ = std::max<int64_t>(1, declare_parameter<int64_t>("stride", 4));
    min_range_ = declare_parameter<double>("min_range", 0.25);
    max_range_ = declare_parameter<double>("max_range", 2.5);
    min_h_ = declare_parameter<double>("min_obstacle_height", 0.05);
    max_h_ = declare_parameter<double>("max_obstacle_height", 0.9);
    floor_band_ = declare_parameter<double>("floor_band", 0.03);
    voxel_ = declare_parameter<double>("voxel", 0.05);
    min_points_ = declare_parameter<int64_t>("min_points", 3);
    floor_voxel_ = declare_parameter<double>("floor_voxel", 0.10);
    max_rate_ = declare_parameter<double>("max_rate", 15.0);
    stats_period_ = declare_parameter<double>("stats_period", 10.0);
    const auto depth_topic = declare_parameter<std::string>("depth_topic", "/camera/depth/image_rect_raw");
    const auto info_topic = declare_parameter<std::string>("info_topic", "/camera/depth/camera_info");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    obstacles_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/camera/obstacles", rclcpp::SensorDataQoS());
    floor_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/camera/floor", rclcpp::SensorDataQoS());
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      info_topic, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { info_ = msg; });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { process(*msg); });
    if (stats_period_ > 0)
      stats_timer_ = create_wall_timer(std::chrono::duration<double>(stats_period_), [this] { log_stats(); });
    RCLCPP_INFO(get_logger(), "%s → /camera/obstacles, /camera/floor in %s (Hindernis %.2f–%.2f m über dem Boden, "
                "Reichweite %.2f–%.2f m, jeder %ld. Pixel, Raster %.2f m ab %ld Punkten)", depth_topic.c_str(),
                target_frame_.c_str(), min_h_, max_h_, min_range_, max_range_, static_cast<long>(stride_), voxel_,
                static_cast<long>(min_points_));
  }

private:
  void process(const sensor_msgs::msg::Image & img)
  {
    if (!info_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Warte auf camera_info");
      return;
    }
    const rclcpp::Time stamp(img.header.stamp);
    if (max_rate_ > 0 && last_stamp_.nanoseconds() > 0 && (stamp - last_stamp_).seconds() < 0.9 / max_rate_) return;
    if (pub_count() == 0) return;
    const bool mm = img.encoding == "16UC1" || img.encoding == "mono16";
    if (!mm && img.encoding != "32FC1") {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Tiefenbild-Format %s nicht unterstützt",
                            img.encoding.c_str());
      return;
    }
    tf2::Transform tf;
    try {
      // statisch (URDF + Treiber): neuester Wert genügt
      tf2::fromMsg(tf_buffer_->lookupTransform(target_frame_, img.header.frame_id, tf2::TimePointZero).transform, tf);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Kein TF %s → %s (Modell/Bringup läuft?): %s",
                           target_frame_.c_str(), img.header.frame_id.c_str(), e.what());
      return;
    }
    last_stamp_ = stamp;
    const auto t0 = std::chrono::steady_clock::now();

    const double fx = info_->k[0], fy = info_->k[4], cx = info_->k[2], cy = info_->k[5];
    if (fx <= 0 || fy <= 0) return;
    obstacle_cells_.clear();
    floor_cells_.clear();
    for (uint32_t v = 0; v < img.height; v += static_cast<uint32_t>(stride_)) {
      const uint8_t * row = img.data.data() + static_cast<size_t>(v) * img.step;
      for (uint32_t u = 0; u < img.width; u += static_cast<uint32_t>(stride_)) {
        float z;
        if (mm) {
          uint16_t d;
          std::memcpy(&d, row + 2 * u, 2);
          z = d * 0.001f;
        } else {
          std::memcpy(&z, row + 4 * u, 4);
        }
        if (!(z >= min_range_ && z <= max_range_)) continue;      // auch 0 = keine Tiefe, NaN
        const tf2::Vector3 p = tf * tf2::Vector3((u - cx) * z / fx, (v - cy) * z / fy, z);
        const float px = static_cast<float>(p.x()), py = static_cast<float>(p.y()), pz = static_cast<float>(p.z());
        if (std::fabs(pz) < 0.15f && (++floor_sample_ % 16) == 0)
          (z < 1.2f ? floor_near_ : floor_far_).push_back(pz);    // Stichprobe für die Neigungsprüfung
        if (pz >= min_h_ && pz <= max_h_) {
          add(obstacle_cells_, px, py, pz, voxel_);
        } else if (std::fabs(pz) <= floor_band_) {
          add(floor_cells_, px, py, 0.0f, floor_voxel_);
        }
      }
    }
    std::vector<float> obst, floor;
    for (const auto & [key, c] : obstacle_cells_)
      if (c.n >= min_points_) obst.insert(obst.end(), {c.x / c.n, c.y / c.n, c.z / c.n});
    for (const auto & [key, c] : floor_cells_) floor.insert(floor.end(), {c.x / c.n, c.y / c.n, 0.0f});
    std_msgs::msg::Header header;
    header.stamp = img.header.stamp;
    header.frame_id = target_frame_;
    obstacles_pub_->publish(make_cloud(header, obst));
    floor_pub_->publish(make_cloud(header, floor));

    ++frames_;
    obstacles_sum_ += obst.size() / 3;
    ms_sum_ += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  }

  static void add(std::unordered_map<int64_t, Cell> & cells, float x, float y, float z, double size)
  {
    Cell & c = cells[cell_key(x, y, z, static_cast<float>(size))];
    c.x += x;
    c.y += y;
    c.z += z;
    ++c.n;
  }

  size_t pub_count() const
  {
    return obstacles_pub_->get_subscription_count() + floor_pub_->get_subscription_count() +
           obstacles_pub_->get_intra_process_subscription_count() + floor_pub_->get_intra_process_subscription_count();
  }

  void log_stats()
  {
    if (frames_ == 0) {
      if (pub_count() > 0)
        RCLCPP_INFO(get_logger(), "Noch keine Tiefenbilder verarbeitet (Kamera und TF %s → Kamera da?)",
                    target_frame_.c_str());
      return;
    }
    const double near = median(floor_near_), far = median(floor_far_);
    const bool tilted = std::fabs(near) > 0.03 || std::fabs(far) > 0.03;
    char text[400];
    std::snprintf(text, sizeof(text), "letzte %.0f s: %zu Bilder, im Mittel %zu Hinderniszellen, %.1f ms je Bild. "
                  "Boden gesehen bei z = %+.1f cm (bis 1,2 m) / %+.1f cm (weiter)", stats_period_, frames_,
                  obstacles_sum_ / frames_, ms_sum_ / frames_, near * 100, far * 100);
    if (tilted)
      RCLCPP_WARN(get_logger(), "%s – Boden liegt nicht bei 0: Kameraneigung prüfen (calibrate mount)", text);
    else
      RCLCPP_INFO(get_logger(), "%s", text);
    frames_ = 0;
    obstacles_sum_ = 0;
    ms_sum_ = 0;
    floor_near_.clear();
    floor_far_.clear();
  }

  std::string target_frame_;
  int64_t stride_, min_points_;
  double min_range_, max_range_, min_h_, max_h_, floor_band_, voxel_, floor_voxel_, max_rate_, stats_period_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  std::unordered_map<int64_t, Cell> obstacle_cells_, floor_cells_;
  std::vector<float> floor_near_, floor_far_;
  size_t frames_ = 0, obstacles_sum_ = 0, floor_sample_ = 0;
  double ms_sum_ = 0;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_pub_, floor_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthObstacles>());
  rclcpp::shutdown();
  return 0;
}
