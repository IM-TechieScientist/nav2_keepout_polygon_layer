#include "keepout_layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <vector>
#include <functional>
#include <geometry_msgs/msg/point32.hpp>

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::KeepoutPolygonLayer, nav2_costmap_2d::Layer)

namespace nav2_costmap_2d
{

void KeepoutPolygonLayer::onInitialize()
{
  RCLCPP_INFO(rclcpp::get_logger("KeepoutLayer"), "onInitialize START");
  
  auto nh = layered_costmap_->getGlobalFrameID(); // Just to check if layered_costmap works
  RCLCPP_INFO(rclcpp::get_logger("KeepoutLayer"), "Global frame: %s", nh.c_str());
  
  declareParameter("keepout_topic", rclcpp::ParameterValue("keepout_polygons"));
  
  std::string topic_name = "keepout_polygons";
  
  try {
    auto node = node_.lock();
    if (node) {
      node->get_parameter(name_ + "." + "keepout_topic", topic_name);
    }
  } catch (...) {
    RCLCPP_WARN(rclcpp::get_logger("KeepoutLayer"), "Could not get parameter, using default");
  }
  
  RCLCPP_INFO(rclcpp::get_logger("KeepoutLayer"), "Creating subscription to: %s", topic_name.c_str());
  
  auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("KeepoutLayer"), "Node lock failed");
    return;
  }
  
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

  try {
    sub_ = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
      topic_name,
      qos,
      std::bind(&KeepoutPolygonLayer::polygonCallback, this, std::placeholders::_1));

    RCLCPP_INFO(rclcpp::get_logger("KeepoutLayer"), "Subscription object created: %s",
                sub_ ? "YES" : "NO");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("KeepoutLayer"), "Exception creating subscription: %s", e.what());
  }
  
  current_ = true;
  enabled_ = true;  
}

void KeepoutPolygonLayer::polygonCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("KeepoutLayer"), "Polygon callback: points=%zu",
              msg->polygon.points.size());
  
  std::lock_guard<std::mutex> lock(mutex_);
  polygons_.push_back(*msg);
  
  RCLCPP_INFO(rclcpp::get_logger("KeepoutLayer"), "Total polygons: %zu", polygons_.size());
}

void KeepoutPolygonLayer::updateBounds(
  double, double, double, double *min_x, double *min_y, double *max_x, double *max_y)
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  for (const auto &poly : polygons_)
  {
    for (const auto &pt : poly.polygon.points)
    {
      *min_x = std::min(*min_x, static_cast<double>(pt.x));
      *min_y = std::min(*min_y, static_cast<double>(pt.y));
      *max_x = std::max(*max_x, static_cast<double>(pt.x));
      *max_y = std::max(*max_y, static_cast<double>(pt.y));
    }
  }
}

bool pointInPolygon(double x, double y, const geometry_msgs::msg::Polygon &poly)
{
  bool inside = false;
  int n = poly.points.size();
  
  for (int i = 0, j = n - 1; i < n; j = i++)
  {
    double xi = poly.points[i].x, yi = poly.points[i].y;
    double xj = poly.points[j].x, yj = poly.points[j].y;
    
    bool intersect = ((yi > y) != (yj > y)) &&
                     (x < (xj - xi) * (y - yi) / (yj - yi + 1e-9) + xi);
    if (intersect)
      inside = !inside;
  }
  return inside;
}

void KeepoutPolygonLayer::updateCosts(
  nav2_costmap_2d::Costmap2D &master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) return;
  
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (polygons_.empty()) {
    return;
  }
  
  double wx, wy;
  
  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      master_grid.mapToWorld(i, j, wx, wy);
      
      for (const auto &poly_stamped : polygons_) {
        const auto &poly = poly_stamped.polygon;
        
        if (pointInPolygon(wx, wy, poly)) {
          master_grid.setCost(i, j, LETHAL_OBSTACLE);
          break;
        }
      }
    }
  }
}

} // namespace nav2_costmap_2d