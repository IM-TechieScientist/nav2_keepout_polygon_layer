#pragma once

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <mutex>

namespace nav2_costmap_2d
{

class KeepoutPolygonLayer : public CostmapLayer
{
public:
  KeepoutPolygonLayer() = default;
  virtual ~KeepoutPolygonLayer() = default;
  
  void onInitialize() override;
  void updateBounds(double, double, double, double*, double*, double*, double*) override;
  void updateCosts(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) override;
  void reset() override { 
    std::lock_guard<std::mutex> lock(mutex_);
    polygons_.clear(); 
  }
  bool isClearable() override { return false; }

private:
  void polygonCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);
  
  std::vector<geometry_msgs::msg::PolygonStamped> polygons_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr sub_;
  std::mutex mutex_;  // Protect polygons_ from concurrent access
};

} // namespace nav2_costmap_2d