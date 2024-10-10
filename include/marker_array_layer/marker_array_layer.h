#ifndef MARKER_ARRAY_LAYER_H_
#define MARKER_ARRAY_LAYER_H_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <mutex>
#include <vector>
#include <memory>

namespace marker_array_layer_namespace
{

class MarkerArrayLayer : public nav2_costmap_2d::Layer
{
public:
  MarkerArrayLayer();

  virtual void reset() override;
  virtual bool isClearable() override;
  
  virtual void onInitialize() override;
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                          double* min_x, double* min_y, double* max_x, double* max_y) override;
  virtual void updateCosts(nav2_costmap_2d::Costmap2D& master_grid, 
                         int min_i, int min_j, int max_i, int max_j) override;

private:
  void markerArrayCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
  
  rcl_interfaces::msg::SetParametersResult 
  parametersCallback(const std::vector<rclcpp::Parameter> & parameters);

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_sub_;
  
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
  
  std::vector<geometry_msgs::msg::Point> marker_positions_;
  
  std::mutex mutex_;
};

} // namespace marker_array_layer_namespace

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(marker_array_layer_namespace::MarkerArrayLayer, nav2_costmap_2d::Layer)

#endif // MARKER_ARRAY_LAYER_H_