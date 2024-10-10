#include "marker_array_layer/marker_array_layer.h"

using nav2_costmap_2d::LETHAL_OBSTACLE;

namespace marker_array_layer_namespace
{

MarkerArrayLayer::MarkerArrayLayer() {}

void MarkerArrayLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    return;
  }

  current_ = true;
  
  marker_array_sub_ = node->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/marker_array", 1, 
    std::bind(&MarkerArrayLayer::markerArrayCallback, this, std::placeholders::_1));

  params_callback_handle_ = node->add_on_set_parameters_callback(
    std::bind(&MarkerArrayLayer::parametersCallback, this, std::placeholders::_1));
    
  node->declare_parameter("enabled", rclcpp::ParameterValue(true));
  enabled_ = node->get_parameter("enabled").as_bool();
}

rcl_interfaces::msg::SetParametersResult 
MarkerArrayLayer::parametersCallback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    if (param.get_name() == "enabled") {
      enabled_ = param.as_bool();
    }
  }

  return result;
}

void MarkerArrayLayer::markerArrayCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
  RCLCPP_WARN(rclcpp::get_logger("marker_array_layer"), 
              "MarkerArray received with %zu markers", msg->markers.size());
  std::lock_guard<std::mutex> lock(mutex_);
  marker_positions_.clear();
  for (const auto& marker : msg->markers)
  {
    for (const auto& point : marker.points)
    {
      marker_positions_.push_back(point);
    }
  }
}

void MarkerArrayLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                  double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;
  RCLCPP_WARN(rclcpp::get_logger("marker_array_layer"), "updateBounds called");
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& point : marker_positions_)
  {
    *min_x = std::min(*min_x, point.x);
    *min_y = std::min(*min_y, point.y);
    *max_x = std::max(*max_x, point.x);
    *max_y = std::max(*max_y, point.y);
  }
}

void MarkerArrayLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                                 int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;
  RCLCPP_WARN(rclcpp::get_logger("marker_array_layer"), "updateCosts called");
  std::lock_guard<std::mutex> lock(mutex_);
  unsigned char cost = LETHAL_OBSTACLE;

  for (const auto& point : marker_positions_)
  {
    unsigned int mx, my;
    if (master_grid.worldToMap(point.x, point.y, mx, my))
    {
      master_grid.setCost(mx, my, cost);
      RCLCPP_WARN(rclcpp::get_logger("marker_array_layer"), "%d, %d", mx, my);
    }
  }
}

} // namespace marker_array_layer_namespace