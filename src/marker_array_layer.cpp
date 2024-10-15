#include "marker_array_layer/marker_array_layer.h"

PLUGINLIB_EXPORT_CLASS(marker_array_layer_namespace::MarkerArrayLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace marker_array_layer_namespace
{

MarkerArrayLayer::MarkerArrayLayer() {}

void MarkerArrayLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  marker_array_sub_ = nh.subscribe("/pred_marker", 1, &MarkerArrayLayer::markerArrayCallback, this);

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &MarkerArrayLayer::ReconfigureCallback, this, _1, _2);
  dsrv_->setCallback(cb);
}

void MarkerArrayLayer::ReconfigureCallback(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void MarkerArrayLayer::markerArrayCallback(const visualization_msgs::MarkerArrayConstPtr& msg)
{
  ROS_WARN("MarkerArray received with %zu markers", msg->markers.size());
  boost::recursive_mutex::scoped_lock lock(lock_);
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
    ROS_WARN("updateBounds called");
  boost::recursive_mutex::scoped_lock lock(lock_);
  for (const auto& point : marker_positions_)
  {
    *min_x = std::min(*min_x, point.x);
    *min_y = std::min(*min_y, point.y);
    *max_x = std::max(*max_x, point.x);
    *max_y = std::max(*max_y, point.y);
  }
  // *min_x = std::min(*min_x, static_x);
  // *min_y = std::min(*min_y, static_y);
  // *max_x = std::max(*max_x, static_x);
  // *max_y = std::max(*max_y, static_y);
}

// void MarkerArrayLayer::onFootprintChanged()
// {
//   ROS_INFO("footprint_changed");
// }

void MarkerArrayLayer::updateCosts(costmap_2d::Costmap2D& master_grid,
                                   int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;
  ROS_WARN("updateCosts called");
  boost::recursive_mutex::scoped_lock lock(lock_);
  unsigned char cost = LETHAL_OBSTACLE;

  for (const auto& point : marker_positions_)
  {
    unsigned int mx, my;
    if (master_grid.worldToMap(point.x, point.y, mx, my))
    {
      master_grid.setCost(mx, my, cost);
      ROS_WARN("%d, %d", mx, my);
    }
  }
  // unsigned int mx, my;
  // if (master_grid.worldToMap(static_x, static_y, mx, my))
  // {
  //   master_grid.setCost(mx, my, cost);
  //   ROS_WARN("%d, %d, %hhu", mx, my, cost);
  // }
}

} // namespace marker_array_layer_namespace
