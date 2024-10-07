#ifndef MARKER_ARRAY_LAYER_H_
#define MARKER_ARRAY_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/MarkerArray.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Point.h>
#include <boost/thread.hpp>

namespace marker_array_layer_namespace
{

class MarkerArrayLayer : public costmap_2d::Layer
{
public:
  MarkerArrayLayer();

  virtual void onInitialize();

  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y, double* max_x, double* max_y);

  virtual void updateCosts(costmap_2d::Costmap2D& master_grid,
                           int min_i, int min_j, int max_i, int max_j);

  // virtual void reset()
  // {
  //   return;
  // }

  // virtual void onFootprintChanged();

  // virtual bool isClearable() {return false;}

private:
  void markerArrayCallback(const visualization_msgs::MarkerArrayConstPtr& msg);

  void ReconfigureCallback(costmap_2d::GenericPluginConfig &config, uint32_t level);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

  ros::Subscriber marker_array_sub_;

  std::vector<geometry_msgs::Point> marker_positions_;

  boost::recursive_mutex lock_;

  // double static_x = 5.0, static_y = 1.0;
};

} // namespace marker_array_layer_namespace

#endif // MARKER_ARRAY_LAYER_H_
