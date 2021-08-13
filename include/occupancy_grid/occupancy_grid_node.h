#ifndef OCCUPANCYGRIDNODE_H
#define OCCUPANCYGRIDNODE_H

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "occupancy_grid/occupancy_grid.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class OccupancyGridNode : public rclcpp::Node {
 public:
  OccupancyGridNode();

 private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  void handleOdom(const nav_msgs::msg::Odometry::SharedPtr odom);
  void handleLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan);
  std::vector<Point2d<double>> convertPolarScantoCartesianScan(
      const sensor_msgs::msg::LaserScan::SharedPtr laser_scan);
  nav_msgs::msg::Odometry prev_odom_{};
  std::unique_ptr<OccupancyGrid> grid_map_;
};

#endif  // OCCUPANCYGRIDNODE_H