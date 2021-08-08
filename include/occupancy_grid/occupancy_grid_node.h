#ifndef OCCUPANCYGRIDNODE_H
#define OCCUPANCYGRIDNODE_H

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
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
  nav_msgs::msg::Odometry prev_odom_{};
};

#endif  // OCCUPANCYGRIDNODE_H