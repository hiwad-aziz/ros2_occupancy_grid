#include "occupancy_grid/occupancy_grid_node.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <chrono>
#include <functional>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;

OccupancyGridNode::OccupancyGridNode() : Node("occupancy_grid")
{
  // Create publisher
  publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("ocg", 10);
  // create subscribers
  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&OccupancyGridNode::handleOdom, this, std::placeholders::_1));
  laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&OccupancyGridNode::handleLaserScan, this, std::placeholders::_1));
}

void OccupancyGridNode::handleOdom(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  // transform based on current and previous odom data
  double delta_x = odom->pose.pose.position.x - prev_odom_.pose.pose.position.x;
  double delta_y = odom->pose.pose.position.y - prev_odom_.pose.pose.position.y;

  // Update previous odometry data
  prev_odom_ = *odom;

  // fill msg and publish grid
}

void OccupancyGridNode::handleLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan)
{
  // update grid based on new laser scan data
  // calculate cartesian coordinate for each point in laser scan
  // use bresenhams line algorithm to update all observed cells
  // fill msg and publish grid
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyGridNode>());
  rclcpp::shutdown();
  return 0;
}