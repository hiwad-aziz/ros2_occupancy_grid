#include "occupancy_grid/occupancy_grid_node.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <chrono>
#include <functional>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;

OccupancyGridNode::OccupancyGridNode()
    : Node("occupancy_grid"), grid_map_{std::make_unique<OccupancyGrid>(20, 0.1)}
{
  // create publisher
  publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("ocg", 10);
  // create subscribers
  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&OccupancyGridNode::handleOdom, this, std::placeholders::_1));
  laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
      std::bind(&OccupancyGridNode::handleLaserScan, this, std::placeholders::_1));
}

void OccupancyGridNode::handleOdom(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  RCLCPP_INFO(this->get_logger(), "Handling odometry data...");
  // transform based on current and previous odom data
  double delta_x = odom->pose.pose.position.x - prev_odom_.pose.pose.position.x;
  double delta_y = odom->pose.pose.position.y - prev_odom_.pose.pose.position.y;
  double delta_yaw = 0.0;

  grid_map_->update(delta_x, delta_y, delta_yaw);

  // update previous odometry data
  prev_odom_ = *odom;

  // fill msg and publish grid
  auto message = nav_msgs::msg::OccupancyGrid();
  grid_map_->toRosMsg(message);
  publisher_->publish(message);
}

void OccupancyGridNode::handleLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan)
{
  RCLCPP_INFO(this->get_logger(), "Handling laser scan data...");
  // update grid based on new laser scan data
  std::vector<Point2d<double>> scan_cartesian = convertPolarScantoCartesianScan(laser_scan);
  grid_map_->update(scan_cartesian);
  // fill msg and publish grid
  auto message = nav_msgs::msg::OccupancyGrid();
  grid_map_->toRosMsg(message);
  message.header.stamp = this->get_clock()->now();
  publisher_->publish(message);
}

std::vector<Point2d<double>> OccupancyGridNode::convertPolarScantoCartesianScan(
    const sensor_msgs::msg::LaserScan::SharedPtr laser_scan)
{
  std::vector<Point2d<double>> scan_cartesian;
  scan_cartesian.reserve(laser_scan->ranges.size());
  float angle = laser_scan->angle_min;
  Point2d<double> cartesian_point;
  for (float range : laser_scan->ranges) {
    cartesian_point.x = range * cos(angle);
    cartesian_point.y = range * sin(angle);
    scan_cartesian.push_back(cartesian_point);
    angle += laser_scan->angle_increment;
  }
  return scan_cartesian;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyGridNode>());
  rclcpp::shutdown();
  return 0;
}