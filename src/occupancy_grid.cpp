#include "occupancy_grid/occupancy_grid.h"

OccupancyGrid::OccupancyGrid() {}

void OccupancyGrid::toRosMsg(nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid_msg) {}

void OccupancyGrid::update(double delta_x, double delta_y, double delta_yaw) {}

void OccupancyGrid::update(int laser_scan) {}

void OccupancyGrid::transform() {}