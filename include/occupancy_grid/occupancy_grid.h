#ifndef OCCUPANCYGRID_H
#define OCCUPANCYGRID_H

#include <Eigen/Dense>

#include "nav_msgs/msg/occupancy_grid.hpp"

class OccupancyGrid {
 public:
  OccupancyGrid();

  void toRosMsg(nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid_msg);
  void update(double delta_x, double delta_y, double delta_yaw);
  void update(int laser_scan);

 private:
  void transform();
  Eigen::MatrixXd map;
};

#endif  // OCCUPANCYGRID_H