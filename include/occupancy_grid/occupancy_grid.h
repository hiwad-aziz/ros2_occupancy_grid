#ifndef OCCUPANCYGRID_H
#define OCCUPANCYGRID_H

#include <Eigen/Dense>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

struct Point2d;
enum class CellState : unsigned int { FREE, OCCUPIED };

class OccupancyGrid {
 public:
  OccupancyGrid(unsigned int grid_size, double cell_size);

  void toRosMsg(nav_msgs::msg::OccupancyGrid& occupancy_grid_msg);
  void update(double delta_x, double delta_y, double delta_yaw);
  void update(const std::vector<Point2d>& laser_scan);

 private:
  void updateCellProbability(int x, int y, CellState state);
  Eigen::MatrixXd map_;
  unsigned int grid_size_{20};
  double cell_size_{0.1};
  unsigned int num_cells_{200};
};

struct Point2d {
  double x;
  double y;
};

#endif  // OCCUPANCYGRID_H