#include "occupancy_grid/occupancy_grid.h"

#include <assert.h>

#include <cmath>

OccupancyGrid::OccupancyGrid(unsigned int grid_size, double cell_size)
    : grid_size_{grid_size}, cell_size_{cell_size}
{
  assert(fmod(grid_size_, cell_size_) == 0.0);
  num_cells_ = grid_size_ / cell_size_;
  map_.resize(num_cells_, num_cells_);
  map_.setConstant(0.5);
}

void OccupancyGrid::toRosMsg(nav_msgs::msg::OccupancyGrid& occupancy_grid_msg)
{
  occupancy_grid_msg.info.width = grid_size_;
  occupancy_grid_msg.info.height = grid_size_;
  occupancy_grid_msg.info.resolution = cell_size_;
  occupancy_grid_msg.info.origin.position.x = 0.0;
  occupancy_grid_msg.info.origin.position.y = 0.0;

  const int num_cells = grid_size_ * grid_size_;
  for (size_t i = 0; i < num_cells; i++) {
    double& occ_prob = map_.data()[i];
    if (occ_prob == 0.5) {
      occupancy_grid_msg.data.push_back(-1);
    }
    else {
      occupancy_grid_msg.data.push_back(occ_prob * 100);
    }
  }
}

void OccupancyGrid::update(double delta_x, double delta_y, double delta_yaw)
{
  Eigen::MatrixXd temp_map = map_;
  int delta_x_grid = delta_x / cell_size_;
  int delta_y_grid = delta_y / cell_size_;
  double cos_dyaw = cos(delta_yaw);
  double sin_dyaw = sin(delta_yaw);
  Eigen::MatrixXd transformation_matrix(3, 3);
  transformation_matrix << cos_dyaw, -sin_dyaw, delta_x_grid, sin_dyaw, cos_dyaw, delta_y_grid, 0.0,
      0.0, 1.0;
  int new_x_grid, new_y_grid;
  for (int x = 0; x < num_cells_; ++x) {
    for (int y = 0; y < num_cells_; ++y) {
      Eigen::VectorXd old_indices(3);
      old_indices << x, y, 1.0;
      Eigen::VectorXd new_indices = transformation_matrix * old_indices;
      map_(old_indices(0), old_indices(1)) = temp_map(x, y);
    }
  }
}

void OccupancyGrid::update(const std::vector<Point2d>& laser_scan)
{
  for (const Point2d& point : laser_scan) {
    updateCellProbability(point.x, point.y, CellState::OCCUPIED);
    std::vector<Point2d> free_cells;
    // TODO: run Bresenham algorithm to get all free cells
    free_cells.push_back({0.0, 0.0});
    for (const Point2d& free_cell : free_cells) {
      updateCellProbability(free_cell.x, free_cell.y, CellState::FREE);
    }
  }
}

void OccupancyGrid::updateCellProbability(int x, int y, CellState state) {}