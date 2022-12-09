#include "occupancy_grid/occupancy_grid.h"

#include <assert.h>

#include <cmath>

OccupancyGrid::OccupancyGrid(unsigned int grid_size, double cell_size)
    : grid_size_{grid_size}, cell_size_{cell_size}
{
  num_cells_ = floor(grid_size_ / cell_size_);
  grid_center_ = {num_cells_ / 2, num_cells_ / 2};
  map_.resize(num_cells_, num_cells_);
  map_.setConstant(0.5);
}

void OccupancyGrid::toRosMsg(nav_msgs::msg::OccupancyGrid& occupancy_grid_msg)
{
  occupancy_grid_msg.info.width = num_cells_;
  occupancy_grid_msg.info.height = num_cells_;
  occupancy_grid_msg.info.resolution = cell_size_;
  occupancy_grid_msg.info.origin.position.x = -grid_center_.x * cell_size_;
  occupancy_grid_msg.info.origin.position.y = -grid_center_.y * cell_size_;
  occupancy_grid_msg.header.frame_id = "laser_frame";

  const int num_cells_grid = num_cells_ * num_cells_;
  for (size_t i = 0; i < num_cells_grid; i++) {
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
  Eigen::MatrixXd temp_map(num_cells_, num_cells_);
  temp_map.setConstant(0.5);
  // Convert position delta to cell delta
  double delta_x_grid = delta_x / cell_size_;
  double delta_y_grid = delta_y / cell_size_;
  double cos_dyaw = cos(delta_yaw);
  double sin_dyaw = sin(delta_yaw);
  Eigen::MatrixXd transformation_matrix(3, 3);
  transformation_matrix << cos_dyaw, -sin_dyaw, delta_x_grid, sin_dyaw, cos_dyaw, delta_y_grid, 0.0,
      0.0, 1.0;
  for (int x = 0; x < num_cells_; ++x) {
    for (int y = 0; y < num_cells_; ++y) {
      Eigen::VectorXd old_indices(3);
      // Translate (x, y) to origin
      old_indices << x - grid_center_.x, y - grid_center_.y, 1.0;
      // Transform according to robot movement
      Eigen::VectorXd new_indices = transformation_matrix * old_indices;
      // Translate back to map indices
      new_indices(0) += grid_center_.x;
      new_indices(1) += grid_center_.y;
      if (isInGridBounds(floor(new_indices(0)), floor(new_indices(1)))) {
        temp_map(floor(new_indices(0)), floor(new_indices(1))) = map_(x, y);
      }
    }
  }
  map_ = temp_map;
}

void OccupancyGrid::update(const std::vector<Point2d<double>>& laser_scan)
{
  // Create vector of free cells and reserve approximate amount of memory for max possible distance
  std::vector<Point2d<int>> free_cells;
  free_cells.reserve(floor(grid_size_));
  for (const Point2d<double>& point : laser_scan) {
    // Convert position of detection to cell indices
    Point2d<int> grid_point{floor(point.x / cell_size_) + grid_center_.x,
                            floor(point.y / cell_size_) + grid_center_.y};
    updateCellProbability(grid_point, CellState::OCCUPIED);
    // Run Bresenham algorithm to get all free cells
    getFreeCells(grid_point, free_cells);
    for (const Point2d<int>& free_cell : free_cells) {
      updateCellProbability(free_cell, CellState::FREE);
    }
    free_cells.clear();
  }
}

void OccupancyGrid::updateCellProbability(const Point2d<int>& point, CellState state)
{
  // Calculate new log odds and add to current log odds
  double log_prob{0.0};
  switch (state) {
    case CellState::FREE:
      log_prob = log(p_free_ / (1.0 - p_free_));
      break;
    case CellState::OCCUPIED:
      log_prob = log(p_occ_ / (1.0 - p_occ_));
      break;
    default:
      log_prob = log(p_prior_ / (1.0 - p_prior_));
      break;
  }
  double current_log_prob = log(map_(point.x, point.y) / (1.0 - map_(point.x, point.y)));
  current_log_prob += log_prob;
  // Convert log odds to probability and update cell
  map_(point.x, point.y) = 1.0 - 1.0 / (1 + exp(current_log_prob));
}

void OccupancyGrid::getFreeCells(const Point2d<int>& detection,
                                 std::vector<Point2d<int>>& free_cells)
{
  int x_start = grid_center_.x;
  int y_start = grid_center_.y;
  int dx = abs(detection.x - x_start);
  int dy = -abs(detection.y - y_start);
  int sx = x_start < detection.x ? 1 : -1;
  int sy = y_start < detection.y ? 1 : -1;
  int error = dx / 2;
  int e2;

  while ((x_start != detection.x) && (y_start != detection.y)) {
    free_cells.push_back({x_start, y_start});
    e2 = 2 * error;
    if (e2 > dy) {
      error += dy;
      x_start += sx;
    }
    if (e2 < dx) {
      error += dx;
      y_start += sy;
    }
  }
}

bool OccupancyGrid::isInGridBounds(int x, int y)
{
  if (!(x >= 0 && x < num_cells_)) return false;
  if (!(y >= 0 && y < num_cells_)) return false;
  return true;
}
