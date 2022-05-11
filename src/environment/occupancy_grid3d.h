#pragma once

#include <Eigen/Dense>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "graph.h"
#include "map3d.h"
#include "node_eigen.h"

namespace game_engine {
class OccupancyGrid3D {
 public:
  OccupancyGrid3D() {}
  ~OccupancyGrid3D();

  // Constructors and assignment operators are deleted to prevent copies due to
  // heap-allocated resouces
  OccupancyGrid3D(const OccupancyGrid3D&) = delete;
  OccupancyGrid3D& operator=(const OccupancyGrid3D&) = delete;
  OccupancyGrid3D(OccupancyGrid3D&& other) noexcept = delete;
  OccupancyGrid3D& operator=(OccupancyGrid3D&& other) noexcept = delete;

  // These functions allow one to load an occupancy grid from various sources
  bool LoadFromFile(const std::string& file_path);
  // sample_delta is the grid cell size, in meters.  safety_bound is how big the
  // "inflation" bubble around obstacles will be, in meters.
  bool LoadFromMap(const Map3D& map, const double sample_delta,
                   const double safety_bound = 0);
  bool LoadFromBuffer(const bool** buffer, const size_t size_x,
                      const size_t size_y, const size_t size_z);

  // Creates a graph representation of this occupancy grid. Every cell has a
  // directed edge to the 8 cells around it.
  Graph3D AsGraph() const;
  // Returns the grid's cell size, in meters.  This value is the length, width,
  // and height of each cube-shaped cell in the grid.
  double GridSize() const { return gridsize_; }
  // These functions return the number of grid cells in the x, y, and z
  // dimensions
  size_t SizeX() const { return size_x_; }
  size_t SizeY() const { return size_y_; }
  size_t SizeZ() const { return size_z_; }
  // Returns the origin of the grid, expressed in meters in the world frame
  Eigen::Vector3d Origin() const { return origin_; }
  // Returns the minimum corner coordinates of the grid cell at index [x,y,z]
  Eigen::Vector3d boxCorner(int x, int y, int z);
  // Returns the center coordinates of the grid cell at index [x,y,z]
  Eigen::Vector3d boxCenter(int x, int y, int z);
  // Returns the 3D grid indices of the cell that contains the input point,
  // which is expressed in meters.
  std::tuple<int, int, int> mapToGridCoordinates(Eigen::Vector3d pt);
  // Indicates whether (true) or not (false) the cell at index [x,y,z] is
  // occupied.
  bool IsOccupied(const size_t z, const size_t y, const size_t x) const;
  // The 3D grid as stored in memory is addressed as a pointer to a pointer to a
  // pointer to a bool.  This function gives you direct access to the occupancy
  // data, should you need it.
  const bool*** Data() const;

 private:
  bool*** data_;
  size_t size_x_, size_y_, size_z_;
  bool heap_allocated_{false};
  Eigen::Vector3d origin_;
  double gridsize_;
};
}  // namespace game_engine
