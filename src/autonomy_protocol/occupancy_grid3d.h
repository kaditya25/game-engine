// Author: Tucker Haydon

#pragma once

#include <string>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "map3d.h"
#include "graph.h"
#include "node_eigen.h"

namespace game_engine {
  class OccupancyGrid3D {
    private:
      bool*** data_;
      size_t size_x_, size_y_, size_z_;
      bool heap_allocated_{false};
      //Added: the origin of occupancy grid in rviz coordinate
      Eigen::Vector3d origin;
      double gridsize;

    public:
      OccupancyGrid3D() {}
      ~OccupancyGrid3D();

      // Prevent copies due to heap-allocated resouces
      OccupancyGrid3D(const OccupancyGrid3D&) = delete;
      OccupancyGrid3D& operator=(const OccupancyGrid3D&) = delete;

      // Prevent moves (for now)
      OccupancyGrid3D& operator=(OccupancyGrid3D&& other) noexcept = delete;
      OccupancyGrid3D(OccupancyGrid3D&& other) noexcept  = delete;

      // Load from various entities
      bool LoadFromFile(const std::string& file_path);
      bool LoadFromMap(const Map3D& map, const double sample_delta, const double safety_bound=0);
      bool LoadFromBuffer(const bool** buffer, const size_t size_x, const size_t size_y, const size_t size_z);

      // Create a graph representation of this occupancy grid. Every cell has a
      // directed edge to the 8 cells around it.
      Graph3D AsGraph() const;
      double GridSize() const;
      size_t SizeX() const;
      size_t SizeY() const;
      size_t SizeZ() const;

      Eigen::Vector3d Origin() const;

      bool IsOccupied(const size_t z, const size_t y, const size_t x) const;

      const bool*** Data() const;
  };
}
