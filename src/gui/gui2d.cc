// Author: Tucker Haydon

#include "gui2d.h"

using namespace geometry;

namespace path_planning {
  namespace {
    struct PolygonView {
      struct Options {
        bool fill{false};
        std::string fill_color{"black"};
        bool transparent{false};
        std::string line_color{"black"};
        Options() {}
      };

      Polygon polygon_;
      Options options_;

      PolygonView(const Polygon& polygon,
                  const Options& options = Options())
        : options_(options),
          polygon_(polygon) {}

      std::vector<Gui2D::CommandUnit> Load() const {
        std::vector<std::pair<double, double>> data;
        data.reserve(this->polygon_.Vertices().size() + 1);
        for(const Point2D& vertex: this->polygon_.Vertices()) {
          data.emplace_back(vertex.x(), vertex.y());
        }
        data.emplace_back(this->polygon_.Vertices()[0].x(), this->polygon_.Vertices()[0].y());

        // For a polygon with a difference color for fill and lines, must draw
        // the polygon twice: once filled and one not filled
        std::vector<Gui2D::CommandUnit> command_units;
        if(options_.fill) {
          std::string command = "'-' ";
          command += "with filledcurves fillcolor rgb '" + options_.fill_color + "' ";
          command += options_.transparent ? "fillstyle transparent solid 0.3 " : "";
          command += ",";
          command_units.emplace_back(command, data);
        }

        {
          std::string command = "'-' ";
          command += "with lines linetype rgb '" + this->options_.line_color + "' ";
          command += ",";
          command_units.emplace_back(command, data);
        }

        return command_units;
      };
    };

    struct PathView {
      struct Options {
        std::string line_color{"black"};
        Options() {}
      };

      std::vector<Point2D> path_;
      Options options_;

      PathView(const std::vector<Point2D>& path,
                  const Options& options = Options())
        : options_(options),
          path_(path) {}

      std::vector<Gui2D::CommandUnit> Load() const {
        std::vector<std::pair<double, double>> data;
        data.reserve(this->path_.size());
        for(const Point2D& point: this->path_) {
          data.emplace_back(point.x(), point.y());
        }

        std::vector<Gui2D::CommandUnit> command_units;
        std::string command = "'-' ";
        // command += "with lines linecolor rgb '" + this->options_.line_color + "' linetype 1 linewidth 2";
        command += "with linespoints linecolor rgb '" + this->options_.line_color + 
          "' linetype 1 linewidth 2 pointtype 7 pointinterval -1 pointsize 1.5";
        command += ",";
        command_units.emplace_back(command, data);

        return command_units;
      };
    };
  }


  bool Gui2D::LoadMap(const Map2D& map) {
    { // Draw boundary
      PolygonView::Options options;
      const std::vector<CommandUnit> command_units 
        = PolygonView(map.Boundary(), options).Load();
      this->command_units_.insert(
          this->command_units_.end(),
          command_units.begin(),
          command_units.end());
    }

    { // Draw obstacles
      PolygonView::Options options;
      options.fill = true;
      options.fill_color = "black";
      for(const Polygon& obstacle: map.Obstacles()) {
        const std::vector<CommandUnit> command_units 
          = PolygonView(obstacle, options).Load();
        this->command_units_.insert(
            this->command_units_.end(),
            command_units.begin(),
            command_units.end());
      }
    }
    return true;
  }
  
  bool Gui2D::LoadSafeFlightCorridor(const std::vector<Polygon>& safe_flight_corridor) {
    for(const Polygon& polygon: safe_flight_corridor) {
      PolygonView::Options options;
      options.fill = true;
      options.fill_color = "green";
      options.line_color = "black";
      options.transparent = true;
      const std::vector<CommandUnit> command_units 
        = PolygonView(polygon, options).Load();
      this->command_units_.insert(
          this->command_units_.end(),
          command_units.begin(),
          command_units.end());
    }
    return true;
  }

  bool Gui2D::LoadPath(const std::vector<Point2D>& path) {
      PathView::Options options;
      const std::vector<CommandUnit> command_units 
        = PathView(path, options).Load();
      this->command_units_.insert(
          this->command_units_.end(),
          command_units.begin(),
          command_units.end());
    return true;
  }

  bool Gui2D::Display() {
    // Turn off the key
    this->gp_ << "set key off" << std::endl;

    // Plot all the data
    this->gp_ << "plot ";
    for(const CommandUnit cu: this->command_units_) {
      this->gp_ << cu.command_ << " ";
    }
    this->gp_ << std::endl;

    for(const CommandUnit cu: this->command_units_) {
      this->gp_.send1d(cu.data_);
    }

    return true;
  }
}
