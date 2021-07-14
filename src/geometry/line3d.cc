#include "line3d.h"

namespace game_engine {
  const Point3D& Line3D::Start() const {
    return this->start_;
  }

  const Point3D& Line3D::End() const {
    return this->end_;
  }

  Point3D Line3D::AsVector() const {
    return this->end_ - this->start_;
  }

  Point3D Line3D::ClosestBoundedPoint(const Point3D& point) const {
    Eigen::Vector3d l = this->AsVector();
    Eigen::Vector3d d = point - this->Start();
    // alpha is the distance along l of the projection of d
    double alpha = l.dot(d)/l.norm();

    // bound to the endpoints
    if (alpha>l.norm())
      return this->End();
    else if (alpha<0)
      return this->Start();

    return alpha*l.normalized()+this->Start();


  }
}
