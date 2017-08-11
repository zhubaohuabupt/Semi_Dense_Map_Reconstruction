// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef POINT_
#define POINT_

#include <boost/noncopyable.hpp>
#include <global.h>

namespace g2o {
  class VertexSBAPointXYZ;
}
typedef g2o::VertexSBAPointXYZ g2oPoint;

namespace DepthEstimation {

class Feature;

typedef Matrix<double, 2, 3> Matrix23d;

/// A 3D point on the surface of the scene.
class Point : boost::noncopyable
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  enum PointType {
    TYPE_DELETED,
    TYPE_CANDIDATE,
    TYPE_UNKNOWN,
    TYPE_GOOD
  };

  static int                  point_counter_;           //!< Counts the number of created points. Used to set the unique id.
  int                         id_;                      //!< Unique ID of the point.
  Vector3d                    pos_;                     //!< 3d pos of the point in the world coordinate frame.
  Vector3d                    normal_;                  //!< Surface normal at point.
  Matrix3d                    normal_information_;      //!< Inverse covariance matrix of normal estimation.
  bool                        normal_set_;              //!< Flag whether the surface normal was estimated or not.
 list<Feature*>              obs_;                     //!< References to keyframes which observe the point.
  PointType                   type_;                    //!< Quality of the point.

  Point(const Vector3d& pos);
  Point(const Vector3d& pos, Feature* ftr);
  ~Point();

  /// Add a reference to a frame.
 // void addFrameRef(Feature* ftr);

  /// Remove reference to a frame.
 // bool deleteFrameRef(Frame* frame);

  /// Initialize point normal. The inital estimate will point towards the frame.
  //void initNormal();

  /// Check whether mappoint has reference to a frame.
  //Feature* findFrameRef(Frame* frame);

  /// Get Frame with similar viewpoint.
  bool getCloseViewObs(const Vector3d& pos, Feature*& obs) const;

};

} // namespace DepthEstimation

#endif
