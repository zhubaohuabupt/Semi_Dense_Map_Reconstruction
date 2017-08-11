
#ifndef FRAME_
#define FRAME_

#include <se3.h>
#include <vikit/math_utils.h>
#include <vikit/abstract_camera.h>
#include <boost/noncopyable.hpp>
#include <global.h>

namespace DepthEstimation {

class Point;
struct Feature;

typedef list<Feature*> Features;
typedef vector<cv::Mat> ImgPyr;

/// A frame saves the image, the associated features and the estimated pose.
class Frame : boost::noncopyable
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int                           id_;                    //!< Unique id of the frame.
  vk::AbstractCamera*           cam_;                   //!< Camera model.
  Sophus::SE3                   T_f_w_;                 //!< Transform (f)rame from (w)orld.
  ImgPyr                        img_pyr_;               //!< Image Pyramid.
  bool                          is_keyframe_;           //!< Was this frames selected as keyframe?

  Frame(vk::AbstractCamera* cam, const cv::Mat& img, int id_);
  ~Frame();

  /// Initialize new frame and create image pyramid.
  void initFrame(const cv::Mat& img);

  /// Creates an image pyramid of half-sampled images.
  void createImgPyramid(const cv::Mat& img_level_0, int n_levels, ImgPyr& pyr);

  /// Check if a point in (w)orld coordinate frame is visible in the image.
  bool isVisible(const Vector3d& xyz_w) const;

  /// Full resolution image stored in the frame.
  inline const cv::Mat& img() const { return img_pyr_[0]; }

  /// Was this frame selected as keyframe?
  inline bool isKeyframe() const { return is_keyframe_; }

  /// Transforms point coordinates in world-frame (w) to camera pixel coordinates (c).
  inline Vector2d w2c(const Vector3d& xyz_w) const { return cam_->world2cam( T_f_w_ * xyz_w ); }

  /// Transforms pixel coordinates (c) to frame unit sphere coordinates (f).
  inline Vector3d c2f(const Vector2d& px) const { return cam_->cam2world(px[0], px[1]); }

  /// Transforms pixel coordinates (c) to frame unit sphere coordinates (f).
  inline Vector3d c2f(const double x, const double y) const { return cam_->cam2world(x, y); }

  /// Transforms point coordinates in world-frame (w) to camera-frams (f).
  inline Vector3d w2f(const Vector3d& xyz_w) const { return T_f_w_ * xyz_w; }

  /// Transforms point from frame unit sphere (f) frame to world coordinate frame (w).
  inline Vector3d f2w(const Vector3d& f) const { return T_f_w_.inverse() * f; }

  /// Projects Point from unit sphere (f) in camera pixels (c).
  inline Vector2d f2c(const Vector3d& f) const { return cam_->world2cam( f ); }

  /// Return the pose of the frame in the (w)orld coordinate frame.
  inline Vector3d pos() const { return T_f_w_.inverse().translation(); }

};
} // namespace DepthEstimation

#endif
