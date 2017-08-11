
#include <stdexcept>
#include <frame.h>
#include <feature.h>
#include <point.h>
#include <config.h>
#include<opencv2/opencv.hpp>
using namespace cv;
namespace DepthEstimation {

Frame::Frame(vk::AbstractCamera* cam, const cv::Mat& img, int  id_ ) : id_(id_), cam_(cam), is_keyframe_(false)
{
  initFrame(img);
}

Frame::~Frame()
{
}

void Frame::initFrame(const cv::Mat& img)
{
  // check image
  if(img.empty() || img.type() != CV_8UC1 || img.cols != cam_->width() || img.rows != cam_->height())
    throw std::runtime_error("Frame: provided image has not the same size as the camera model or image is not grayscale");
  // Build Image Pyramid
  createImgPyramid(img, max(Config::nPyrLevels(), Config::kltMaxLevel()+1), img_pyr_);

}

bool Frame::isVisible(const Vector3d& xyz_w) const
{
  Vector3d xyz_f = T_f_w_*xyz_w;
  if(xyz_f.z() < 0.0)
    return false; // point is behind the camera
  Vector2d px = f2c(xyz_f);
  if(px[0] >= 0.0 && px[1] >= 0.0 && px[0] < cam_->width() && px[1] < cam_->height())
    return true;
  return false;
}

void Frame::createImgPyramid(const cv::Mat& img_level_0, int n_levels, ImgPyr& pyr)
    {
      pyr.resize(n_levels);
      pyr[0] = img_level_0;

      for(int i=1; i<n_levels; ++i)
      {
        pyr[i] = cv::Mat(pyr[i-1].rows/2, pyr[i-1].cols/2, CV_8U);
        cv::pyrDown(pyr[i-1],pyr[i]);
      }
    }
} // namespace DepthEstimation
