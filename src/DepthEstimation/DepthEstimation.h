/*This file is part of AntennaDetect-SemiDenseReconstruction.
 * DepthEstimation.h
 *
 *  Created on: 2017年6月1日
 *      Author: Zuber
 */
#ifndef DEPTHESTIMATION_
#define DEPTHESTIMATION_

#include<vector>
#include <vikit/performance_monitor.h>
#include <global.h>
#include <matcher.h>
#include<opencv2/opencv.hpp>
#include"ComData.h"
#include <vikit/pinhole_camera.h>
#include<algorithm>
using namespace cv;
using namespace std;
namespace DepthEstimation {

class Frame;
class Feature;
class Point;
class Frame;
typedef boost::shared_ptr<Frame> FramePtr;
/// A seed is a probabilistic depth estimate for a single pixel.
struct Seed
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static int batch_counter;
  static int seed_counter;
  int batch_id;                //!< Batch id is the id of the keyframe for which the seed was created.
  int id;                      //!< Seed ID, only used for visualization.
  Feature* ftr;                //!< Feature in the keyframe for which the depth should be computed.
  float a;                     //!< a of Beta distribution: When high, probability of inlier is large.
  float b;                     //!< b of Beta distribution: When high, probability of outlier is large.
  float mu;                    //!< Mean of normal distribution.
  float z_range;               //!< Max range of the possible depth.
  float sigma2;                //!< Variance of normal distribution.
  bool IsValue;
  uchar lineID;
  Seed(Feature* ftr, float depth_mean);
  Seed(Feature* ftr, float depth_mean,uchar lineID);
  Seed(Feature* ftr, float depth_befor, float sigma2_befor,float a_befor,float b_befor);
};

class DepthEstimation
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 typedef unique_lock<mutex> lock_t;

  /// Depth-filter config parameters
  struct Options
  {
    bool check_ftr_angle;                       //!< gradient features are only updated if the epipolar line is orthogonal to the gradient.
    bool epi_search_1d;                         //!< restrict Gauss Newton in the epipolar search to the epipolar line.
    bool verbose;                               //!< display output.
    bool use_photometric_disparity_error;       //!< use photometric disparity error instead of 1px error in tau computation.
    int max_n_kfs;                              //!< maximum number of keyframes for which we maintain seeds.
    double sigma_i_sq;                          //!< image noise.
    double seed_convergence_sigma2_thresh;      //!< threshold on depth uncertainty for convergence.
    Options()
    : check_ftr_angle(false),
      epi_search_1d(false),
      verbose(false),
      use_photometric_disparity_error(false),
      max_n_kfs(3),
      sigma_i_sq(5e-4),
      seed_convergence_sigma2_thresh(200.0)
    {}
  } options_;

  DepthEstimation(ComData* pComData);
   virtual ~DepthEstimation();

  /// Compute the uncertainty of the measurement.
  static double computeTau(
      const SE3& T_ref_cur,
      const Vector3d& f,
      const double z,
      const double px_error_angle);
  ComData *pComData;
  vFrame mvFrame;

  vk::AbstractCamera* cam;
  virtual void CalDepth(cv::Mat &DEPTH_RESULT)=0;
  virtual void  testshow()=0;
protected:

   vk::PerformanceMonitor permon_;       //!< Separate performance monitor since the DepthEstimation runs in a parallel thread.
   Matcher matcher_;
   Sophus::SE3 Mat2SE3(const cv::Mat& T);
};
void GenerateFalseMap(cv::Mat &src, cv::Mat &disp);
} // namespace DepthEstimation

#endif
