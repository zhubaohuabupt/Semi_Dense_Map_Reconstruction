/*This file is part of AntennaDetect-SemiDenseReconstruction.
 * DepthEstimation.cpp
 *
 *  Created on: 2017年6月1日
 *      Author: Zuber
 */
#include<iostream>
#include <algorithm>
#include <vikit/math_utils.h>
#include <vikit/abstract_camera.h>
#include <vikit/vision.h>
#include <global.h>
#include <DepthEstimation.h>
#include <frame.h>
#include <point.h>
#include <feature.h>
#include <matcher.h>
#include <config.h>
#include <stdio.h>        // for printf()
#include <sys/time.h>    // for gettimeofday()
#include <unistd.h>        // for sleep()

namespace DepthEstimation {

int Seed::batch_counter = 0;
int Seed::seed_counter = 0;

Seed::Seed(Feature* ftr, float depth_mean) :
    batch_id(batch_counter),
    id(seed_counter++),
    ftr(ftr),
    a(10),
    b(10),
    mu(depth_mean),
    z_range(7),//////
    sigma2(z_range*z_range/36),
    IsValue(false)
{}
Seed::Seed(Feature* ftr, float depth_mean,uchar lineID_):
      batch_id(batch_counter),
      id(seed_counter++),
      ftr(ftr),
      a(10),
      b(10),
      mu(depth_mean),
      z_range(7),//////
      sigma2(z_range*z_range/36),
      IsValue(false),
      lineID(lineID_)
  {}
 Seed:: Seed(Feature* ftr, float depth_befor, float sigma2_befor,float a_befor,float b_befor):
      batch_id(batch_counter),
      id(seed_counter++),
      ftr(ftr),
      a(a_befor),
      b(b_befor),
      mu(depth_befor),
      z_range(5),////
      sigma2(sigma2_befor),
      IsValue(false)
  {}
DepthEstimation::DepthEstimation(ComData*ComData_): pComData(ComData_)
{
    float fx, fy,cx, cy,bf;
    int width, height;
    if(!pComData->getCamPara(fx,fy,cx,cy,bf,height,width))
    {
        cerr<<"Cam Para is not Init  In  DepthEstimation"<<endl;
    }
    cam=new vk::PinholeCamera(width, height,fx, fy,cx, cy, 0,0,0,0,0);
}

DepthEstimation::~DepthEstimation()
{
}

double DepthEstimation::computeTau(
      const SE3& T_ref_cur,
      const Vector3d& f,
      const double z,
      const double px_error_angle)
{
  Vector3d t(T_ref_cur.translation());
  Vector3d a = f*z-t;
  double t_norm = t.norm();
  double a_norm = a.norm();
  double alpha = acos(f.dot(t)/t_norm); // dot product
  double beta = acos(a.dot(-t)/(t_norm*a_norm)); // dot product
  double beta_plus = beta + px_error_angle;
  double gamma_plus = PI-alpha-beta_plus; // triangle angles sum to PI
  double z_plus = t_norm*sin(beta_plus)/sin(gamma_plus); // law of sines
  return (z_plus - z); // tau
}



  SE3 DepthEstimation::Mat2SE3(const cv::Mat& T)
  {
  Matrix3d rotation;
  rotation<<T.at<float>(0,0),T.at<float>(0,1),T.at<float>(0,2),
            T.at<float>(1,0),T.at<float>(1,1),T.at<float>(1,2),
           T.at<float>(2,0),T.at<float>(2,1),T.at<float>(2,2) ;

  Vector3d translation(3);
  translation<<T.at<float>(0,3),T.at<float>(1,3),T.at<float>(2,3);

  //转变回SE3
  return SE3(rotation,translation);

  }

  void GenerateFalseMap(cv::Mat &src, cv::Mat &disp)
  {
      disp=cv::Mat::zeros(src.size(),CV_8UC3);
      // color map
      float max_val = 255.0f;
      float map[8][4] = {{0,0,0,114},{0,0,1,185},{1,0,0,114},{1,0,1,174},
                         {0,1,0,114},{0,1,1,185},{1,1,0,114},{1,1,1,0}};
      float sum = 0;
      for (int i=0; i<8; i++)
        sum += map[i][3];

      float weights[8]; // relative   weights
      float cumsum[8];  // cumulative weights
      cumsum[0] = 0;
      for (int i=0; i<7; i++) {
        weights[i]  = sum/map[i][3];
        cumsum[i+1] = cumsum[i] + map[i][3]/sum;
      }

      int height_ = src.rows;
      int width_ = src.cols;
      // for all pixels do
      for (int v=0; v<height_; v++) {
        for (int u=0; u<width_; u++) {

          // get normalized value
          float val = std::min(std::max(src.data[v*width_ + u]/max_val,0.0f),1.0f);

          // find bin
          int i;
          for (i=0; i<7; i++)
            if (val<cumsum[i+1])
              break;

          // compute red/green/blue values
          float   w = 1.0-(val-cumsum[i])*weights[i];
          uchar r = (uchar)((w*map[i][0]+(1.0-w)*map[i+1][0]) * 255.0);
          uchar g = (uchar)((w*map[i][1]+(1.0-w)*map[i+1][1]) * 255.0);
          uchar b = (uchar)((w*map[i][2]+(1.0-w)*map[i+1][2]) * 255.0);
          //rgb内存连续存放
          disp.data[v*width_*3 + 3*u + 0] = b;
          disp.data[v*width_*3 + 3*u + 1] = g;
          disp.data[v*width_*3 + 3*u + 2] = r;
        }
      }
  }
}
