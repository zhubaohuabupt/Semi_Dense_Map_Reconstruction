/*This file is part of AntennaDetect-SemiDenseReconstruction.
 * DynamicEstimation.h
 *
 *  Created on: 2017年6月1日
 *      Author: Zuber
 */
#ifndef DYNAMICESTIMATION_
#define DYNAMICESTIMATION_
#include"DepthEstimation.h"
using namespace cv;
using namespace std;
namespace DepthEstimation {
class converge_point
{
public:
 Feature * ftr;
 Point  * point3d;
converge_point(Feature* ftr_,Point*  point3d_):ftr(ftr_),point3d(point3d_){}
};

class kfconvergmsg
{
public:
  int KeypointN;
  float ConvergRatio;
  vector<converge_point*> Kfconvergpoints;
   std::list<Seed* > seeds_inthisKF;
kfconvergmsg():KeypointN(0),ConvergRatio(0){}

};
class DynamicEstimation:public  DepthEstimation
{
public:
  DynamicEstimation(ComData* pComData);
   virtual ~DynamicEstimation();
    virtual void CalDepth( cv::Mat &DEPTH_RESULT);
    virtual void testshow();
protected:
          static int ALLConvergePointNum;
           list<FramePtr> pKeyFrames;
          std::list<Seed, aligned_allocator<Seed> > seeds_;
          map<int,kfconvergmsg> mapconvergePoints;//按帧存储收敛点
          FramePtr lastKF;
         /// Initialize new seeds from a frame.
         void initializeSeeds(FramePtr frame,float newKeyframeMEANdepth);
         void initializeSeedstest(FramePtr frame);
         /// Bayes update of the seed, x is the measurement, tau2 the measurement uncertainty
         static void updateSeed(
             const float x,
             const float tau2,
             Seed* seed);
         static void updateSeedkalman(
             const float x,
             const float tau2,
             Seed* seed);
         /// Return a reference to the seeds. This is NOT THREAD SAFE!
         bool propragatedepth( FramePtr frame,const vector<cv::Point>&newdetect,FramePtr lastKF,std::list<Seed, aligned_allocator<Seed> > &allseeds,
                                                   float newKeyframeMINdepth,float newKeyframeMEANdepth);

};

} // namespace DepthEstimation

#endif
