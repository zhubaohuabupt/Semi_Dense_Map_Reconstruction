/*This file is part of AntennaDetect-SemiDenseReconstruction.
 * AntennaDetectThread.cpp
 *
 *  Created on: 2017年5月24日
 *      Author: Zuber
 */
#include"AntennaDetectThread.h"
#include"Config.h"
#include <unistd.h>
void AntennaDetectThread::run()
{
   cv::Mat leftimg;
   cv::Mat depthimg;
   SystemPattern mSystemPattern=mOnlyDetect;
   VSLAMstate mVSLAMstate=sIdle;
   Detect_Wireline detect;

   //图源无图时，等待
  {
      ulock lock(pComData->mImgSourceMutex);
     if(pComData->pImgSource==NULL)
           pComData->ImgSourcecond.wait(lock);
  }
   while(1)
   {

         //从公共区图源内存读图
    if( pComData->getpImgSourceLeftDepth(leftimg,depthimg))
       {

           std::vector<std::vector<cv::Point> >AntennaPoints;
           //帧率计算单元
           setFPS( );
           //天线检测
           detect.detect_depth(leftimg,depthimg,AntennaPoints);
           //写公共区天线检测缓存
           pComData->setpDetectedAntennaCache(leftimg,depthimg,AntennaPoints);

            pComData->getVSLAMstate(mVSLAMstate);
            if (mVSLAMstate!=sTracking)
                  mSystemPattern=mOnlyDetect;
            if(AntennaPoints.size()>0)//检测到天线
                    mSystemPattern=mDepthEstimation;
           pComData->setSystemPattern(mSystemPattern);
       }
   }
}
void AntennaDetectThread::startThread()
{
    pAntennaDetectThread=new thread(&AntennaDetectThread::run,this);
}
void AntennaDetectThread::setFPS( )
{
    static int cnt=0;
    static std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    if(cnt==10)
    {
           std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
           float FPS= 10/std::chrono::duration_cast<std::chrono::duration<double> >(end - begin).count();
           pComData->setFPS_AntennaDetection(FPS);
           begin=std::chrono::steady_clock::now();
           cnt=0;
    }
   ++cnt;
}
