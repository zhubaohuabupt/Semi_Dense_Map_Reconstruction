/*This file is part of AntennaDetect-SemiDenseReconstruction.
 * DepthEstimationThread.cpp
 *
 *  Created on: 2017年6月2日
 *      Author: Zuber
 */
#include <frame.h>
#include <point.h>
#include <feature.h>
#include <matcher.h>
#include"DepthEstimationThread.h"
void GenerateFalseMap(cv::Mat &src, cv::Mat &disp)  ;

DepthEstimationThread::DepthEstimationThread(ComData*ComData_):pComData(ComData_){}

void DepthEstimationThread::run()
{


        while(1)
        {
                {
                   ulock lock(pComData->mdpVSLAMCacheMutex);
                     if(pComData->mdpVSLAMCache.size()==0)
                          pComData->startDepthEstimationcond.wait(lock);
                     cout<<"start DepthEsttimation ."<<endl;
                }
            DepthEstimation::DynamicEstimation *pDynamicEstimation=new DepthEstimation::DynamicEstimation(pComData);
            DepthEstimation::StaticEstimation *pStaticEstimation=new DepthEstimation::StaticEstimation(pComData);
            DepthEstimation::DepthEstimation *pDepthEstimation=pStaticEstimation;
                    SystemPattern mSystemPattern=mDepthEstimation;
                    VSLAMstate mVSLAMstate=sIdle;
                                    while(1)
                                    {
                                             pComData->getSystemPattern(mSystemPattern);
                                             pComData->getVSLAMstate(mVSLAMstate);
                                                if (mSystemPattern==mOnlyDetect||mVSLAMstate==sLost)   break;
                                             if(pComData->getmdpVSLAMCache(pDepthEstimation->mvFrame))
                                             {
                                                 cv::Mat DetectedAntenna;
                                                 //帧率计算单元
                                                 setFPS( );
                                                 //深度估计
                                                 pDepthEstimation->CalDepth(DetectedAntenna);    
                                              //   pDepthEstimation->testshow();
                                                 pComData->setpDepthofAntenna(DetectedAntenna);
                                            }
                                   }
                                pComData->setSystemPattern(SystemPattern(mOnlyDetect));
                                   pComData->setFPS_DepthEstimation(0);
                                    delete pStaticEstimation;
                                    delete pDynamicEstimation;
        }
    }


void DepthEstimationThread::startThread()
{
  pDepthEstimationThread = new std::thread(&DepthEstimationThread::run, this);
}

SE3 DepthEstimationThread::Mat2SE3(const cv::Mat& T)
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
void DepthEstimationThread ::setFPS( )
{
    static int cnt=0;
    static std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    if(cnt==10)
    {
           std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
           float FPS= 10/std::chrono::duration_cast<std::chrono::duration<double> >(end - begin).count();
           pComData->setFPS_DepthEstimation(FPS);
           begin=std::chrono::steady_clock::now();
           cnt=0;
    }
   ++cnt;
}
