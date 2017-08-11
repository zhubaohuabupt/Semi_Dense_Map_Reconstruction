/*This file is part of AntennaDetect-SemiDenseReconstruction.
 * PointCloudProcessingThread.cpp
 *
 *  Created on: 2017年5月24日
 *      Author: Zuber
 */
#include"PointCloudProcessingThread.h"
#include <unistd.h>
void PointCloudProcessingThread::run()
{
    cv::Mat leftimg;
    cv::Mat dispimg;
   float fx,fy,cx,cy,bf;
   int height,width;
   if( !pComData->getCamPara(fx,fy,cx,cy,bf,height,width))
     std::cerr<<"Camera Parameter has not init "<<endl;

    Point3D_Processing::cam cam_(bf, fx, cx, cy, width, height);

    Point3D_Processing::Point3D_Cluster method1(cam_,3, 30);
    Point3D_Processing::DepthConnectedcheck method2(cam_,50, 10);
    Point3D_Processing::Denoise3DPoint *denose = &method2;
    Point3D_Processing::BilateralInterPolation InterPolation(bf,100,100,10);

    //图源无图时，等待
           {
               ulock lock(pComData->mImgSourceMutex);
              if(pComData->pImgSource==NULL)
                    pComData->ImgSourcecond.wait(lock);
           }
       while(1)
       {

              //从公共区图源内存读图
          if(pComData->getpImgSourceLeftDisp(leftimg,dispimg))
            {
              //帧率计算单元
                setFPS();
               //去噪
               denose->Denose3DPoint( leftimg, dispimg);
              cv::Mat Denoise = denose->GetFalseColorDenoisedisp();
               cv::Mat noise = denose->GetFalseColorNoisedisp();
               cv::Mat Originaldisp = denose->GetFalseColorOriginaldisp();
              //平滑
               cv::Mat afterSmoothing = InterPolation(leftimg, denose->SemiDenseDisp,denose->DeNoiseDisp);
               //写公共区半稠密点云缓存
               pComData->setpSemiDenseDepth(Originaldisp,noise,Denoise,afterSmoothing);
            }
       }
 }

void PointCloudProcessingThread::startThread()
{
    pPointCloudProcessingThread=new std::thread (&PointCloudProcessingThread::run,this);
}
void PointCloudProcessingThread ::setFPS()
{
    static int cnt=0;
    static std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    if(cnt==10)
    {
           std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
           float FPS= 10/std::chrono::duration_cast<std::chrono::duration<double> >(end - begin).count();
           pComData->setFPS_PointCloudProcessing(FPS);
           begin=std::chrono::steady_clock::now();
           cnt=0;
    }
   ++cnt;
}
