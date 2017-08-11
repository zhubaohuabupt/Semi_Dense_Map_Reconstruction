#include<iostream>
#include "ComData.h"
#include "runCamera2.h"
#include "AntennaDetectThread.h"
#include "PointCloudProcessingThread.h"
#include "VSLAMThread.h"
#include"ViewThread.h"
#include"DepthEstimationThread.h"
#include <unistd.h>
#include <X11/Xlib.h>
using namespace std;
void  setFPS( ComData*pComData);
//人嘉相机视差图转深度图
void DISP2DEPTH(const cv::Mat& dispimg,cv::Mat&depthimg ,float bf)
{
   int height=dispimg.rows;
   int width=dispimg.cols;
   for(int row=0;row<height;row++)
       for(int col=0;col<width;col++)
       {
           if(dispimg.at<uchar>(row,col)<128&&dispimg.at<uchar>(row,col)!=0)
            depthimg.at<float>(row,col)  = bf/(dispimg.at<uchar>(row,col)/4);

           else if(dispimg.at<uchar>(row,col)>=128)
           depthimg.at<float>(row,col) = bf/(dispimg.at<uchar>(row,col)/2-32);

           else
          depthimg.at<float>(row,col)=0;
       }
}
int main(int argc, char **argv)
{
      if(argc != 3)
          {
              cerr << endl << "Wrong input format!" << endl;
              return 1;
          }
      XInitThreads();
//读取配置参数
      string SettingPath = argv[1];
      string VocabularyPath=argv[2];
      cv::FileStorage settings(SettingPath, cv::FileStorage::READ);
      float fx= settings["Camera_fx"];
      float fy= settings["Camera_fy"];
      float cx= settings["Camera_cx"];
      float cy= settings["Camera_cy"];
      float bf = settings["Camera_bf"];
      float width = settings["Camera_width"];
      float height = settings["Camera_height"];
      CamPara  pCamPara(fx,fy,cx,cy,bf,height,width);

       cv::Mat leftimg(height,width,CV_8UC1),dispimg(height,width,CV_8UC1);
       cv::Mat depthimg(height,width,CV_32FC1);

/**********************************初始化并开启各个线程*******************************************/
        ComData* ComData_=new ComData() ;
        ComData_->setCamPara(pCamPara);
         //开启天线检测线程
        AntennaDetectThread * pAntennaDetectThread=new AntennaDetectThread(ComData_);
             pAntennaDetectThread->startThread();
        //开启点云去噪平滑线程
        PointCloudProcessingThread * pPointCloudProcessingThread=new PointCloudProcessingThread(ComData_);
             pPointCloudProcessingThread->startThread();
        //开启显示线程
       ViewThread* pViewThread=new  ViewThread(ComData_);
             pViewThread->startThread();
        //开启VSLAM线程
       VSLAMThread*pVSLAMThread=new  VSLAMThread(SettingPath,VocabularyPath,ComData_);
            pVSLAMThread->startThread();
          //开启深度估计线程
        DepthEstimationThread * pDepthEstimationThread=new DepthEstimationThread(ComData_);
              pDepthEstimationThread->startThread();
        //主线程
        while(1)
        {
            setFPS(ComData_);
            usleep(20000);
            //读图
            runCamera2(leftimg,dispimg);
            DISP2DEPTH(dispimg,depthimg,bf);
            //写抓图缓存
            ComData_->swapImgSourceCache(leftimg,depthimg,dispimg);
            //更新图源
           ComData_->UpdatepImgSource(leftimg,depthimg,dispimg);

        }
}

void  setFPS( ComData*pComData)
{
    static int cnt=0;
    static std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    if(cnt==10)
    {
           std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
           float FPS= 10/std::chrono::duration_cast<std::chrono::duration<double> >(end - begin).count();
           pComData->setFPS_main(FPS);
           begin=std::chrono::steady_clock::now();
           cnt=0;
    }
   ++cnt;
}
