#include<iostream>
#include "ComData.h"
#include "runCamera2.h"
#include "AntennaDetect.h"
#include "PointCloudProcessingThread.h"
#include "VSLAMThread.h"
#include"ViewThread.h"
#include"DepthEstimationThread.h"
#include"Config.h"
#include <unistd.h>
#include <X11/Xlib.h>
using namespace std;
void  setFPS( ComData*pComData);
int loadImg(cv::Mat&left,cv::Mat &depth,const string &IMGdataPath);
//人嘉相机视差图转深度图
void DISP2DEPTH(const cv::Mat& dispimg,cv::Mat&depthimg ,float bf)
{
   int height=dispimg.rows;
   int width=dispimg.cols;
   for(int row=0;row<height;row++)
       for(int col=0;col<width;col++)
       {
           if(dispimg.at<uchar>(row,col)!=0)
            depthimg.at<float>(row,col)  = bf/(dispimg.at<uchar>(row,col)/4);
           else
          depthimg.at<float>(row,col)=0;
       }
}
int main(int argc, char **argv)
{
      if(argc !=4 )
          {
              cerr << endl << "Wrong input format!" << endl;
              return 1;
          }
      XInitThreads();
//读取配置参数
      string SettingPath = argv[1];
      string VocabularyPath=argv[2];
      string IMGdataPath=argv[3];
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
        ComData_->setDatasetPath(IMGdataPath);
        ComData_->setCamPara(pCamPara);
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
 /**********************************定义天线检测*******************************************/
        Detect_Wireline detect;
        SystemPattern mSystemPattern=mOnlyDetect;
        //主线程
        while(1)
        {
            //读图
             if(loadImg(leftimg,dispimg ,IMGdataPath))
             {
               static int  ID_show=0;

                DISP2DEPTH(dispimg,depthimg,bf);
                //写抓图缓存
                ComData_->swapImgSourceCache(leftimg,depthimg,dispimg);
                //更新图源
               ComData_->UpdatepImgSource(leftimg,depthimg,dispimg);
               //检测天线
               std::vector<std::vector<cv::Point> >AntennaPoints;
               //天线检测
               detect.detect_depth(leftimg,depthimg,AntennaPoints);
               ComData_->setImageReadID(++ID_show);
                setFPS(ComData_);
               //写公共区天线检测缓存
               ComData_->setpDetectedAntennaCache(leftimg,depthimg,AntennaPoints);
                if(AntennaPoints.size()>0)//检测到天线
                        mSystemPattern=mDepthEstimation;
               ComData_->setSystemPattern(mSystemPattern);

//               //显示
//               cv::Mat AntennaMat=cv::Mat::zeros(leftimg.rows,leftimg.cols,CV_8UC1);
//               Points2Mat(AntennaPoints,AntennaMat);
//             imshow("",AntennaMat);  cv::waitKey(5);

            }
           else
             {
                 std::cerr<<"no image to read "<<std::endl;
             }
        }
}


int  loadImg(cv::Mat&left,cv::Mat &disp,const string &IMGdataPath)
{
                static int frame=1000;
                static int framdend=5000;
                if (frame <= framdend)
                {
                    //本地图片途径和图片后缀名
                    const string leftpath=IMGdataPath+"/01";
                    const string disppath=IMGdataPath+"/disp";
                    char name1[80];
                    char name2[80];
                    sprintf(name1,"/left_%d.png",frame);
                    sprintf(name2,"/disp_%d.png",frame);
                    const string leftname = leftpath + string(name1);
                    const string dispname = disppath + string(name2);
                    left = imread(leftname, 0);
                    disp = imread(dispname, 0);
                    frame++;
                    return 1;
                }

                else
                {
                    cerr<<"ERROR: load image "<<endl;
                    return 0 ;
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
           pComData->setFPS_AntennaDetection(FPS);
           begin=std::chrono::steady_clock::now();
           cnt=0;
    }
   ++cnt;
}
