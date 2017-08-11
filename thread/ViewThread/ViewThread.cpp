/*This file is part of AntennaDetect-SemiDenseReconstruction.
 * ViewThread.cpp
 *
 *  Created on: 2017年5月25日
 *      Author: Zuber
 */
#include"ViewThread.h"
using namespace std;
void ViewThread::run()
{

int mImageWidth=752;
int mImageHeight=480;
 float windowHeight = 1000;
 float windowWidth = 1410;
 pangolin::CreateWindowAndBind("AntennaDetect-SemiDenseReconstruction",windowWidth,windowHeight);
 // 3D Mouse handler requires depth testing to be enabled
 glEnable(GL_DEPTH_TEST);
 // Issue specific OpenGl we might need
 glEnable (GL_BLEND);
 glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
 //pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
 //初始化显示图像大小
 float imageHeight = 300;
 float imageWidth = imageHeight/480*752;
 //初始化图例大小
  float panelHeight = 30;
 float panelWidth = imageWidth;
 //初始化显示条大小
 float consoleHeight = 200;
 float consoleWidth = imageWidth;



 //used to show console1
 pangolin::CreatePanel("console1").SetBounds(pangolin::Attach::Pix(windowHeight-imageHeight-panelHeight),0,pangolin::Attach::Pix(imageWidth+480/300*40),pangolin::Attach::Pix(consoleWidth+imageWidth));
 pangolin::Var<string> bSystemPattern("console1.SystemPattern","Only Detect");
 pangolin::Var<string> bVSLAMstate("console1.VSLAMstate","Idle");
  pangolin::Var<float> FPS_main("console1.FPS-ImgCaptureThread",0,100,true);
 pangolin::Var<float> FPS_AntennaDetect("console1.FPS-AntennaDetectThread",0,100,true);
 pangolin::Var<float> FPS_DepthEstimation("console1.FPS-DepthEstimationThread",0,100,true);
 pangolin::Var<float> FPS_PointCloudProcessing("console1.FPS-PointCloudProcessingThread",0,100,true);
 pangolin::Var<float> FPS_VSLAM("console1.FPS-VSLAMThread",0,100,true);


 pangolin::CreatePanel("1").SetBounds(pangolin::Attach::Pix(windowHeight-2*(imageHeight+panelHeight)),0,pangolin::Attach::Pix(imageWidth),pangolin::Attach::Pix(consoleWidth+imageWidth));
 //used to show console2
 pangolin::CreatePanel("console2").SetBounds(pangolin::Attach::Pix(windowHeight-2*(panelHeight+imageHeight)),0,0,pangolin::Attach::Pix(imageWidth));
 pangolin::Var<int > VSLAMCacheSize("console2.VSLAMCacheSize",0,100,true);
 pangolin::Var<int > FrameID("console2.FrameID",0,10000,true);
pangolin::Var<string> DataSetPath("console2.DataSetPath","");


 //显示左图
 pangolin::CreatePanel("leftImage_menu").SetBounds(1.0,pangolin::Attach::Pix(windowHeight-panelHeight),pangolin::Attach::Pix(panelWidth),pangolin::Attach::Pix(panelWidth+imageWidth));
 pangolin::Var<string>leftImage_bar("leftImage_menu. Left Image","");
 pangolin::View& left_image = pangolin::Display("left_image")
     .SetBounds(pangolin::Attach::Pix(windowHeight-panelHeight),pangolin::Attach::Pix(windowHeight-panelHeight-imageHeight),pangolin::Attach::Pix(panelWidth),pangolin::Attach::Pix(panelWidth+imageWidth),752.0/480.0)
     .SetLock(pangolin::LockLeft, pangolin::LockTop);
 pangolin::GlTexture imageTexture(mImageWidth,mImageHeight,GL_LUMINANCE,false,0,GL_LUMINANCE,GL_UNSIGNED_BYTE);


 //显示天线图
pangolin::CreatePanel("detectedAntenna_menu").SetBounds(1.0,pangolin::Attach::Pix(windowHeight-panelHeight),0.0,pangolin::Attach::Pix(panelWidth));
pangolin::Var<string>detectedAntenna_bar("detectedAntenna_menu. Antenna","");
 pangolin::View& detectedAntenna = pangolin::Display("detectedAntenna")
     .SetBounds(pangolin::Attach::Pix(windowHeight-panelHeight),pangolin::Attach::Pix(windowHeight-(panelHeight+imageHeight)),0,pangolin::Attach::Pix(imageWidth),752.0/480.0)
     .SetLock(pangolin::LockLeft, pangolin::LockTop);
 pangolin::GlTexture detectedAntennaTexture(mImageWidth,mImageHeight,GL_LUMINANCE,false,0,GL_LUMINANCE,GL_UNSIGNED_BYTE);

// 天线深度图
 pangolin::CreatePanel("AntennaDepth_menu").SetBounds(pangolin::Attach::Pix(windowHeight-(panelHeight+imageHeight)),pangolin::Attach::Pix(windowHeight-(panelHeight+imageHeight)-panelHeight),0.0,pangolin::Attach::Pix(panelWidth));
 pangolin::Var<string>AntennaDepth_bar("AntennaDepth_menu. Depth of Antenna ","");
 pangolin::View& AntennaDepth = pangolin::Display("AntennaDepth")
      .SetBounds(pangolin::Attach::Pix(windowHeight-(panelHeight+imageHeight)-panelHeight),pangolin::Attach::Pix(windowHeight-2*(panelHeight+imageHeight)),0,pangolin::Attach::Pix(imageWidth),752./480.0)
      .SetLock(pangolin::LockLeft, pangolin::LockTop);
  pangolin::GlTexture AntennaDepthTexture(mImageWidth,mImageHeight,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);

//彩条
  pangolin::View& colorbar = pangolin::Display("colorbar")
       .SetBounds(pangolin::Attach::Pix(windowHeight-(panelHeight+imageHeight)),pangolin::Attach::Pix(windowHeight-2*(panelHeight+imageHeight)),pangolin::Attach::Pix(imageWidth),pangolin::Attach::Pix(imageWidth+480/300*40),58./480.0)
       .SetLock(pangolin::LockLeft, pangolin::LockTop);
   pangolin::GlTexture colorbarTexture(40,mImageHeight,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);
   cv::Mat colorbar_=cv::imread("/home/baohua/BX/Config/bar.png");
   uchar pgcolorbar[3*40*480];
   int b = 0;
   for(int j=479;j>=0;j--)
       for(int i=0;i<40;i++,b++)
           for(int c=0,k=2;k>=0;k--,c++)
           {
           pgcolorbar[3*b+c] = colorbar_.at<cv::Vec3b>(j,i)[k];
         }
//含噪声的半稠密点云
  pangolin::CreatePanel("OriginSemiDenseDepth_menu").SetBounds(1.0,pangolin::Attach::Pix(windowHeight-panelHeight),pangolin::Attach::Pix(panelWidth+panelWidth),pangolin::Attach::Pix(panelWidth+panelWidth+panelWidth));
  pangolin::Var<string>OriginSemiDenseDepth_bar("OriginSemiDenseDepth_menu.OriginSemiDenseDepth ","");
  pangolin::View& OriginSemiDenseDepth = pangolin::Display("OriginSemiDenseDepth")
       .SetBounds(pangolin::Attach::Pix(windowHeight-panelHeight),pangolin::Attach::Pix(windowHeight-(panelHeight+imageHeight)),pangolin::Attach::Pix(panelWidth+panelWidth),pangolin::Attach::Pix(panelWidth+panelWidth+panelWidth),752./480.0)
       .SetLock(pangolin::LockLeft, pangolin::LockTop);
   pangolin::GlTexture OriginSemiDenseDepthTexture(mImageWidth,mImageHeight,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);

//噪声检测图
   pangolin::CreatePanel("NoisedSemiDenseDepth_menu").SetBounds(pangolin::Attach::Pix(windowHeight-(panelHeight+imageHeight)),pangolin::Attach::Pix(windowHeight-(panelHeight+imageHeight)-panelHeight),pangolin::Attach::Pix(panelWidth+panelWidth),pangolin::Attach::Pix(panelWidth+panelWidth+panelWidth));
   pangolin::Var<string>NoisedSemiDenseDepth_bar("NoisedSemiDenseDepth_menu.Noise ","");
   pangolin::View& NoisedSemiDenseDepth = pangolin::Display("NoisedSemiDenseDepth")
        .SetBounds(pangolin::Attach::Pix(windowHeight-(panelHeight+imageHeight)-panelHeight),pangolin::Attach::Pix(windowHeight-2*(panelHeight+imageHeight)),pangolin::Attach::Pix(panelWidth+panelWidth),pangolin::Attach::Pix(panelWidth+panelWidth+panelWidth),752./480.0)
        .SetLock(pangolin::LockLeft, pangolin::LockTop);
    pangolin::GlTexture NoisedSemiDenseDepthTexture(mImageWidth,mImageHeight,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);

//去噪平滑后的半稠密点云
    pangolin::CreatePanel("SmoothedSemiDenseDepth_menu").SetBounds(pangolin::Attach::Pix(windowHeight-2*(panelHeight+imageHeight)),pangolin::Attach::Pix(windowHeight-2*(panelHeight+imageHeight)-panelHeight),pangolin::Attach::Pix(panelWidth+panelWidth),pangolin::Attach::Pix(panelWidth+panelWidth+panelWidth));
    pangolin::Var<string>SmoothedSemiDenseDepth_bar("SmoothedSemiDenseDepth_menu.AfterProcessing ","");
    pangolin::View& SmoothedSemiDenseDepth = pangolin::Display("SmoothedSemiDenseDepth")
         .SetBounds(pangolin::Attach::Pix(windowHeight-2*(panelHeight+imageHeight)-panelHeight),pangolin::Attach::Pix(windowHeight-3*(panelHeight+imageHeight)),pangolin::Attach::Pix(panelWidth+panelWidth),pangolin::Attach::Pix(panelWidth+panelWidth+panelWidth),752./480.0)
         .SetLock(pangolin::LockLeft, pangolin::LockTop);
     pangolin::GlTexture SmoothedSemiDenseDepthTexture(mImageWidth,mImageHeight,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);

     cv::Mat left;
     std::vector<std::vector<cv::Point>> AntennaPoints;
     cv::Mat depthofAntenna;
     cv::Mat OriginSemiDenseDepthIMG;
     cv::Mat NoisedSemiDenseDepthIMG;
     cv::Mat DenosedSemiDenseDepthIMG;
     cv::Mat SmoothedSemiDenseDepthIMG;
          //图源无图时，等待
           {
               ulock lock(pComData->mImgSourceMutex);
              if(pComData->pImgSource==NULL)
                    pComData->ImgSourcecond.wait(lock);
           }
 uchar pgleft[752*480];
 uchar pgAntenna[752*480];
 uchar pgAntennaDepth[3*752*480];
 uchar pgOriginSemiDenseDepth[3*752*480];
 uchar pgNoisedSemiDenseDepth[3*752*480];
 uchar pgSmoothedSemiDenseDepth[3*752*480];
 SystemPattern mSystemPattern;
 VSLAMstate mVSLAMstate;
   while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        //显示系统状态
        pComData->getSystemPattern(mSystemPattern);
        if(mSystemPattern==0)
            bSystemPattern=string("Only Detect");
        else
            bSystemPattern=string("Depth Estimation");

        //显示VSLAM状态
     pComData->getVSLAMstate(mVSLAMstate);
         if(mVSLAMstate==0)
             bVSLAMstate=string("Idle");
         else if(mVSLAMstate==1)
             bVSLAMstate=string("Tracking");
         else
             bVSLAMstate=string("Lost");
       //显示各个线程帧率
        FPS_AntennaDetect=pComData->getFPS_AntennaDetection();
        FPS_DepthEstimation=pComData->getFPS_DepthEstimation();
        FPS_PointCloudProcessing =pComData->getFPS_PointCloudProcessing();
        FPS_VSLAM=pComData->getFPS_VSLAM();
        FPS_main=pComData->getFPS_main();
        VSLAMCacheSize=pComData->getmdpVSLAMCacheSize();
        FrameID =pComData->getImageID();
        DataSetPath=pComData->getDatasetPath();

         leftImage_bar=string("");
         detectedAntenna_bar=string("");
         AntennaDepth_bar=string("");
         OriginSemiDenseDepth_bar=string("");
         NoisedSemiDenseDepth_bar=string("");
         SmoothedSemiDenseDepth_bar=string("");
         /**＊**＊*＊*＊*＊**＊*＊*＊*＊**＊**＊*＊**＊**＊**＊**＊**＊*＊*＊**＊*更新显示数据**********************************************************/
                  //左图
             if(pComData->getpImgSourceLeft(left))
                {
                    int a = 0;
                    for(int j=479;j>=0;j--)
                        for(int i=0;i<752;i++,a++)
                        {
                            pgleft[a] = left.at<uchar>(j,i);
                        }

                 }
               //天线
                  if(pComData->getpDetectedAntennaCacheA(AntennaPoints))
                  {
                        cv::Mat Antenna=cv::Mat::zeros(pComData->pCamPara->height,pComData->pCamPara->width,CV_8UC1);
                        Points2Mat(AntennaPoints,Antenna);
                        //膨胀
                      cv::  Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(3,3),cv:: Point( 1, 1 ));
                      cv::dilate(Antenna , Antenna ,   element, cv::Point(-1,-1), 1, cv::BORDER_CONSTANT );
                         int a = 0;
                         for(int j=479;j>=0;j--)
                             for(int i=0;i<752;i++,a++)
                             {
                                 pgAntenna[a] = Antenna.at<uchar>(j,i);
                             }
                  }
                   //天线深度
                       if(pComData->getpDepthofAntenna(depthofAntenna))
                       {

                           cv::Mat ucharshow,color;
                           FLOAT2UCHAR(depthofAntenna,ucharshow);
                                GenerateFalseMap(ucharshow,color);

                               //膨胀
                             cv::  Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(3,3),cv:: Point( 1, 1 ));
                             cv::dilate(color , color ,   element, cv::Point(-1,-1), 1, cv::BORDER_CONSTANT );
                              int b = 0;
                              for(int j=479;j>=0;j--)
                                  for(int i=0;i<752;i++,b++)
                                      for(int c=0,k=2;k>=0;k--,c++)
                                      {
                                          pgAntennaDepth[3*b+c] = color.at<cv::Vec3b>(j,i)[k];
                                      }
                       }
                      if(pComData->getpSemiDenseDepth(OriginSemiDenseDepthIMG,NoisedSemiDenseDepthIMG,DenosedSemiDenseDepthIMG,  SmoothedSemiDenseDepthIMG))
                      {

                    //含噪声的半稠密点云
                          int b = 0;
                          for(int j=479;j>=0;j--)
                              for(int i=0;i<752;i++,b++)
                                  for(int c=0,k=2;k>=0;k--,c++)
                                  {
                                  pgOriginSemiDenseDepth[3*b+c] = OriginSemiDenseDepthIMG.at<cv::Vec3b>(j,i)[k];
                                }
                    //噪声检测图
                           b = 0;
                          for(int j=479;j>=0;j--)
                              for(int i=0;i<752;i++,b++)
                                  for(int c=0,k=2;k>=0;k--,c++)
                                  {
                                  pgNoisedSemiDenseDepth[3*b+c] = NoisedSemiDenseDepthIMG.at<cv::Vec3b>(j,i)[k];
                                }

                     //去噪平滑后的半稠密点云
                          b = 0;
                         for(int j=479;j>=0;j--)
                             for(int i=0;i<752;i++,b++)
                                 for(int c=0,k=2;k>=0;k--,c++)
                                 {
                                 pgSmoothedSemiDenseDepth[3*b+c] = SmoothedSemiDenseDepthIMG.at<cv::Vec3b>(j,i)[k];
                               }
                      }

  /**＊**＊*＊*＊*＊**＊*＊*＊*＊**＊**＊*＊**＊**＊**＊**＊**＊*＊*＊**＊* 上传显示数据**********************************************************/

                      //左图
                      imageTexture.Upload(pgleft,GL_LUMINANCE,GL_UNSIGNED_BYTE);
                      left_image.Activate();
                      glColor3f(1.0,1.0,1.0);
                      imageTexture.RenderToViewport();

                      //天线
                      detectedAntennaTexture.Upload(pgAntenna,GL_LUMINANCE,GL_UNSIGNED_BYTE);
                      detectedAntenna.Activate();
                      glColor3f(1.0,1.0,1.0);
                      detectedAntennaTexture.RenderToViewport();

                      //天线深度
                      AntennaDepthTexture.Upload(pgAntennaDepth,GL_RGB,GL_UNSIGNED_BYTE);
                       AntennaDepth.Activate();
                       glColor3f(1.0,1.0,1.0);
                       AntennaDepthTexture.RenderToViewport();


                       //彩条
                       colorbarTexture.Upload(pgcolorbar,GL_RGB,GL_UNSIGNED_BYTE);
                        colorbar.Activate();
                        glColor3f(1.0,1.0,1.0);
                        colorbarTexture.RenderToViewport();
                       //含噪声的半稠密点云
                       OriginSemiDenseDepthTexture.Upload(pgOriginSemiDenseDepth,GL_RGB,GL_UNSIGNED_BYTE);
                       OriginSemiDenseDepth.Activate();
                       glColor3f(1.0,1.0,1.0);
                       OriginSemiDenseDepthTexture.RenderToViewport();
                      //噪声检测图
                       NoisedSemiDenseDepthTexture.Upload(pgNoisedSemiDenseDepth,GL_RGB,GL_UNSIGNED_BYTE);
                       NoisedSemiDenseDepth.Activate();
                       glColor3f(1.0,1.0,1.0);
                       NoisedSemiDenseDepthTexture.RenderToViewport();
                      //去噪平滑后的半稠密点云
                       SmoothedSemiDenseDepthTexture.Upload(pgSmoothedSemiDenseDepth,GL_RGB,GL_UNSIGNED_BYTE);
                       SmoothedSemiDenseDepth.Activate();
                       glColor3f(1.0,1.0,1.0);
                      SmoothedSemiDenseDepthTexture.RenderToViewport();

                       pangolin::FinishFrame();

      }
}
 void ViewThread::startThread()
 {
     pViewThread=new std::thread(&ViewThread::run,this);
 }

 void ViewThread::FLOAT2UCHAR(const cv::Mat&Depthfloat,cv::Mat &Depthuchar)
 {

    Depthuchar=cv::Mat::zeros(Depthfloat.size(),CV_8UC1);
    int height=Depthfloat.rows;
    int width=Depthfloat.cols;
    for(int row=0;row<height;row++)
        for(int col=0;col<width;col++)
        {
            if(Depthfloat.at<float>(row,col)!=0)
            {
            Depthuchar.at<uchar>(row,col)=10*Depthfloat.at<float>(row,col);
            }
         }

 }
 void ViewThread::GenerateFalseMap(cv::Mat &src, cv::Mat &disp)
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
