/*This file is part of AntennaDetect-SemiDenseReconstruction.
 * ComData.h
 *
 *  Created on: 2017年5月22日
 *      Author: Zuber
 */
#ifndef  COMDATA
#define COMDATA
#include<map>
#include<deque>
#include<vector>
#include<string>
#include<highgui.h>
#include<cv.h>
#include<mutex>
#include<condition_variable>
#include<thread>

typedef std::unique_lock< std::mutex> ulock;
//相机参数
class CamPara
{
public:
    CamPara(float fx_,float fy_,float cx_,float cy_,float bf_,int height_,int width_);
 const   float fx;
 const   float fy;
 const   float cx;
 const   float cy;
 const   float bf;
 const int height;
 const  int width;

};

//此数据结构
//生产:VSLAM线程
//消费:深度估计线程
class vFrame
{
    public:
    vFrame( int FrameID_,float* Scenemeandepth_,bool IsKF_,const cv::Mat& left_,const cv::Mat& depth_,const cv::Mat& TCW_,const std::vector<std::vector<cv::Point>>& DetectedAntenna_);
     vFrame();
    int FrameID;//生产于VSLAM线程
    float SceneMEANdepth ;//场景平均深度
    float SceneMINdepth ;//场景最小深度
    bool IsKF;
    cv::Mat left;
    cv::Mat depth;
    cv::Mat TCW;
    std::vector<std::vector<cv::Point>> DetectedAntenna;
};

//系统运行模式
enum SystemPattern{ mOnlyDetect=0,mDepthEstimation=1};
//VSLAM状态
enum VSLAMstate { sIdle=0,sTracking=1,sLost=2 };


//图源存储
class  ImgSource
{
public:
    ImgSource( const cv::Mat&left_,const cv::Mat& depth_,const cv::Mat& disp);
     cv::Mat left;
     cv::Mat depth;
      cv::Mat disp;
};
//半稠密点云存储
class SemiDenseDepthProcessed
{
public:
    SemiDenseDepthProcessed(const cv::Mat&OriginSemiDenseDepth_,const cv::Mat&NoisedSemiDenseDepth,const cv::Mat&DenosedSemiDenseDepth,const cv::Mat& SmoothedSemiDenseDepth);
      cv::Mat OriginSemiDenseDepth;
      cv::Mat NoisedSemiDenseDepth;
      cv::Mat DenosedSemiDenseDepth;
      cv::Mat SmoothedSemiDenseDepth;
};

//天线存储
class DetectedAntennaCache
{
public:
    DetectedAntennaCache(  const cv::Mat& left_, const cv::Mat&depth_, const std::vector<std::vector<cv::Point>>& DetectedAntenna_);
     cv::Mat left;
     cv::Mat depth;
     std::vector<std::vector<cv::Point>> DetectedAntenna;
};

class ComData
{
public:

    ComData();
  /**************************************************数据区*******************************************/
  //相机参数    1
const CamPara *  pCamPara ;

//VSLAM缓存，将用于深度估计的数据   2
std::deque<vFrame*> mdpVSLAMCache;


//半稠密点云  生产于去噪平滑线程   3
SemiDenseDepthProcessed *pSemiDenseDepth;

//系统运行模式        4
SystemPattern mSystemPattern;

//VSLAM状态           5
VSLAMstate  mVSLAMstate;

//抓图缓存              6
ImgSource* Cache1;
ImgSource* Cache2;
ImgSource* pImgSource;


//天线检测缓存,用于VSLAM            7
DetectedAntennaCache* pDetectedAntennaCache;

//天线深度求取结果  8
cv::Mat mDepthofAntenna;


//天线检测线程实时帧率
float FPS_AntennaDetection;

//深度估计线程实时帧率
float FPS_DepthEstimation;

//点云处理线程实时帧率
float FPS_PointCloudProcessing;

//VSLAM线程实时帧率
float FPS_VSLAM;

//抓图线程帧率
float FPS_main;
int ID;
std::string DatasetPath;
  /**************************************************交互区*******************************************/
//互斥锁
std::mutex pCamParaMutex;//2
std::mutex mdpVSLAMCacheMutex;//2
std::mutex pSemiDenseDepthMutex;//3
std::mutex SystemPatternMutex;//4
std::mutex mVSLAMstateMutex;//5
std::mutex mDepthofAntennaMutex;//8

std::mutex mFPS_AntennaDetectionMutex;
std::mutex mFPS_DepthEstimationMutex;
std::mutex mFPS_PointCloudProcessingMutex;
std::mutex mFPS_VSLAMMutex;
std::mutex mFPS_mainMutex;
std::mutex ID_Mutex;
std::mutex DatasetPath_Mutex;
//条件变量

std::condition_variable startVOcond;//系统模式切换为深度估计时,通知VSLAM线程开始工作
std::condition_variable ImgSourcecond;//图源有图时，唤醒其他线程
std::condition_variable startDepthEstimationcond;//VSLAM开始后，唤醒深度估计线程
std::condition_variable mDetectedAntennaCacheCond;//天线检测
//读写相机参数1
int setCamPara(const CamPara&) ;
int getCamPara(float&,float&,float&,float&,float&,int&,int&) ;//

//读写天线检测线程实时帧率
void setFPS_AntennaDetection(float FPS_AntennaDetection);
float getFPS_AntennaDetection();

//读写深度估计线程实时帧率
void  setFPS_DepthEstimation(float FPS_DepthEstimation);
float getFPS_DepthEstimation();

//读写点云处理线程实时帧率
void  setFPS_PointCloudProcessing(float FPS_PointCloudProcessing);
float getFPS_PointCloudProcessing();

//读写VSLAM线程实时帧率
void  setFPS_VSLAM(float FPS_VSLAM);
float getFPS_VSLAM();

//读写主(抓图)线程实时帧率
void setFPS_main(float FPS_main);
float getFPS_main();

//读写图像ID
void setImageReadID(int ID);
int getImageID();

//读写图集路径
void setDatasetPath(const std::string&DatasetPath);
std::string getDatasetPath();

//读写VSLAMstate5
int setVSLAMstate(const VSLAMstate&);
int getVSLAMstate(VSLAMstate&);

//读写系统运行模式4
int setSystemPattern(const SystemPattern&);
int getSystemPattern( SystemPattern&);



//生产:去噪平滑线程
//消费:显示线程
//读写去噪后的半稠密深度图3
int setpSemiDenseDepth(const cv::Mat&,const cv::Mat&,const cv::Mat&,const cv::Mat&);
//返回值:0空,1非空
int getpSemiDenseDepth(  cv::Mat&, cv::Mat&, cv::Mat&, cv::Mat&);



/**************************图源***＊*＊*＊*＊*＊*＊*＊*＊*＊*＊*/
//生产:抓图主线程
//消费:天线检测，去噪平滑线程，显示线程

//一写多读内存访问设计
//读者计数器
static int mImgSourceReaderCounter;
//读者计数锁
std::mutex mImgSourceReaderCounterMutex;
//一级(读写权限)锁，保证读写内存顺序－－时间优先
std::mutex mImgSourceRWMutex;
//二级(内存)锁
std::mutex mImgSourceMutex;

//交换更新读图缓存
void swapImgSourceCache(const cv::Mat&left,const cv::Mat&depth,const cv::Mat&disp);

//更新图源地址　　//返回值:0空,1非空
int UpdatepImgSource(const cv::Mat&left,const cv::Mat&depth,const cv::Mat&disp);
//从图源读图
int getpImgSourceLeftDepth( cv::Mat&left, cv::Mat&depth);
int getpImgSourceLeftDisp( cv::Mat&left, cv::Mat&disp);
int getpImgSourceLeft( cv::Mat&left_);

/**************************图源***＊*＊*＊*＊*＊*＊*＊*＊**＊*＊*/



/**************************天线检测缓存***＊*＊*＊*＊*＊*＊*＊*＊**＊*＊*/
//生产:天线检测线程
//消费:VSLAM线程，显示线程

//一写多读内存访问设计
//读者计数器
static int mDetectedAntennaCacheReaderCounter;
//读者计数器锁
std::mutex  mDetectedAntennaCacheReaderCounterMutex;
//一级(读写权限)锁，保证读写内存顺序－－时间优先
std::mutex pDetectedAntennaCacheRWMutex;//
//二级(内存)锁
std::mutex pDetectedAntennaCacheMutex;//7

//读写天线检测缓存7
int setpDetectedAntennaCache(const cv::Mat&left,const cv::Mat&depth,const std::vector<std::vector<cv::Point>>&detectedAntenna);
//返回值:0空,1非空
int getpDetectedAntennaCacheLDA( cv::Mat&left, cv::Mat&depth, std::vector<std::vector<cv::Point>>&detectedAntenna);
//返回值:0空,1非空
int getpDetectedAntennaCacheA( std::vector<std::vector<cv::Point>>&detectedAntenna);

/**************************天线检测缓存***＊*＊*＊*＊*＊*＊*＊*＊**＊*＊*/




//读写VSLAM缓存2
int setmdpVSLAMCache(int FramID ,float* Scenemeandepth ,bool IsKF,const cv::Mat&left,
                    const cv::Mat&depth,const cv::Mat&TRC,const std::vector<std::vector<cv::Point>>&DetectedAntenna);
//返回值:0空,1非空
int getmdpVSLAMCache( vFrame&) ;
//获取VSLAM缓存大小
int getmdpVSLAMCacheSize();

void clearVSLAMCache();


//读写天线深度缓存8
int setpDepthofAntenna(const cv::Mat& );
//返回值:0空,1非空
int getpDepthofAntenna(cv::Mat&) ;


};
#endif
