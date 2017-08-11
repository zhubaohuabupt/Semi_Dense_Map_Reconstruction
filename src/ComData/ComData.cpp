/*This file is part of AntennaDetect-SemiDenseReconstruction.
 * ComData.cpp
 *
 *  Created on: 2017年5月22日
 *      Author: Zuber
 */
#include"ComData.h"
//相机参数构造
CamPara::CamPara(float fx_,float fy_,float cx_,float cy_,float bf_,int height_,int width_):fx(fx_),fy(fy_),cx(cx_),cy(cy_),bf(bf_),height(height_),width(width_){}

//读图缓存构造
ImgSource::ImgSource(  const cv::Mat& left_, const cv::Mat&depth_,const cv::Mat&disp_):left(left_.clone()),depth(depth_.clone()),disp(disp_.clone()){}

//天线检测缓存构造
DetectedAntennaCache::DetectedAntennaCache(  const cv::Mat& left_, const cv::Mat&depth_, const std::vector<std::vector<cv::Point>>& DetectedAntenna_)
{
    this->left=left_.clone();
    this->depth=depth_.clone();
    this->DetectedAntenna=DetectedAntenna_;
}

//半稠密点云构造
SemiDenseDepthProcessed::SemiDenseDepthProcessed(const cv::Mat&OriginSemiDenseDepth_,const cv::Mat&NoisedSemiDenseDepth_,
                                                 const cv::Mat&DenosedSemiDenseDepth_,const cv::Mat& SmoothedSemiDenseDepth_)
{
    this->OriginSemiDenseDepth=OriginSemiDenseDepth_.clone();
    this->NoisedSemiDenseDepth=NoisedSemiDenseDepth_.clone();
    this->DenosedSemiDenseDepth=DenosedSemiDenseDepth_.clone();
    this->SmoothedSemiDenseDepth=SmoothedSemiDenseDepth_.clone();
}
//VSLAM缓存构造
vFrame::vFrame( int FrameID_,float *Scenedepth_,bool IsKF_,const cv::Mat& left_,const cv::Mat& depth_,const cv::Mat& TCW_,const std::vector<std::vector<cv::Point>>& DetectedAntenna_):
    FrameID(FrameID_),IsKF(IsKF_),  left(left_.clone()), depth(depth_.clone()), TCW(TCW_.clone()) ,DetectedAntenna(DetectedAntenna_)
{
this->SceneMINdepth=Scenedepth_[0];
this->SceneMEANdepth=Scenedepth_[1];
}
vFrame::vFrame():FrameID(0),IsKF(false),SceneMINdepth(NULL)
{
}
//公共区构造初始化
ComData::ComData()
{
this->pCamPara=NULL;
this->pImgSource=NULL;
this->Cache1=pImgSource;
this->Cache2=NULL;
this->pDetectedAntennaCache=NULL;
this->pSemiDenseDepth=NULL;
this->mSystemPattern=mOnlyDetect;
this->mVSLAMstate=sIdle;

}
//读写相机参数
int ComData::setCamPara(const CamPara&CamPara_)
{
    ulock lock(pCamParaMutex);
    pCamPara=new CamPara(CamPara_.fx,CamPara_.fy,CamPara_.cx,CamPara_.cy,CamPara_.bf,CamPara_.height,CamPara_.width);
    std::cout<<"Camera Parameter Init Sucess "<<std::endl;
    return 1;
}
int ComData::getCamPara(float&fx_,float&fy_,float&cx_,float&cy_,float&bf_,int&height_,int&width_)
{
 ulock lock(pCamParaMutex);
    if(pCamPara==NULL)
        return 0;
fx_=this->pCamPara->fx;
fy_=this->pCamPara->fy;
cx_=this->pCamPara->cx;
cy_=this->pCamPara->cy;
bf_=this->pCamPara->bf;
height_=this->pCamPara->height;
width_=this->pCamPara->width;
return 1;
}
//读写天线检测线程帧率
void ComData::setFPS_AntennaDetection(float FPS_AntennaDetection)
{
    ulock lock(mFPS_AntennaDetectionMutex);
    this->FPS_AntennaDetection=FPS_AntennaDetection;
}

float ComData::getFPS_AntennaDetection()
{
 ulock lock(mFPS_AntennaDetectionMutex);
 return this->FPS_AntennaDetection;
}
//读写深度估计线程帧率
void  ComData::setFPS_DepthEstimation(float FPS_DepthEstimation)
{
    ulock lock(mFPS_DepthEstimationMutex);
     this->FPS_DepthEstimation=FPS_DepthEstimation;
}

float ComData::getFPS_DepthEstimation()
{
      ulock lock(mFPS_DepthEstimationMutex);
       return this->FPS_DepthEstimation;
}
//读写点云处理线程帧率
void  ComData::setFPS_PointCloudProcessing(float FPS_PointCloudProcessing)
{
    ulock lock(mFPS_PointCloudProcessingMutex);
    this->FPS_PointCloudProcessing=FPS_PointCloudProcessing;
}

float ComData::getFPS_PointCloudProcessing()
{
 ulock lock(mFPS_PointCloudProcessingMutex);
  return this->FPS_PointCloudProcessing;
}

//读写VSLAM线程帧率
void ComData::setFPS_VSLAM(float FPS_VSLAM)
{
    ulock lock(mFPS_VSLAMMutex);
      this->FPS_VSLAM=FPS_VSLAM;
}

float  ComData::getFPS_VSLAM( )
{
    ulock lock(mFPS_VSLAMMutex);
     return this->FPS_VSLAM;
}

//读写主(抓图)线程实时帧率
void ComData::setFPS_main(float FPS_main)
{
ulock lock(mFPS_mainMutex);
this->FPS_main=FPS_main;
}
float ComData::getFPS_main( )
{
    ulock lock(mFPS_mainMutex);
    return this->FPS_main;
}
void ComData::setImageReadID(int ID)
{
    ulock lock(ID_Mutex);
    this->ID=ID;
}
int ComData::getImageID()
{
        ulock lock (ID_Mutex);
        return this->ID;
}

void ComData::setDatasetPath(const std::string&DatasetPath)
{
    ulock lock(DatasetPath_Mutex);
            this->DatasetPath=DatasetPath;
}
std::string ComData::getDatasetPath()
{
        ulock lock(DatasetPath_Mutex);
        return  this->DatasetPath;
}


 int ComData::mImgSourceReaderCounter=0;
 void ComData::swapImgSourceCache(const cv::Mat&left,const cv::Mat&depth,const cv::Mat&disp)
 {
        if(Cache1==NULL)
             Cache1=new ImgSource(left,depth,disp);
        if(Cache2==NULL)
              Cache2=new ImgSource(left,depth,disp);
    if(pImgSource==Cache1)
        {
            Cache2->left=left.clone();
            Cache2->depth=depth.clone();
            Cache2->disp=disp.clone();
        }
    else
        {
            Cache1->left=left.clone();
            Cache1->depth=depth.clone();
            Cache1->disp=disp.clone();
        }
 }
//读写图源6
int ComData::UpdatepImgSource(const cv::Mat& left,const cv::Mat&depth,const cv::Mat&disp)
{
        ulock lock(mImgSourceRWMutex) ;
        {
           ulock lock(mImgSourceMutex) ;
           if (pImgSource==Cache1)
            pImgSource=Cache2;
          else
            pImgSource= Cache1;
        }
        ImgSourcecond.notify_all();
    return 1;
}
int ComData::getpImgSourceLeftDepth( cv::Mat&left_, cv::Mat&depth_)
{
  if(pImgSource==NULL)
      return 0;
   {
   ulock lock(mImgSourceRWMutex) ;
        mImgSourceReaderCounterMutex.lock();
         if(mImgSourceReaderCounter==0)
               mImgSourceMutex.lock();
            ++ mImgSourceReaderCounter;
       mImgSourceReaderCounterMutex.unlock();
    }

  //开始内存拷贝
    ImgSource* pt=pImgSource;
    left_=pt->left.clone();
    depth_=pt->depth.clone();
      //内存拷贝完毕
    mImgSourceReaderCounterMutex.lock();
            --mImgSourceReaderCounter;
    if(mImgSourceReaderCounter==0)
        mImgSourceMutex.unlock();
    mImgSourceReaderCounterMutex.unlock();

    return 1;
}

int ComData::getpImgSourceLeftDisp( cv::Mat&left_, cv::Mat&disp_)
{
    if(pImgSource==NULL)
      return 0;
    {
  ulock lock(mImgSourceRWMutex) ;
    mImgSourceReaderCounterMutex.lock();
    if(mImgSourceReaderCounter==0)
          mImgSourceMutex.lock();
      ++ mImgSourceReaderCounter;
    mImgSourceReaderCounterMutex.unlock();
    }
      //开始内存拷贝
    ImgSource* pt=pImgSource;
    left_=pt->left.clone();
    disp_=pt->disp.clone();
     //内存拷贝完毕
     mImgSourceReaderCounterMutex.lock();
       -- mImgSourceReaderCounter;
     if(mImgSourceReaderCounter==0)
         mImgSourceMutex.unlock();
      mImgSourceReaderCounterMutex.unlock();
    return 1;
}

int ComData::getpImgSourceLeft( cv::Mat&left_)
{
    if(pImgSource==NULL)
      return 0;
    {
  ulock lock(mImgSourceRWMutex) ;
    mImgSourceReaderCounterMutex.lock();
    if(mImgSourceReaderCounter==0)
          mImgSourceMutex.lock();
      ++ mImgSourceReaderCounter;
    mImgSourceReaderCounterMutex.unlock();
    }
      //开始内存拷贝
    ImgSource* pt=pImgSource;
    left_=pt->left.clone();
     //内存拷贝完毕
     mImgSourceReaderCounterMutex.lock();
       -- mImgSourceReaderCounter;
     if(mImgSourceReaderCounter==0)
         mImgSourceMutex.unlock();
      mImgSourceReaderCounterMutex.unlock();
    return 1;
}


//读写天线检测缓存7
int ComData::mDetectedAntennaCacheReaderCounter=0;
int ComData::setpDetectedAntennaCache(const cv::Mat&left,const cv::Mat&depth,const std::vector<std::vector<cv::Point>>&detectedAntenna)
{
    ulock lock(pDetectedAntennaCacheRWMutex) ;
    {
      ulock lock(pDetectedAntennaCacheMutex) ;


          if(this->pDetectedAntennaCache==NULL)
              this->pDetectedAntennaCache=new DetectedAntennaCache(left,depth,detectedAntenna);
          else
            {

                this->pDetectedAntennaCache->left=left.clone();
                this->pDetectedAntennaCache->depth=depth.clone();
                 this->pDetectedAntennaCache->DetectedAntenna= detectedAntenna;
            }
          mDetectedAntennaCacheCond.notify_all();
    }
    return 1;
}
int ComData::getpDetectedAntennaCacheLDA( cv::Mat&left_, cv::Mat&depth_, std::vector<std::vector<cv::Point>>&detectedAntenna_)
{
    if(this->pDetectedAntennaCache==NULL)
          return 0;
    {
    ulock lock(pDetectedAntennaCacheRWMutex) ;  {
          ulock lock(mDetectedAntennaCacheReaderCounterMutex);
               if(this->mDetectedAntennaCacheReaderCounter==0)
                   pDetectedAntennaCacheMutex.lock();
              ++ this->mDetectedAntennaCacheReaderCounter;
             }
    }

    //开始内存拷贝
   left_=this->pDetectedAntennaCache->left.clone();
   depth_=this->pDetectedAntennaCache->depth.clone();
   detectedAntenna_=this->pDetectedAntennaCache->DetectedAntenna;

 //内存拷贝完毕
    mDetectedAntennaCacheReaderCounterMutex.lock();
      --this->mDetectedAntennaCacheReaderCounter;
    if(this->mDetectedAntennaCacheReaderCounter==0)
        pDetectedAntennaCacheMutex.unlock();

    mDetectedAntennaCacheReaderCounterMutex.unlock();

  return 1;
}
int ComData::getpDetectedAntennaCacheA(std::vector<std::vector<cv::Point>>&detectedAntenna_)
{
    if(this->pDetectedAntennaCache==NULL)
          return 0;
    {
    ulock lock(pDetectedAntennaCacheRWMutex) ;  {
          ulock lock(mDetectedAntennaCacheReaderCounterMutex);
               if(this->mDetectedAntennaCacheReaderCounter==0)
                   pDetectedAntennaCacheMutex.lock();
              ++ this->mDetectedAntennaCacheReaderCounter;
             }
    }
    //开始内存拷贝
   detectedAntenna_=this->pDetectedAntennaCache->DetectedAntenna;
 //内存拷贝完毕
    mDetectedAntennaCacheReaderCounterMutex.lock();
      --this->mDetectedAntennaCacheReaderCounter;
    if(this->mDetectedAntennaCacheReaderCounter==0) 
        pDetectedAntennaCacheMutex.unlock();

    mDetectedAntennaCacheReaderCounterMutex.unlock();

  return 1;
}


//读写VSLAM缓存2
int ComData::setmdpVSLAMCache(int FramID_ ,float *Scenedepth_ ,bool IsKF_,const cv::Mat&left_,
                                                const cv::Mat&depth_,const cv::Mat&TRC_,const std::vector<std::vector<cv::Point>>&DetectedAntenna_)
{
 ulock lock(mdpVSLAMCacheMutex) ;
 mdpVSLAMCache.push_back(new vFrame(FramID_,Scenedepth_,IsKF_,left_,depth_,TRC_,DetectedAntenna_));
 startDepthEstimationcond.notify_one();
return 1;
}

 int ComData::getmdpVSLAMCache( vFrame&vFrame_)
{
 ulock lock(mdpVSLAMCacheMutex) ;
  vFrame* DataFromCache=mdpVSLAMCache.front();
  if(mdpVSLAMCache.size()<1)
       return 0;
 vFrame_.FrameID=DataFromCache->FrameID;
 vFrame_.IsKF=DataFromCache->IsKF;
 vFrame_.SceneMINdepth=DataFromCache->SceneMINdepth;
 vFrame_.SceneMEANdepth=DataFromCache->SceneMEANdepth;
 vFrame_.left=DataFromCache->left.clone();
 vFrame_.depth=DataFromCache->depth.clone();
 vFrame_.TCW=DataFromCache->TCW.clone();
 vFrame_.DetectedAntenna=DataFromCache->DetectedAntenna;
 //用后删除
  vFrame* pt=mdpVSLAMCache.front();
  delete pt;
  mdpVSLAMCache.pop_front();

 return 1;
}

int ComData:: getmdpVSLAMCacheSize()
{
    ulock lock(mdpVSLAMCacheMutex) ;
    return this->mdpVSLAMCache.size();
}
void ComData::clearVSLAMCache()
{
 ulock lock(mdpVSLAMCacheMutex) ;
 while(mdpVSLAMCache.size()>0)
     {
         vFrame* pt=mdpVSLAMCache.front();
         delete pt;
         mdpVSLAMCache.pop_front();
     }
 return;
}

//读写VSLAMstate
int ComData::setVSLAMstate(const VSLAMstate&VSLAMstate_)
{
    ulock lock(mVSLAMstateMutex) ;
    this->mVSLAMstate=VSLAMstate_;
    return 1;
}

int ComData::getVSLAMstate( VSLAMstate&VSLAMstate_)
{
      //ulock lock(mVSLAMstateMutex) ;
      std::unique_lock<std::mutex> lock(mVSLAMstateMutex);
   VSLAMstate_=this->mVSLAMstate;
    return 1;
}


//读写系统运行模式
int ComData::setSystemPattern(const SystemPattern&SystemPattern_)
{
   ulock lock(SystemPatternMutex) ;
   this->mSystemPattern=SystemPattern_;
   if(SystemPattern_==mDepthEstimation)
   startVOcond.notify_all();
  return 1;
}
int ComData::getSystemPattern( SystemPattern&SystemPattern_)
{
    ulock lock(SystemPatternMutex) ;
   SystemPattern_= this->mSystemPattern;
   return 1;
}


//读写去噪后的半稠密深度图
int ComData::setpSemiDenseDepth(const cv::Mat&OriginSemiDenseDepth_,const cv::Mat&NoisedSemiDenseDepth_,const cv::Mat&DenosedSemiDenseDepth_,const cv::Mat&SmoothedSemiDenseDepth_)
{
 ulock lock(pSemiDenseDepthMutex) ;
     if(this->pSemiDenseDepth==NULL)
         this->pSemiDenseDepth=new SemiDenseDepthProcessed(OriginSemiDenseDepth_,NoisedSemiDenseDepth_,DenosedSemiDenseDepth_,SmoothedSemiDenseDepth_);
    else
     {
          this-> pSemiDenseDepth->OriginSemiDenseDepth=OriginSemiDenseDepth_.clone();
          this-> pSemiDenseDepth->NoisedSemiDenseDepth=NoisedSemiDenseDepth_.clone();
          this-> pSemiDenseDepth->DenosedSemiDenseDepth=DenosedSemiDenseDepth_.clone();
          this-> pSemiDenseDepth->SmoothedSemiDenseDepth=SmoothedSemiDenseDepth_.clone();
     }
 return 1;
}
int ComData::getpSemiDenseDepth(  cv::Mat&OriginSemiDenseDepth_, cv::Mat&NoisedSemiDenseDepth_, cv::Mat&DenosedSemiDenseDepth_, cv::Mat&SmoothedSemiDenseDepth_)
{
    ulock lock(pSemiDenseDepthMutex) ;
    if(this->pSemiDenseDepth==NULL)
        return 0 ;
   OriginSemiDenseDepth_= this->pSemiDenseDepth->OriginSemiDenseDepth.clone();
   NoisedSemiDenseDepth_= this->pSemiDenseDepth->NoisedSemiDenseDepth.clone();
   DenosedSemiDenseDepth_=this->pSemiDenseDepth->DenosedSemiDenseDepth.clone();
  SmoothedSemiDenseDepth_= this->pSemiDenseDepth->SmoothedSemiDenseDepth.clone();
    return 1;
}


//读写天线深度缓存
int ComData::setpDepthofAntenna( const cv::Mat&depth )
{
    ulock lock(mDepthofAntennaMutex) ;
            this->mDepthofAntenna=depth.clone();
    return 1;
}
int ComData::getpDepthofAntenna(cv::Mat& DepthofAntenna_)
{
     ulock lock(mDepthofAntennaMutex) ;
     if(this->mDepthofAntenna.empty())
       return 0;
    DepthofAntenna_=this->mDepthofAntenna.clone();
     return 1;
}
