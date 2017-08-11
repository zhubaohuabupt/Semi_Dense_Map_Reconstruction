/*This file is part of AntennaDetect-SemiDenseReconstruction.
 * StaticEstimation.cpp
 *
 *  Created on: 2017年6月1日
 *      Author: Zuber
 */
#include"StaticEstimation.h"
#include <frame.h>
#include <point.h>
#include <feature.h>
#include <matcher.h>
#include"Config.h"
namespace DepthEstimation {

StaticEstimation::StaticEstimation(ComData*ComData_):DepthEstimation(ComData_),SHOW(false){}
void StaticEstimation::CalDepth( cv::Mat &DEPTH_RESULT)
   {
   mFrameDeque.push_back(mvFrame);
   //延迟一定帧
       if(mFrameDeque.size()<Neighborframesize)
           return;
     vFrame SlideFrame=mFrameDeque[KeyframeIDinNeighborWindow];
    if(SlideFrame.IsKF)
    {
        //公共区数据转换
         FramePtr keyframe;
         keyframe.reset(new Frame(cam,SlideFrame.left,SlideFrame.FrameID)) ;
         keyframe->T_f_w_=Mat2SE3(SlideFrame.TCW);
         //设定待估计区域
         getSourcePoint(keyframe,SlideFrame.DetectedAntenna,SlideFrame.SceneMEANdepth);
        vector<FramePtr> NeighborFrames;
        //计算不确定性相关参数
        const double focal_length = keyframe->cam_->errorMultiplier2();
        double px_noise = 1.0;
        double px_error_angle = atan(px_noise/(2.0*focal_length))*2.0; // law of chord (sehnensatz)
                                                                                                                                                                                                                        //imshow("key ",keyframe->img());
        //更新用于匹配的图像序列缓存
        for(int i=0;i<mFrameDeque.size();i++)
        {
            if(i==KeyframeIDinNeighborWindow) continue;
            FramePtr sqframe;
            sqframe.reset(new Frame(cam,mFrameDeque[i].left,mFrameDeque[i].FrameID)) ;
            sqframe->T_f_w_=Mat2SE3(mFrameDeque[i].TCW);
            NeighborFrames.push_back(sqframe);
                                                                                                                                                                                                                         //   imshow("sub ",sqframe->img());
                                                                                                                                                                                                                         //   waitKey(0);
        }

        int matchnum=0;
        //对设定的区域逐一开始计算深度
        for(int i=0;i<allseeds_.size();i++)
        {
         list<Seed>::iterator it=allseeds_[i].begin();
            while(it!=allseeds_[i].end())
                {
                   vector<SingleObs> MultiObs;
                    for(int NeighborFramecnt=0;NeighborFramecnt<NeighborFrames.size();NeighborFramecnt++)
                    {
                        FramePtr frame=NeighborFrames[NeighborFramecnt];
                        // check if point is visible in the current image
                        SE3 T_ref_cur = it->ftr->frame->T_f_w_ *frame->T_f_w_.inverse();
                        const Vector3d xyz_f(T_ref_cur.inverse()*(it->mu * it->ftr->f) );
                        if(xyz_f.z() < 0.0)
                             continue; // behind the camera
                        if(!frame->cam_->isInFrame(frame->f2c(xyz_f).cast<int>()))
                             continue;  // point does not project in image
                        float z_max  = it->mu + sqrt(it->sigma2);
                        float z_min= max(it->mu - sqrt(it->sigma2), 0.00000001f);
                        double z;
                        //极线找匹配
                         if(!matcher_.findEpipolarMatchDirect( *it->ftr->frame, *frame, *it->ftr, it->mu, z_min, z_max, z))
                         {
                           it->b++; // increase outlier probability when no match was found
                           continue;
                         }
                         // 计算不确定性
                         double tau = computeTau(T_ref_cur, it->ftr->f, z, px_error_angle);
                         //存储
                        MultiObs.push_back(SingleObs(z,tau*tau));
                    }
                   //从Ｎ帧中匹配成功的次数若小于一定数量，则放弃改点
                 if(MultiObs.size()<3)  {
                       it->mu=-1;//天线上无效深度置－１
                       it++;  continue;
                 }
                 vector<SingleObs> Obsaftercheck;
                 //从多次匹配结果里找出最大相容观测值的集合
                 if( !findValueObswithKaFang(MultiObs,Obsaftercheck) )
                 {
                      it->mu=-1;
                     continue;
                 }
                //多次测量值融合
                fuseMultiValueObs(Obsaftercheck,it->mu,it->sigma2,it->IsValue);   //  cout<<"  Obsaftercheck  "<<Obsaftercheck.size()<<endl;
                    matchnum++;
                     ++it;
                }
          }
        //滤波
        Denose_Smoothing(DEPTH_RESULT);                                                                                                                                                                                                            // cout<<"matchnum.size() "<<matchnum<<endl;
     //  if(SHOW)   show();
        allseeds_.clear();
    }
mFrameDeque.pop_front();
}

void StaticEstimation::getSourcePoint(FramePtr frame, const std::vector<std::vector<cv::Point>>&DetectedAntenna,float MEANdepth)
{
       for(int i=0;i<DetectedAntenna.size();i++){
           std::list<Seed, aligned_allocator<Seed> > seeds_;
       for(int j=0;j<DetectedAntenna[i].size();j++)
       {
            Feature* ftr=new Feature(frame.get(),Vector2d(DetectedAntenna[i][j].x,DetectedAntenna[i][j].y),0);
         seeds_.push_back(Seed(ftr, MEANdepth));
       }
       allseeds_.push_back(seeds_);
       }
}
bool StaticEstimation::findValueObswithKaFang( const vector<SingleObs>&MultiObs,vector<SingleObs>&aftercheck)
{
    vector<SingleObs> tmpMultiObs;
    for(auto it=MultiObs.begin();it!=MultiObs.end();it++)
        if(it->depth<100&&it->depth>0.5)
            tmpMultiObs.push_back(*it);
    int num=tmpMultiObs.size();
    cv::Mat A=cv::Mat::zeros(num,num,CV_8UC1);
//构造检测结果矩阵
    for(int j=0;j<num;j++)
    for(int i=j;i<num;i++){
        bool re_=KaFangcheck(tmpMultiObs[i],tmpMultiObs[j]);
                     A.at<uchar>(j,i)=re_;
                    A.at<uchar>(i,j)=re_;
    }
//提取最大相容观测集合
int max_CompatibleObsnum=0;
int max_result_row=-1;
   for(int j=0;j<num;j++)
   {
       int rowmax_CompatibleObsnum=0;
        for(int i=0;i<num;i++)
        {
           if( A.at<uchar>(j,i)==true)
               rowmax_CompatibleObsnum++;
        }
        if(rowmax_CompatibleObsnum>max_CompatibleObsnum)
               max_result_row=j;
   }
//提去最大相容观测值集合
    for(int i=0;i<num;i++)
    {
        if(A.at<uchar>(max_result_row,i)==true)
            aftercheck.push_back(tmpMultiObs[i]);
    }
    if(aftercheck.size()<3)
        return false;
    return true;
}
bool StaticEstimation::KaFangcheck(const SingleObs&obs1,const SingleObs&obs2)
{
    //KaFang check
return std::pow(obs1.depth-obs2.depth,2)*(obs1.var+obs2.var)/(obs1.var*obs2.var)<5.99;

}
void StaticEstimation::fuseMultiValueObs( const vector<SingleObs>&MultiObs,float& Depthafterfuse,float& Varafterfuse,bool &IsValue)
{
    //计算融合后方差
    for(auto it=MultiObs.begin();it!=MultiObs.end();it++){
      Varafterfuse+=1/it->var;
    }
    Varafterfuse=1/Varafterfuse;
    //计算融合后深度值
   for(auto it=MultiObs.begin();it!=MultiObs.end();it++) {
        Depthafterfuse+=it->depth/it->var;
   }
        Depthafterfuse*=Varafterfuse;
        IsValue=true;
}
void StaticEstimation::Denose_Smoothing(cv::Mat &DEPTH_RESULT)
 {
         cv::Mat useForFilter=cv::Mat::zeros(cam->height(),cam->width(),CV_32FC1);
         for(int i=0;i<allseeds_.size();i++)
         for(auto it=allseeds_[i].begin();it!=allseeds_[i].end();it++)
           {
                        useForFilter.at<float>(it->ftr->px[1],it->ftr->px[0])=it->mu;
            }

       //对天线点云滤波

            int row=useForFilter.rows;
            int col=useForFilter.cols;
            int winsize=21;

            for(int j=0;j<row;j++)
                for(int i=0;i<col;i++)
                {
                if(useForFilter.at<float>(j,i)==0)  continue;
                int lx=i-winsize>0?i-winsize:0;
                int rx=i+winsize>col?col:i+winsize;
                int uy=j-winsize>0?j-winsize:0;
                int dy=j+winsize>row?row:j+winsize;
                vector<float > NeighborDepth;
                        for(int winrow=uy;winrow<dy;winrow++)
                           for(int wincol=lx;wincol<rx;wincol++)
                           {
                              if(useForFilter.at<float>(winrow,wincol)==0||useForFilter.at<float>(winrow,wincol)==-1)  continue;
                               NeighborDepth.push_back(useForFilter.at<float>(winrow,wincol));
                           }

                 if(NeighborDepth.size()<1)
                 {
                      useForFilter.at<float>(j,i)=0;
                      continue;
                 }
                 sort(NeighborDepth.begin(),NeighborDepth.end());
                  float meandepthwirline=NeighborDepth[NeighborDepth.size()/2];
                  useForFilter.at<float>(j,i)=meandepthwirline;
                }
            //插值
        cv::Mat tmpstorage=Mat::zeros(cam->height(),cam->width(),CV_32FC1);
         for(int j=0;j<row;j++)
             for(int i=0;i<col;i++)
             {
             if(useForFilter.at<float>(j,i)==0)  continue;
             int lx=i-winsize>0?i-winsize:0;
             int rx=i+winsize>col?col:i+winsize;
             int uy=j-winsize>0?j-winsize:0;
             int dy=j+winsize>row?row:j+winsize;
             vector<float > NeighborDepth;
                     for(int winrow=uy;winrow<dy;winrow++)
                        for(int wincol=lx;wincol<rx;wincol++)
                        {
                           if(useForFilter.at<float>(winrow,wincol)==0||useForFilter.at<float>(winrow,wincol)==-1)  continue;
                            NeighborDepth.push_back(useForFilter.at<float>(winrow,wincol));
                        }
              if(NeighborDepth.size()<1)
              {
                   tmpstorage.at<float>(j,i)=0;
                   continue;
              }
           //   sort(NeighborDepth.begin(),NeighborDepth.end());
              float mean=accumulate(NeighborDepth.begin(),NeighborDepth.end(),0 )/NeighborDepth.size();
           //   float meandepthwirline=NeighborDepth[NeighborDepth.size()/2];
               tmpstorage.at<float>(j,i)=mean;
             }
        DEPTH_RESULT=tmpstorage.clone();
   //更新三维点内存,更改为滤波后的深度值
         for(int i=0;i<allseeds_.size();i++)
         for(auto it=allseeds_[i].begin();it!=allseeds_[i].end();it++)
         {
             int px_kf=it->ftr->px[0];
             int py_kf=it->ftr->px[1];
           it->mu =tmpstorage.at<float>(py_kf,px_kf);
         }


}
void StaticEstimation::testshow()
{
    this->SHOW=true;
}
void StaticEstimation::show()
{
    cv::Mat show=cv::Mat::zeros(cam->height(),cam->width(),CV_8UC1);
  for(int i=0;i<allseeds_.size();i++)
  for(auto it=allseeds_[i].begin();it!=allseeds_[i].end();it++)
  {
       show.at<uchar>(it->ftr->px[1],it->ftr->px[0])=30*it->mu;
  }
cv::Mat color;
GenerateFalseMap(show,color);
imshow("Depth of Antenna ",color);
waitKey(5);
}
}
