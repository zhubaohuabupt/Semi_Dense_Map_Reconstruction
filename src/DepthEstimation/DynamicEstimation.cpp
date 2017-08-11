/*This file is part of AntennaDetect-SemiDenseReconstruction.
 * DynamicEstimation.cpp
 *
 *  Created on: 2017年6月1日
 *      Author: Zuber
 */
#include <boost/bind.hpp>
#include <boost/math/distributions/normal.hpp>
#include <global.h>
#include <DynamicEstimation.h>
#include <frame.h>
#include <point.h>
#include <feature.h>
#include <matcher.h>
#include <config.h>
namespace DepthEstimation {

DynamicEstimation::DynamicEstimation(ComData*ComData_):DepthEstimation(ComData_),lastKF(NULL){}
DynamicEstimation::~DynamicEstimation(){}

void DynamicEstimation::CalDepth(cv::Mat &DEPTH_RESULT )
{
   //公共区数据转换
    FramePtr frame;
    frame.reset(new Frame(cam,mvFrame.left,mvFrame.FrameID)) ;
    frame->T_f_w_=Mat2SE3(mvFrame.TCW);
   //计算误匹配一个误差不确定性的参数
  const double focal_length = frame->cam_->errorMultiplier2();
  double px_noise = 1.0;
  double px_error_angle = atan(px_noise/(2.0*focal_length))*2.0; // law of chord (sehnensatz)
cout<<" seeds_.size()"<<seeds_.size()<<endl;
 list<Seed>::iterator it=seeds_.begin();

int matchnum=0;
  while( it!=seeds_.end())
  {
    // check if seed is not already too old
    if((Seed::batch_counter - it->batch_id) > options_.max_n_kfs) {
      it = seeds_.erase(it);
      continue;
    }

    // check if point is visible in the current image
    SE3 T_ref_cur = it->ftr->frame->T_f_w_ * frame->T_f_w_.inverse();
    const Vector3d xyz_f(T_ref_cur.inverse()*(it->mu * it->ftr->f) );////////////////////////////////////////////////////////////////////
    if(xyz_f.z() < 0.0)  {
      ++it; // behind the camera
      continue;
    }
    if(!frame->cam_->isInFrame(frame->f2c(xyz_f).cast<int>())) {
      ++it; // point does not project in image
      continue;
    }

     float z_max  = it->mu + sqrt(it->sigma2);//////////////////////////////////////////////////////////////////////////////////////////////////////
    float z_min= max(it->mu - sqrt(it->sigma2), 0.00000001f);//////////////////////////////////////////////////////////////////////////////////////////////////////
    double z;
   double inittau=-1;
   //极线找匹配
    if(!matcher_.findEpipolarMatchDirect( *it->ftr->frame, *frame, *it->ftr, it->mu, z_min, z_max, z))
    {
      it->b++; // increase outlier probability when no match was found
      ++it;
      continue;
    }
    matchnum++;
    // compute tau
    double tau = computeTau(T_ref_cur, it->ftr->f, z, px_error_angle);
  // tau*=std::pow(2.7,(std::min(2.0d,inittau)-2));
  //cout<<"        new_z       "<<z<<endl;

    // update the estimate
//#define showupdate
#ifdef showupdate
  cout<<"old_z         "<<it->mu<<endl;
  cout<<"old_sigma "<<it->sigma2<<endl;
  cout<<"old_a         "<<it->a<<endl;

  cout<<"        tau            "<<tau<<endl;
  cout<<"        new_z       "<<z<<endl;
#endif
  updateSeed(z, tau*tau, &*it);

  #ifdef showupdate
    cout<<"              new_a  "<<it->a<<endl;
   cout<<"               z_update "<<it->mu<<endl;
   cout<<"              sigma_update "<<it->sigma2<<endl;
   #endif

				
    // if the seed has converged, we initialize a new candidate point and remove the seed
    if(sqrt(it->sigma2) < it->z_range/10)///////////////////////////////////////////////////////////////////////////////////////
    {
      assert(it->ftr->point == NULL); // TODO this should not happen anymore
      Vector3d xyz_world(it->ftr->frame->T_f_w_.inverse() * (it->ftr->f * (it->mu)));
      Point* point = new Point(xyz_world, it->ftr);
     //it->ftr->point = point;
       //存储收敛的点
      converge_point* cvg_point=new converge_point(it->ftr,point);
      mapconvergePoints[it->ftr->frame->id_].Kfconvergpoints.push_back(cvg_point);
      it = seeds_.erase(it);
      ++ALLConvergePointNum;
    }
    else if(isnan(z_min))
    {
      cerr<<("z_min is NaN")<<endl;
      it = seeds_.erase(it);
    }
    else
      ++it;
  }
   cout<<"matchnum :" <<matchnum<<endl;
   //添加关键帧上的种子
   if(mvFrame.IsKF)
        {
           frame->is_keyframe_=true;
           pKeyFrames.push_back(frame);
           if(frame->isKeyframe())
             initializeSeeds(frame,mvFrame.SceneMEANdepth);
           lastKF=frame;
        }
   }




void DynamicEstimation::initializeSeeds( FramePtr frame,float newKeyframeMEANdepth)
{
      Features new_features;
        for(int i=0;i<mvFrame.DetectedAntenna.size();i++)
           for(int j=0;j<mvFrame.DetectedAntenna[i].size();j++)
                 new_features.push_back(new Feature(frame.get(),Vector2d(mvFrame.DetectedAntenna[i][j].x,mvFrame.DetectedAntenna[i][j].y),0));
      //  imshow("wire ",mvFrame.DetectedAntenna);
      //  waitKey(3);
    //记录每个关键帧一共提取的特征点数目，为计算收敛率做准备。
    mapconvergePoints[frame->id_].KeypointN=new_features.size();

  ++Seed::batch_counter;
  std::for_each(new_features.begin(), new_features.end(), [&](Feature* ftr){
    seeds_.push_back(Seed(ftr, newKeyframeMEANdepth));
  });

  if(options_.verbose)
    SVO_INFO_STREAM("DepthEstimation: Initialized "<<new_features.size()<<" new seeds");
//  seeds_updating_halt_ = false;
}
//void DynamicEstimation::initializeSeedstest(FramePtr frame)
//{
//    vector<cv::Point> smallthing;
//    {
//    lock_t lock(stereoD_mutex);
//      detect_sm.detect_depth(frame->img_pyr_[0],stereoD,smallthing);//detect_wieline
//    }
////在这样的情况下，我们选择给新种子初始一个大方差和深度均值  1 第一个关键帧　２上一个关键帧的种子为０
////其他情况下　我们把上一个关键帧的深度向新的关键帧传递，旨在让种子更快的收敛．
//if(seeds_.size()==0||(lastKF!=NULL&&mapconvergePoints[lastKF->id_].seeds_inthisKF.size()==0))
//           initializeSeeds(frame);
//else if(!propragatedepth(frame,smallthing, lastKF,seeds_))
//    initializeSeeds(frame) ;
//#define showdetectWirline
//#ifdef  showdetectWirline
//   cv::Mat wire=Mat::zeros(stereoD.rows,stereoD.cols,CV_8UC1);
//    for(auto it=smallthing.begin();it!=smallthing.end();it++)
//      {
//        wire.at<uchar>(it->y,it->x)=255;
//       }

//    imshow("wire ",wire);

//    waitKey(3);
//#endif
//    //std::cout<<"新提取天线　 "<<smallthing.size()<<std::endl;
//    //记录每个关键帧一共提取的特征点数目，为计算收敛率做准备。
//    mapconvergePoints[frame->id_].KeypointN=smallthing.size();

//}

void DynamicEstimation::updateSeed(const float x, const float tau2, Seed* seed)
{
  float norm_scale = seed->sigma2 + tau2;//////////////////////////////////////
  if(std::isnan(norm_scale))
    return;
  boost::math::normal_distribution<float> nd(seed->mu, norm_scale);
  float s2 = 1./(1./seed->sigma2 + 1./tau2);
  float m = s2*(seed->mu/seed->sigma2 + x/tau2);
  float C1 = seed->a/(seed->a+seed->b) * boost::math::pdf(nd, x);
  float C2 = seed->b/(seed->a+seed->b) * 1./seed->z_range;
  float normalization_constant = C1 + C2;
  C1 /= normalization_constant;
  C2 /= normalization_constant;
  float f = C1*(seed->a+1.)/(seed->a+seed->b+1.) + C2*seed->a/(seed->a+seed->b+1.);
  float e = C1*(seed->a+1.)*(seed->a+2.)/((seed->a+seed->b+1.)*(seed->a+seed->b+2.))
          + C2*seed->a*(seed->a+1.0f)/((seed->a+seed->b+1.0f)*(seed->a+seed->b+2.0f));

  // update parameters
  float mu_new = C1*m+C2*seed->mu;
  seed->sigma2 = C1*(s2 + m*m) + C2*(seed->sigma2 + seed->mu*seed->mu) - mu_new*mu_new;
  seed->mu = mu_new;
  seed->a = (e-f)/(f-e/f);
  seed->b = seed->a*(1.0f-f)/f;
}
void DynamicEstimation::updateSeedkalman(const float newz, const float newtau2, Seed* seed)
{
 float old_sigma2=seed->sigma2;
 float old_z=seed->mu;
seed->sigma2 =newtau2*seed->sigma2/(newtau2+seed->sigma2);
 seed->mu = newz/old_sigma2*seed->sigma2+old_z/newtau2*seed->sigma2;
}

bool DynamicEstimation::propragatedepth( FramePtr currentframe,const vector<cv::Point>&newdetect,FramePtr lastKF, std::list<Seed, aligned_allocator<Seed> > &allseeds,
                                                                   float newKeyframeMINdepth,float newKeyframeMEANdepth)
{
   if(mapconvergePoints.find(lastKF->id_)==mapconvergePoints.end())
   {
       cout<<" PropragateError, depth_filter.propragatedepth()"<<endl;
       return false;
   }
   Features new_features;
   for(auto it=newdetect.begin();it!=newdetect.end();it++)
       new_features.push_back(new Feature(currentframe.get(),Vector2d(it->x,it->y),0));
//用来记录上一个关键帧的所有种子投影至当前帧的　／位置／深度／方差／
  Mat SeedInlastKFprojectTothisKF=Mat::zeros(currentframe->img().size(),CV_32FC3);
   for(auto lastkfseed:mapconvergePoints[lastKF->id_].seeds_inthisKF)
   {
             //开机坐标系转换到上一个关键帧
            Vector3d P3D_in_lastkf( lastkfseed->ftr->f * (lastkfseed->mu));
            //开机坐标系转换到当前帧
           Vector3d P3D_in_cf(currentframe->T_f_w_*lastKF->T_f_w_.inverse()*P3D_in_lastkf);

           Vector2d uv_in_cf=currentframe->cam_->world2cam(P3D_in_cf);
                       if(currentframe->cam_->isInFrame(currentframe->f2c(P3D_in_cf).cast<int>()))
                       {
                           float depthIncurrentframe=P3D_in_cf[2];
                            SeedInlastKFprojectTothisKF.at<Vec3f>(uv_in_cf[1],uv_in_cf[0])[0]=1;//投到的坐标标志１
                            SeedInlastKFprojectTothisKF.at<Vec3f>(uv_in_cf[1],uv_in_cf[0])[1]=depthIncurrentframe;//投在当前帧上的深度
                            SeedInlastKFprojectTothisKF.at<Vec3f>(uv_in_cf[1],uv_in_cf[0])[2]=(float)depthIncurrentframe/P3D_in_lastkf[2]*lastkfseed->sigma2/4;   //传播到当前帧的不确定性
                    //        cout<<"   SeedInlastKFprojectTothisKF.at<Vec3b>(uv_in_cf[1],uv_in_cf[0])[2] "<< SeedInlastKFprojectTothisKF.at<Vec3f>(uv_in_cf[1],uv_in_cf[0])[2]<<endl;
                         //        cout<<"   SeedInlastKFprojectTothisKF.at<Vec3b>(uv_in_cf[1],uv_in_cf[0])[1] "<<depthIncurrentframe<<endl;
                       }
   }

   ++Seed::batch_counter;
   for(auto features:new_features)
   {
       if(SeedInlastKFprojectTothisKF.at<Vec3b>(features->px[1],features->px[0])[0]==1)
       {
           float depth_lastKF=SeedInlastKFprojectTothisKF.at<Vec3b>(features->px[1],features->px[0])[1];
           float sigma2_lastKF=SeedInlastKFprojectTothisKF.at<Vec3b>(features->px[1],features->px[0])[2];
           Seed *seed= new Seed(features,depth_lastKF , sigma2_lastKF,10,10);
             seeds_.push_back(*seed);
             mapconvergePoints[currentframe->id_].seeds_inthisKF.push_back(seed);
       }
       else
       {
           Seed *seed= new Seed(features ,newKeyframeMEANdepth);
             seeds_.push_back(*seed);
             mapconvergePoints[currentframe->id_].seeds_inthisKF.push_back(seed);
       }
   }
   if(options_.verbose)
     SVO_INFO_STREAM("DepthEstimation: Initialized "<<new_features.size()<<" new seeds");
}

int DynamicEstimation::ALLConvergePointNum=0;
void DynamicEstimation:: testshow()
 {
                   cout<<"ALLConvergePointNum "<<ALLConvergePointNum<<endl;

                     float convergeRatio=0.2;
                             //统计各个关键帧种子的收敛率
                             for(map<int, kfconvergmsg>::iterator it=mapconvergePoints.begin();it!=mapconvergePoints.end();++it)
                             {
                                 it->second.ConvergRatio=(float)it->second.Kfconvergpoints.size()/it->second.KeypointN;
                             }
                             //倒序查找满足收敛率的最新关键帧
                             for(auto it=mapconvergePoints.cend();it!=mapconvergePoints.cbegin();)
                             {      --it;//cend()返回迭代器前一个才是map最后一个元素。
                                     //cout<<"收敛率"<<it->second.ConvergRatio<<endl;
                                 if(it->second.ConvergRatio>convergeRatio){
                                     //送去显示
                                     {

                                         cv::Mat showmedian=cv::Mat::zeros(480,752,CV_8UC1);

                                             for(auto itt=it->second.Kfconvergpoints.begin();itt!=it->second.Kfconvergpoints.end();++itt)
                                             {       //关键帧上的深度
                                                      Vector3d P3D_in_kf((*itt)->ftr->frame->T_f_w_*(*itt)->point3d->pos_);
                                                      float depthInKF=P3D_in_kf[2];
                                                      showmedian.at<uchar>((*itt)->ftr->px[1],(*itt)->ftr->px[0])=depthInKF*20;
                                             }
                                             Mat color;
                                             GenerateFalseMap(showmedian,color);
                                             imshow("c",color);waitKey(5);
                                     }
                                   break;
                                 }
                             }
         }

}//end DynamicEstimation.cpp
