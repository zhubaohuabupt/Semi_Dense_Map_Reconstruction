/*This file is part of AntennaDetect-SemiDenseReconstruction.
 * VSLAMThread.cpp
 *
 *  Created on: 2017年5月26日
 *      Author: Zuber
 */
#include"VSLAMThread.h"
VSLAMThread:: VSLAMThread(const string& ConfigPath_,const string &VocPath_,ComData*ComData_)
 {
            this->ConfigPath=ConfigPath_;
            this->VocPath=VocPath_;
            this->pComData=ComData_;
 }
 void VSLAMThread::run()
 {
     SystemPattern  mSystemPattern=mOnlyDetect;
    ORB_SLAM2:: eState VSLAMstate_=ORB_SLAM2::NOT_READY;
    VSLAMstate mVSLAMstate=sIdle;

    int mode = 1;
   //vSLAM constructor
 ORB_SLAM2:: VSLAM vSLAM(this->VocPath, this->ConfigPath, mode, true);

     while(1)
     {

            //当系统模式为mOnlyDetect时，VSLAM线程休眠，否则开启.
            {
                 ulock lock(pComData->SystemPatternMutex);
                   if(mSystemPattern==mOnlyDetect)
                        pComData->startVOcond.wait(lock);
            }
//              int mode = 1;
//             //vSLAM constructor
//           ORB_SLAM2:: VSLAM vSLAM(this->VocPath, this->ConfigPath, mode, true);

             cv::Mat imleft;
             cv::Mat depth;
             vector<vector<cv::Point>> Antenna;
             bool isKeyFrame;
             cv::Mat pose;
           float SceneDepth[2];
           int FramedID=0;
                while(1)
                    {  
                        if(pComData->getpDetectedAntennaCacheLDA(imleft,depth,Antenna))
                            {
                                //帧率计算单元
                                 setFPS();
                                 //计算位姿
                                 vSLAM.doTracking(imleft, depth, 0, pose,isKeyFrame,SceneDepth);
                                 if((char)waitKey(2)=='r')
                                     vSLAM.requestSystemReset();
                                 if((char)waitKey(2)=='1')
                                     vSLAM.setMode(1);
                                 if((char)waitKey(2)=='2')
                                     vSLAM.setMode(2);
                                 if((char)waitKey(2)=='s')
                                     vSLAM.saveCovisibilityGraph();
                                 if((char)waitKey(2)=='q')
                                 {
                                     break;
                                 }

                              vSLAM.getState(VSLAMstate_);
                              if (VSLAMstate_==ORB_SLAM2::TRACKING)//////////////////////////
                                 mVSLAMstate=sTracking;
                              else
                                  mVSLAMstate=sLost;
                              //实时向公共区更新VSLAM状态
                              pComData->setVSLAMstate(mVSLAMstate);
                              //如果系统状态被其他线程设定为mOnlyDetect 或 VSLAM跟丢　 则停止本阶段深度估计
                             pComData->getSystemPattern(mSystemPattern);
                             if(mSystemPattern==mOnlyDetect||mVSLAMstate==sLost)
                             {
                                 //本阶段VSLAM结束，并把系统模式切换为mOnlyDetect，把VSLAM状态切换成空闲
                                   mVSLAMstate=sIdle;
                                 pComData->setVSLAMstate(mVSLAMstate);
                                 mSystemPattern=mOnlyDetect;
                                 pComData->setSystemPattern(mSystemPattern);
                                 pComData->setFPS_VSLAM(0);//写帧率
                                 //清空VSLAM缓存
                                 pComData->clearVSLAMCache();
                                 break;
                             }


                               pComData->setmdpVSLAMCache(FramedID,SceneDepth,isKeyFrame,imleft,depth,pose.inv(),Antenna);
                              ++FramedID;            
                     }
                }
vSLAM.requestSystemReset();
                //     vSLAM.shutDown();
         }
  }

 void VSLAMThread::startThread()
 {
this->pVSLAMThread=new std::thread (&VSLAMThread::run,this);
 }

 void VSLAMThread ::setFPS()
 {
     static int cnt=0;
     static std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
     if(cnt==10)
     {
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            float FPS= 10/std::chrono::duration_cast<std::chrono::duration<double> >(end - begin).count();
            pComData->setFPS_VSLAM(FPS);//写
            begin=std::chrono::steady_clock::now();
            cnt=0;
     }
    ++cnt;
 }
