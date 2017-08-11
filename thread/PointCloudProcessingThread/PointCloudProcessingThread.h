/*This file is part of AntennaDetect-SemiDenseReconstruction.
 * PointCloudProcessingThread.h
 *
 *  Created on: 2017年5月24日
 *      Author: Zuber
 */
#include "Method_DensityCluster.h"
#include"Method_DepthConnectedcheck.h";
#include"BilateralInterPolation.h"
#include"ComData.h"
#include<iostream>
class PointCloudProcessingThread
{
public:
      PointCloudProcessingThread(ComData*ComData_):pComData(ComData_){}
      void run();
      void startThread();
      void  setFPS();
private:
      ComData* pComData;
      std::thread * pPointCloudProcessingThread;
};
