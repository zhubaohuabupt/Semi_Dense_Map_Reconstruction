/*This file is part of AntennaDetect-SemiDenseReconstruction.
 * VSLAMThread.h
 *
 *  Created on: 2017年5月26日
 *      Author: Zuber
 */
#include "VSLAM.h"
#include "ComData.h"
#include <iostream>

class VSLAMThread
{
public:
      VSLAMThread(const string& ConfigPath,const string &VocPath,ComData*ComData_);
      void run();
      void startThread();
      void  setFPS();
private:
       std:: string ConfigPath;
      std:: string VocPath;
      ComData* pComData;
      std::thread* pVSLAMThread;
};
