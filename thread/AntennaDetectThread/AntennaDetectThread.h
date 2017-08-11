/*This file is part of AntennaDetect-SemiDenseReconstruction.
 * AntennaDetectThread.h
 *
 *  Created on: 2017年5月24日
 *      Author: Zuber
 */
#include<iostream>
#include"AntennaDetect.h"
#include"ComData.h"
class  AntennaDetectThread
{
public:
    AntennaDetectThread(ComData * ComData_):pComData(ComData_){}
    void run();
    void startThread();
    void setFPS( );
private:
      std::thread * pAntennaDetectThread;
      ComData * pComData;
};
