/*This file is part of AntennaDetect-SemiDenseReconstruction.
 * ViewThread.h
 *
 *  Created on: 2017年5月25日
 *      Author: Zuber
 */
#include <pangolin/pangolin.h>
#include"ComData.h"
#include"Config.h"
class ViewThread
{
public:
  ViewThread(ComData*ComData_):pComData(ComData_){}
  void run();
  void startThread();
  void GenerateFalseMap(cv::Mat &src, cv::Mat &disp);
  void FLOAT2UCHAR(const cv::Mat&,cv::Mat &);
private:
std::thread *pViewThread;
ComData* pComData;
};
