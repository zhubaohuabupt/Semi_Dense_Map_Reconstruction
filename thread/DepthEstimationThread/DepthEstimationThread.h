/*This file is part of AntennaDetect-SemiDenseReconstruction.
 * DepthEstimationThread.h
 *
 *  Created on: 2017年6月2日
 *      Author: Zuber
 */
#include"ComData.h"
#include"DynamicEstimation.h"
#include"StaticEstimation.h"
class DepthEstimationThread
{
public:
        DepthEstimationThread(ComData*ComData_);
        void run();
        void startThread();
        Sophus::SE3 Mat2SE3(const cv::Mat& T);
        void setFPS( );
private:
        ComData*pComData;
        std::thread *pDepthEstimationThread;

};
