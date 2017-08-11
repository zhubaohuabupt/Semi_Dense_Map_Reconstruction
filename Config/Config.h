#ifndef CONFIG_
#define CONFIG_

#define Neighborframesize 10
#define KeyframeIDinNeighborWindow 5
#include<vector>
#include<cv.h>
#include"opencv2/opencv.hpp"
void Points2Mat(const std::vector<std::vector<cv::Point>>&AntennaPoints,cv::Mat &AntennaMat);
#endif
