 #include<chrono>
#include"Config.h"
void Points2Mat(const std::vector<std::vector<cv::Point>>&AntennaPoints,cv::Mat &AntennaMat)
{
    if(AntennaPoints.size()<1)
    {
       // std::cerr<<"天线检测vector容器为空"<<std::endl;
        return;
    }
    for(int i=0;i<AntennaPoints.size();i++)
        for(int j=0;j<AntennaPoints[i].size();j++)
     AntennaMat.at<uchar>(AntennaPoints[i][j].y,AntennaPoints[i][j].x)=255;
}


