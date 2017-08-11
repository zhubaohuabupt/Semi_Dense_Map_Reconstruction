#pragma once
#include<vector>
#include<opencv2/opencv.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv/highgui.h>
#include<math.h>
using namespace std;
namespace Point3D_Processing
{
	class BilateralInterPolation {
	public:

    BilateralInterPolation(float bf,float sigmaGeo_ , float sigmaCol_ , float winsize_ );
	
	inline float CalGrayDif(const cv::Mat& leftpic, const cv::Point& p1, const cv::Point &p2)
	{
		int colordif = leftpic.at<uchar>(p1.y, p1.x) - leftpic.at<uchar>(p2.y, p2.x);
		return colordif*colordif;
	}
	inline float CalGeoDif( const cv::Point& p1, const cv::Point &p2)
	{
		return ((p1.y-p2.y)*(p1.y-p2.y)+(p1.x-p2.x)*(p1.x-p2.x));
	}
	cv::Mat operator()(const cv::Mat& leftpic, const cv::Mat&SemiDenseDisp,const cv::Mat&DenoseDisp);
	public:
		float sigmaGeo;
		float sigmaCol;
		float winsize;
        float bf;
	};
}
