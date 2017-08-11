/******************************************************************************************************
本工程实现了两种三维点去噪算法，一种是基于三维点聚类的去噪算法，另一种是基于视差图的连通区域检测的去噪算法。
作者：朱保华
时间：2017/4/13

派生类 DepthConnectedcheck实现的是基于视差图的连通区域检测的去噪算法
*******************************************************************************************************/
#pragma once
#include<vector>
#include<deque>
#include<opencv2/opencv.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<highgui.h>
#include<math.h>
#include"Denoise3DPoint.h"
using namespace std;
namespace Point3D_Processing
{
	class DepthConnectedcheck:public Denoise3DPoint
	{
	public:
        DepthConnectedcheck(cam cam_,int minsize,int maxgap);
		template <typename T>
        void filterSpecklesImpl(cv::Mat& img, int newVal, int maxSpeckleSize, int maxDiff, cv::Mat& _buf);
        void Denose3DPoint(const cv::Mat& left,const cv::Mat&disp);
		void GetNoise3DPoint();//获取纯噪声图
		int minsize;//连通域最少包含像素的个数
		int maxgap;//
		cv::Mat buf_;
	};

}
