/******************************************************************************************************
本工程实现了两种三维点去噪算法，一种是基于三维点聚类的去噪算法，另一种是基于视差图的连通区域检测的去噪算法。
作者：朱保华
时间：2017/4/13
Denoise3DPoint 基类
Point3D_Cluster和DepthConnectedcheck是其两个派生类，分别实现不同算法。
输入参数：左图 视差图 相机参数 处理参数
输出：原半稠密视差图  去噪后的视差图  噪声图
*******************************************************************************************************/
#pragma once
#include<vector>
#include<opencv2/opencv.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<highgui.h>
#include<math.h>
using namespace std;
namespace Point3D_Processing
{
	enum _Pointproperty { havenotprocess = 0, isborderpoint = 1, iscorepoint = 2 };
	class Point_3D
	{

	public:
		Point_3D(const cv::Point& Point2D_, const cv::Scalar& Point3D_);
		cv::Point Point2D;
		cv::Scalar Point3D;
		_Pointproperty Pointproperty;
		int classID;
	};
	class cam
	{
	public:
		cam(float bf_, float f_, float cx_, float cy_, int width_, int height_) :bf(bf_), f(f_), cx(cx_), cy(cy_), width(width_), height(height_) {}
		const float bf;
		const float f;
		const float cx;
		const float cy;
		const int width;
		const int height;
	};
	class Denoise3DPoint
	{
	public:
        Denoise3DPoint(cam cam_);
		~Denoise3DPoint();
		cam *Cam_;
		void disptodepth(const cv::Mat&disp, float bf, cv::Mat& depth);
        void getstrongtexture(const cv::Mat&left, const cv::Mat&disp, cv::Mat&text);//获取强纹理区域
		cv::Mat GetFalseColorDenoisedisp();//获取去噪图
		cv::Mat GetFalseColorNoisedisp();//获取纯噪声图
		cv::Mat GetFalseColorOriginaldisp();//获取原来半稠密深度图

        virtual void Denose3DPoint(const cv::Mat&left,const cv::Mat &disp_) = 0;//去噪处理  公共接口
		cv::Mat left;
		cv::Mat disp;
		cv::Mat SemiDenseDisp;//强纹理深度图
		cv::Mat NoiseDisp;//纯噪声图
		cv::Mat DeNoiseDisp;//去噪后的图
	};
	void GenerateFalseMap(cv::Mat &src, cv::Mat &disp);
    void dispTodepthShow(const cv::Mat&Depthfloat,float bf,cv::Mat &Depthuchar);
	
}
