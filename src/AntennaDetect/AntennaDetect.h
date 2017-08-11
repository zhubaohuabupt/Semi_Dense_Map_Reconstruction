#ifndef ANTENNADETECT_
#define ANTENNADETECT_
/*This file is part of AntennaDetect.
 *
 *  Created on: 2016年10月22日
 *      Author: Zuber
 */
#include<iostream>
#include<opencv2/opencv.hpp>
#include <algorithm>
#include<cmath>
#include<vector>
using namespace std;
using namespace cv;

	struct region
	{
	 vector<Point> Pionts;
	 float neighbor_region_value_avg;
	 float neighbor_region_value_var;
	};
    class line_
    {
    public:
        vector<Point> PiontsofLine;
        float k;
        float b;
    };
	class Detect_Wireline
	{
	public:
		Detect_Wireline();

		 //函数功能：对输入灰度图的每一个像素点用sobel算子检测梯度，当该像素点的梯度大于阈值thred时，蓝色显示（第一通道为255）
		 void detect_grad(const Mat&grayinput,Mat&grad,float thred);
		
         //函数功能： 霍夫线段检测
         //输入：待检测图像
         //输出：检测线段显示图和线段上的所有点
         void houghcheck(const Mat&input,Mat&output,vector<line_>& pointsofLines_k);

         //函数功能：给定起始点坐标，画出直线。并把直线上所有点的坐标输出
         //输入：将要被绘制的图像、直线的起始点坐标
         //输出：绘制完毕的图像、直线上所有点的坐标。
         void putline_into_pic( Mat& pic,vector<Vec4i>& lines,vector<line_>& pointsofLines_k);


		//函数功能：从梯度图初步去除杂乱梯度区域。
		//输入:梯度图，待检测灰度图，检测窗口大小。
		//输出：去除后的图像
        void detectsmallobject(const Mat&gradinput,Mat&outputwithsamllobject,int windowsize);

        //函数功能：计算候选天线的斜率
        float CalLineSlope(const region&);
        //函数功能：斜率检测,排除斜率不符合要求的线
        void  SlopeCheck(const 	vector<region >& regions,vector<region >&AfterSlopeCheck);

		//函数功能：根据深度信息检测直线：把一些周围深度方差比较小并且深度均值比较小的直线去掉，因为这样的线很可能是天线影子
		//输入：待检测的直线、检测窗口大小、视差搜索最大值，深度图
		//输出: 检测后图像
        void  dispcheck( const vector<region> regions,const Mat&disp,Mat &out,vector<region>& regions_afterdispcheck,int windoesize=15,int Dmax=100);
        void  dispcheck( const vector<region> regions,const Mat&disp,vector<cv::Point>& Points,vector<region>& regions_afterdispcheck,int windowsize=15,int Dmax=100);
       void  depthcheck( const vector<region> regions,const Mat&depth,vector<vector<cv::Point>>& Points,vector<region>& regions_afterdispcheck,int windowsize=15,int Dmax=100);
		
		void  detect(const cv::Mat &inputsrc,const cv::Mat &disp,vector<cv::Point>& Points);
        void  detect_depth(const cv::Mat &inputsrc,const cv::Mat &depth,vector<vector<cv::Point>>& Points);
		 int width;
		 int height;
		 vector<Point> Allpoint_afterhough;
	     Mat graysrc;
		//函数功能：四舍五入取整
		inline int round(double r)  
		{  
			return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5);  
		} 
		//修改参数
		 void change_THRED_GRAD(int numyouwantchange);
		 void change_THRED_REMOVE_DISORDER(float numyouwantchange);
	    
		void change_WINDOWSIZE_REMOVE_DISORDER(int numyouwantchange);

		private:
			//梯度提取阈值
			int THRED_GRAD;
			//杂乱细节去除阈值
			float THRED_REMOVE_DISORDER;
			//深度抑制阈值
			int THRED_DEPTH_MEAN;
			int THRED_DEPTH_VAR;
		    //杂乱细节去除窗口大小
			int WINDOWSIZE_REMOVE_DISORDER;
	};
#endif
