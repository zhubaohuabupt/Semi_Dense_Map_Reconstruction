/******************************************************************************************************
本工程实现了两种三维点去噪算法，一种是基于三维点聚类的去噪算法，另一种是基于视差图的连通区域检测的去噪算法。
作者：朱保华
时间：2017/4/13

派生类 Point3D_Cluster实现的是基于三维点聚类的去噪算法
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

//三维点密度聚类去噪
class Point3D_Cluster:public Denoise3DPoint
{
	
public:
    Point3D_Cluster(cam cam_, float Radiusratio_, double MinPtsInWindow_);

	~Point3D_Cluster(void);
	//是否是核心点（点密度大）：
	bool IsCorePoint(Point_3D &P, vector<vector<Point_3D>>& All3DPoint,deque<Point_3D*>&NeighborPointdeque);

	inline float CaculatePointDistance(const Point_3D& P1,const Point_3D& P2)
	{
		return sqrt(    (P1.Point3D.val[0]-P2.Point3D.val[0])*(P1.Point3D.val[0]-P2.Point3D.val[0])+
						(P1.Point3D.val[1]-P2.Point3D.val[1])*(P1.Point3D.val[1]-P2.Point3D.val[1])+
						(P1.Point3D.val[2]-P2.Point3D.val[2])*(P1.Point3D.val[2]-P2.Point3D.val[2]) );
	}
	inline float CaculateDepthResolution(int d)
	{
		float bf = Cam_->bf;
		return (bf / d - bf / (d + 1));
	}
    void Get3dPoint(vector< vector<Point_3D> >&All3DPoint);//获取三维点
    void runcluster(vector<vector<Point_3D>>&All3DPoint);
    void Denose3DPoint(  const cv::Mat&left,const cv::Mat& disp);
    cv::Mat showEveryCluster3DPiont(const vector<vector<Point_3D>>&All3DPoint);
    int classnum;
	bool showAllcluster;
    cv::Mat allcluster;

private:
	float  Radiusratio;//半径控制
	double MinPtsInWindow;//点数控制
};
//去噪



}
