/******************************************************************************************************
本工程实现了两种三维点去噪算法，一种是基于三维点聚类的去噪算法，另一种是基于视差图的连通区域检测的去噪算法。
作者：朱保华
时间：2017/4/13
*******************************************************************************************************/
#include "Method_DensityCluster.h"
#include<map>
namespace Point3D_Processing
{

Point3D_Cluster::Point3D_Cluster(cam cam_,float Radiusratio_,double MinPtsInWindow_)
    :Denoise3DPoint( cam_),Radiusratio(Radiusratio_), MinPtsInWindow(MinPtsInWindow_),classnum(1), showAllcluster(false)
{

}
Point3D_Cluster::~Point3D_Cluster(void) {}
void Point3D_Cluster::Get3dPoint( vector< vector<Point_3D> >&All3DPoint)
{
    cv::Mat depth;
    disptodepth(disp, Cam_->bf, depth);
    getstrongtexture(left, disp, SemiDenseDisp);
    for (int row = 0; row<SemiDenseDisp.rows; row++)
    {
        vector<Point3D_Processing::Point_3D> row3dpoint;
        for (int col = 0; col<SemiDenseDisp.cols; col++)
        {
            if (SemiDenseDisp.at<uchar>(row, col) == 0)
                row3dpoint.push_back(Point3D_Processing::Point_3D(cv::Point(-1, -1), cv::Scalar(0, 0, 0)));
            else
            {
                float z = depth.at<float>(row, col);
                float x = z*(col - Cam_->cx) / Cam_->f;
                float y = z*(row - Cam_->cy) / Cam_->f;
                row3dpoint.push_back(Point3D_Processing::Point_3D(cv::Point(col, row), cv::Scalar(x, y, z)));
            }
        }
        All3DPoint.push_back(row3dpoint);
    }
}
bool Point3D_Cluster::IsCorePoint(Point_3D &P, vector<vector<Point_3D>>& All3DPoint,deque<Point_3D*>&NeighborPointdeque)
{
	
	
	int num=0;
	vector<Point_3D*> thispointneighbor;
	int x=P.Point2D.x;
	int y=P.Point2D.y;
    int halfwinsize=5 ;
	int colbegin=x-halfwinsize>0?x-halfwinsize:0;
	int colend= disp.cols-1<x+halfwinsize? disp.cols-1:x+halfwinsize;
	int rowbegin=y-halfwinsize>0?y-halfwinsize:0;
	int rowend= disp.rows-1<y+halfwinsize? disp.rows-1:y+halfwinsize;

	for(int row=rowbegin;row<rowend;row++)
		for(int col=colbegin;col<colend;col++)
		{
			if(All3DPoint[row][col].Point2D.x==-1) continue;
			float dist=CaculatePointDistance(P,All3DPoint[row][col]);
			//if(dist<P.Point3D.val[2]*Radiusratio&&dist!=0)//      密度可达             dist==0 意思是该点本身
			//据深度分辨率自适应调节邻居半径
			if (dist<CaculateDepthResolution(disp.at<uchar>(P.Point2D.y, P.Point2D.x)) * Radiusratio&&dist != 0)//密度可达      dist==0 意思是该点本身
			{
				num++;
				if(All3DPoint[row][col].classID==-1)//未被处理
				thispointneighbor.push_back(&All3DPoint[row][col]);			
					
			}
		}		
	//cout<<"tatol neighbor  "<<num<<endl;
	if(num>MinPtsInWindow)
	{            
		for(auto it=thispointneighbor.begin();it<thispointneighbor.end();it++)//核心点的密度可达邻居
		{
			(*it)->classID=classnum;
			NeighborPointdeque.push_back(*it);

		}
				//cout<<"new push  "<<thispointneighbor.size()<<endl;
		
		return true;
	}
	else
		return false;

}
void Point3D_Cluster::runcluster(vector<vector<Point_3D>>&All3DPoint)
{
	for(int i=0;i<All3DPoint.size();i++)
	for(auto it=All3DPoint[i].begin();it<All3DPoint[i].end();it++)
	{ 
	  if(it->Point2D.x==-1) continue;
		deque<Point_3D*> NeighborPointdeque;
																										
		if(it->Pointproperty!=havenotprocess)//判断从集合中取出的点是否被标记，如果标记了，代表着已经处理过了。不再处理
			{
				
				continue;
			}
		else
			{
				if(IsCorePoint(*it,All3DPoint,NeighborPointdeque))
					{
					
						it->Pointproperty=iscorepoint;
						it->classID=classnum;
														//	cout<<"NeighborPointdeque.size()"<<NeighborPointdeque.size()<<endl;
				    	
						while (!NeighborPointdeque.empty())
						{


							Point_3D* PointfromDeque = NeighborPointdeque.front();//从队列中取邻域点


							if (IsCorePoint(*PointfromDeque, All3DPoint, NeighborPointdeque))
							{
								PointfromDeque->Pointproperty = iscorepoint;//标记处理过的点为核心点
								PointfromDeque->classID = classnum;
							}
							else
							{//获取纯噪声图, 存储在NoiseDisp
								NoiseDisp.at<uchar>(PointfromDeque->Point2D.y, PointfromDeque->Point2D.x) = disp.at<uchar>(PointfromDeque->Point2D.y, PointfromDeque->Point2D.x);;
								PointfromDeque->Pointproperty = isborderpoint;//标记处理过的点为噪点
								PointfromDeque->classID = 0;
							}
							NeighborPointdeque.pop_front();
						}
						if (NeighborPointdeque.empty())
							classnum++;
					}
		
				else
					{
					//获取纯噪声图, 存储在NoiseDisp
					NoiseDisp.at<uchar>(it->Point2D.y,it->Point2D.x)= disp.at<uchar>(it->Point2D.y, it->Point2D.x);
						it->Pointproperty=isborderpoint;//标记处理过的点为噪点
						it->classID=0;
					}
			}
	}
	for (int i = 0; i<All3DPoint.size(); i++)
		for (int j = 0; j<All3DPoint[i].size(); j++)
		{
			if (All3DPoint[i][j].classID == 0)
					continue;
			else // 检测后的去噪图像存储在DeNoiseDisp
				DeNoiseDisp.at<uchar>(All3DPoint[i][j].Point2D.y, All3DPoint[i][j].Point2D.x) = disp.at<uchar>(All3DPoint[i][j].Point2D.y, All3DPoint[i][j].Point2D.x);
		}
	if(showAllcluster)
	{ 
		 allcluster=showEveryCluster3DPiont(All3DPoint);
		 imshow("allcluster", allcluster); 
		 cvWaitKey(10);
		 imwrite("聚类结果图.png", allcluster);
	}																			//cout<<"classnum" <<classnum<<endl;
																				/*int noisecnt=0,valuedata=0;
																				for(int i=0;i<All3DPoint.size();i++)
																				for(auto it=All3DPoint[i].begin();it<All3DPoint[i].end();it++)
																				{
																					if(it->Pointproperty==1)
																					noisecnt++;
																					if(it->Pointproperty==2)
																						valuedata++;
																				}
																				cout<<"noisecnt   "<<noisecnt<<endl;
																				cout<<"valuedata  "<<valuedata<<endl;*/	

}


void Point3D_Cluster::Denose3DPoint(const cv::Mat& left_,const cv::Mat& disp_)
{
    NoiseDisp = cv::Mat::zeros(Cam_->height, Cam_->width, CV_8UC1);
    DeNoiseDisp = cv::Mat::zeros(Cam_->height, Cam_->width, CV_8UC1);
    vector< vector<Point_3D> >All3DPoint;
    this->left=left_.clone();
    this->disp=disp_.clone();
    Get3dPoint(All3DPoint);
	runcluster(All3DPoint);
}
cv::Mat Point3D_Cluster::showEveryCluster3DPiont(const vector<vector<Point_3D>>&All3DPoint)
{
	vector <int >  colormap;
	for (int i = 0; i < classnum; i++)//灰度分级
		colormap.push_back((unsigned char)(255/ 25)*i);
	
	cv::Mat showcolor = cv::Mat::zeros(Cam_->height, Cam_->width,CV_8UC1);
	for (int i = 0; i < All3DPoint.size(); i++)
		for (int j = 0; j < All3DPoint[i].size(); j++)
		{
			if (All3DPoint[i][j].classID == -1 || All3DPoint[i][j].classID == 0) continue;
			showcolor.at<uchar>(All3DPoint[i][j].Point2D.y, All3DPoint[i][j].Point2D.x) = colormap[All3DPoint[i][j].classID]; 
		}
	
	cv::Mat color;
	GenerateFalseMap(showcolor, color);
	return color.clone();
}
}
