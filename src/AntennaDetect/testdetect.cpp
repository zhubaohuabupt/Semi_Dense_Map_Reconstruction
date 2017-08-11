/*This file is part of AntennaDetect.
 *
 *  Created on: 2016年10月22日
 *      Author: Zuber
 */
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<sstream>
#include<opencv2/core/core.hpp>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include "runCamera2.h"
#include "AntennaDetect.h"
using namespace std;
using namespace cv;
void GenerateFalseMap(cv::Mat &src, cv::Mat &disp)  
{  
    // color map  
    float max_val = 255.0f;  
    float map[8][4] = {{0,0,0,114},{0,0,1,185},{1,0,0,114},{1,0,1,174},  
                       {0,1,0,114},{0,1,1,185},{1,1,0,114},{1,1,1,0}};  
    float sum = 0;  
    for (int i=0; i<8; i++)  
      sum += map[i][3];  
  
    float weights[8]; // relative   weights  
    float cumsum[8];  // cumulative weights  
    cumsum[0] = 0;  
    for (int i=0; i<7; i++) {  
      weights[i]  = sum/map[i][3];
      cumsum[i+1] = cumsum[i] + map[i][3]/sum;  
    }  
  
    int height_ = src.rows;  
    int width_ = src.cols;  
    // for all pixels do  
    for (int v=0; v<height_; v++) {  
      for (int u=0; u<width_; u++) {  
  
        // get normalized value  
        float val = std::min(std::max(src.data[v*width_ + u]/max_val,0.0f),1.0f);  
  
        // find bin  
        int i;  
        for (i=0; i<7; i++)  
          if (val<cumsum[i+1])  
            break;  
  
        // compute red/green/blue values  
        float   w = 1.0-(val-cumsum[i])*weights[i];  
        uchar r = (uchar)((w*map[i][0]+(1.0-w)*map[i+1][0]) * 255.0);  
        uchar g = (uchar)((w*map[i][1]+(1.0-w)*map[i+1][1]) * 255.0);  
        uchar b = (uchar)((w*map[i][2]+(1.0-w)*map[i+1][2]) * 255.0);  

        //rgb内存连续存放  
        disp.data[v*width_*3 + 3*u + 0] = b;  
        disp.data[v*width_*3 + 3*u + 1] = g;  
        disp.data[v*width_*3 + 3*u + 2] = r;  
      }  
    }  
}
int main(int argc, char **argv)
{
    if(argc != 2)
    {
        cerr << endl << "Wrong input format!" << endl;
        return 1;
    }

    string SettingPath = argv[1];
    cv::FileStorage settings(SettingPath, cv::FileStorage::READ);
    float width = settings["Camera_width"];
    float height = settings["Camera_height"];
    float bf = settings["Camera_bf"];

    Mat img_left(height,width,CV_8UC1);
    Mat img_disp(height,width,CV_8UC1);
    Mat depth(height,width,CV_32FC1);
Detect_Wireline  detect_sm;
//给相机初始化留出时间
	int i=0;

	while(i++<30)
	{
      runCamera2(img_left, img_disp);
	waitKey(10);
	}

    while(1)
    {
	
        runCamera2(img_left, img_disp);
      //  img_left=imread("/media/baohua/media/wireline/02/01/left_277.png");
        // img_disp=imread("/media/baohua/media/wireline/02/disp/disp_277.png");
vector<vector<cv::Point> >smallthing;

 //imshow("dispcolor",img_disp);
 //imshow("left",img_left);
// waitKey(0);
//img_left=imread("/media/baohua/media/wireline/05/01/left_280.png",0);
  // img_disp=imread("/media/baohua/media/wireline/05/disp/disp_280.png",0);

for(int row=0;row<height;row++)
    for(int col=0;col<width;col++)
    {
        if(img_disp.at<uchar>(row,col)<128&&img_disp.at<uchar>(row,col)!=0)
         depth.at<float>(row,col)  = bf/(img_disp.at<uchar>(row,col)/4);
        else if(img_disp.at<uchar>(row,col)>=128)
        depth.at<float>(row,col) = bf/(img_disp.at<uchar>(row,col)/2-32);
        else
       depth.at<float>(row,col)=0;
    }

//显示彩色视差图
Mat dispcolor(img_disp.size(),CV_8UC3);
GenerateFalseMap(img_disp,dispcolor);
 imshow("dispcolor",dispcolor);
detect_sm.detect_depth(img_left,depth,smallthing);

      

 	Mat SHOW=Mat::zeros(img_disp.rows,img_disp.cols,CV_8UC1);
    for(int i=0;i<smallthing.size();i++) {
        for(int j=0;j<smallthing[i].size();j++)
        SHOW.at<uchar>(smallthing[i][j].y, smallthing[i][j].x)=255;
	   }

    imshow("WIRL",SHOW);
    imshow("left",img_left);
waitKey( 5);
//imwrite("/home/baohua/color.png",dispcolor);
//imwrite("/home/baohua/img_left.png",img_left);
//imwrite("/home/baohua/WIRL.png",SHOW);


    }
return 0;
}

