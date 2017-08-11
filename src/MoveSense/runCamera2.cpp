#include "Camera.h"
#include "CameraCmd.h"
#include <iostream>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include "MoveSenseCamera2.h"

#define FRAMERATE_MESSURE 0

using namespace std;
using namespace cv;
int sel = CAM_STEREO_752X480_LRD_30FPS;
movesense::MoveSenseCamera c(sel);

int alpha_slider_1 = 40;   //desire_bin
int alpha_slider_2 = 4;   //P1
int alpha_slider_3 = 16;   //P2_min
int alpha_slider_4 = 32;  //P2_Gamma

void setCamera2Parameter(int alpha_slider_1=40,int alpha_slider_2=4,int alpha_slider_3=16,int alpha_slider_4=32){
	c.SetDesireBin(alpha_slider_1);
	c.SetSM_P1(alpha_slider_2);
	c.SetSM_P2(alpha_slider_3);
	c.SetSM_P3(alpha_slider_4);
	c.OpenCamera();
}

void closeCamera(){
	c.CloseCamera();
}

bool first=true;

int runCamera2(Mat &left1,Mat &disp1)
{
	if(first){
		setCamera2Parameter( alpha_slider_1=40, alpha_slider_2=4, alpha_slider_3=16, alpha_slider_4=32);
		first=false;
	}
	static unsigned char *data_752x480_lr  = new unsigned char[752*480*2];
	static unsigned char *data_752x480_lrd = new unsigned char[752*480*3];
	static unsigned char *data_376x240_lr  = new unsigned char[376*240*2];
	static unsigned char *data_376x240_lrd = new unsigned char[376*240*3];


	static int wait_key = 3;
	static bool save = false;
	static int frame = 0;

		int len_752x480_lr  = 752*480*2;
		int len_752x480_lrd = 752*480*3;
		int len_376x240_lr  = 376*240*2;
		int len_376x240_lrd = 376*240*3;
		cv::Mat left(240,376,CV_8UC1),right(240,376,CV_8UC1),dip(240,376,CV_8UC1);

		//cv::Mat left1(480,752,CV_8UC1);
        cv::Mat right1(480,752,CV_8UC1);
        //cv::Mat dip1(480,752,CV_8UC1);

		if (sel >= 190 && sel <= 194)
		{
			c.GetImageData(data_376x240_lrd ,len_376x240_lrd);
			for(int i = 0 ; i < 240; i++)
			{
				memcpy(left.data+376*i,data_376x240_lrd+i*376*3,376);
				memcpy(right.data+376*i,data_376x240_lrd+i*376*3+376,376);
				memcpy(dip.data+376*i,data_376x240_lrd+i*376*3+376*2,376);
			}
			//if(isDisp) medianBlur(right,right,3);
			cv::imshow("left",left);
			cv::imshow("right",right);
			cv::imshow("disp",dip);

		}
		else if(sel >= 195 && sel <= 204)
		{
			c.GetImageData(data_376x240_lr ,len_376x240_lr);
			for(int i = 0 ; i < 240; i++)
			{
				memcpy(left.data+376*i,data_376x240_lr+i*376*2,376);
				memcpy(right.data+376*i,data_376x240_lr+i*376*2+376,376);
			}
			//if(isDisp) medianBlur(right,right,3);
			cv::imshow("left",left);
			cv::imshow("right",right);
		}
		else if(sel >= 206 && sel <= 210)
		{
			c.GetImageData(data_752x480_lrd ,len_752x480_lrd);
			for(int i = 0 ; i < 480; i++)
			{
				memcpy(left1.data+752*i,data_752x480_lrd+i*752*3,752);
				memcpy(right1.data+752*i,data_752x480_lrd+i*752*3+752,752);
                memcpy(disp1.data+752*i,data_752x480_lrd+i*752*3+752*2,752);
			}
           // imwrite("/home/baohua/lefttt.png",left1);
           // imwrite("/home/baohua/righttt.png",right1);
            //waitKey(500);
           // cv::imshow("left",left1);
           // cv::imshow("right",right1);
          //  cv::imshow("disp",disp1);   waitKey(500);
		}
		else
		{
			c.GetImageData(data_752x480_lr ,len_752x480_lr);
			for(int i = 0 ; i < 480; i++)
			{
				memcpy(left1.data+752*i,data_752x480_lr+i*752*2,752);
                memcpy(disp1.data+752*i,data_752x480_lr+i*752*2+752,752);
			}
		}

		char key = cv::waitKey(wait_key);
		if(save)
		{
			char name_l[100];
			char name_r[100];
			sprintf(name_l,"data/01/left_%d.bmp",frame);
			sprintf(name_r,"data/02/right_%d.bmp",frame);
			imwrite(name_l,left1);
			imwrite(name_r,right1);
			frame++;
			//save = false;
		}
		if(key == 'S')
		{
			save = true;
		}else if(key == 'A')
		{
			c.SetUndistort(true);
		}else if(key == 'B')
		{
			c.SetUndistort(false);
		}else if(key == 'E')
		{
			c.SetSM_HoleFill(true);
		}else if(key == 'F')
		{
			c.SetSM_HoleFill(false);
		}else if(key == 'G')
		{
			c.SetSM_LRCheck(true);
		}else if(key == 'H')
		{
			c.SetSM_LRCheck(false);
		}else if(key == 'I')
		{
			c.SetSM_Subpixel(true);
		}else if(key == 'J')
		{
			c.SetSM_Subpixel(false);
		}else if(key == 'K')
		{
			c.SetHDR(true);
		}else if(key == 'L')
		{
			c.SetHDR(false);
		}else if(key == 'O')
		{
			c.SetSM_MedianFilter(true);
		}else if(key == 'P')
		{
			c.SetSM_MedianFilter(false);
		}else if(key == 'M')
		{
			wait_key = (wait_key==10)?0:10;
		}
//	}
	return 0;
}
