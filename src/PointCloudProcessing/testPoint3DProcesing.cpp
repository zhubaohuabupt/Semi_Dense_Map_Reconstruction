#include<iostream>
#include<deque>
#include<vector>
#include "Method_DensityCluster.h"
#include"Method_DepthConnectedcheck.h";
#include"BilateralInterPolation.h"
#include "runCamera2.h"
using namespace std;
using namespace cv;

  
     void GenerateFalseMap(cv::Mat &src, cv::Mat &disp);
	 
  int main(int argc, char **argv)
  {
      bool save = false;

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
          float   f = settings["Camera_fx"];
          float  cx = settings["Camera_cx"];
          float   cy = settings["Camera_cy"];

//          float bf = 387;
//          float f = 721;
//          float cx = 610;
//          float cy = 172;


          Mat left(height,width,CV_8UC1);
          Mat disp(height,width,CV_8UC1);
          Mat depth(height,width,CV_32FC1);
	  while(1){

        runCamera2(left, disp);
	// left = imread("E:\\wireline\\02\\01\\left_203.png", 0);
	 //disp= cv::imread("E:\\wireline\\02\\disp\\disp_203.png", 0);
	
	 Point3D_Processing::cam cam_(bf, f, cx, cy, left.cols, left.rows);
     Point3D_Processing::Point3D_Cluster method1(cam_,3, 30);
     Point3D_Processing::DepthConnectedcheck method2(cam_, 50, 10);
     Point3D_Processing::Denoise3DPoint *denose = &method1;
    //去噪
     denose->Denose3DPoint(left, disp);
     //获取结果
	 cv::Mat Denoise = denose->GetFalseColorDenoisedisp();
	 cv::Mat noise = denose->GetFalseColorNoisedisp();
	 cv::Mat Originaldisp = denose->GetFalseColorOriginaldisp();

     imshow("SemiDensePointCloudWithNoise ", Originaldisp);
     imshow("Noise ",noise);
     imshow("Denoised  ", Denoise);
    //平滑处理
     Point3D_Processing::BilateralInterPolation InterPolation(bf,100,100,10);
	 cv::Mat result = InterPolation(left, denose->SemiDenseDisp,denose->DeNoiseDisp),colorre;
	 GenerateFalseMap(result, colorre);
     imshow("Smoothing ", colorre);
	 
	 
	if(save)
	{
		 imwrite("左图.png ", left);
		 imwrite("原来半稠密深度图.png ", Originaldisp);
		 imwrite("纯噪声图.png ", noise);
		 imwrite("去噪图.png ", Denoise);
		 imwrite("插值后.png ", colorre);
	}
    cv::waitKey(10);
   }
  }


 
  void GenerateFalseMap(cv::Mat &src, cv::Mat &disp)
{
	  disp = cv::Mat::zeros(src.size(), CV_8UC3);
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
