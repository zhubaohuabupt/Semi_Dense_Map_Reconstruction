/******************************************************************************************************
本工程实现了两种三维点去噪算法，一种是基于三维点聚类的去噪算法，另一种是基于视差图的连通区域检测的去噪算法。
作者：朱保华
时间：2017/4/13
*******************************************************************************************************/
#include"Denoise3DPoint.h"
namespace Point3D_Processing
{

	Point_3D::Point_3D(const cv::Point& Point2D_, const cv::Scalar& Point3D_) :
		Point2D(Point2D_), Point3D(Point3D_), Pointproperty(havenotprocess), classID(-1) {}

    Denoise3DPoint::Denoise3DPoint(cam cam_)
	{

		Cam_ = new cam(cam_.bf, cam_.f, cam_.cx, cam_.cy, cam_.width, cam_.height);
        NoiseDisp = cv::Mat::zeros(cam_.height, cam_.width, CV_8UC1);
        DeNoiseDisp = cv::Mat::zeros(cam_.height, cam_.width, CV_8UC1);
	}
	Denoise3DPoint::~Denoise3DPoint()
	{
		delete Cam_;
	}

    void Denoise3DPoint::disptodepth(const cv::Mat&dispimg, float bf, cv::Mat& depthimg)
	{
        depthimg = cv::Mat::zeros(dispimg.size(), CV_32FC1);
           int height=dispimg.rows;
           int width=dispimg.cols;

           for(int row=0;row<height;row++)
               for(int col=0;col<width;col++)
               {

                   if(dispimg.at<uchar>(row,col)<128&&dispimg.at<uchar>(row,col)!=0)
                    depthimg.at<float>(row,col)  = (float)bf/dispimg.at<uchar>(row,col)*4;

                   else if(dispimg.at<uchar>(row,col)>=128)
                   depthimg.at<float>(row,col) = bf/(dispimg.at<uchar>(row,col)/2-32);

                   else
                  depthimg.at<float>(row,col)=0;
               }
	}
    void Denoise3DPoint::getstrongtexture(const cv::Mat&left, const cv::Mat&disp, cv::Mat&strongtexture)
    {
        strongtexture = cv::Mat::zeros(left.size(), CV_8UC1);
        cv::Mat dst_x, dst_y, dst;

        Sobel(left, dst_x, left.depth(), 1, 0);
        Sobel(left, dst_y, left.depth(), 0, 1);
        convertScaleAbs(dst_x, dst_x);
        convertScaleAbs(dst_y, dst_y);
        addWeighted(dst_x, 0.5, dst_y, 0.5, 0, dst);

        for (int row = 0; row<disp.rows; row++)
            for (int col = 0; col<disp.cols; col++)
            {
                if (dst.at<uchar>(row, col)>=5 && disp.at<uchar>(row, col) != 0)
                    strongtexture.at<uchar>(row, col) = disp.at<uchar>(row, col);
            }
    }
	cv::Mat Denoise3DPoint::GetFalseColorDenoisedisp()
	{
        cv::Mat color,depthuchar;
        dispTodepthShow(DeNoiseDisp, Cam_->bf,depthuchar);
        GenerateFalseMap(depthuchar, color);
        return color.clone();
	}
	cv::Mat Denoise3DPoint::GetFalseColorNoisedisp()
	{
        cv::Mat color,depthuchar;
        dispTodepthShow(NoiseDisp, Cam_->bf,depthuchar);
        GenerateFalseMap(depthuchar, color);
        return color.clone();
	}
	cv::Mat Denoise3DPoint::GetFalseColorOriginaldisp()
	{
        cv::Mat color, depthuchar;
        dispTodepthShow(SemiDenseDisp,Cam_->bf,depthuchar);
        GenerateFalseMap(depthuchar, color);
        return color.clone();
	}
    void dispTodepthShow(const cv::Mat&dispimg,float bf,cv::Mat &Depthuchar)
    {

       Depthuchar=cv::Mat::zeros(dispimg.size(),CV_8UC1);
       int height=dispimg.rows;
       int width=dispimg.cols;
       for(int row=0;row<height;row++)
           for(int col=0;col<width;col++)
           {
               if(dispimg.at<uchar>(row,col)<128&&dispimg.at<uchar>(row,col)!=0)
                Depthuchar.at<uchar>(row,col)  = 10*bf/(float)dispimg.at<uchar>(row,col)*4;
               else if(dispimg.at<uchar>(row,col)>=128)
               Depthuchar.at<uchar>(row,col) = 10*bf/(float)(dispimg.at<uchar>(row,col)/2-32);
               else
              Depthuchar.at<uchar>(row,col)=0;
            }
    }

    void GenerateFalseMap(cv::Mat &src, cv::Mat &disp)
	{
		disp = cv::Mat::zeros(src.size(), CV_8UC3);
		// color map
		float max_val = 255.0f;
		float map[8][4] = { { 0,0,0,114 },{ 0,0,1,185 },{ 1,0,0,114 },{ 1,0,1,174 },
		{ 0,1,0,114 },{ 0,1,1,185 },{ 1,1,0,114 },{ 1,1,1,0 } };
		float sum = 0;
		for (int i = 0; i<8; i++)
			sum += map[i][3];

		float weights[8]; // relative   weights
		float cumsum[8];  // cumulative weights
		cumsum[0] = 0;
		for (int i = 0; i<7; i++) {
			weights[i] = sum / map[i][3];
			cumsum[i + 1] = cumsum[i] + map[i][3] / sum;
		}

		int height_ = src.rows;
		int width_ = src.cols;
		// for all pixels do
		for (int v = 0; v<height_; v++) {
			for (int u = 0; u<width_; u++) {

				// get normalized value
				float val = std::min(std::max(src.data[v*width_ + u] / max_val, 0.0f), 1.0f);

				// find bin
				int i;
				for (i = 0; i<7; i++)
					if (val<cumsum[i + 1])
						break;

				// compute red/green/blue values
				float   w = 1.0 - (val - cumsum[i])*weights[i];
				uchar r = (uchar)((w*map[i][0] + (1.0 - w)*map[i + 1][0]) * 255.0);
				uchar g = (uchar)((w*map[i][1] + (1.0 - w)*map[i + 1][1]) * 255.0);
				uchar b = (uchar)((w*map[i][2] + (1.0 - w)*map[i + 1][2]) * 255.0);
				//rgb内存连续存放
				disp.data[v*width_ * 3 + 3 * u + 0] = b;
				disp.data[v*width_ * 3 + 3 * u + 1] = g;
				disp.data[v*width_ * 3 + 3 * u + 2] = r;
			}
		}
	}
}
