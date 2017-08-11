/******************************************************************************************************
本工程实现了两种三维点去噪算法，一种是基于三维点聚类的去噪算法，另一种是基于视差图的连通区域检测的去噪算法。
作者：朱保华
时间：2017/4/13


*******************************************************************************************************/
#include"Method_DepthConnectedcheck.h"
namespace Point3D_Processing
{
    DepthConnectedcheck::DepthConnectedcheck(cam cam_,int minsize_, int maxgap_)
        :Denoise3DPoint(cam_), minsize(minsize_), maxgap(maxgap_) {}


void DepthConnectedcheck::Denose3DPoint(const cv::Mat&left,const cv::Mat &disp)
{
	getstrongtexture(left, disp, SemiDenseDisp);
	DeNoiseDisp = SemiDenseDisp.clone();
	filterSpecklesImpl<uchar>(DeNoiseDisp, 0, minsize, maxgap, buf_);//连通区域检测  检测后的去噪图像存储在DeNoiseDisp
	GetNoise3DPoint();//获取纯噪声图,存储在NoiseDisp

}

void DepthConnectedcheck::GetNoise3DPoint()
{
    NoiseDisp=cv::Mat::zeros(Cam_->height,Cam_->width,CV_8UC1);
	for(int row=0;row<Cam_->height;row++)
		for (int col = 0; col < Cam_->width; col++)
		{
			if (SemiDenseDisp.at<uchar>(row, col) != 0 && DeNoiseDisp.at<uchar>(row, col) == 0)
				NoiseDisp.at<uchar>(row, col) = SemiDenseDisp.at<uchar>(row, col);
		}
}

template <typename T>
void DepthConnectedcheck::filterSpecklesImpl(cv::Mat& img, int newVal, int maxSpeckleSize, int maxDiff, cv::Mat& _buf)
{
	//filterSpecklesImpl<short>(img, newVal, maxSpeckleSize, maxDiff, _buf); 
	using namespace cv;
	int width = img.cols, height = img.rows, npixels = width*height;
	size_t bufSize = npixels*(int)(sizeof(cv::Point_<short>) + sizeof(int) + sizeof(uchar)); //存了三部分东西
	if (!_buf.isContinuous() || !_buf.data || _buf.cols*_buf.rows*_buf.elemSize() < bufSize)
		_buf.create(1, (int)bufSize, CV_8U);
	uchar* buf = _buf.data;
	int i, j, dstep = (int)(img.step / sizeof(T));
	int* labels = (int*)buf;
	buf += npixels * sizeof(labels[0]);
	cv::Point_<short>* wbuf = (cv::Point_<short>*)buf;
	buf += npixels * sizeof(wbuf[0]);
	uchar* rtype = (uchar*)buf;
	int curlabel = 0;
	// clear out label assignments 
	memset(labels, 0, npixels * sizeof(labels[0]));
	for (i = 0; i < height; i++)
	{
		T* ds = img.ptr<T>(i);
		int* ls = labels + width*i;
		for (j = 0; j < width; j++)
		{
			if (ds[j] != newVal) // not a bad disparity 
			{
				if (ls[j]) // has a label, check for bad label 
				{

					if (rtype[ls[j]]) // small region, zero out disparity 
						ds[j] = (T)newVal;
				}
				// no label, assign and propagate 
				else
				{
					cv::Point_<short>* ws = wbuf; // initialize wavefront 
					cv::Point_<short> p((short)j, (short)i); // current pixel 
					curlabel++; // next label 
					int count = 0; // current region size 
					ls[j] = curlabel;
					// wavefront propagation 
					while (ws >= wbuf) // wavefront not empty 
					{
						count++;
						// put neighbors onto wavefront 
						T* dpp = &img.at<T>(p.y, p.x);
						T dp = *dpp; //p(x,y)的像素值
						int* lpp = labels + width*p.y + p.x;
						if (p.y < height - 1 && !lpp[+width] && dpp[+dstep] != newVal && std::abs(dp - dpp[+dstep]) <= maxDiff)
						{
							lpp[+width] = curlabel;
							*ws++ = cv::Point_<short>(p.x, p.y + 1);
						}
						if (p.y > 0 && !lpp[-width] && dpp[-dstep] != newVal && std::abs(dp - dpp[-dstep]) <= maxDiff)
						{
							lpp[-width] = curlabel;
							*ws++ = cv::Point_<short>(p.x, p.y - 1);
						}
						if (p.x < width - 1 && !lpp[+1] && dpp[+1] != newVal && std::abs(dp - dpp[+1]) <= maxDiff)
						{
							lpp[+1] = curlabel;
							*ws++ = cv::Point_<short>(p.x + 1, p.y);
						}
						if (p.x > 0 && !lpp[-1] && dpp[-1] != newVal && std::abs(dp - dpp[-1]) <= maxDiff)
						{
							lpp[-1] = curlabel;
							*ws++ = cv::Point_<short>(p.x - 1, p.y);
						}
						// pop most recent and propagate 
						// NB: could try least recent, maybe better convergence 
						p = *--ws;
					}

					// assign label type 
					if (count <= maxSpeckleSize) // speckle region 
					{
						rtype[ls[j]] = 1; // small region label 
						ds[j] = (T)newVal;
					}
					else
						rtype[ls[j]] = 0; // large region label 
				}
			}
		}

	}
}

}
