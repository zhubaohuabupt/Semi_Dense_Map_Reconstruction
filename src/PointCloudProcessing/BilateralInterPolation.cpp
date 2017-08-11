#include"BilateralInterPolation.h"
#include"Denoise3DPoint.h"
namespace Point3D_Processing
{
    BilateralInterPolation::BilateralInterPolation(float bf,float sigmaGeo_ = 10, float sigmaCol_ = 10, float winsize_ = 11)
        :bf(bf),sigmaGeo(sigmaGeo_), sigmaCol(sigmaCol_), winsize(winsize_) {}
cv::Mat  BilateralInterPolation::operator()(const cv::Mat& leftpic, const cv::Mat&SemiDenseDisp,const cv::Mat&DenoseDisp)
{
		int width = DenoseDisp.cols;
		int height = DenoseDisp.rows;
		int halfwinsize = winsize / 2;
		cv::Mat DispAfterInterPolation = DenoseDisp.clone();
	for(int row= halfwinsize;row<height- halfwinsize;row++)
		for (int col = halfwinsize; col < width- halfwinsize; col++)
		{
			if (DenoseDisp.at<uchar>(row, col) == 0&& SemiDenseDisp.at<uchar>(row, col)!=0)
			{ 
			int colbegin = col - halfwinsize>0 ? col - halfwinsize : 0;
			int colend = DenoseDisp.cols - 1<col + halfwinsize ? DenoseDisp.cols - 1 : col + halfwinsize;
			int rowbegin = row- halfwinsize>0 ? row - halfwinsize : 0;
			int rowend = DenoseDisp.rows - 1<row + halfwinsize ? DenoseDisp.rows - 1 : row + halfwinsize;
			double DispValueAfterInterPolation = 0;
			float weightsum = 0;
			for (int j = rowbegin; j<=rowend; j++)
				for (int i = colbegin; i <= colend; i++)
				{
					
					int neighbordisp = DenoseDisp.at<uchar>(j, i);
					if (neighbordisp == 0) continue;
					float GeoDif = CalGeoDif(cv::Point(col, row), cv::Point(i, j)); 
					float GrayDif = CalGrayDif(leftpic, cv::Point(col, row), cv::Point(i, j)); 
					float weight = 1 / pow(2.7, GeoDif / sigmaGeo + GrayDif / sigmaCol);
					weightsum += weight;
					DispValueAfterInterPolation += weight*neighbordisp;
				}
			
			DispValueAfterInterPolation /= weightsum;
			
			DispAfterInterPolation.at<uchar>(row, col) = DispValueAfterInterPolation;
		}
        }
    cv::Mat color,depthuchar;
    dispTodepthShow(DispAfterInterPolation, bf,depthuchar);
    GenerateFalseMap(depthuchar, color);
    return color.clone();
	}
}
