#include <VSLAM.h>
using namespace ORB_SLAM2;
using namespace cv;
void LoadImages(const string &dataset_path, int id, string &vstrImageFilenamesRGB,
                string &vstrImageFilenamesD);
int main()
{
    string SettingPath = "../Camera_configs/EuRoC_MH01.yaml";		//config file path
    string dataset_path = "/media/baohua/media/SLAM_DATA/mav0_MH01";		//dataset path
	string vstrImageFilenamesRGB, vstrImageFilenamesD;

	int mode = 1;	//initial mode

	//vSLAM constructor
	VSLAM vSLAM("../Vocabulary/ORBvoc.bin", SettingPath, mode, true);

	//get camera parameters
    cv::FileStorage settings(SettingPath, cv::FileStorage::READ);
    float width = settings["Camera_width"];
    float height = settings["Camera_height"];
    float bf = settings["Camera_bf"];
	
	cv::Mat imRGB, imD;
    cv::Mat disparity(height,width,CV_32FC1);
    cv::Mat depth(height,width,CV_32FC1);
	bool isKeyFrame;
	cv::Mat pose;

    // Main loop
    for(int ni=1300; ni<3000; ni++)
    {

		LoadImages(dataset_path, ni, vstrImageFilenamesRGB, vstrImageFilenamesD);
        imRGB = cv::imread(vstrImageFilenamesRGB,CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(vstrImageFilenamesD,CV_LOAD_IMAGE_UNCHANGED);

		//deal with disparity map
		for(int i=0;i<imD.rows;i++)
			for(int j=0;j<imD.cols;j++)
			{
				disparity.at<float>(i,j) = (float)imD.at<cv::Vec3b>(i,j)[1]/100 + imD.at<cv::Vec3b>(i,j)[0];			
				depth.at<float>(i,j) = bf/disparity.at<float>(i,j);
			}
		//tracking
		vSLAM.doTracking(imRGB, depth, 0, pose,isKeyFrame);

		//request reset
		if((char)waitKey(2)=='r') 
			vSLAM.requestSystemReset();
		//set mode to MODE1
		if((char)waitKey(2)=='1')
			vSLAM.setMode(1);
		//set mode to MODE2
		if((char)waitKey(2)=='2')
			vSLAM.setMode(2);
		//quit program
		if((char)waitKey(2)=='q')
		{
			break;
		}
	}
	map<long,cv::Mat> allPoses;
	vSLAM.shutDown();
	vSLAM.saveCovisibilityGraph();	//save covisibility map
	vSLAM.savePoses(allPoses);	//save poses of all of the frame
    vSLAM.saveTrajactory();
	return 0;
}
void LoadImages(const string &dataset_path, int id, string &vstrImageFilenamesRGB,
                string &vstrImageFilenamesD)
{
    char num[5];
    sprintf(num,"%d",id);
    vstrImageFilenamesRGB = dataset_path +"/01/"+ "left_" + num +".png";
    vstrImageFilenamesD = dataset_path +"/disp/"+ "disp_" + num +".png";
}
