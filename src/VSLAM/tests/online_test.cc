#include <VSLAM.h>
#include "runCamera2.h"
using namespace ORB_SLAM2;
using namespace cv;

int main()
{
    string SettingPath = "/home/baohua/BX/Config/MVB1-0022.yaml";

    int mode = 1;

    //vSLAM constructor
    VSLAM vSLAM("/home/baohua/BX/src/VSLAM/Vocabulary/ORBvoc.bin", SettingPath, mode, true);

    //get camera parameters
    cv::FileStorage settings(SettingPath, cv::FileStorage::READ);
    float width = settings["Camera_width"];
    float height = settings["Camera_height"];
    float bf = settings["Camera_bf"];

    cv::Mat imRGB(height,width,CV_8UC1);
    cv::Mat imD(height,width,CV_8UC1);
    cv::Mat depth(height,width,CV_32FC1);
    bool isKeyFrame;
    cv::Mat pose;
  float SceneDepth[2];
    // Main loop
    while(1)
    {
        runCamera2(imRGB, imD);

        //deal with disparity map
        for(int i=0;i<imD.rows;i++)
            for(int j=0;j<imD.cols;j++)
            {
                if(imD.at<uchar>(i,j)<128&&imD.at<uchar>(i,j)!=0)
                 depth.at<float>(i,j)  = bf/(imD.at<uchar>(i,j)/4);
                else if(imD.at<uchar>(i,j)>=128)
                depth.at<float>(i,j) = bf/(imD.at<uchar>(i,j)/2-32);
                else
               depth.at<float>(i,j)=0;
            }

        vSLAM.doTracking(imRGB, depth, 0, pose,isKeyFrame,SceneDepth);

        if((char)waitKey(2)=='r')
            vSLAM.requestSystemReset();
        if((char)waitKey(2)=='1')
            vSLAM.setMode(1);
        if((char)waitKey(2)=='2')
            vSLAM.setMode(2);
        if((char)waitKey(2)=='s')
            vSLAM.saveCovisibilityGraph();
        if((char)waitKey(2)=='q')
        {
            break;
        }
    }
    map<long,cv::Mat> allPoses;
    vSLAM.shutDown();

    vSLAM.savePoses(allPoses);
    return 0;
}
