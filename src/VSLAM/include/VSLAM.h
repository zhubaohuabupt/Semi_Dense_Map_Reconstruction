#ifndef _VSLAM_H_
#define _VSLAM_H_

#include "System.h"
#include <mutex>
namespace ORB_SLAM2
{
//SLAM state
enum eState
{
	NOT_READY = -1,
	INITIALIZING = 0,
	TRACKING = 1,
	RELOCALIZING = 2,
	SHUT_DOWN = 3
};

class VSLAM
{
public:
	~VSLAM();
	// Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
	VSLAM(const string& strVocPath, const string& strSettingPath, const int mode, const bool bViewer);

	// Process the given rgbd frame. Depthmap must be registered to the RGB frame.
    // Input left: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Input depth: Float (CV_32F).
	// Input id: frame id.
    // Returns the camera pose (empty if tracking fails).
    int doTracking(const cv::Mat& left, const cv::Mat& depth, const long& id, Mat& pose, bool& bIsKeyFrame,float*SceneDepth);

	//reset system, only available for mode1.
	int requestSystemReset();

	//change mode for vSLAM(mode1: tracking + LBA; mode2:relocalization + tracking)
	int setMode(int mode);

	//get camera pose and pic id
	int getPose(cv::Mat& pose, long& id);

	//get camera state
	int getState(eState& state);

	//get vSLAM mode(mode1: tracking + LBA; mode2:relocalization + tracking)
	int getMode(int& mode);

	int modeConfig();

	//shut down SLAM system
	void shutDown();

	int saveCovisibilityGraph();

	int loadCovisibilityGraph();

	bool checkResetFinish();

    //save poses for all of the frames
	//this function must be called after calling function "shutDown()"
	int savePoses(std::map<long,cv::Mat>& allPoses);

	bool mbReset;
	void saveTrajactory();
private:
	System* mpSystem;
	int mMode;
	long mID;
	cv::Mat mPose;
	eState mState;
	const string paramPath;
	//mutex
	std::mutex mMutexPose;
	std::mutex mMutexID;
	std::mutex mMutexState;
	std::mutex mMutexMode;
	
	
	
};

}
#endif

