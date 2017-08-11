#include "VSLAM.h"

using namespace cv;
namespace ORB_SLAM2
{	
	VSLAM::~VSLAM()
	{ }

	//constructor
	VSLAM::VSLAM(const string& strVocPath, const string& strSettingPath, const int mode, const bool bViewer):mState(NOT_READY),mbReset(false)
	{
		{
			unique_lock<mutex> lock(mMutexMode);
			mMode = mode;
		}

		mpSystem = new System(strVocPath, strSettingPath, System::RGBD, mode, bViewer);

		if(mode==2)
		{
			cout<<"Loading co-visibility Map..."<<endl;
			bool bKFload=mpSystem->LoadKFFile("./saveinformation/KeyFrameInformation.bin","./saveinformation/KeyFrameSize.txt");
			if(!bKFload)
			{
				cerr << "Wrong path to KeyFrameInformation. " << endl;
				cerr << "Failed to open: " << "KeyFrameInformation.bin" << endl;
				exit(-1);
			}
			cerr << "KeyFrameInformation loaded! " << endl;

			bool bMPload=mpSystem->LoadMPFile("./saveinformation/MapPointInformation.bin");
			if(!bMPload)
			{
				cerr << "Wrong path to MapPointInformation. " << endl;
				cerr << "Failed to open at: " << "MapPointInformation.bin" << endl;
				exit(-1);
			}
			cerr << "MapPointInformation loaded! " << endl;


			bool bKFcreate=mpSystem->createKeyFrameDatabase();
			if(bKFcreate)
			   cerr<<"KeyFrameDatabase created!"<<endl;
		}
		mState = INITIALIZING;
	}
	int VSLAM::loadCovisibilityGraph()
	{
		unique_lock<mutex> lock1(mMutexMode);
		if(mMode==2)
		{
			cout<<"Loading co-visibility Map..."<<endl;
			bool bKFload=mpSystem->LoadKFFile("./saveinformation/KeyFrameInformation.bin","./saveinformation/KeyFrameSize.txt");
			if(!bKFload)
			{
				cerr << "Wrong path to KeyFrameInformation. " << endl;
				cerr << "Failed to open: " << "KeyFrameInformation.bin" << endl;
				return -1;
			}
			cerr << "KeyFrameInformation loaded! " << endl;

			bool bMPload=mpSystem->LoadMPFile("./saveinformation/MapPointInformation.bin");
			if(!bMPload)
			{
				cerr << "Wrong path to MapPointInformation. " << endl;
				cerr << "Failed to open at: " << "MapPointInformation.bin" << endl;
				return -1;
			}
			cerr << "MapPointInformation loaded! " << endl;


			bool bKFcreate=mpSystem->createKeyFrameDatabase();
			if(bKFcreate)
			   cerr<<"KeyFrameDatabase created!"<<endl;
			return 0;
		}
		else
			return -1;
	}
	//do tracking
    int VSLAM::doTracking(const cv::Mat& left, const cv::Mat& depth, const long& id, Mat& pose, bool& bIsKeyFrame,float* SceneDepth)
	{
		{
			unique_lock<mutex> lock(mMutexState);
			if(mState==SHUT_DOWN)
			{
				cout<<"SLAM system is shutdown, couldn't do tracking any way..."<<endl;
				return -1;
			}
			else if(mState==NOT_READY)
				mState = INITIALIZING;
		}
        pose = mpSystem->TrackRGBD(left, depth, id, bIsKeyFrame,SceneDepth);
		if(pose.empty()==1)
		{
			unique_lock<mutex> lock(mMutexState);
			mState = RELOCALIZING;
			return -1;
		}
		Mat Rwc = pose.rowRange(0,3).colRange(0,3).t();
		Mat twc = -Rwc*pose.rowRange(0,3).col(3);
		for(int i=0;i<3;i++)
			for(int j=0;j<3;j++)
			{
				pose.at<float>(i,j) = Rwc.at<float>(i,j);
			}
		for(int i=0;i<3;i++)
			pose.at<float>(i,3) = twc.at<float>(i,0);
		{
			unique_lock<mutex> lock(mMutexPose);
			mPose = pose.clone();
		}
		{
			unique_lock<mutex> lock(mMutexID);
			mID = id;
		}
		
		unique_lock<mutex> lock(mMutexState);
		mState = TRACKING;
		return 0;

			
	}

	//reset
	int VSLAM::requestSystemReset()
	{
		{
			unique_lock<mutex> lock(mMutexState);
			if(mState==SHUT_DOWN)
			{
				cout<<"SLAM system is shutdown, couldn't reset any way..."<<endl;
				return -1;
			}
			else if(mState==NOT_READY)
			{
				cout<<"SLAM system is not ready, couldn't reset any way..."<<endl;
				return -1;
			}
				
		}
		unique_lock<mutex> lock1(mMutexMode);
		if(mMode==1)
		{
			mbReset = true;
			mpSystem->Reset();
			{
				unique_lock<mutex> lock(mMutexState);
				mState = INITIALIZING;
			}
		}
		return 0;
	}

	//set mode
	int VSLAM::setMode(int mode)
	{
		{
			unique_lock<mutex> lock(mMutexState);
			if(mState==SHUT_DOWN)
			{
				cout<<"SLAM system is shutdown, couldn't set mode any way..."<<endl;
				return -1;
			}
			else if(mState==NOT_READY)
			{
				cout<<"SLAM system is not ready, couldn't set mode any way..."<<endl;
				return -1;
			}
		}
		unique_lock<mutex> lock1(mMutexMode);
		if(mode==mMode)
			return 0;
		else if(mode==1)
		{
			mMode = mode;
			mpSystem->DeactivateLocalizationMode();
			return 0;
		}
		else if(mode==2)
		{
			mMode = mode;
			mpSystem->ActivateLocalizationMode();
			return 0;
		}
		else
		{
			mMode = 0;
			return -1;
		}
			
	}
	//get pose
	int VSLAM::getPose(cv::Mat& pose, long& id)
	{
		{
			unique_lock<mutex> lock(mMutexState);
			if(mState==SHUT_DOWN)
			{
				cout<<"SLAM system is shutdown, couldn't get pose any way..."<<endl;
				return -1;
			}
			else if(mState==NOT_READY)
			{
				cout<<"SLAM system is not ready, couldn't get pose any way..."<<endl;
				return -1;
			}
		}
		{
			unique_lock<mutex> lock(mMutexPose);
			pose = mPose.clone();
		}
		{
			unique_lock<mutex> lock(mMutexID);
			id = mID;
		}
		if(pose.empty()==1)
			return -1;
		else
			return 0;
	}
	//get SLAM state
	int VSLAM::getState(eState& state)
	{
		unique_lock<mutex> lock(mMutexState);
		state = mState;
		return 0;
	}
	//get mode
	int VSLAM::getMode(int& mode)
	{
		{
			unique_lock<mutex> lock(mMutexState);
			if(mState==SHUT_DOWN)
			{
				cout<<"SLAM system is shutdown, couldn't get mode any way..."<<endl;
				return -1;
			}
			else if(mState==NOT_READY)
			{
				cout<<"SLAM system is not ready, couldn't get mode any way..."<<endl;
				return -1;
			}
		}
		unique_lock<mutex> lock1(mMutexMode);
		mode = mMode;
		return 0;
	}

	void VSLAM::shutDown()
	{
		mpSystem->Shutdown();
		unique_lock<mutex> lock(mMutexState);
		mState = SHUT_DOWN;
	}
	
	int VSLAM::saveCovisibilityGraph()
	{
		{		
			unique_lock<mutex> lock(mMutexState);
			if(mState==NOT_READY)
			{
				cout<<"Couldn't save graph, SLAM system was not ready, no graph to save!"<<endl;
				return -1;
			}
		}

		mpSystem->mpLocalMapper->RequestStop();
		// Wait until Local Mapping has effectively stopped
		while(!mpSystem->mpLocalMapper->isStopped())
		{
			usleep(1000);
		}

		unique_lock<mutex> lock1(mMutexMode);
		
		cout<<"Saving co-visibility Map..."<<endl;

		mpSystem->mpKeyFrameDatabase->clearOld("./saveinformation/KeyFrameInformation.bin","./saveinformation/MapPointInformation.bin");
		if(mpSystem->mpMap->OutputKF("./saveinformation/KeyFrameSize.txt")==-1)
		{
			cout<<"wrong path to save KF message!"<<endl;
			mpSystem->mpLocalMapper->Release();
			return -1;
		}
		if(mpSystem->mpMap->OutputMP()==-1)
		{
			cout<<"wrong path to save MapPoints message!"<<endl;
			mpSystem->mpLocalMapper->Release();
			return -1;
		}
		mpSystem->mpLocalMapper->Release();
	}

	int VSLAM::savePoses(map<long,cv::Mat>& allPoses)
	{
		{
			unique_lock<mutex> lock(mMutexState);
			if(mState==NOT_READY)
			{
				cout<<"Couldn't save poses, shut down SLAM system first!"<<endl;
				return -1;
			}	
		}

		mpSystem->mpLocalMapper->RequestStop();
		// Wait until Local Mapping has effectively stopped
		while(!mpSystem->mpLocalMapper->isStopped())
		{
			usleep(1000);
		}

		cout << "Saving camera trajectory " << endl;
		allPoses.clear();
		vector<KeyFrame*> vpKFs = mpSystem->mpMap->GetAllKeyFrames();
		sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

		// Transform all keyframes so that the first keyframe is at the origin.
		// After a loop closure the first keyframe might not be at the origin.
		cv::Mat Two = vpKFs[0]->GetPoseInverse();

		// Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
		// We need to get first the keyframe pose and then concatenate the relative transformation.
		// Frames not localized (tracking failure) are not saved.

		// For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
		// which is true when tracking failed (lbL).
		list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpSystem->mpTracker->mlpReferences.begin();
		list<double>::iterator lT = mpSystem->mpTracker->mlFrameTimes.begin();
		list<bool>::iterator lbL = mpSystem->mpTracker->mlbLost.begin();
		for(list<cv::Mat>::iterator lit=mpSystem->mpTracker->mlRelativeFramePoses.begin(),
		    lend=mpSystem->mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
		{
		    if(*lbL)
		        continue;

		    KeyFrame* pKF = *lRit;

		    cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

		    // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
		    while(pKF->isBad())
		    {
		        Trw = Trw*pKF->mTcp;
		        pKF = pKF->GetParent();
		    }

		    Trw = Trw*pKF->GetPose()*Two;

		    cv::Mat Tcw = (*lit)*Trw;
		    cv::Mat Twc = Tcw.inv();
			allPoses.insert(make_pair((long)*lT,Twc));
		}
		mpSystem->mpLocalMapper->Release();
		cout << endl << "trajectory saved!" << endl;
	}
	bool VSLAM::checkResetFinish()
	{
		return mpSystem->checkIsResetFinish()&&!mbReset;
	}
    void VSLAM:: saveTrajactory()
    {
     mpSystem->SaveTrajectoryTUM("CameraTrajectory.txt");
    }
}
