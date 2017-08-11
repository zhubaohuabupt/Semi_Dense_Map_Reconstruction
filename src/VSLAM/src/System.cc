/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/



#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <time.h>

bool has_suffix(const std::string &str, const std::string &suffix) {
  std::size_t index = str.find(suffix, str.size() - suffix.size());
  return (index != std::string::npos);
}

namespace ORB_SLAM2
{
bool isFirstFrame = true;
System::~System()
{

}
System::System(const string &strVocFile, const string &strSettingsFile,  const eSensor sensor,
               const int mode, const bool bUseViewer):mSensor(sensor),mbReset(false),mbActivateLocalizationMode(false),
        mbDeactivateLocalizationMode(false)
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";
	if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

	if(mode==1)
		cout<<"Running MODE 1"<<endl;
	else if(mode==2)
	{
		mbActivateLocalizationMode = true;
		cout<<"Running MODE 2"<<endl;
	}
	else
	{
		cout<<"wrong MODE selection!"<<endl;
		exit(-1);
	}
    

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);

    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }


    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;
    clock_t tStart = clock();
    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = false; // chose loading method based on file extension
    if (has_suffix(strVocFile, ".txt"))
		bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    else
		bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Failed to open at: " << strVocFile << endl;
        exit(-1);
    }
    printf("Vocabulary loaded in %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Map
    mpMap = new Map();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor, mode);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    //Initialize the Viewer thread and launch
    mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
    //if(bUseViewer)
     //   mptViewer = new thread(&Viewer::Run, mpViewer);

    mpTracker->SetViewer(mpViewer);

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

}
cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }   

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }
            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    return mpTracker->GrabImageStereo(imLeft,imRight,timestamp);
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const long &timestamp, bool& bIsKeyFrame,float *SceneDepth)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }    

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
			mpTracker->mState = mpTracker->LOST;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    	unique_lock<mutex> lock(mMutexReset);
    	if(mbReset)
    	{
            mpTracker->Reset();
            mbReset = false;
    	}
    }

    Mat TCW;
    if(USE_IMU==true)
       TCW= mpTracker->GrabImageRGBD_IMUdata(im,depthmap,timestamp,IMUdetaT_t_tlast, bIsKeyFrame,SceneDepth);
    else
    {
         TCW=  mpTracker->GrabImageRGBD(im,depthmap,timestamp, bIsKeyFrame,SceneDepth);
     }

      return TCW;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    return mpTracker->GrabImageMonocular(im,timestamp);
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    mpViewer->RequestFinish();

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished()  ||
          !mpViewer->isFinished()      || mpLoopCloser->isRunningGBA())
    {
        usleep(5000);
    }

    pangolin::BindToContext("ORB-SLAM2: Map Viewer");
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
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
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

bool System::LoadKFFile(const string &strKFFile,const string &kfsz)
{

  fstream f;
  f.open(strKFFile.c_str(), ios_base::in|ios::binary);
  if(!f.is_open())
	return 0;
  ifstream ff;
  ff.open(kfsz.c_str());
  if(!ff.is_open())
	return 0;
  int ikfsz;
  string s;
  getline(ff,s);
  stringstream ss;
  ss << s;
  ss >> ikfsz;
  ff.close();
  int kfcounter=0;


  while(!f.eof())
     {
       if(f.fail())break;
       else{

    //int...float...
    f.read((char*)&kmnId, sizeof(kmnId));
    f.read((char*)&knNextId, sizeof(knNextId));
    f.read((char*)&kmnFrameId,sizeof(kmnFrameId));
    f.read((char*)&kmTimeStamp,sizeof(kmTimeStamp));
    f.read((char*)&kmnGridCols,sizeof(kmnGridCols));
    f.read((char*)&kmnGridRows,sizeof(kmnGridRows));
    f.read((char*)&kmfGridElementWidthInv,sizeof(kmfGridElementWidthInv));
    f.read((char*)&kmfGridElementHeightInv,sizeof(kmfGridElementHeightInv));
    f.read((char*)&kmnTrackReferenceForFrame,sizeof(kmnTrackReferenceForFrame));
    f.read((char*)&kmnFuseTargetForKF,sizeof(kmnFuseTargetForKF));

    f.read((char*)&kmnBALocalForKF,sizeof(kmnBALocalForKF));
    f.read((char*)&kmnBAFixedForKF,sizeof(kmnBAFixedForKF));
    f.read((char*)&kmnLoopQuery,sizeof(kmnLoopQuery));
    f.read((char*)&kmnLoopWords,sizeof(kmnLoopWords));
    f.read((char*)&kmLoopScore,sizeof(kmLoopScore));
    f.read((char*)&kmnRelocQuery,sizeof(kmnRelocQuery));
    f.read((char*)&kmnRelocWords,sizeof(kmnRelocWords));
    f.read((char*)&kmRelocScore,sizeof(kmRelocScore));
    f.read((char*)&kmnBAGlobalForKF,sizeof(kmnBAGlobalForKF));

    f.read((char*)&kfx,sizeof(kfx));
    f.read((char*)&kfy,sizeof(kfy));
    f.read((char*)&kcx,sizeof(kcx));
    f.read((char*)&kcy,sizeof(kcy));
    f.read((char*)&kinvfx,sizeof(kinvfx));
    f.read((char*)&kinvfy,sizeof(kinvfy));
    f.read((char*)&kmbf,sizeof(kmbf));
    f.read((char*)&kmb,sizeof(kmb));
    f.read((char*)&kmThDepth,sizeof(kmThDepth));
    f.read((char*)&kN,sizeof(kN));

    f.read((char*)&kmnScaleLevels,sizeof(kmnScaleLevels));
    f.read((char*)&kmfScaleFactor,sizeof(kmfScaleFactor));
    f.read((char*)&kmfLogScaleFactor,sizeof(kmfLogScaleFactor));
    f.read((char*)&kmnMinX,sizeof(kmnMinX));
    f.read((char*)&kmnMinY,sizeof(kmnMinY));
    f.read((char*)&kmnMaxX,sizeof(kmnMaxX));
    f.read((char*)&kmnMaxY,sizeof(kmnMaxY));
    f.read((char*)&kmbFirstConnection,sizeof(kmbFirstConnection));//cout<<kmbFirstConnection<<endl;getchar();
    f.read((char*)&kmbNotErase,sizeof(kmbNotErase));
    f.read((char*)&kmbToBeErased,sizeof(kmbToBeErased));

    f.read((char*)&kmbBad,sizeof(kmbBad));
    f.read((char*)&kmHalfBaseline,sizeof(kmHalfBaseline));//cout<<sizeof(kmHalfBaseline)<<" "<<kmHalfBaseline<<endl;getchar();

    bool bcont;


    //Mat...

    //mTcwGBA(4,4,32F) 
    kmTcwGBA.release(); 
    f.read((char*)&bcont,sizeof(bcont));
    if(bcont==true)
      {
       kmTcwGBA.create(4,4,CV_32F);
       f.read((char*)&(kmTcwGBA.at<float>(0,0)),sizeof(kmTcwGBA.at<float>(0,0)));
       f.read((char*)&(kmTcwGBA.at<float>(0,1)),sizeof(kmTcwGBA.at<float>(0,1)));
       f.read((char*)&(kmTcwGBA.at<float>(0,2)),sizeof(kmTcwGBA.at<float>(0,2)));
       f.read((char*)&(kmTcwGBA.at<float>(0,3)),sizeof(kmTcwGBA.at<float>(0,3)));

       f.read((char*)&(kmTcwGBA.at<float>(1,0)),sizeof(kmTcwGBA.at<float>(1,0)));
       f.read((char*)&(kmTcwGBA.at<float>(1,1)),sizeof(kmTcwGBA.at<float>(1,1)));
       f.read((char*)&(kmTcwGBA.at<float>(1,2)),sizeof(kmTcwGBA.at<float>(1,2)));
       f.read((char*)&(kmTcwGBA.at<float>(1,3)),sizeof(kmTcwGBA.at<float>(1,3)));

       f.read((char*)&(kmTcwGBA.at<float>(2,0)),sizeof(kmTcwGBA.at<float>(2,0)));
       f.read((char*)&(kmTcwGBA.at<float>(2,1)),sizeof(kmTcwGBA.at<float>(2,1)));
       f.read((char*)&(kmTcwGBA.at<float>(2,2)),sizeof(kmTcwGBA.at<float>(2,2)));
       f.read((char*)&(kmTcwGBA.at<float>(2,3)),sizeof(kmTcwGBA.at<float>(2,3)));

       f.read((char*)&(kmTcwGBA.at<float>(3,0)),sizeof(kmTcwGBA.at<float>(3,0)));
       f.read((char*)&(kmTcwGBA.at<float>(3,1)),sizeof(kmTcwGBA.at<float>(3,1)));
       f.read((char*)&(kmTcwGBA.at<float>(3,2)),sizeof(kmTcwGBA.at<float>(3,2)));
       f.read((char*)&(kmTcwGBA.at<float>(3,3)),sizeof(kmTcwGBA.at<float>(3,3)));
        }   

    //kmTcwBefGBA
    kmTcwBefGBA.release();              
    f.read((char*)&bcont,sizeof(bcont));//cout<<"222222222"<<" "<<bcont<<endl;getchar();
    if(bcont==true)
      {
       kmTcwBefGBA.create(4,4,CV_32F);
       f.read((char*)&(kmTcwBefGBA.at<float>(0,0)),sizeof(kmTcwBefGBA.at<float>(0,0)));
       f.read((char*)&(kmTcwBefGBA.at<float>(0,1)),sizeof(kmTcwBefGBA.at<float>(0,1)));
       f.read((char*)&(kmTcwBefGBA.at<float>(0,2)),sizeof(kmTcwBefGBA.at<float>(0,2)));
       f.read((char*)&(kmTcwBefGBA.at<float>(0,3)),sizeof(kmTcwBefGBA.at<float>(0,3)));

       f.read((char*)&(kmTcwBefGBA.at<float>(1,0)),sizeof(kmTcwBefGBA.at<float>(1,0)));
       f.read((char*)&(kmTcwBefGBA.at<float>(1,1)),sizeof(kmTcwBefGBA.at<float>(1,1)));
       f.read((char*)&(kmTcwBefGBA.at<float>(1,2)),sizeof(kmTcwBefGBA.at<float>(1,2)));
       f.read((char*)&(kmTcwBefGBA.at<float>(1,3)),sizeof(kmTcwBefGBA.at<float>(1,3)));

       f.read((char*)&(kmTcwBefGBA.at<float>(2,0)),sizeof(kmTcwBefGBA.at<float>(2,0)));
       f.read((char*)&(kmTcwBefGBA.at<float>(2,1)),sizeof(kmTcwBefGBA.at<float>(2,1)));
       f.read((char*)&(kmTcwBefGBA.at<float>(2,2)),sizeof(kmTcwBefGBA.at<float>(2,2)));
       f.read((char*)&(kmTcwBefGBA.at<float>(2,3)),sizeof(kmTcwBefGBA.at<float>(2,3)));

       f.read((char*)&(kmTcwBefGBA.at<float>(3,0)),sizeof(kmTcwBefGBA.at<float>(3,0)));
       f.read((char*)&(kmTcwBefGBA.at<float>(3,1)),sizeof(kmTcwBefGBA.at<float>(3,1)));
       f.read((char*)&(kmTcwBefGBA.at<float>(3,2)),sizeof(kmTcwBefGBA.at<float>(3,2)));
       f.read((char*)&(kmTcwBefGBA.at<float>(3,3)),sizeof(kmTcwBefGBA.at<float>(3,3)));
       } 

    //kmDescriptors(N*32)
    kmDescriptors.create(kN,32, CV_8U);          
    for(int i=0;i<kmDescriptors.rows;++i)
        {
         stringstream tempss;
         unsigned char *p = kmDescriptors.ptr<unsigned char>(i);
         for(int j=0;j<kmDescriptors.cols;++j,++p)
            {
             int tempintdesc;
             f.read((char*)&tempintdesc,sizeof(tempintdesc));
             *p=(unsigned char)tempintdesc;
             }

        }

    //kmTcp  
    kmTcp.release();            
    f.read((char*)&bcont,sizeof(bcont));
    if(bcont==true)
      {
       kmTcp.create(4,4,CV_32F);
       f.read((char*)&(kmTcp.at<float>(0,0)),sizeof(kmTcp.at<float>(0,0)));
       f.read((char*)&(kmTcp.at<float>(0,1)),sizeof(kmTcp.at<float>(0,1)));
       f.read((char*)&(kmTcp.at<float>(0,2)),sizeof(kmTcp.at<float>(0,2)));
       f.read((char*)&(kmTcp.at<float>(0,3)),sizeof(kmTcp.at<float>(0,3)));

       f.read((char*)&(kmTcp.at<float>(1,0)),sizeof(kmTcp.at<float>(1,0)));
       f.read((char*)&(kmTcp.at<float>(1,1)),sizeof(kmTcp.at<float>(1,1)));
       f.read((char*)&(kmTcp.at<float>(1,2)),sizeof(kmTcp.at<float>(1,2)));
       f.read((char*)&(kmTcp.at<float>(1,3)),sizeof(kmTcp.at<float>(1,3)));

       f.read((char*)&(kmTcp.at<float>(2,0)),sizeof(kmTcp.at<float>(2,0)));
       f.read((char*)&(kmTcp.at<float>(2,1)),sizeof(kmTcp.at<float>(2,1)));
       f.read((char*)&(kmTcp.at<float>(2,2)),sizeof(kmTcp.at<float>(2,2)));
       f.read((char*)&(kmTcp.at<float>(2,3)),sizeof(kmTcp.at<float>(2,3)));

       f.read((char*)&(kmTcp.at<float>(3,0)),sizeof(kmTcp.at<float>(3,0)));
       f.read((char*)&(kmTcp.at<float>(3,1)),sizeof(kmTcp.at<float>(3,1)));
       f.read((char*)&(kmTcp.at<float>(3,2)),sizeof(kmTcp.at<float>(3,2)));
       f.read((char*)&(kmTcp.at<float>(3,3)),sizeof(kmTcp.at<float>(3,3)));
        }

    //kmK  
    kmK.release();            
    f.read((char*)&bcont,sizeof(bcont));
    if(bcont==true)
      {
       kmK.create(3,3,CV_32F);
       f.read((char*)&(kmK.at<float>(0,0)),sizeof(kmK.at<float>(0,0)));
       f.read((char*)&(kmK.at<float>(0,1)),sizeof(kmK.at<float>(0,1)));
       f.read((char*)&(kmK.at<float>(0,2)),sizeof(kmK.at<float>(0,2)));

       f.read((char*)&(kmK.at<float>(1,0)),sizeof(kmK.at<float>(1,0)));
       f.read((char*)&(kmK.at<float>(1,1)),sizeof(kmK.at<float>(1,1)));
       f.read((char*)&(kmK.at<float>(1,2)),sizeof(kmK.at<float>(1,2)));

       f.read((char*)&(kmK.at<float>(2,0)),sizeof(kmK.at<float>(2,0)));
       f.read((char*)&(kmK.at<float>(2,1)),sizeof(kmK.at<float>(2,1)));
       f.read((char*)&(kmK.at<float>(2,2)),sizeof(kmK.at<float>(2,2)));
        }

    //kTcw 
    kTcw.release();             
    f.read((char*)&bcont,sizeof(bcont));
    if(bcont==true)
      {
       kTcw.create(4,4,CV_32F);
       f.read((char*)&(kTcw.at<float>(0,0)),sizeof(kTcw.at<float>(0,0)));
       f.read((char*)&(kTcw.at<float>(0,1)),sizeof(kTcw.at<float>(0,1)));
       f.read((char*)&(kTcw.at<float>(0,2)),sizeof(kTcw.at<float>(0,2)));
       f.read((char*)&(kTcw.at<float>(0,3)),sizeof(kTcw.at<float>(0,3)));

       f.read((char*)&(kTcw.at<float>(1,0)),sizeof(kTcw.at<float>(1,0)));
       f.read((char*)&(kTcw.at<float>(1,1)),sizeof(kTcw.at<float>(1,1)));
       f.read((char*)&(kTcw.at<float>(1,2)),sizeof(kTcw.at<float>(1,2)));
       f.read((char*)&(kTcw.at<float>(1,3)),sizeof(kTcw.at<float>(1,3)));

       f.read((char*)&(kTcw.at<float>(2,0)),sizeof(kTcw.at<float>(2,0)));
       f.read((char*)&(kTcw.at<float>(2,1)),sizeof(kTcw.at<float>(2,1)));
       f.read((char*)&(kTcw.at<float>(2,2)),sizeof(kTcw.at<float>(2,2)));
       f.read((char*)&(kTcw.at<float>(2,3)),sizeof(kTcw.at<float>(2,3)));

       f.read((char*)&(kTcw.at<float>(3,0)),sizeof(kTcw.at<float>(3,0)));
       f.read((char*)&(kTcw.at<float>(3,1)),sizeof(kTcw.at<float>(3,1)));
       f.read((char*)&(kTcw.at<float>(3,2)),sizeof(kTcw.at<float>(3,2)));
       f.read((char*)&(kTcw.at<float>(3,3)),sizeof(kTcw.at<float>(3,3)));
       }

    //kTwc 
    kTwc.release();             
    f.read((char*)&bcont,sizeof(bcont));
    if(bcont==true)
      {
       kTwc.create(4,4,CV_32F);
       f.read((char*)&(kTwc.at<float>(0,0)),sizeof(kTwc.at<float>(0,0)));
       f.read((char*)&(kTwc.at<float>(0,1)),sizeof(kTwc.at<float>(0,1)));
       f.read((char*)&(kTwc.at<float>(0,2)),sizeof(kTwc.at<float>(0,2)));
       f.read((char*)&(kTwc.at<float>(0,3)),sizeof(kTwc.at<float>(0,3)));

       f.read((char*)&(kTwc.at<float>(1,0)),sizeof(kTwc.at<float>(1,0)));
       f.read((char*)&(kTwc.at<float>(1,1)),sizeof(kTwc.at<float>(1,1)));
       f.read((char*)&(kTwc.at<float>(1,2)),sizeof(kTwc.at<float>(1,2)));
       f.read((char*)&(kTwc.at<float>(1,3)),sizeof(kTwc.at<float>(1,3)));

       f.read((char*)&(kTwc.at<float>(2,0)),sizeof(kTwc.at<float>(2,0)));
       f.read((char*)&(kTwc.at<float>(2,1)),sizeof(kTwc.at<float>(2,1)));
       f.read((char*)&(kTwc.at<float>(2,2)),sizeof(kTwc.at<float>(2,2)));
       f.read((char*)&(kTwc.at<float>(2,3)),sizeof(kTwc.at<float>(2,3)));

       f.read((char*)&(kTwc.at<float>(3,0)),sizeof(kTwc.at<float>(3,0)));
       f.read((char*)&(kTwc.at<float>(3,1)),sizeof(kTwc.at<float>(3,1)));
       f.read((char*)&(kTwc.at<float>(3,2)),sizeof(kTwc.at<float>(3,2)));
       f.read((char*)&(kTwc.at<float>(3,3)),sizeof(kTwc.at<float>(3,3)));
       }

    //kOw 
    kOw.release();   
    f.read((char*)&bcont,sizeof(bcont));
    if(bcont==true)             
       {
       kOw.create(3,1,CV_32F);
       f.read((char*)&(kOw.at<float>(0,0)),sizeof(kOw.at<float>(0,0)));
       f.read((char*)&(kOw.at<float>(1,0)),sizeof(kOw.at<float>(1,0)));
       f.read((char*)&(kOw.at<float>(2,0)),sizeof(kOw.at<float>(2,0)));
       }

    //kCw
    kCw.release();
    f.read((char*)&bcont,sizeof(bcont));
    if(bcont==true)             
       {
       kCw.create(3,1,CV_32F);
       f.read((char*)&(kCw.at<float>(0,0)),sizeof(kCw.at<float>(0,0)));
       f.read((char*)&(kCw.at<float>(1,0)),sizeof(kCw.at<float>(1,0)));
       f.read((char*)&(kCw.at<float>(2,0)),sizeof(kCw.at<float>(2,0)));
       f.read((char*)&(kCw.at<float>(3,0)),sizeof(kCw.at<float>(3,0)));
       }

    //vector

    //kmvKeys(N*7)
    kmvKeys.clear();
    kmvKeys.resize(kN);
    for(int i=0;i<kN;i++)
      {
         f.read((char*)&(kmvKeys[i].pt.x),sizeof(kmvKeys[i].pt.x));
         f.read((char*)&(kmvKeys[i].pt.y),sizeof(kmvKeys[i].pt.y));

         f.read((char*)&(kmvKeys[i].size),sizeof(kmvKeys[i].size));
         f.read((char*)&(kmvKeys[i].angle),sizeof(kmvKeys[i].angle));
         f.read((char*)&(kmvKeys[i].response),sizeof(kmvKeys[i].response));
         f.read((char*)&(kmvKeys[i].octave),sizeof(kmvKeys[i].octave));
         f.read((char*)&(kmvKeys[i].class_id),sizeof(kmvKeys[i].class_id));                         
       }

    //kmvKeysUn(mvKeysUn.size()*7)
    kmvKeysUn.clear();
    int mvkeysunsize;
    f.read((char*)&(mvkeysunsize),sizeof(mvkeysunsize));
    kmvKeysUn.resize(mvkeysunsize);              
    for(int i=0;i<kmvKeysUn.size();i++)
       {
         f.read((char*)&(kmvKeysUn[i].pt.x),sizeof(kmvKeysUn[i].pt.x));
         f.read((char*)&(kmvKeysUn[i].pt.y),sizeof(kmvKeysUn[i].pt.y));

         f.read((char*)&(kmvKeysUn[i].size),sizeof(kmvKeysUn[i].size));
         f.read((char*)&(kmvKeysUn[i].angle),sizeof(kmvKeysUn[i].angle));
         f.read((char*)&(kmvKeysUn[i].response),sizeof(kmvKeysUn[i].response));
         f.read((char*)&(kmvKeysUn[i].octave),sizeof(kmvKeysUn[i].octave));
         f.read((char*)&(kmvKeysUn[i].class_id),sizeof(kmvKeysUn[i].class_id));
        }

    //kmvuRight
    kmvuRight.clear();
    int mvurightsize;
    f.read((char*)&(mvurightsize),sizeof(mvurightsize));
    kmvuRight.resize(mvurightsize);
    for(int i=0;i<kmvuRight.size();i++)
       f.read((char*)&(kmvuRight[i]),sizeof(kmvuRight[i]));

    //kmvDepth
    kmvDepth.clear();
    int mvdepthsize;
    f.read((char*)&(mvdepthsize),sizeof(mvdepthsize));
    kmvDepth.resize(mvdepthsize);
    for(int i=0;i<kmvDepth.size();i++)
       f.read((char*)&(kmvDepth[i]),sizeof(kmvDepth[i]));

    //kmvScaleFactors
    kmvScaleFactors.clear();
    int mvscalefactorssize;
    f.read((char*)&(mvscalefactorssize),sizeof(mvscalefactorssize));
    kmvScaleFactors.resize(mvscalefactorssize);
    for(int i=0;i<kmvScaleFactors.size();i++)
       f.read((char*)&(kmvScaleFactors[i]),sizeof(kmvScaleFactors[i]));

    //kmvLevelSigma2
    kmvLevelSigma2.clear();
    int mvLevelSigma2size;
    f.read((char*)&(mvLevelSigma2size),sizeof(mvLevelSigma2size));
    kmvLevelSigma2.resize(mvLevelSigma2size);
    for(int i=0;i<kmvLevelSigma2.size();i++)
       f.read((char*)&(kmvLevelSigma2[i]),sizeof(kmvLevelSigma2[i]));

    //kmvInvLevelSigma2
    kmvInvLevelSigma2.clear();
    int mvInvLevelSigma2size;
    f.read((char*)&(mvInvLevelSigma2size),sizeof(mvInvLevelSigma2size));
    kmvInvLevelSigma2.resize(mvInvLevelSigma2size);
    for(int i=0;i<kmvInvLevelSigma2.size();i++)
       f.read((char*)&(kmvInvLevelSigma2[i]),sizeof(kmvInvLevelSigma2[i]));

              
    //kmvpMapPoints
    kmvpMapPoints.clear();
    int MPCounter;
    f.read((char*)&(MPCounter),sizeof(MPCounter));
    for(int i=0;i<MPCounter;i++)
       {
        int tempindex;
        long unsigned int kMPId;
        f.read((char*)&tempindex,sizeof(tempindex));
        f.read((char*)&kMPId,sizeof(kMPId)); 
        kmvpMapPoints[tempindex]=kMPId; 
        }
             
    //kmGrid
    kmGrid.resize(64);
    for(int i=0;i<kmGrid.size();i++)
       {
        kmGrid[i].resize(48);
        for(int j=0;j<kmGrid[i].size();j++)
           {
            int lsize;
            f.read((char*)&lsize,sizeof(lsize));
            for(int k=0;k<lsize;k++)
                {
                 size_t st;
                 f.read((char*)&st,sizeof(st));
                 kmGrid[i][j].push_back(st);
                 }
            }
       }

    //kmvpOrderedConnectedKeyFrames              
    kmvpOrderedConnectedKeyFrames.clear();
    int mvpOrderedConnectedKeyFramessize;
    f.read((char*)&mvpOrderedConnectedKeyFramessize,sizeof(mvpOrderedConnectedKeyFramessize));
    kmvpOrderedConnectedKeyFrames.resize(mvpOrderedConnectedKeyFramessize);
    for(int i=0;i<kmvpOrderedConnectedKeyFrames.size();i++)
       f.read((char*)&(kmvpOrderedConnectedKeyFrames[i]),sizeof(kmvpOrderedConnectedKeyFrames[i]));

    //kmvOrderedWeights              
    kmvOrderedWeights.clear();
    int mvOrderedWeightssize;
    f.read((char*)&mvOrderedWeightssize,sizeof(mvOrderedWeightssize));
    kmvOrderedWeights.resize(mvOrderedWeightssize);
    for(int i=0;i<kmvOrderedWeights.size();i++)
       f.read((char*)&(kmvOrderedWeights[i]),sizeof(kmvOrderedWeights[i]));


    //map

    //kmBowVec(n*(1 2))
    kmBowVec.clear();
    int mBowVecsize;
    f.read((char*)&mBowVecsize,sizeof(mBowVecsize));
    for(int i=0;i<mBowVecsize;i++)
       {
        unsigned int wdid;
        double wdval;
        f.read((char*)&wdid,sizeof(wdid));
        f.read((char*)&wdval,sizeof(wdval));

        kmBowVec[wdid]=wdval;   
        }    

     //kmFeatVec
     kmFeatVec.clear();
     int mFeatVecsize;
     f.read((char*)&mFeatVecsize,sizeof(mFeatVecsize));
     for(int i=0;i<mFeatVecsize;i++)
       {
        unsigned int wdid;
        int sz;
        f.read((char*)&wdid,sizeof(wdid));
	f.read((char*)&sz,sizeof(sz));
        for(int j=0;j<sz;j++)
           {
            int desid;
            f.read((char*)&desid,sizeof(desid));
            kmFeatVec[wdid].push_back(desid);                       
            }
        }   

    //kmConnectedKeyFrameWeights(n 1 2)
    kmConnectedKeyFrameWeights.clear();
    int mConnectedKeyFrameWeightssize;
    f.read((char*)&mConnectedKeyFrameWeightssize,sizeof(mConnectedKeyFrameWeightssize));
    for(int i=0;i<mConnectedKeyFrameWeightssize;i++)
       {
        long unsigned int kfid;
        int conweights;
        f.read((char*)&kfid,sizeof(kfid));
        f.read((char*)&conweights,sizeof(conweights));
        kmConnectedKeyFrameWeights[kfid]=conweights;                              
        }

    //set

    //kmspChildrens
    kmspChildrens.clear();
    int mspChildrenssize;
    f.read((char*)&mspChildrenssize,sizeof(mspChildrenssize)); 
    for(int i=0;i<mspChildrenssize;i++)
       {
        long unsigned int childid;
        f.read((char*)&childid,sizeof(childid)); 
        kmspChildrens.insert(childid);
        }

    //kmspLoopEdges
    kmspLoopEdges.clear();
    int mspLoopEdgessize;
    f.read((char*)&mspLoopEdgessize,sizeof(mspLoopEdgessize)); 
    for(int i=0;i<mspLoopEdgessize;i++)
       {
        long unsigned int loopedges;
        f.read((char*)&loopedges,sizeof(loopedges)); 
        kmspLoopEdges.insert(loopedges);
        }

    //pointer

    //kmpParent
    f.read((char*)&bcont,sizeof(bcont));
    if(bcont==true)
       f.read((char*)&kmpParent,sizeof(kmpParent));


    KeyFrame* pKF = new KeyFrame(this); 
    LoadKFDatabase.push_back(pKF);              

    kfcounter++;
    if(kfcounter==ikfsz)
       break;  

           }//end else...
              
         }//end while...

//mpParent
for(int i=0;i<LoadKFDatabase.size();i++)
    {
     KeyFrame* pKFi = LoadKFDatabase[i];
     if(pKFi->mnId==0)
        {
        pKFi->mpParent=NULL;
        continue;
        }

     long unsigned int idprt=pKFi->IdmpParent;
     for(int j=0;j<LoadKFDatabase.size();j++)
        {
        KeyFrame* pKFiprt = LoadKFDatabase[j];
        if(pKFiprt->mnId==idprt)
           pKFi->mpParent=pKFiprt;
         }
    
    }

//mspChildrens
for(int i=0;i<LoadKFDatabase.size();i++)
    {
     KeyFrame* pKFi = LoadKFDatabase[i];
     if(pKFi->IdmspChildrens.size()==0)
        continue;
     for(set<long unsigned int>::iterator sit=pKFi->IdmspChildrens.begin(), send=pKFi->IdmspChildrens.end(); sit!=send; sit++)
         {
           long unsigned int idchild=(*sit);
           for(int j=0;j<LoadKFDatabase.size();j++)
               {
                 KeyFrame* pKFichild = LoadKFDatabase[j]; 
                 if(pKFichild->mnId==idchild)
                    pKFi->mspChildrens.insert(pKFichild);             
                }

          }
     
     } 

//mspLoopEdges
for(int i=0;i<LoadKFDatabase.size();i++)
    {
     KeyFrame* pKFi = LoadKFDatabase[i];
     if(pKFi->IdmspLoopEdges.size()==0)
        continue;
     for(set<long unsigned int>::iterator sit=pKFi->IdmspLoopEdges.begin(), send=pKFi->IdmspLoopEdges.end(); sit!=send; sit++)
         {
           long unsigned int idlpedges=(*sit);
           for(int j=0;j<LoadKFDatabase.size();j++)
               {
                 KeyFrame* pKFilpedges = LoadKFDatabase[j]; 
                 if(pKFilpedges->mnId==idlpedges)
                    pKFi->mspLoopEdges.insert(pKFilpedges);             
                }

          }
     
     } 

//mvpOrderedConnectedKeyFrames 
for(int i=0;i<LoadKFDatabase.size();i++)
    {
     KeyFrame* pKFi = LoadKFDatabase[i];
     if(pKFi->IdmvpOrderedConnectedKeyFrames.size()==0)
        continue;
     for(int j=0;j<pKFi->IdmvpOrderedConnectedKeyFrames.size();j++)
        {
         long unsigned int idcnctkf=pKFi->IdmvpOrderedConnectedKeyFrames[j];
         for(int k=0;k<LoadKFDatabase.size();k++)
            {
             KeyFrame* pKFilconnected = LoadKFDatabase[k]; 
             if(pKFilconnected->mnId==idcnctkf)
                pKFi->mvpOrderedConnectedKeyFrames.push_back(pKFilconnected); 
             }
         }

     }
 
//mConnectedKeyFrameWeights
for(int i=0;i<LoadKFDatabase.size();i++)
    {
     KeyFrame* pKFi = LoadKFDatabase[i];   
     if(pKFi->IdmConnectedKeyFrameWeights.size()==0)
        continue;
     for(auto mit=pKFi->IdmConnectedKeyFrameWeights.begin();mit!=pKFi->IdmConnectedKeyFrameWeights.end();mit++)
        {
         long unsigned int kfid=mit->first;
         int cnctweights=mit->second;
         for(int j=0;j<LoadKFDatabase.size();j++)
             {
              KeyFrame* pKFiconnected=LoadKFDatabase[j];
              if(pKFiconnected->mnId==kfid)                 
                   pKFi->mConnectedKeyFrameWeights[pKFiconnected]=cnctweights;                  
              }

         }

     }

for(int i=0;i<LoadKFDatabase.size();i++)
    {
     KeyFrame* pKFi = LoadKFDatabase[i];   
     mpMap->mspKeyFrames.insert(pKFi); 
    }

    return true;
}


bool System::LoadMPFile(const string &strMPFile)
{
  fstream f;
  f.open(strMPFile.c_str(), ios_base::in|ios::binary);
  if(!f.is_open())
	return 0;
  while(!f.eof())
     {
       if(f.fail())break;
       else{
    //int...float...
    f.read((char*)&mmnId,sizeof(mmnId));
    f.read((char*)&mnNextId,sizeof(mnNextId));
    f.read((char*)&mmnFirstKFid,sizeof(mmnFirstKFid));
    f.read((char*)&mmnFirstFrame,sizeof(mmnFirstFrame));
    f.read((char*)&mnObs,sizeof(mnObs));
    f.read((char*)&mmTrackProjX,sizeof(mmTrackProjX));
    f.read((char*)&mmTrackProjY,sizeof(mmTrackProjY));
    f.read((char*)&mmTrackProjXR,sizeof(mmTrackProjXR));

    f.read((char*)&mmbTrackInView,sizeof(mmbTrackInView));
    f.read((char*)&mmnTrackScaleLevel,sizeof(mmnTrackScaleLevel));
    f.read((char*)&mmTrackViewCos,sizeof(mmTrackViewCos));
    f.read((char*)&mmnTrackReferenceForFrame,sizeof(mmnTrackReferenceForFrame));
    f.read((char*)&mmnLastFrameSeen,sizeof(mmnLastFrameSeen));
    f.read((char*)&mmnBALocalForKF,sizeof(mmnBALocalForKF));
    f.read((char*)&mmnFuseCandidateForKF,sizeof(mmnFuseCandidateForKF));
    f.read((char*)&mmnLoopPointForKF,sizeof(mmnLoopPointForKF));
    f.read((char*)&mmnCorrectedByKF,sizeof(mmnCorrectedByKF));

    f.read((char*)&mmnCorrectedReference,sizeof(mmnCorrectedReference));
    f.read((char*)&mmnBAGlobalForKF,sizeof(mmnBAGlobalForKF));
    f.read((char*)&mmnVisible,sizeof(mmnVisible));
    f.read((char*)&mmnFound,sizeof(mmnFound));
    f.read((char*)&mmbBad,sizeof(mmbBad));
    f.read((char*)&mmfMinDistance,sizeof(mmfMinDistance));
    f.read((char*)&mmfMaxDistance,sizeof(mmfMaxDistance));

    bool bcont;

    //Mat
         
    //mmPosGBA
    mmPosGBA.release();  
    f.read((char*)&bcont,sizeof(bcont));
    if(bcont==true)      
      {
       mmPosGBA.create(3,1,CV_32F);
       f.read((char*)&(mmPosGBA.at<float>(0,0)),sizeof(mmPosGBA.at<float>(0,0)));
       f.read((char*)&(mmPosGBA.at<float>(1,0)),sizeof(mmPosGBA.at<float>(1,0)));
       f.read((char*)&(mmPosGBA.at<float>(2,0)),sizeof(mmPosGBA.at<float>(2,0)));
       }

    //mmWorldPos
    mmWorldPos.release();  
    f.read((char*)&bcont,sizeof(bcont));
    if(bcont==true)      
      {
       mmWorldPos.create(3,1,CV_32F);
       f.read((char*)&(mmWorldPos.at<float>(0,0)),sizeof(mmWorldPos.at<float>(0,0)));
       f.read((char*)&(mmWorldPos.at<float>(1,0)),sizeof(mmWorldPos.at<float>(1,0)));
       f.read((char*)&(mmWorldPos.at<float>(2,0)),sizeof(mmWorldPos.at<float>(2,0)));
       }

    //mmNormalVector
    mmNormalVector.release();  
    f.read((char*)&bcont,sizeof(bcont));
    if(bcont==true)      
      {
       mmNormalVector.create(3,1,CV_32F);
       f.read((char*)&(mmNormalVector.at<float>(0,0)),sizeof(mmNormalVector.at<float>(0,0)));
       f.read((char*)&(mmNormalVector.at<float>(1,0)),sizeof(mmNormalVector.at<float>(1,0)));
       f.read((char*)&(mmNormalVector.at<float>(2,0)),sizeof(mmNormalVector.at<float>(2,0)));
       }

    //mmDescriptor(32)
    mmDescriptor.create(1,32,CV_8U);
    unsigned char *p = mmDescriptor.ptr<unsigned char>();
    for(int i=0;i<mmDescriptor.cols;++i,++p)
       {
        int tempintdesc;
        f.read((char*)&tempintdesc,sizeof(tempintdesc));
        *p=(unsigned char)tempintdesc;

        }
    //cout<<endl;getchar();

    //map

    //mmObservations
    mmObservations.clear();
    int mObservationssize;
    f.read((char*)&mObservationssize,sizeof(mObservationssize));
    for(int i=0;i<mObservationssize;i++)
       {
        long unsigned int kfid;
        size_t obser;
        f.read((char*)&kfid,sizeof(kfid));
        f.read((char*)&obser,sizeof(obser));
        mmObservations[kfid]=obser;                              
        }

    //pointer

    //mmpRefKF
    bifmmpRefKFnull=false;
    f.read((char*)&bcont,sizeof(bcont));
    if(bcont==true) 
       f.read((char*)&mmpRefKF,sizeof(mmpRefKF));
    else
       bifmmpRefKFnull=true;

    //mmpReplaced
    bifreplacenull=false;
    f.read((char*)&bcont,sizeof(bcont));
    if(bcont==true)
       f.read((char*)&mmpReplaced,sizeof(mmpReplaced));
    else
       bifreplacenull=true;

            
    if(mmWorldPos.data!=NULL)
      {
       MapPoint* pMP = new MapPoint(this); 
       LoadMPDatabase.push_back(pMP); 
        }            

            }//end else
       }//end while







//mpReplaced
for(int i=0;i<LoadMPDatabase.size();i++)
    {
     MapPoint* pMPi = LoadMPDatabase[i];
     if(bifreplacenull==true)
        {
        pMPi->mpReplaced=NULL;
        continue;
        }

     long unsigned int idreplaced=pMPi->IdmpReplaced;
     for(int j=0;j<LoadMPDatabase.size();j++)
        {
        MapPoint* pMPireplaced = LoadMPDatabase[j];
        if(pMPireplaced->mnId==idreplaced)
           pMPi->mpReplaced=pMPireplaced;
         }

    }

//mpRefKF
for(int i=0;i<LoadMPDatabase.size();i++)
    {
     MapPoint* pMPi = LoadMPDatabase[i];
     if(bifmmpRefKFnull==true)
        {
        pMPi->mpRefKF=NULL;
        continue;
        }
     long unsigned int idRefKF=pMPi->IdmpRefKF;

     for(int j=0;j<LoadKFDatabase.size();j++)
         {
          KeyFrame* RefKF=LoadKFDatabase[j];
          if(RefKF->mnId==idRefKF)
             pMPi->mpRefKF=RefKF;
          }

     }



//mObservations
for(int i=0;i<LoadMPDatabase.size();i++)
    {
     MapPoint* pMPi = LoadMPDatabase[i];   
     if(pMPi->IdmObservations.size()==0)
        continue;
     for(auto mit=pMPi->IdmObservations.begin();mit!=pMPi->IdmObservations.end();mit++)
        {
         long unsigned int kfid=mit->first;
         size_t mpindex=mit->second;
         for(int j=0;j<LoadKFDatabase.size();j++)
             {
              KeyFrame* pKFiobser=LoadKFDatabase[j];
              if(pKFiobser->mnId==kfid)                 
                   pMPi->mObservations[pKFiobser]=mpindex;                  
              }

         }

     }

//Connect KeyFrames and MapPoints
addmvpMapPoints();

for(int i=0;i<LoadMPDatabase.size();i++)
    {
     MapPoint* pMPi = LoadMPDatabase[i]; 
     mpMap->mspMapPoints.insert(pMPi);
     }

return true;
}



bool System::createKeyFrameDatabase()
{
   for(int i=0;i<LoadKFDatabase.size();i++)
      {
        KeyFrame* pKFi=LoadKFDatabase[i];
        mpKeyFrameDatabase->add(pKFi);

      }

//about pointers
mpTracker->mpKeyFrameDB=mpKeyFrameDatabase;

return true;
}

void System::addmvpMapPoints()
{
   for(int i=0;i<LoadKFDatabase.size();i++)
      {
        KeyFrame* pKFi=LoadKFDatabase[i];
        (pKFi->mvpMapPoints).resize(pKFi->N);
        for(auto mit=pKFi->IdmvpMapPoints.begin();mit!=pKFi->IdmvpMapPoints.end();mit++)
           {
             int mpindex=mit->first;
             long unsigned int mpid=mit->second;
             for(int j=0;j<LoadMPDatabase.size();j++)
                 {
                   MapPoint* pMPi=LoadMPDatabase[j];
                   if(pMPi->mnId==mpid)
                      (pKFi->mvpMapPoints)[mpindex]=pMPi;

                  }            
            }

       }

}
bool System::checkIsResetFinish()
{
    unique_lock<mutex> lock(mMutexReset);
    return !mbReset;
}
////////////////////////////////////////ZBH//////////////////////
bool System::USE_IMU=false;
int  System::inteval=0;
/////////////////////////////////////////////////////////////////
} //namespace ORB_SLAM
