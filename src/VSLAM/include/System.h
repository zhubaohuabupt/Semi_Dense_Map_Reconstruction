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


#ifndef SYSTEM_H
#define SYSTEM_H

#include<string>
#include<thread>
#include<opencv2/core/core.hpp>

#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"
namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class Tracking;
class LocalMapping;
class LoopClosing;

class System
{
public:
    // Input sensor
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2
    };
public:
	~System();
    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,const int mode, const bool bUseViewer);

    // Proccess the given stereo frame. Images must be synchronized and rectified.
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp);

    // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
    // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Input depthmap: Float (CV_32F).
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const long &timestamp, bool& bIsKeyFrame,float* SceneDepth);

    // Proccess the given monocular frame
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp);

    // This stops local mapping thread (map building) and performs only camera tracking.
    void ActivateLocalizationMode();
    // This resumes local mapping thread and performs SLAM again.
    void DeactivateLocalizationMode();

    // Reset the system (clear map)
    void Reset();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();

    // Save camera trajectory in the TUM RGB-D dataset format.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveTrajectoryTUM(const string &filename);

    // Save keyframe poses in the TUM RGB-D dataset format.
    // Use this function in the monocular case.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveKeyFrameTrajectoryTUM(const string &filename);

    // Save camera trajectory in the KITTI dataset format.
    // Call first Shutdown()
    // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
    void SaveTrajectoryKITTI(const string &filename);

    // TODO: Save/Load functions
    // SaveMap(const string &filename);
    // LoadMap(const string &filename);

	// KeyFrame database for place recognition (relocalization and loop detection).
    KeyFrameDatabase* mpKeyFrameDatabase;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    Map* mpMap;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    LocalMapping* mpLocalMapper;

    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    LoopClosing* mpLoopCloser;

    bool LoadKFFile(const string &strKFFile,const string &kfsz);
    bool LoadMPFile(const string &strMPFile);
    bool LoadDBFile(const string &strDBFile);

    void addmvpMapPoints();

    bool createKeyFrameDatabase();
    
    //About KeyFrame

    std::vector<KeyFrame*> LoadKFDatabase;

    long unsigned int kmnId;
    long unsigned int knNextId; 
    long unsigned int kmnFrameId;
    double kmTimeStamp;
    int kmnGridCols;
    int kmnGridRows;
    float kmfGridElementWidthInv;
    float kmfGridElementHeightInv;
    long unsigned int kmnTrackReferenceForFrame;
    long unsigned int kmnFuseTargetForKF;
    long unsigned int kmnBALocalForKF;
    long unsigned int kmnBAFixedForKF;
    long unsigned int kmnLoopQuery;
    int kmnLoopWords;
    float kmLoopScore;
    long unsigned int kmnRelocQuery;
    int kmnRelocWords;
    float kmRelocScore;
    long unsigned int kmnBAGlobalForKF;
    float kfx,kfy,kcx,kcy,kinvfx,kinvfy,kmbf,kmb,kmThDepth;
    int kN;

    int kmnScaleLevels;
    float kmfScaleFactor;
    float kmfLogScaleFactor;
    int kmnMinX,kmnMinY,kmnMaxX,kmnMaxY;
    bool kmbFirstConnection;
    bool kmbNotErase,kmbToBeErased,kmbBad;
    float kmHalfBaseline;

    
    cv::Mat kmTcwGBA;
    cv::Mat kmTcwBefGBA;
    cv::Mat kmDescriptors;
    cv::Mat kmTcp;
    cv::Mat kmK;
    cv::Mat kTcw;
    cv::Mat kTwc;
    cv::Mat kOw;
    cv::Mat kCw;

    std::vector<cv::KeyPoint> kmvKeys;
    std::vector<cv::KeyPoint> kmvKeysUn;
    std::vector<float> kmvuRight;
    std::vector<float> kmvDepth;
    std::vector<float> kmvScaleFactors;
    std::vector<float> kmvLevelSigma2; 
    std::vector<float> kmvInvLevelSigma2;
    std::map<int,long unsigned int> kmvpMapPoints;               //MapPoint*
    std::vector< std::vector <std::vector<size_t> > > kmGrid;
    std::vector<long unsigned int> kmvpOrderedConnectedKeyFrames;//KeyFrame*
    std::vector<int> kmvOrderedWeights;

    DBoW2::BowVector kmBowVec;
    DBoW2::FeatureVector kmFeatVec;  
    //std::map<unsigned int,double> kmBowVec;
    //std::map<unsigned int,std::vector<unsigned int>> kmFeatVec;  
    std::map<long unsigned int,int> kmConnectedKeyFrameWeights;  //KeyFrame*

    std::set<long unsigned int> kmspChildrens;                   //KeyFrame*
    std::set<long unsigned int> kmspLoopEdges;                   //KeyFrame*
    
    long unsigned int kmpParent;                                  //KeyFrame*      


    //about MapPoint

    std::vector<MapPoint*> LoadMPDatabase;

    long unsigned int mmnId;
    long unsigned int mnNextId;
    long int mmnFirstKFid;
    long int mmnFirstFrame;
    int mnObs;
    float mmTrackProjX;
    float mmTrackProjY;
    float mmTrackProjXR;
    bool mmbTrackInView;
    int mmnTrackScaleLevel;
    float mmTrackViewCos;
    long unsigned int mmnTrackReferenceForFrame;
    long unsigned int mmnLastFrameSeen;
    long unsigned int mmnBALocalForKF;
    long unsigned int mmnFuseCandidateForKF;
    long unsigned int mmnLoopPointForKF;
    long unsigned int mmnCorrectedByKF;
    long unsigned int mmnCorrectedReference;  
    long unsigned int mmnBAGlobalForKF;
    int mmnVisible;
    int mmnFound;
    bool mmbBad;
    float mmfMinDistance;
    float mmfMaxDistance;

    cv::Mat mmPosGBA;
    cv::Mat mmWorldPos;
    cv::Mat mmNormalVector;
    cv::Mat mmDescriptor;

    std::map<long unsigned int,size_t> mmObservations;            //KeyFrame*

    long unsigned int mmpRefKF;                                   //KeyFrame*
    long unsigned int mmpReplaced;                                //MapPoint*

    bool bifreplacenull;
    bool bifmmpRefKFnull;

    //about InvertedFile
    std::vector<list<KeyFrame*> > ImvInvertedFile;
   
	// Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    Tracking* mpTracker;
    static bool USE_IMU;
    static int inteval;
    cv::Mat IMUdetaT_t_tlast;

	//about reset
	bool checkIsResetFinish();
private:

    // Input sensor
    eSensor mSensor;

    // ORB vocabulary used for place recognition and feature matching.
    ORBVocabulary* mpVocabulary;  

    // The viewer draws the map and the current camera pose. It uses Pangolin.
    Viewer* mpViewer;

    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;


    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    std::thread* mptLocalMapping;
    std::thread* mptLoopClosing;
    std::thread* mptViewer;


    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;

    // Change mode flags
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;
};

}// namespace ORB_SLAM

#endif // SYSTEM_H
