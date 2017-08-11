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

#include "KeyFrame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include  <mutex>

#include "System.h"

extern int numofpoints;
using namespace std;

namespace ORB_SLAM2
{

long unsigned int KeyFrame::nNextId=0;

KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB):
    mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
    fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
    mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
    mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
    mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
    mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
    mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
    mnMaxY(F.mnMaxY), mK(F.mK), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
    mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
    mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb/2), mpMap(pMap)
{
    mnId=nNextId++;
    setKeyPoints();//get  five keypoint for choose localmap
 /*   cv::Mat showpts=F.mpORBextractorLeft->mvImagePyramid[0].clone();
    for(int i=0;i<5;i++)
    {cout<< FivePts[i].pts.pt<<endl;
        circle(showpts,Point(  FivePts[i].pts.pt.x,FivePts[i].pts.pt.y  ),  5,cv::Scalar(255,0,0));

    }
    cv::imshow("",showpts);
    cv::waitKey(0);
*/
    mGrid.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++)
            mGrid[i][j] = F.mGrid[i][j];
    }

    SetPose(F.mTcw);    
}

KeyFrame::KeyFrame(System* pSLAM):
    mnId(pSLAM->kmnId), mnFrameId(pSLAM->kmnFrameId), mTimeStamp(pSLAM->kmTimeStamp), mnGridCols(pSLAM->kmnGridCols),
    mnGridRows(pSLAM->kmnGridRows), mfGridElementWidthInv(pSLAM->kmfGridElementWidthInv), mfGridElementHeightInv(pSLAM->kmfGridElementHeightInv),
    mnTrackReferenceForFrame(pSLAM->kmnTrackReferenceForFrame), mnFuseTargetForKF(pSLAM->kmnFuseTargetForKF), mnBALocalForKF(pSLAM->kmnBALocalForKF),
    mnBAFixedForKF(pSLAM->kmnBAFixedForKF), mnLoopQuery(pSLAM->kmnLoopQuery), mnLoopWords(pSLAM->kmnLoopWords), mLoopScore(pSLAM->kmLoopScore),
    mnRelocQuery(pSLAM->kmnRelocQuery), mnRelocWords(pSLAM->kmnRelocWords), mRelocScore(pSLAM->kmRelocScore),
    mnBAGlobalForKF(pSLAM->kmnBAGlobalForKF), fx(pSLAM->kfx), fy(pSLAM->kfy), cx(pSLAM->kcx), cy(pSLAM->kcy),
    invfx(pSLAM->kinvfx), invfy(pSLAM->kinvfy), mbf(pSLAM->kmbf), mb(pSLAM->kmb), mThDepth(pSLAM->kmThDepth), N(pSLAM->kN),
    mnScaleLevels(pSLAM->kmnScaleLevels), mfScaleFactor(pSLAM->kmfScaleFactor),  mfLogScaleFactor(pSLAM->kmfLogScaleFactor), 
    mnMinX(pSLAM->kmnMinX), mnMinY(pSLAM->kmnMinY), mnMaxX(pSLAM->kmnMaxX), mnMaxY(pSLAM->kmnMaxY), mbFirstConnection(pSLAM->kmbFirstConnection),
    mbNotErase(pSLAM->kmbNotErase), mbToBeErased(pSLAM->kmbToBeErased), mbBad(pSLAM->kmbBad), mHalfBaseline(pSLAM->kmHalfBaseline),
    mDescriptors(pSLAM->kmDescriptors.clone()), mK(pSLAM->kmK.clone()), mvKeys(pSLAM->kmvKeys), mvKeysUn(pSLAM->kmvKeysUn), mvuRight(pSLAM->kmvuRight),
    mvDepth(pSLAM->kmvDepth), mvScaleFactors(pSLAM->kmvScaleFactors), mvLevelSigma2(pSLAM->kmvLevelSigma2), 
    mvInvLevelSigma2(pSLAM->kmvInvLevelSigma2), mGrid(pSLAM->kmGrid), mvOrderedWeights(pSLAM->kmvOrderedWeights)
   
{   
    nNextId=pSLAM->knNextId;
   setKeyPoints();//get  five keypoint for choose localmap
    if(pSLAM->kmTcwGBA.data!=NULL)
       mTcwGBA = pSLAM->kmTcwGBA.clone();

    if(pSLAM->kmTcwBefGBA.data!=NULL)
       mTcwBefGBA = pSLAM->kmTcwBefGBA.clone();

    if(pSLAM->kmTcp.data!=NULL)
       mTcp = pSLAM->kmTcp.clone();
    

    if(pSLAM->kTcw.data!=NULL)
       Tcw = pSLAM->kTcw.clone();

    if(pSLAM->kTwc.data!=NULL)
       Twc = pSLAM->kTwc.clone();

    if(pSLAM->kOw.data!=NULL)
       Ow = pSLAM->kOw.clone();

    if(pSLAM->kCw.data!=NULL)
       Cw = pSLAM->kCw.clone();

    mBowVec = pSLAM->kmBowVec;
    mFeatVec = pSLAM->kmFeatVec;

    IdmvpMapPoints = pSLAM->kmvpMapPoints;
    IdmvpOrderedConnectedKeyFrames = pSLAM->kmvpOrderedConnectedKeyFrames;
    IdmConnectedKeyFrameWeights = pSLAM->kmConnectedKeyFrameWeights;
    IdmspChildrens = pSLAM->kmspChildrens;
    IdmspLoopEdges = pSLAM->kmspLoopEdges;
    IdmpParent = pSLAM->kmpParent;

    KFandId[this]=mnId;

}


void KeyFrame::ComputeBoW()
{
    if(mBowVec.empty() || mFeatVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void KeyFrame::SetPose(const cv::Mat &Tcw_)
{
    unique_lock<mutex> lock(mMutexPose);
    Tcw_.copyTo(Tcw);
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    Ow = -Rwc*tcw;

    Twc = cv::Mat::eye(4,4,Tcw.type());
    Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    Ow.copyTo(Twc.rowRange(0,3).col(3));
    cv::Mat center = (cv::Mat_<float>(4,1) << mHalfBaseline, 0 , 0, 1);
    Cw = Twc*center;
}

cv::Mat KeyFrame::GetPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.clone();
}

cv::Mat KeyFrame::GetPoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return Twc.clone();
}

cv::Mat KeyFrame::GetCameraCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Ow.clone();
}

cv::Mat KeyFrame::GetStereoCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Cw.clone();
}


cv::Mat KeyFrame::GetRotation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).colRange(0,3).clone();
}

cv::Mat KeyFrame::GetTranslation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).col(3).clone();
}

void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight)
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(!mConnectedKeyFrameWeights.count(pKF))
            mConnectedKeyFrameWeights[pKF]=weight;
        else if(mConnectedKeyFrameWeights[pKF]!=weight)
            mConnectedKeyFrameWeights[pKF]=weight;
        else
            return;
    }

    UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles()
{
    unique_lock<mutex> lock(mMutexConnections);
    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
       vPairs.push_back(make_pair(mit->second,mit->first));

    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());    
}

set<KeyFrame*> KeyFrame::GetConnectedKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    set<KeyFrame*> s;
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
        s.insert(mit->first);
    return s;
}

vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mvpOrderedConnectedKeyFrames;
}

vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    unique_lock<mutex> lock(mMutexConnections);
    if((int)mvpOrderedConnectedKeyFrames.size()<N)
        return mvpOrderedConnectedKeyFrames;
    else
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);

}

vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(const int &w)
{
    unique_lock<mutex> lock(mMutexConnections);

    if(mvpOrderedConnectedKeyFrames.empty())
        return vector<KeyFrame*>();

    vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,KeyFrame::weightComp);
    if(it==mvOrderedWeights.end())
        return vector<KeyFrame*>();
    else
    {
        int n = it-mvOrderedWeights.begin();
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
    }
}

int KeyFrame::GetWeight(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexConnections);
    if(mConnectedKeyFrameWeights.count(pKF))
        return mConnectedKeyFrameWeights[pKF];
    else
        return 0;
}

void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=pMP;
}

void KeyFrame::EraseMapPointMatch(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}

void KeyFrame::EraseMapPointMatch(MapPoint* pMP)
{
    int idx = pMP->GetIndexInKeyFrame(this);
    if(idx>=0)
        mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}


void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP)
{
    mvpMapPoints[idx]=pMP;
}

set<MapPoint*> KeyFrame::GetMapPoints()
{
    unique_lock<mutex> lock(mMutexFeatures);
    set<MapPoint*> s;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        MapPoint* pMP = mvpMapPoints[i];
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}

int KeyFrame::TrackedMapPoints(const int &minObs)
{
    unique_lock<mutex> lock(mMutexFeatures);

    int nPoints=0;
    const bool bCheckObs = minObs>0;
    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = mvpMapPoints[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(bCheckObs)
                {
                    if(mvpMapPoints[i]->Observations()>=minObs)
                        nPoints++;
                }
                else
                    nPoints++;
            }
        }
    }

    return nPoints;
}

vector<MapPoint*> KeyFrame::GetMapPointMatches()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints;
}

MapPoint* KeyFrame::GetMapPoint(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints[idx];
}

void KeyFrame::UpdateConnections()
{
    map<KeyFrame*,int> KFcounter;

    vector<MapPoint*> vpMP;

    {
        unique_lock<mutex> lockMPs(mMutexFeatures);
        vpMP = mvpMapPoints;
    }

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    for(vector<MapPoint*>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;

        if(!pMP)
            continue;

        if(pMP->isBad())
            continue;

        map<KeyFrame*,size_t> observations = pMP->GetObservations();

        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            if(mit->first->mnId==mnId)
                continue;
            KFcounter[mit->first]++;
        }
    }

    // This should not happen
    if(KFcounter.empty())
        return;

    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    int nmax=0;
    KeyFrame* pKFmax=NULL;
    int th = 15;

    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(KFcounter.size());
    for(map<KeyFrame*,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
    {
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pKFmax=mit->first;
        }
        if(mit->second>=th)
        {
            vPairs.push_back(make_pair(mit->second,mit->first));
            (mit->first)->AddConnection(this,mit->second);
        }
    }

    if(vPairs.empty())
    {
        vPairs.push_back(make_pair(nmax,pKFmax));
        pKFmax->AddConnection(this,nmax);
    }

    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    {
        unique_lock<mutex> lockCon(mMutexConnections);

        // mspConnectedKeyFrames = spConnectedKeyFrames;
        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

        if(mbFirstConnection && mnId!=0)
        {
            mpParent = mvpOrderedConnectedKeyFrames.front();
            mpParent->AddChild(this);
            mbFirstConnection = false;
        }

    }
}

void KeyFrame::AddChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.erase(pKF);
}

void KeyFrame::ChangeParent(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mpParent = pKF;
    pKF->AddChild(this);
}

set<KeyFrame*> KeyFrame::GetChilds()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens;
}

KeyFrame* KeyFrame::GetParent()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mpParent;
}

bool KeyFrame::hasChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens.count(pKF);
}

void KeyFrame::AddLoopEdge(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspLoopEdges.insert(pKF);
}

set<KeyFrame*> KeyFrame::GetLoopEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspLoopEdges;
}

void KeyFrame::SetNotErase()
{
    unique_lock<mutex> lock(mMutexConnections);
    mbNotErase = true;
}

void KeyFrame::SetErase()
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mspLoopEdges.empty())
        {
            mbNotErase = false;
        }
    }

    if(mbToBeErased)
    {
        SetBadFlag();
    }
}

void KeyFrame::SetBadFlag()
{   
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mnId==0)
            return;
        else if(mbNotErase)
        {
            mbToBeErased = true;
            return;
        }
    }

    for(map<KeyFrame*,int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
        mit->first->EraseConnection(this);

    for(size_t i=0; i<mvpMapPoints.size(); i++)
        if(mvpMapPoints[i])
            mvpMapPoints[i]->EraseObservation(this);
    {
        unique_lock<mutex> lock(mMutexConnections);
        unique_lock<mutex> lock1(mMutexFeatures);

        mConnectedKeyFrameWeights.clear();
        mvpOrderedConnectedKeyFrames.clear();

        // Update Spanning Tree
        set<KeyFrame*> sParentCandidates;
        sParentCandidates.insert(mpParent);

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        while(!mspChildrens.empty())
        {
            bool bContinue = false;

            int max = -1;
            KeyFrame* pC;
            KeyFrame* pP;

            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(), send=mspChildrens.end(); sit!=send; sit++)
            {
                KeyFrame* pKF = *sit;
                if(pKF->isBad())
                    continue;

                // Check if a parent candidate is connected to the keyframe
                vector<KeyFrame*> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
                {
                    for(set<KeyFrame*>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                    {
                        if(vpConnected[i]->mnId == (*spcit)->mnId)
                        {
                            int w = pKF->GetWeight(vpConnected[i]);
                            if(w>max)
                            {
                                pC = pKF;
                                pP = vpConnected[i];
                                max = w;
                                bContinue = true;
                            }
                        }
                    }
                }
            }

            if(bContinue)
            {
                pC->ChangeParent(pP);
                sParentCandidates.insert(pC);
                mspChildrens.erase(pC);
            }
            else
                break;
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        if(!mspChildrens.empty())
            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)
            {
                (*sit)->ChangeParent(mpParent);
            }

        mpParent->EraseChild(this);
        mTcp = Tcw*mpParent->GetPoseInverse();
        mbBad = true;
    }


    mpMap->EraseKeyFrame(this);
    mpKeyFrameDB->erase(this);
}

bool KeyFrame::isBad()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mbBad;
}

void KeyFrame::EraseConnection(KeyFrame* pKF)
{
    bool bUpdate = false;
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mConnectedKeyFrameWeights.count(pKF))
        {
            mConnectedKeyFrameWeights.erase(pKF);
            bUpdate=true;
        }
    }

    if(bUpdate)
        UpdateBestCovisibles();
}

vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=mnGridCols)
        return vIndices;

    const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=mnGridRows)
        return vIndices;

    const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool KeyFrame::IsInImage(const float &x, const float &y) const
{
    return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
}

cv::Mat KeyFrame::UnprojectStereo(int i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeys[i].pt.x;
        const float v = mvKeys[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

        unique_lock<mutex> lock(mMutexPose);
        return Twc.rowRange(0,3).colRange(0,3)*x3Dc+Twc.rowRange(0,3).col(3);
    }
    else
        return cv::Mat();
}

float KeyFrame::ComputeSceneMedianDepth(const int q)
{
    vector<MapPoint*> vpMapPoints;
    cv::Mat Tcw_;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPose);
        vpMapPoints = mvpMapPoints;
        Tcw_ = Tcw.clone();
    }

    vector<float> vDepths;
    vDepths.reserve(N);
    cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);
    Rcw2 = Rcw2.t();
    float zcw = Tcw_.at<float>(2,3);
    for(int i=0; i<N; i++)
    {
        if(mvpMapPoints[i])
        {
            MapPoint* pMP = mvpMapPoints[i];
            cv::Mat x3Dw = pMP->GetWorldPos();
            float z = Rcw2.dot(x3Dw)+zcw;
            vDepths.push_back(z);
        }
    }

    sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}

int KeyFrame::saveKeyFrame(const std::string &filename)
{   
    fstream f;
    f.open(filename.c_str(),ios_base::out|ios::binary|ios::app);
	if(!f.is_open())
		return -1;
    //int...float...(10 9 10 10 2)
    f.write((char*)&mnId,sizeof(mnId));
    f.write((char*)&nNextId,sizeof(nNextId));
    f.write((char*)&mnFrameId,sizeof(mnFrameId));
    f.write((char*)&mTimeStamp,sizeof(mTimeStamp));
    f.write((char*)&mnGridCols,sizeof(mnGridCols));
    f.write((char*)&mnGridRows,sizeof(mnGridRows));
    f.write((char*)&mfGridElementWidthInv,sizeof(mfGridElementWidthInv));
    f.write((char*)&mfGridElementHeightInv,sizeof(mfGridElementHeightInv));
    f.write((char*)&mnTrackReferenceForFrame,sizeof(mnTrackReferenceForFrame));
    f.write((char*)&mnFuseTargetForKF,sizeof(mnFuseTargetForKF));

    f.write((char*)&mnBALocalForKF,sizeof(mnBALocalForKF));
    f.write((char*)&mnBAFixedForKF,sizeof(mnBAFixedForKF));
    f.write((char*)&mnLoopQuery,sizeof(mnLoopQuery));
    f.write((char*)&mnLoopWords,sizeof(mnLoopWords));
    f.write((char*)&mLoopScore,sizeof(mLoopScore));
    f.write((char*)&mnRelocQuery,sizeof(mnRelocQuery));
    f.write((char*)&mnRelocWords,sizeof(mnRelocWords));
    f.write((char*)&mRelocScore,sizeof(mRelocScore));
    f.write((char*)&mnBAGlobalForKF,sizeof(mnBAGlobalForKF));

    f.write((char*)&fx,sizeof(fx));
    f.write((char*)&fy,sizeof(fy));
    f.write((char*)&cx,sizeof(cx));
    f.write((char*)&cy,sizeof(cy));
    f.write((char*)&invfx,sizeof(invfx));
    f.write((char*)&invfy,sizeof(invfy));
    f.write((char*)&mbf,sizeof(mbf));
    f.write((char*)&mb,sizeof(mb));
    f.write((char*)&mThDepth,sizeof(mThDepth));
    f.write((char*)&N,sizeof(N));

    f.write((char*)&mnScaleLevels,sizeof(mnScaleLevels));
    f.write((char*)&mfScaleFactor,sizeof(mfScaleFactor));
    f.write((char*)&mfLogScaleFactor,sizeof(mfLogScaleFactor));
    f.write((char*)&mnMinX,sizeof(mnMinX));
    f.write((char*)&mnMinY,sizeof(mnMinY));
    f.write((char*)&mnMaxX,sizeof(mnMaxX));
    f.write((char*)&mnMaxY,sizeof(mnMaxY));
    f.write((char*)&mbFirstConnection,sizeof(mbFirstConnection));//cout<<mbFirstConnection<<endl;getchar();
    f.write((char*)&mbNotErase,sizeof(mbNotErase));
    f.write((char*)&mbToBeErased,sizeof(mbToBeErased));

    f.write((char*)&mbBad,sizeof(mbBad));
    f.write((char*)&mHalfBaseline,sizeof(mHalfBaseline));

    bool strnull=false;
    bool strcont=true;

    //Mat...

    //mTcwGBA(4,4,32F)    
    if(mTcwGBA.data==NULL)
       f.write((char*)&strnull,sizeof(strnull));
    else{
       f.write((char*)&strcont,sizeof(strcont));

       f.write((char*)&(mTcwGBA.at<float>(0,0)),sizeof(mTcwGBA.at<float>(0,0)));
       f.write((char*)&(mTcwGBA.at<float>(0,1)),sizeof(mTcwGBA.at<float>(0,1)));
       f.write((char*)&(mTcwGBA.at<float>(0,2)),sizeof(mTcwGBA.at<float>(0,2)));
       f.write((char*)&(mTcwGBA.at<float>(0,3)),sizeof(mTcwGBA.at<float>(0,3)));

       f.write((char*)&(mTcwGBA.at<float>(1,0)),sizeof(mTcwGBA.at<float>(1,0)));
       f.write((char*)&(mTcwGBA.at<float>(1,1)),sizeof(mTcwGBA.at<float>(1,1)));
       f.write((char*)&(mTcwGBA.at<float>(1,2)),sizeof(mTcwGBA.at<float>(1,2)));
       f.write((char*)&(mTcwGBA.at<float>(1,3)),sizeof(mTcwGBA.at<float>(1,3)));

       f.write((char*)&(mTcwGBA.at<float>(2,0)),sizeof(mTcwGBA.at<float>(2,0)));
       f.write((char*)&(mTcwGBA.at<float>(2,1)),sizeof(mTcwGBA.at<float>(2,1)));
       f.write((char*)&(mTcwGBA.at<float>(2,2)),sizeof(mTcwGBA.at<float>(2,2)));
       f.write((char*)&(mTcwGBA.at<float>(2,3)),sizeof(mTcwGBA.at<float>(2,3)));

       f.write((char*)&(mTcwGBA.at<float>(3,0)),sizeof(mTcwGBA.at<float>(3,0)));
       f.write((char*)&(mTcwGBA.at<float>(3,1)),sizeof(mTcwGBA.at<float>(3,1)));
       f.write((char*)&(mTcwGBA.at<float>(3,2)),sizeof(mTcwGBA.at<float>(3,2)));
       f.write((char*)&(mTcwGBA.at<float>(3,3)),sizeof(mTcwGBA.at<float>(3,3)));
         }

    //mTcwBefGBA(4,4,F)
    if(mTcwBefGBA.data==NULL)
       f.write((char*)&strnull,sizeof(strnull));
    else{
       f.write((char*)&strcont,sizeof(strcont));

       f.write((char*)&(mTcwBefGBA.at<float>(0,0)),sizeof(mTcwBefGBA.at<float>(0,0)));
       f.write((char*)&(mTcwBefGBA.at<float>(0,1)),sizeof(mTcwBefGBA.at<float>(0,1)));
       f.write((char*)&(mTcwBefGBA.at<float>(0,2)),sizeof(mTcwBefGBA.at<float>(0,2)));
       f.write((char*)&(mTcwBefGBA.at<float>(0,3)),sizeof(mTcwBefGBA.at<float>(0,3)));

       f.write((char*)&(mTcwBefGBA.at<float>(1,0)),sizeof(mTcwBefGBA.at<float>(1,0)));
       f.write((char*)&(mTcwBefGBA.at<float>(1,1)),sizeof(mTcwBefGBA.at<float>(1,1)));
       f.write((char*)&(mTcwBefGBA.at<float>(1,2)),sizeof(mTcwBefGBA.at<float>(1,2)));
       f.write((char*)&(mTcwBefGBA.at<float>(1,3)),sizeof(mTcwBefGBA.at<float>(1,3)));

       f.write((char*)&(mTcwBefGBA.at<float>(2,0)),sizeof(mTcwBefGBA.at<float>(2,0)));
       f.write((char*)&(mTcwBefGBA.at<float>(2,1)),sizeof(mTcwBefGBA.at<float>(2,1)));
       f.write((char*)&(mTcwBefGBA.at<float>(2,2)),sizeof(mTcwBefGBA.at<float>(2,2)));
       f.write((char*)&(mTcwBefGBA.at<float>(2,3)),sizeof(mTcwBefGBA.at<float>(2,3)));

       f.write((char*)&(mTcwBefGBA.at<float>(3,0)),sizeof(mTcwBefGBA.at<float>(3,0)));
       f.write((char*)&(mTcwBefGBA.at<float>(3,1)),sizeof(mTcwBefGBA.at<float>(3,1)));
       f.write((char*)&(mTcwBefGBA.at<float>(3,2)),sizeof(mTcwBefGBA.at<float>(3,2)));
       f.write((char*)&(mTcwBefGBA.at<float>(3,3)),sizeof(mTcwBefGBA.at<float>(3,3)));
         }
  
    //mDescriptors(N*32)          
    for(int i=0;i<mDescriptors.rows;++i)
        {
         stringstream ss;
         const unsigned char *p = mDescriptors.ptr<unsigned char>(i);
         for(int j=0;j<mDescriptors.cols;++j,++p)
            {
             int tempintdesc=(int)*p;
             f.write((char*)&tempintdesc,sizeof(tempintdesc));
             //cout<<tempintdesc<<" ";
             }
         //cout<<endl;getchar();
        }

    //mTcp(4,4,F)
    if(mTcp.data==NULL)
       f.write((char*)&strnull,sizeof(strnull));
    else{
       f.write((char*)&strcont,sizeof(strcont));

       f.write((char*)&(mTcp.at<float>(0,0)),sizeof(mTcp.at<float>(0,0)));
       f.write((char*)&(mTcp.at<float>(0,1)),sizeof(mTcp.at<float>(0,1)));
       f.write((char*)&(mTcp.at<float>(0,2)),sizeof(mTcp.at<float>(0,2)));
       f.write((char*)&(mTcp.at<float>(0,3)),sizeof(mTcp.at<float>(0,3)));

       f.write((char*)&(mTcp.at<float>(1,0)),sizeof(mTcp.at<float>(1,0)));
       f.write((char*)&(mTcp.at<float>(1,1)),sizeof(mTcp.at<float>(1,1)));
       f.write((char*)&(mTcp.at<float>(1,2)),sizeof(mTcp.at<float>(1,2)));
       f.write((char*)&(mTcp.at<float>(1,3)),sizeof(mTcp.at<float>(1,3)));

       f.write((char*)&(mTcp.at<float>(2,0)),sizeof(mTcp.at<float>(2,0)));
       f.write((char*)&(mTcp.at<float>(2,1)),sizeof(mTcp.at<float>(2,1)));
       f.write((char*)&(mTcp.at<float>(2,2)),sizeof(mTcp.at<float>(2,2)));
       f.write((char*)&(mTcp.at<float>(2,3)),sizeof(mTcp.at<float>(2,3)));

       f.write((char*)&(mTcp.at<float>(3,0)),sizeof(mTcp.at<float>(3,0)));
       f.write((char*)&(mTcp.at<float>(3,1)),sizeof(mTcp.at<float>(3,1)));
       f.write((char*)&(mTcp.at<float>(3,2)),sizeof(mTcp.at<float>(3,2)));
       f.write((char*)&(mTcp.at<float>(3,3)),sizeof(mTcp.at<float>(3,3)));
        }

    //mK(3,3,32F)
    if(mK.data==NULL)
       f.write((char*)&strnull,sizeof(strnull));
    else{
       f.write((char*)&strcont,sizeof(strcont));

       f.write((char*)&(mK.at<float>(0,0)),sizeof(mK.at<float>(0,0)));
       f.write((char*)&(mK.at<float>(0,1)),sizeof(mK.at<float>(0,1)));
       f.write((char*)&(mK.at<float>(0,2)),sizeof(mK.at<float>(0,2)));

       f.write((char*)&(mK.at<float>(1,0)),sizeof(mK.at<float>(1,0)));
       f.write((char*)&(mK.at<float>(1,1)),sizeof(mK.at<float>(1,1)));
       f.write((char*)&(mK.at<float>(1,2)),sizeof(mK.at<float>(1,2)));

       f.write((char*)&(mK.at<float>(2,0)),sizeof(mK.at<float>(2,0)));
       f.write((char*)&(mK.at<float>(2,1)),sizeof(mK.at<float>(2,1)));
       f.write((char*)&(mK.at<float>(2,2)),sizeof(mK.at<float>(2,2)));
           }    

    //Tcw(4,4,F)
    if(Tcw.data==NULL)
       f.write((char*)&strnull,sizeof(strnull));
    else{
       f.write((char*)&strcont,sizeof(strcont));

       f.write((char*)&(Tcw.at<float>(0,0)),sizeof(Tcw.at<float>(0,0)));
       f.write((char*)&(Tcw.at<float>(0,1)),sizeof(Tcw.at<float>(0,1)));
       f.write((char*)&(Tcw.at<float>(0,2)),sizeof(Tcw.at<float>(0,2)));
       f.write((char*)&(Tcw.at<float>(0,3)),sizeof(Tcw.at<float>(0,3)));

       f.write((char*)&(Tcw.at<float>(1,0)),sizeof(Tcw.at<float>(1,0)));
       f.write((char*)&(Tcw.at<float>(1,1)),sizeof(Tcw.at<float>(1,1)));
       f.write((char*)&(Tcw.at<float>(1,2)),sizeof(Tcw.at<float>(1,2)));
       f.write((char*)&(Tcw.at<float>(1,3)),sizeof(Tcw.at<float>(1,3)));

       f.write((char*)&(Tcw.at<float>(2,0)),sizeof(Tcw.at<float>(2,0)));
       f.write((char*)&(Tcw.at<float>(2,1)),sizeof(Tcw.at<float>(2,1)));
       f.write((char*)&(Tcw.at<float>(2,2)),sizeof(Tcw.at<float>(2,2)));
       f.write((char*)&(Tcw.at<float>(2,3)),sizeof(Tcw.at<float>(2,3)));

       f.write((char*)&(Tcw.at<float>(3,0)),sizeof(Tcw.at<float>(3,0)));
       f.write((char*)&(Tcw.at<float>(3,1)),sizeof(Tcw.at<float>(3,1)));
       f.write((char*)&(Tcw.at<float>(3,2)),sizeof(Tcw.at<float>(3,2)));
       f.write((char*)&(Tcw.at<float>(3,3)),sizeof(Tcw.at<float>(3,3)));
        }

    //Twc(4,4,F)
    if(Twc.data==NULL)
       f.write((char*)&strnull,sizeof(strnull));
    else{
       f.write((char*)&strcont,sizeof(strcont));

       f.write((char*)&(Twc.at<float>(0,0)),sizeof(Twc.at<float>(0,0)));
       f.write((char*)&(Twc.at<float>(0,1)),sizeof(Twc.at<float>(0,1)));
       f.write((char*)&(Twc.at<float>(0,2)),sizeof(Twc.at<float>(0,2)));
       f.write((char*)&(Twc.at<float>(0,3)),sizeof(Twc.at<float>(0,3)));

       f.write((char*)&(Twc.at<float>(1,0)),sizeof(Twc.at<float>(1,0)));
       f.write((char*)&(Twc.at<float>(1,1)),sizeof(Twc.at<float>(1,1)));
       f.write((char*)&(Twc.at<float>(1,2)),sizeof(Twc.at<float>(1,2)));
       f.write((char*)&(Twc.at<float>(1,3)),sizeof(Twc.at<float>(1,3)));

       f.write((char*)&(Twc.at<float>(2,0)),sizeof(Twc.at<float>(2,0)));
       f.write((char*)&(Twc.at<float>(2,1)),sizeof(Twc.at<float>(2,1)));
       f.write((char*)&(Twc.at<float>(2,2)),sizeof(Twc.at<float>(2,2)));
       f.write((char*)&(Twc.at<float>(2,3)),sizeof(Twc.at<float>(2,3)));

       f.write((char*)&(Twc.at<float>(3,0)),sizeof(Twc.at<float>(3,0)));
       f.write((char*)&(Twc.at<float>(3,1)),sizeof(Twc.at<float>(3,1)));
       f.write((char*)&(Twc.at<float>(3,2)),sizeof(Twc.at<float>(3,2)));
       f.write((char*)&(Twc.at<float>(3,3)),sizeof(Twc.at<float>(3,3)));
        }

    //Ow(3,1,F)
    if(Ow.data==NULL)
       f.write((char*)&strnull,sizeof(strnull));
    else{
       f.write((char*)&strcont,sizeof(strcont));

       f.write((char*)&(Ow.at<float>(0,0)),sizeof(Ow.at<float>(0,0)));
       f.write((char*)&(Ow.at<float>(1,0)),sizeof(Ow.at<float>(1,0)));
       f.write((char*)&(Ow.at<float>(2,0)),sizeof(Ow.at<float>(2,0)));
        }

    //Cw(4,1,F)
    if(Cw.data==NULL)
       f.write((char*)&strnull,sizeof(strnull));
    else{
       f.write((char*)&strcont,sizeof(strcont));

       f.write((char*)&(Cw.at<float>(0,0)),sizeof(Cw.at<float>(0,0)));
       f.write((char*)&(Cw.at<float>(1,0)),sizeof(Cw.at<float>(1,0)));
       f.write((char*)&(Cw.at<float>(2,0)),sizeof(Cw.at<float>(2,0)));
       f.write((char*)&(Cw.at<float>(3,0)),sizeof(Cw.at<float>(3,0)));
        }

    //vector

    //mvKeys(N*7)
    for(int i=0;i<N;i++)
       {
         f.write((char*)&(mvKeys[i].pt.x),sizeof(mvKeys[i].pt.x));
         f.write((char*)&(mvKeys[i].pt.y),sizeof(mvKeys[i].pt.y));

         f.write((char*)&(mvKeys[i].size),sizeof(mvKeys[i].size));
         f.write((char*)&(mvKeys[i].angle),sizeof(mvKeys[i].angle));
         f.write((char*)&(mvKeys[i].response),sizeof(mvKeys[i].response));
         f.write((char*)&(mvKeys[i].octave),sizeof(mvKeys[i].octave));
         f.write((char*)&(mvKeys[i].class_id),sizeof(mvKeys[i].class_id));
        }

    //mvKeysUn(mvKeysUn.size()*7)
    int mvkeysunsize=mvKeysUn.size();
    f.write((char*)&(mvkeysunsize),sizeof(mvkeysunsize));
    for(int i=0;i<mvKeysUn.size();i++)
       {
         f.write((char*)&(mvKeysUn[i].pt.x),sizeof(mvKeysUn[i].pt.x));
         f.write((char*)&(mvKeysUn[i].pt.y),sizeof(mvKeysUn[i].pt.y));

         f.write((char*)&(mvKeysUn[i].size),sizeof(mvKeysUn[i].size));
         f.write((char*)&(mvKeysUn[i].angle),sizeof(mvKeysUn[i].angle));
         f.write((char*)&(mvKeysUn[i].response),sizeof(mvKeysUn[i].response));
         f.write((char*)&(mvKeysUn[i].octave),sizeof(mvKeysUn[i].octave));
         f.write((char*)&(mvKeysUn[i].class_id),sizeof(mvKeysUn[i].class_id));
        }

    //mvuRight
    int mvurightsize=mvuRight.size();
    f.write((char*)&(mvurightsize),sizeof(mvurightsize));
    for(int i=0;i<mvuRight.size();i++)
       f.write((char*)&(mvuRight[i]),sizeof(mvuRight[i]));

    //mvDepth
    int mvdepthsize=mvDepth.size();
    f.write((char*)&(mvdepthsize),sizeof(mvdepthsize));
    for(int i=0;i<mvDepth.size();i++)
       f.write((char*)&(mvDepth[i]),sizeof(mvDepth[i]));

    //mvScaleFactors
    int mvscalefactorssize=mvScaleFactors.size();
    f.write((char*)&(mvscalefactorssize),sizeof(mvscalefactorssize));
    for(int i=0;i<mvScaleFactors.size();i++)
       f.write((char*)&(mvScaleFactors[i]),sizeof(mvScaleFactors[i]));

    //mvLevelSigma2
    int mvLevelSigma2size=mvLevelSigma2.size();
    f.write((char*)&(mvLevelSigma2size),sizeof(mvLevelSigma2size));
    for(int i=0;i<mvLevelSigma2.size();i++)
       f.write((char*)&(mvLevelSigma2[i]),sizeof(mvLevelSigma2[i]));

    //mvInvLevelSigma2
    int mvInvLevelSigma2size=mvInvLevelSigma2.size();
    f.write((char*)&(mvInvLevelSigma2size),sizeof(mvInvLevelSigma2size));
    for(int i=0;i<mvInvLevelSigma2.size();i++)
       f.write((char*)&(mvInvLevelSigma2[i]),sizeof(mvInvLevelSigma2[i]));

    //mvpMapPoints(num+mnId)    
    int MPCounter=0;
    for(auto i=0;i<mvpMapPoints.size();i++)
       {        
         if(mvpMapPoints[i]!=NULL)            
            MPCounter++;                              
       }
    f.write((char*)&(MPCounter),sizeof(MPCounter));
    for(int i=0;i<mvpMapPoints.size();i++)
       {        
         if(mvpMapPoints[i]!=NULL) 
            {           
            f.write((char*)&i,sizeof(i));
            f.write((char*)&(mvpMapPoints[i]->mnId),sizeof(mvpMapPoints[i]->mnId)); //cout<<i<<" "<<mvpMapPoints[i]->mnId<<endl;getchar();
            }                           
       }  

    //mGrid(64*48*n+st)
    //f<<mGrid.size()<<" ";
    for(auto i=0;i!=mGrid.size();i++)
        {
          vector<vector<size_t> >layer1=mGrid[i];
          //f<<layer1.size()<<" ";
          for(auto j=0;j!=layer1.size();j++)
             {
               vector<size_t>layer2=layer1[j];

               int layer2size=layer2.size();
               f.write((char*)&layer2size,sizeof(layer2size));

               for(auto k=0;k!=layer2.size();k++)
                   {
                     size_t st=layer2[k];
                     f.write((char*)&st,sizeof(st));
                    }
              }
      
         }

    //mvpOrderedConnectedKeyFrames
    int mvpOrderedConnectedKeyFramessize=mvpOrderedConnectedKeyFrames.size();
    f.write((char*)&mvpOrderedConnectedKeyFramessize,sizeof(mvpOrderedConnectedKeyFramessize));
    for(int i=0;i<mvpOrderedConnectedKeyFrames.size();i++)
        f.write((char*)&(mvpOrderedConnectedKeyFrames[i]->mnId),sizeof(mvpOrderedConnectedKeyFrames[i]->mnId)); 

    //mvOrderedWeights
    int mvOrderedWeightssize=mvOrderedWeights.size();
    f.write((char*)&mvOrderedWeightssize,sizeof(mvOrderedWeightssize));
    for(int i=0;i<mvOrderedWeights.size();i++)
       f.write((char*)&(mvOrderedWeights[i]),sizeof(mvOrderedWeights[i]));

    //map

    //mBowVec(n*(1 2))
    int mBowVecsize=mBowVec.size();
    f.write((char*)&mBowVecsize,sizeof(mBowVecsize));  
    for(DBoW2::BowVector::const_iterator vit=mBowVec.begin(), vend=mBowVec.end(); vit!=vend; vit++)
        {
          f.write((char*)&(vit->first),sizeof(vit->first));
          f.write((char*)&(vit->second),sizeof(vit->second));//cout<<vit->first<<" "<<vit->second<<endl;getchar();
         }

    //mFeatVec(n1 1 n2 2)
    int mFeatVecsize=mFeatVec.size();
    f.write((char*)&mFeatVecsize,sizeof(mFeatVecsize));   
    for(DBoW2::FeatureVector::const_iterator vit=mFeatVec.begin(), vend=mFeatVec.end(); vit!=vend; vit++)
        {
          f.write((char*)&(vit->first),sizeof(vit->first));
          int mfeatsecondsize=vit->second.size();
          f.write((char*)&mfeatsecondsize,sizeof(mfeatsecondsize));

          for(size_t i=0;i!=vit->second.size();i++) 
              f.write((char*)&(vit->second[i]),sizeof(vit->second[i]));     
         }

    //mConnectedKeyFrameWeights(n 1 2)
    int mConnectedKeyFrameWeightssize=mConnectedKeyFrameWeights.size();
    f.write((char*)&mConnectedKeyFrameWeightssize,sizeof(mConnectedKeyFrameWeightssize));
    for(auto mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
        {
          f.write((char*)&(mit->first->mnId),sizeof(mit->first->mnId));
          f.write((char*)&(mit->second),sizeof(mit->second));
         }

    //set

    //mspChildrens
    int mspChildrenssize=mspChildrens.size();
    f.write((char*)&mspChildrenssize,sizeof(mspChildrenssize));
    for(set<KeyFrame*>::iterator sit=mspChildrens.begin(), send=mspChildrens.end(); sit!=send; sit++)
        f.write((char*)&((*sit)->mnId),sizeof((*sit)->mnId));
       
    //mspLoopEdges
    int mspLoopEdgessize=mspLoopEdges.size();
    f.write((char*)&mspLoopEdgessize,sizeof(mspLoopEdgessize));
    for(set<KeyFrame*>::iterator sit=mspLoopEdges.begin(), send=mspLoopEdges.end(); sit!=send; sit++)
        f.write((char*)&((*sit)->mnId),sizeof((*sit)->mnId));

    //pointer

    //mpParent
    if(mpParent!=NULL){
       f.write((char*)&strcont,sizeof(strcont));
       f.write((char*)&(mpParent->mnId),sizeof(mpParent->mnId));
        }
    else
       f.write((char*)&strnull,sizeof(strnull));

    f.close();
    return 0;
}
void KeyFrame::setKeyPoints()
{
    for(int i=0;i!=mvKeys.size();i++)
    {   Five_pt candidate;
        candidate.pts=mvKeys[i];
        candidate.depth=mvDepth[i];

        if( candidate.depth>0.5&&candidate.depth<100) checkKeyPoints(candidate);
    }
project3d();//get pos3d
}

void KeyFrame::checkKeyPoints(const Five_pt& ftr)
{
  const int cu = cx;
  const int cv = cy;

  // center pixel
  if(FivePts[0] .depth== 0)
    FivePts[0] = ftr;
  else if(std::max(std::fabs(ftr.pts.pt.x-cu), std::fabs(ftr.pts.pt.y-cv))
        < std::max(std::fabs(FivePts[0].pts.pt.x-cu), std::fabs(FivePts[0].pts.pt.y-cv)))
    FivePts[0] = ftr;

  if(ftr.pts.pt.x >= cu && ftr.pts.pt.y>= cv)
  {
    if(FivePts[1] .depth== 0)
    {
      FivePts[1] = ftr;
    }
    else if((ftr.pts.pt.x-cu) * (ftr.pts.pt.y-cv)
          > (FivePts[1].pts.pt.x-cu) * (FivePts[1].pts.pt.y-cv))
      FivePts[1] = ftr;
  }
  if(ftr.pts.pt.x >= cu &&ftr.pts.pt.y < cv)
  {
    if(FivePts[2] .depth== 0)
      FivePts[2]= ftr;
    else if((ftr.pts.pt.x-cu) * (ftr.pts.pt.y-cv)
          <(FivePts[2].pts.pt.x-cu) * (FivePts[2].pts.pt.y-cv))
      FivePts[2]= ftr;
  }
  if(ftr.pts.pt.x < cu && ftr.pts.pt.y  < cv)
  {
    if(FivePts[3].depth == 0)
      FivePts[3] = ftr;
    else if((ftr.pts.pt.x-cu) * (ftr.pts.pt.y-cv)
          > (FivePts[3].pts.pt.x-cu) * (FivePts[3].pts.pt.y-cv))
      FivePts[3] = ftr;
  }
  if(ftr.pts.pt.x  < cu && ftr.pts.pt.y >= cv)
  {
    if(FivePts[4].depth == 0)
      FivePts[4] = ftr;
    else if((ftr.pts.pt.x-cu) *(ftr.pts.pt.y-cv)
          < (FivePts[4].pts.pt.x-cu) * (FivePts[4].pts.pt.y-cv))
      FivePts[4] = ftr;
  }
}
void KeyFrame::project3d()
{
    for(int i=0;i<5;i++)
    {
         float z=FivePts[i].depth;
        if(z>0)
        {
            const float u = FivePts[i].pts.pt.x;
            const float v = FivePts[i].pts.pt.y;
            const float x = (u-cx)*z*invfx;
            const float y = (v-cy)*z*invfy;
            FivePts[i].Point3d = (cv::Mat_<float>(3,1) << x, y,z);
        }
    }
}

} //namespace ORB_SLAM
