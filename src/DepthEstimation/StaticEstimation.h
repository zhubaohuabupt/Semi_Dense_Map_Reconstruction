/*This file is part of AntennaDetect-SemiDenseReconstruction.
 * StaticEstimation.h
 *
 *  Created on: 2017年6月1日
 *      Author: Zuber
 */
#include"DepthEstimation.h"
#include<deque>
namespace DepthEstimation {

class SingleObs
{
public:
    SingleObs( float depth_, float var_):depth(depth_),var(var_){}
    float depth;
    float var;
};
class StaticEstimation: public DepthEstimation
{
public:
    StaticEstimation(ComData*ComData_);
    virtual void CalDepth( cv::Mat &DEPTH_RESULT);
    virtual void testshow();
private:
    std::deque<vFrame> mFrameDeque;
    vector< std::list<Seed, aligned_allocator<Seed> > >allseeds_;
    //设定待匹配像素点
    void getSourcePoint(FramePtr frame, const std::vector<std::vector<cv::Point>>& DetectedAntenna,float MEANdepth);
    //从多次匹配结果里找出最大相容观测值的集合
    bool findValueObswithKaFang( const vector<SingleObs>&MultiObs, vector<SingleObs>&aftercheck);
    bool KaFangcheck(const SingleObs&obs1,const SingleObs&obs2);
    //融合多次观测值
    void fuseMultiValueObs( const vector<SingleObs>&MultiObs,float& Depthafterfuse,float &Varafterfuse,bool&);
    //去噪平滑
    void Denose_Smoothing(cv::Mat &);
    void show();
    bool SHOW;
};
}
