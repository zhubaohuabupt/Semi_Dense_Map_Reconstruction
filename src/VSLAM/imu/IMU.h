#ifndef IMU_H
#define IMU_H

#include <iostream>
#include <vector>
#include <fstream>

#include"Thirdparty/Sophus/sophus/se3.h"
#include "include/Frame.h"
#include "eigen_utils.h"
#include<string>
#include<stdio.h>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace Sophus;
namespace ORB_SLAM2
{

using namespace std;
using namespace Eigen;

class IMU
{
    private:
        double imu_delta_t, last_imu_delta_t;//imu sample period
	const Matrix<double,3,1> earth_acc;
	const Matrix<double,3,1> earth_omega;
	Matrix<double,3,1> ba, bg;
	SE3* T_imu_cam;
	Quaternion<double> q_imu2ground;
    public:
         string imudatapath;//
         int inteval;
        Matrix<double,3,1> earth_acc_in_w;
        Matrix<double,3,1> earth_omega_in_w;

	Matrix<double,3,1> r_new, r_old;
	Matrix<double,3,1> v_old, v_new;
  	SE3 T_test;
	
	double t_imu;//timestamp of imu
	double delta_t;//equal to imu_delta_t in common, be used to do integration
	vector<vector<double> > imu_measurements;
 	vector<double> last_measurement;
	vector<double> measurement;//store imu measurement
	ifstream imu_data;//csv file of imu
	ifstream pic_data;//csv file of dataset
	ifstream imu_data_;
       ifstream GT_trans;
	double pic_timestamp;
	double pic_last_timestamp;
	Matrix<double,3,1> acc;//acceleration measured by imu
	Matrix<double,3,1> omega;//angle velocity measured by imu

        //Euler
	double heading, attitude, bank;
    private:
	void get_pic_timestamp(); 
        void readpic_timestamp_inteval(int inteval);//
    public:
	IMU(const string& imudatapath_,int inteval_);
	void sync(int readskipnum);
	void load_imu_data();
	
       void calculate_pose( cv::Mat& deta_tnext_tcur_imu);
 void calculate_pose_( cv::Mat& deta_tnext_tcur_imu);
 void calculate_pose( Sophus::SE3 & deta_tnext_tcur_imu);
	void integration(const vector<double>& imu_data);
        void trans_to_Euler(Quaternion<double> q);
};

}

#endif
