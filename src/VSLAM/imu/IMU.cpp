#include "imu/IMU.h"
#include <iomanip>
Quaternion<double> q_old;
Quaternion<double> q_new ;
namespace ORB_SLAM2
{

IMU::IMU( const string& imudatapath_,int inteval_):imu_delta_t(0),earth_acc(0,0,-9.8),earth_omega(0,0,0)
{
imudatapath=imudatapath_;
inteval=inteval_;
 ///////////////////mh01   
#ifdef mh01
   q_imu2ground.w() = 0.541956;
    q_imu2ground.x() = -0.064676987;
    q_imu2ground.y() = 0.009810731;
    q_imu2ground.z() = -0.093543;
#endif
#define v202
#ifdef v202
     q_imu2ground.w() = 0.592105;
    q_imu2ground.x() = 0.022393;
    q_imu2ground.y() = -0.805188;
    q_imu2ground.z() = 0.024127;
#endif

    v_old<<0,0,0;
    ba<<0,0,0;//0.0004,0.0004,0.0004;
     bg<<0,0,0;
    Matrix3d rotation;
    Vector3d translation(-0.021640145, -0.064676987, 0.0098107306);


    rotation<< 0.016350092, -0.99980514, 0.011061917,
               0.99971676, 0.016537997, 0.017113992,
                -0.017293599, 0.010778969, 0.99979235;
    T_imu_cam = new SE3(rotation, translation);
    T_imu_cam->translation() = translation;
    earth_acc_in_w = (q_imu2ground*T_imu_cam->unit_quaternion()).conjugate()._transformVector(earth_acc);//check
    Matrix<double,3,1> earth_acc_in_imu;
    earth_acc_in_imu = q_imu2ground.conjugate()._transformVector(earth_acc);//check
    //cout<<"earth_acc_in_w"<<earth_acc_in_w(0,0)<<","<<earth_acc_in_w(1,0)<<","<<earth_acc_in_w(2,0)<<endl;
    //cout<<"earth_acc_in_imu"<<earth_acc_in_imu(0,0)<<","<<earth_acc_in_imu(1,0)<<","<<earth_acc_in_imu(2,0)<<endl;
   
string picdata_Path=imudatapath+"/pic_data/data.csv";
string imudata_Path=imudatapath+"/imu_data/data.csv";
pic_data.open(picdata_Path);
imu_data.open(imudata_Path);
imu_data_.open(imudata_Path);
cout<<"picdata_Path : "<<picdata_Path<<endl; cout<<"inteval:  "<<inteval<<endl; 
    /*trans_to_Euler(q_imu2ground);
    cout<<"heading:"<<heading<<endl;
    cout<<"attitude:"<<attitude<<endl;
    cout<<"bank:"<<bank<<endl;*/
}

void IMU::load_imu_data()
{
    measurement.clear();
    measurement.reserve(7);
    char a[25],c[25];
    double b,t_temp;
    for(int i=0;i<7;i++)
    {
        if(i==6)
	{
	    imu_data.getline(a,25);
	    imu_data_.getline(c,25);
	}
	else
	{
	    imu_data.getline(a,25,',');
	    imu_data_.getline(c,25,',');
	}        
	if(i==0)
	{
	    t_temp = atof(c);
	}
	b = atof(a);
        measurement.push_back(b);
    }
    t_imu = measurement[0];
    imu_delta_t = t_temp - t_imu;
}



void IMU::calculate_pose( cv::Mat& deta_tnext_tcur_imu)
{
//相机到imu的矩阵
     Mat T_imu_cam_Mat=cv::Mat(4,4,CV_32FC1);
     T_imu_cam_Mat.at<float>(0,0)=0.016350092, T_imu_cam_Mat.at<float>(0,1)=-0.99980514,T_imu_cam_Mat.at<float>(0,2)=0.011061917,T_imu_cam_Mat.at<float>(0,3)=-0.021640145;
     T_imu_cam_Mat.at<float>(1,0)= 0.99971676, T_imu_cam_Mat.at<float>(1,1)=0.016537997,T_imu_cam_Mat.at<float>(1,2)=0.017113992,T_imu_cam_Mat.at<float>(1,3)=-0.064676987;
     T_imu_cam_Mat.at<float>(2,0)=  -0.017293599, T_imu_cam_Mat.at<float>(2,1)=0.010778969,T_imu_cam_Mat.at<float>(2,2)=0.99979235,T_imu_cam_Mat.at<float>(2,3)=0.0098107306;
	 T_imu_cam_Mat.at<float>(3,0)=  0.0, T_imu_cam_Mat.at<float>(3,1)=0.0,T_imu_cam_Mat.at<float>(3,2)=0.0,T_imu_cam_Mat.at<float>(3,3)=1;

//得到t时刻imu  到  t时刻相机的矩阵//此时积的是变化量，
Mat tmp_IMU_camt=T_imu_cam_Mat;//*last_frame->mTcw;

Matrix3d rotation;
rotation<<tmp_IMU_camt.at<float>(0,0),tmp_IMU_camt.at<float>(0,1),tmp_IMU_camt.at<float>(0,2),
		   tmp_IMU_camt.at<float>(1,0),tmp_IMU_camt.at<float>(1,1),tmp_IMU_camt.at<float>(1,2),
		  tmp_IMU_camt.at<float>(2,0),tmp_IMU_camt.at<float>(2,1),tmp_IMU_camt.at<float>(2,2) ;
//cout<<"rotation   "<<rotation<<endl;
VectorXd translation(3);
translation<<tmp_IMU_camt.at<float>(0,3),tmp_IMU_camt.at<float>(1,3),tmp_IMU_camt.at<float>(2,3);
	
//转变回SE3
SE3 T_IMU_camt(rotation,translation);

 r_old = T_IMU_camt.inverse().translation();
Quaternion<double> qq;
qq= T_IMU_camt.inverse().unit_quaternion().conjugate();//
q_old =qq;
 //  get_pic_timestamp();
get_pic_timestamp();
get_pic_timestamp();
get_pic_timestamp();
//get_pic_timestamp();

//cout<<fixed<<setprecision(22)<<"pic_last_timestamp: "<<pic_last_timestamp<<endl<<" timu  "<<t_imu<<endl;;
//getchar();
    while(t_imu+imu_delta_t<=pic_timestamp)
    {  

        if(pic_last_timestamp-t_imu<imu_delta_t&&pic_last_timestamp-t_imu>0)
	{

	    delta_t = (t_imu + imu_delta_t - pic_last_timestamp)*1e-9;

	    integration(last_measurement);

	    r_old = r_new;
	    v_old = v_new;
	    q_old = q_new;
	}
	else
	{

     		delta_t = imu_delta_t*1e-9;
		integration(measurement);
		r_old = r_new;
	        v_old = v_new;
	        q_old = q_new;	


	}
        last_imu_delta_t = imu_delta_t;
	load_imu_data();
    }
    if(t_imu+imu_delta_t>pic_timestamp)
    {
	delta_t = (pic_timestamp - t_imu)*1e-9;
	integration(measurement);
	r_old = r_new;
	v_old = v_new;
	q_old = q_new;
	last_measurement.clear();
	last_measurement = measurement;
    }
    pic_last_timestamp = pic_timestamp;



   SE3  newframeT= SE3(Matrix3d::Identity(), Vector3d::Zero());
    newframeT.setQuaternion(q_new.conjugate());
   newframeT.translation() = r_new;
//此时newframeT  是t+1时刻的imu到t时刻相机的相对位姿
  //SE3->Mat
Matrix3d rot_new=newframeT.rotation_matrix();
Vector3d trans_new=newframeT.translation();
 Mat newframeT_camt_imu=cv::Mat(4,4,CV_32FC1);
      newframeT_camt_imu.at<float>(0,0)=rot_new(0,0),newframeT_camt_imu.at<float>(0,1)=rot_new(0,1),newframeT_camt_imu.at<float>(0,2)=rot_new(0,2),  newframeT_camt_imu.at<float>(0,3)=0;//trans_new(0);
       newframeT_camt_imu.at<float>(1,0)=rot_new(1,0),newframeT_camt_imu.at<float>(1,1)=rot_new(1,1),newframeT_camt_imu.at<float>(1,2)=rot_new(1,2),  newframeT_camt_imu.at<float>(1,3)=0;//trans_new(1);

      newframeT_camt_imu.at<float>(2,0)=rot_new(2,0),newframeT_camt_imu.at<float>(2,1)=rot_new(2,1),newframeT_camt_imu.at<float>(2,2)=rot_new(2,2),  newframeT_camt_imu.at<float>(2,3)=0;//trans_new(2);
     
     newframeT_camt_imu.at<float>(3,0)=0,newframeT_camt_imu.at<float>(3,1)=0,newframeT_camt_imu.at<float>(3,2)=0,  newframeT_camt_imu.at<float>(3,3)=1;

            
 deta_tnext_tcur_imu=(newframeT_camt_imu*T_imu_cam_Mat).inv();


}

//////////////////

void IMU::calculate_pose_( cv::Mat& deta_tnext_tcur_imu)
{
//相机到imu的矩阵
     Mat T_imu_cam_Mat=cv::Mat(4,4,CV_32FC1);
     T_imu_cam_Mat.at<float>(0,0)=0.016350092, T_imu_cam_Mat.at<float>(0,1)=-0.99980514,T_imu_cam_Mat.at<float>(0,2)=0.011061917,T_imu_cam_Mat.at<float>(0,3)=-0.021640145;
     T_imu_cam_Mat.at<float>(1,0)= 0.99971676, T_imu_cam_Mat.at<float>(1,1)=0.016537997,T_imu_cam_Mat.at<float>(1,2)=0.017113992,T_imu_cam_Mat.at<float>(1,3)=-0.064676987;
     T_imu_cam_Mat.at<float>(2,0)=  -0.017293599, T_imu_cam_Mat.at<float>(2,1)=0.010778969,T_imu_cam_Mat.at<float>(2,2)=0.99979235,T_imu_cam_Mat.at<float>(2,3)=0.0098107306;
	 T_imu_cam_Mat.at<float>(3,0)=  0.0, T_imu_cam_Mat.at<float>(3,1)=0.0,T_imu_cam_Mat.at<float>(3,2)=0.0,T_imu_cam_Mat.at<float>(3,3)=1;

//得到t时刻imu  到  t时刻相机的矩阵//此时积的是变化量，
Mat tmp_IMU_camt=T_imu_cam_Mat;//*last_frame->mTcw;

Matrix3d rotation;
rotation<<tmp_IMU_camt.at<float>(0,0),tmp_IMU_camt.at<float>(0,1),tmp_IMU_camt.at<float>(0,2),
		   tmp_IMU_camt.at<float>(1,0),tmp_IMU_camt.at<float>(1,1),tmp_IMU_camt.at<float>(1,2),
		  tmp_IMU_camt.at<float>(2,0),tmp_IMU_camt.at<float>(2,1),tmp_IMU_camt.at<float>(2,2) ;
//cout<<"rotation   "<<rotation<<endl;
VectorXd translation(3);
translation<<tmp_IMU_camt.at<float>(0,3),tmp_IMU_camt.at<float>(1,3),tmp_IMU_camt.at<float>(2,3);
	
//转变回SE3
SE3 T_IMU_camt(rotation,translation);
SE3 eye4=SE3(Matrix3d::Identity(), Vector3d::Zero());
 r_old = eye4.translation();
Quaternion<double> qq;
qq= eye4.unit_quaternion().conjugate();//
q_old =qq;

readpic_timestamp_inteval(inteval);

//cout<<fixed<<setprecision(22)<<"pic_last_timestamp: "<<pic_last_timestamp<<endl<<" timu  "<<t_imu<<endl;;

    while(t_imu+imu_delta_t<=pic_timestamp)
    {  

        if(pic_last_timestamp-t_imu<imu_delta_t&&pic_last_timestamp-t_imu>0)
	{

	    delta_t = (t_imu + imu_delta_t - pic_last_timestamp)*1e-9;

	    integration(last_measurement);

	    r_old = r_new;
	    v_old = v_new;
	    q_old = q_new;
	}
	else
	{

     		delta_t = imu_delta_t*1e-9;
		integration(measurement);
		r_old = r_new;
	        v_old = v_new;
	        q_old = q_new;	


	}
        last_imu_delta_t = imu_delta_t;
	load_imu_data();
    }
    if(t_imu+imu_delta_t>pic_timestamp)
    {
	delta_t = (pic_timestamp - t_imu)*1e-9;
	integration(measurement);
	r_old = r_new;
	v_old = v_new;
	q_old = q_new;
	last_measurement.clear();
	last_measurement = measurement;
    }
    pic_last_timestamp = pic_timestamp;



   SE3  newframeT= SE3(Matrix3d::Identity(), Vector3d::Zero());
    newframeT.setQuaternion(q_new.conjugate());
   newframeT.translation() = r_new;
//此时newframeT  是t+1时刻的imu到t时刻相机的相对位姿
  //SE3->Mat
Matrix3d rot_new=newframeT.rotation_matrix();
Vector3d trans_new=newframeT.translation();
 Mat newframeT_camt_imu=cv::Mat(4,4,CV_32FC1);
      newframeT_camt_imu.at<float>(0,0)=rot_new(0,0),newframeT_camt_imu.at<float>(0,1)=rot_new(0,1),newframeT_camt_imu.at<float>(0,2)=rot_new(0,2),  newframeT_camt_imu.at<float>(0,3)=0;//trans_new(0);
       newframeT_camt_imu.at<float>(1,0)=rot_new(1,0),newframeT_camt_imu.at<float>(1,1)=rot_new(1,1),newframeT_camt_imu.at<float>(1,2)=rot_new(1,2),  newframeT_camt_imu.at<float>(1,3)=0;//trans_new(1);

      newframeT_camt_imu.at<float>(2,0)=rot_new(2,0),newframeT_camt_imu.at<float>(2,1)=rot_new(2,1),newframeT_camt_imu.at<float>(2,2)=rot_new(2,2),  newframeT_camt_imu.at<float>(2,3)=0;//trans_new(2);
     
     newframeT_camt_imu.at<float>(3,0)=0,newframeT_camt_imu.at<float>(3,1)=0,newframeT_camt_imu.at<float>(3,2)=0,  newframeT_camt_imu.at<float>(3,3)=1;

      ///Timu_t_t+1      
 deta_tnext_tcur_imu=((T_imu_cam_Mat.inv())*newframeT_camt_imu*T_imu_cam_Mat).inv();


}
////////////////////

void IMU::calculate_pose( Sophus::SE3 & deta_tnext_tcur_imu)
{


SE3 T_IMU_camt=*T_imu_cam;

 r_old = T_IMU_camt.inverse().translation();
Quaternion<double> qq;
qq= T_IMU_camt.inverse().unit_quaternion().conjugate();//
q_old =qq;
    get_pic_timestamp();

get_pic_timestamp();
get_pic_timestamp();
get_pic_timestamp();

cout<<fixed<<setprecision(22)<<"pic_last_timestamp: "<<pic_last_timestamp<<endl<<" timu  "<<t_imu<<endl;;
//getchar();
    while(t_imu+imu_delta_t<=pic_timestamp)
    {  

        if(pic_last_timestamp-t_imu<imu_delta_t&&pic_last_timestamp-t_imu>0)
	{

	    delta_t = (t_imu + imu_delta_t - pic_last_timestamp)*1e-9;

	    integration(last_measurement);

	    r_old = r_new;
	    v_old = v_new;
	    q_old = q_new;
	}
	else
	{

     		delta_t = imu_delta_t*1e-9;
		integration(measurement);
		r_old = r_new;
	        v_old = v_new;
	        q_old = q_new;	


	}
        last_imu_delta_t = imu_delta_t;
	load_imu_data();
    }
    if(t_imu+imu_delta_t>pic_timestamp)
    {
	delta_t = (pic_timestamp - t_imu)*1e-9;
	integration(measurement);
	r_old = r_new;
	v_old = v_new;
	q_old = q_new;
	last_measurement.clear();
	last_measurement = measurement;
    }
    pic_last_timestamp = pic_timestamp;



   SE3  newframeT= SE3(Matrix3d::Identity(), Vector3d::Zero());
    newframeT.setQuaternion(q_new.conjugate());
   newframeT.translation() = r_new;
//此时newframeT  是t+1时刻的imu到t时刻相机的相对位姿
  

            
 deta_tnext_tcur_imu=(newframeT*(*T_imu_cam)).inverse();


}

void IMU::integration(const vector<double>& imu_data)
{
    
    for(int i=0;i<3;i++)
    {   
        omega(i,0) = imu_data[i+1] - bg[i];
    	acc(i,0) = imu_data[i+4] - ba[i];
    }

   // cout<<"omega"<<omega(0,0)<<","<<omega(1,0)<<","<<omega(2,0)<<endl;
    //cout<<"acc"<<acc(0,0)<<","<<acc(1,0)<<","<<acc(2,0)<<endl;
    Quaternion<double> qe = rvec2quat(earth_omega*delta_t);   


    q_new = q_old*qe;


    Quaternion<double> qb=rvec2quat(-omega*delta_t);//q_imu(new)_imu(old)

    q_new = qb*q_new;//q_imu(new)_w

    Eigen::Matrix<double,3,1> vel_inc1=(q_old.conjugate()._transformVector(acc*delta_t)+q_new.conjugate()._transformVector(acc*delta_t))*0.5;

    Eigen::Matrix<double,3,1> vel_inc2 = (earth_acc_in_w-2*earth_omega.cross(v_old))*delta_t;
  
    v_new = v_old+vel_inc1+vel_inc2;
    r_new = r_old+(v_new+v_old)*delta_t*0.5;
}

void IMU::sync(int readskipnum)
{   
    
    t_imu = 0;
    char a[25];
    for(int i=0;i<7;i++)
    {

        if(i==6)
		{

	           imu_data_.getline(a,25);	

		}
	else
	    imu_data_.getline(a,25,',');


    }
readpic_timestamp_inteval(readskipnum);
    //get_pic_timestamp();

    load_imu_data();

    while(t_imu+imu_delta_t<=pic_timestamp)
	 load_imu_data();
    if(t_imu+imu_delta_t>pic_timestamp&&t_imu!=pic_timestamp)
	last_measurement = measurement;
    pic_last_timestamp = pic_timestamp;
   // cout<<fixed<<setprecision(25)<<"t_imu :"<<t_imu<<"pic_timestamp: "<<pic_timestamp<<endl;
//getchar();
}
void IMU::get_pic_timestamp()
{
    char a[25];
    double b;
    pic_data.getline(a,25,',');
    b = atof(a);
    pic_timestamp = b;
    pic_data.getline(a,25);//just aim to skip pic's name
}
void IMU::readpic_timestamp_inteval(int inteval)
{
for(int i=0;i<inteval+1;i++)
  get_pic_timestamp();
}
void IMU::trans_to_Euler(Quaternion<double> q)
{
    double change = 3.1415926/180;
    if(q.x()*q.y()+q.z()*q.w()>0.499)
    {
	heading = 2*atan2(q.x(),q.w())/change;
	attitude = 3.1415926/2/change;
	bank = 0;
    }
    else if(q.x()*q.y()+q.z()*q.w()<-0.499)
    {
	heading = -2*atan2(q.x(),q.w())/change;
	attitude = -3.1415926/2/change;
	bank = 0;
    }
    else
    {    
    	heading = atan2(2*q.y()*q.w()-2*q.x()*q.z(),1-2*q.y()*q.y()-2*q.z()*q.z())/change;
    	attitude = asin(2*q.x()*q.y()+2*q.z()*q.w())/change;
    	bank = atan2(2*q.x()*q.w()-2*q.y()*q.z(),1-2*q.x()*q.x()-2*q.z()*q.z())/change;
    }
}
}
