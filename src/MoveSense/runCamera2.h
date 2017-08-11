/*
 * runCamera.h
 *
 *  Created on: 2016-5-14
 *      Author: robot
 */

#ifndef RUNCAMERA2_H_
#define RUNCAMERA2_H_
#include<cv.h>
void setCamera2Parameter(int alpha_slider_1=40,int alpha_slider_2=4,int alpha_slider_3=16,int alpha_slider_4=32);
void closeCamera();
int runCamera2(cv::Mat &left1,cv::Mat &disp);
void on_trackbar_1( int, void* );
void on_trackbar_2( int, void* );
void on_trackbar_3( int, void* );
void on_trackbar_4( int, void* );

#endif /* RUNCAMERA_H_ */
