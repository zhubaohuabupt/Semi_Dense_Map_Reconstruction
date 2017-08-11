#include "Camera.h"
#include "MoveSenseCamera2.h"

namespace movesense{

	Camera *g_camif = 0;

MoveSenseCamera::MoveSenseCamera(CameraMode mode)
{
	g_camif = new Camera(mode);
}


MoveSenseCamera::~MoveSenseCamera(void)
{
	delete g_camif;
}


MoveSenseErrorCode MoveSenseCamera::OpenCamera(){
	return g_camif->OpenCamera();
}


void MoveSenseCamera::CloseCamera(){
	g_camif->CloseCamera();
}
	

int MoveSenseCamera::GetImageData(unsigned char * &data, int &len){
	return g_camif->GetImageData(data,len);
}


void MoveSenseCamera::SetGainValue(int value){
	g_camif->SetGainValue(value);
}

void MoveSenseCamera::SetExposureValue(int value){
	g_camif->SetExposureValue(value);
}

void MoveSenseCamera::SetAutoExposure(bool value){
	g_camif->SetAutoExposure(value);
}
void MoveSenseCamera::SetAutoGain(bool value){
	g_camif->SetAutoGain(value);	
}

void MoveSenseCamera::SetSM_P1(unsigned int value){
	g_camif->SetSM_P1(value);
}

void MoveSenseCamera::SetSM_P2(unsigned int value){
	g_camif->SetSM_P2(value);
}

void MoveSenseCamera::SetSM_P3(unsigned int value){
	g_camif->SetSM_P3(value);
}

void MoveSenseCamera::SetSM_HoleFill(bool onoff){

	g_camif->SetSM_HoleFill(onoff);
}

void MoveSenseCamera::SetSM_LRCheck(bool onoff){
	g_camif->SetSM_LRCheck(onoff);
}

void MoveSenseCamera::SetSM_Subpixel(bool onoff){
	g_camif->SetSM_Subpixel(onoff);
}

void MoveSenseCamera::SetSM_MedianFilter(bool onoff){
	g_camif->SetSM_MedianFilter(onoff);
}

void MoveSenseCamera::SetHDR(bool onoff){
	g_camif->SetHDR(onoff);
}

void MoveSenseCamera::SetUndistort(bool onoff){
	g_camif->SetUndistort(onoff);
}

void MoveSenseCamera::SetCameraMode(CameraMode cameraMode){
	g_camif->SetCameraMode(cameraMode);	
}

CameraMode MoveSenseCamera::GetCameraMode(){
	return 	g_camif->GetCameraMode();
}

void MoveSenseCamera::SetDesireBin(int value)
{
	g_camif->SetDesireBin(value);
}

}
