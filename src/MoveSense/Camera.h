#pragma once
#include "CameraMode.h"
#include "MoveSenseErrorCode.h"
namespace movesense
{

using namespace movesense;

	typedef enum{
		MOVESENSE_USB20,
		MOVESENSE_USB30,
	}MoveSenseUSBSpeed;

class Camera
{

public:
	Camera(CameraMode mode = CAM_STEREO_752X480_LR_30FPS);
	~Camera();

public:
	MoveSenseErrorCode OpenCamera();


	void CloseCamera();
	

	int GetImageData(unsigned char * &data, int &len);


	void SetGainValue(int value);
	void SetExposureValue(int value);

	void SetDesireBin(int value);
	void SetAutoExposure(bool value);
	void SetAutoGain(bool value);

	void SetSM_P1(unsigned int value);

	void SetSM_P2(unsigned int value);

	void SetSM_P3(unsigned int value);

	void SetSM_HoleFill(bool onoff);

	void SetSM_LRCheck(bool onoff);

	void SetSM_Subpixel(bool onoff);

	void SetSM_MedianFilter(bool onoff);

	void SetHDR(bool onoff);

	void SetUndistort(bool onoff);

	void CameraCmd(unsigned char);
	void SetCameraMode(CameraMode cameraMode);
	CameraMode GetCameraMode();

private:
	bool m_camera_opened;
	MoveSenseUSBSpeed  m_usbspeed;
	CameraMode m_mode;
};

}

