#ifndef MoveSenseCamera_H_
#define MoveSenseCamera_H_

#include "CameraMode.h"
#include "MoveSenseErrorCode.h"

namespace movesense{

class MoveSenseCamera
{
public:
	MoveSenseCamera(CameraMode mode = CAM_STEREO_752X480_LR_30FPS);
	~MoveSenseCamera(void);


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

	void SetCameraMode(CameraMode cameraMode);
	CameraMode GetCameraMode();
};
}
#endif
