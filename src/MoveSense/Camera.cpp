#include "Camera.h"
#include "VL4IF.h"
#include "CameraCmd.h"
#include "pid_vid.h"

namespace movesense
{

	VL4_IF *g_cds = NULL;


	Camera::Camera(CameraMode mode):m_camera_opened(false),m_usbspeed(MOVESENSE_USB20)
	{
		int w = 752,h = 480;
		CAMERA_FPS fps = CAMERA_FPS_30;
		switch(mode)
		{
		case CAM_STEREO_752X480_LR_60FPS:
			w = 752;
			h = 480;
			fps = CAMERA_FPS_60;
			break;
		case CAM_STEREO_752X480_LR_30FPS:
			w = 752;
			h = 480;
			fps = CAMERA_FPS_30;
			break;
		case CAM_STEREO_752X480_LR_15FPS:
			w = 752;
			h = 480;
			fps = CAMERA_FPS_15;
			break;
		case CAM_STEREO_752X480_LR_10FPS:
			w = 752;
			h = 480;
			fps = CAMERA_FPS_10;
			break;
		case CAM_STEREO_752X480_LD_60FPS:
			w = 752;
			h = 480;
			fps = CAMERA_FPS_60;
			break;
		case CAM_STEREO_752X480_LD_30FPS:
			w = 752;
			h = 480;
			fps = CAMERA_FPS_30;
			break;
		case CAM_STEREO_752X480_LD_15FPS:
			w = 752;
			h = 480;
			fps = CAMERA_FPS_15;
			break;
		case CAM_STEREO_752X480_LD_10FPS:
			w = 752;
			h = 480;
			fps = CAMERA_FPS_10;
			break;
		case CAM_STEREO_752X480_LRD_60FPS:
			w = 752;
			h = 720;
			fps = CAMERA_FPS_60;
			break;
		case CAM_STEREO_752X480_LRD_30FPS:
			w = 752;
			h = 720;
			fps = CAMERA_FPS_30;
			break;
		case CAM_STEREO_752X480_LRD_15FPS:
			w = 752;
			h = 720;
			fps = CAMERA_FPS_15;
			break;
		case CAM_STEREO_752X480_LRD_10FPS:
			w = 752;
			h = 720;
			fps = CAMERA_FPS_10;
			break;
		case CAM_STEREO_376X240_LR_60FPS:
			w = 376;
			h = 240;
			fps = CAMERA_FPS_60;
			break;
		case CAM_STEREO_376X240_LR_30FPS:
			w = 376;
			h = 240;
			fps = CAMERA_FPS_30;
			break;
		case CAM_STEREO_376X240_LR_15FPS:
			w = 376;
			h = 240;
			fps = CAMERA_FPS_15;
			break;
		case CAM_STEREO_376X240_LR_10FPS:
			w = 376;
			h = 240;
			fps = CAMERA_FPS_10;
			break;
		case CAM_STEREO_376X240_LD_60FPS:
			w = 376;
			h = 240;
			fps = CAMERA_FPS_60;
			break;
		case CAM_STEREO_376X240_LD_30FPS:
			w = 376;
			h = 240;
			fps = CAMERA_FPS_30;
			break;
		case CAM_STEREO_376X240_LD_15FPS:
			w = 376;
			h = 240;
			fps = CAMERA_FPS_15;
			break;
		case CAM_STEREO_376X240_LD_10FPS:
			w = 376;
			h = 240;
			fps = CAMERA_FPS_10;
			break;
		case CAM_STEREO_376X240_LRD_60FPS:
			w = 376;
			h = 360;
			fps = CAMERA_FPS_60;
			break;
		case CAM_STEREO_376X240_LRD_30FPS:
			w = 376;
			h = 360;
			fps = CAMERA_FPS_30;
			break;
		case CAM_STEREO_376X240_LRD_15FPS:
			w = 376;
			h = 360;
			fps = CAMERA_FPS_15;
			break;
		case CAM_STEREO_376X240_LRD_10FPS:
			w = 376;
			h = 360;
			fps = CAMERA_FPS_10;
			break;
		default:break;
		}
		g_cds = new VL4_IF(w,h,fps);
		m_mode = mode;
	}


	Camera::~Camera(void)
	{
		delete g_cds;
	}

	MoveSenseErrorCode Camera::OpenCamera()
	{
		string name = g_cds->ScanCamera(g_vid_usb,g_pid_usb2);
		if(name.empty())
		{
			name = g_cds->ScanCamera(g_vid_usb,g_pid_usb3);
			if(name.empty())
			{
				return MS_ERROR;
			}else
			{
				m_usbspeed = MOVESENSE_USB20;
			}
		}else
		{
			m_usbspeed = MOVESENSE_USB30;
		}
		if(g_cds->OpenCamera(name)) //have test 2.0
		{
//			if(m_mode==CAM_STEREO_752X480_LD_60FPS || m_mode==CAM_STEREO_752X480_LD_30FPS || m_mode==CAM_STEREO_752X480_LD_25FPS
//					||m_mode==CAM_STEREO_752X480_LD_15FPS||m_mode==CAM_STEREO_752X480_LD_10FPS)
//				g_cds->CameraCmd(SETTINGS_SHOW_LD);
//			else
//				g_cds->CameraCmd(SETTINGS_SHOW_LR);
			g_cds->CameraCmd(m_mode);
			g_cds->CameraCmd(SETTINGS_CAPSTART);
			return MS_SUCCESS;
		}
		return MS_ERROR;
	}

	//�ر����
	void Camera::CloseCamera()
	{
		g_cds->CloseCamera();
	}

	int Camera::GetImageData(unsigned char * &data, int &len)
	{
		return g_cds->GetImageData(data,len);;
	}

	void Camera::SetGainValue(int value)
	{
		g_cds->SetGainValue(value);
	}
	void Camera::SetExposureValue(int value)
	{
		g_cds->SetExposureValue(value);
	}

	void Camera::SetAutoExposure(bool value)
	{
		if(value) g_cds->CameraCmd(SETTINGS_AUTOEXP_ON);
		else g_cds->CameraCmd(SETTINGS_AUTOEXP_OFF);
	}
	void Camera::SetAutoGain(bool value)
	{
		if(value) g_cds->CameraCmd(SETTINGS_AUTOGAIN_ON);
		else g_cds->CameraCmd(SETTINGS_AUTOGAIN_OFF);
	}

	void Camera::SetSM_P1(unsigned int value)
	{
		g_cds->SetSM_P1(value);
	}

	void Camera::SetSM_P2(unsigned int value)
	{
		g_cds->SetSM_P2(value);
	}

	void Camera::SetSM_P3(unsigned int value)
	{
		g_cds->SetSM_P3(value);
	}

	void Camera::SetSM_HoleFill(bool onoff)
	{
		if(onoff)
			g_cds->CameraCmd(SETTINGS_HOLEFILL_ON);
		else
			g_cds->CameraCmd(SETTINGS_HOLEFILL_OFF);
	}

	void Camera::SetSM_LRCheck(bool onoff)
	{
		if(onoff)
			g_cds->CameraCmd(SETTINGS_LRCHECK_ON);
		else
			g_cds->CameraCmd(SETTINGS_LRCHECK_OFF);	
	}

	void Camera::SetSM_Subpixel(bool onoff)
	{
		if(onoff)
			g_cds->CameraCmd(SETTINGS_SUBPIX_ON);
		else
			g_cds->CameraCmd(SETTINGS_SUBPIX_OFF);	
	}

	void Camera::SetSM_MedianFilter(bool onoff)
	{
		if(onoff)
			g_cds->CameraCmd(SETTINGS_MEDIAN_ON);
		else
			g_cds->CameraCmd(SETTINGS_MEDIAN_OFF);		
	}

	void Camera::SetHDR(bool onoff)
	{
		if(onoff)
			g_cds->CameraCmd(SETTINGS_HDRON);
		else
			g_cds->CameraCmd(SETTINGS_HDROFF);	
	}

	void Camera::SetUndistort(bool onoff)
	{
		if(onoff)
			g_cds->CameraCmd(SETTINGS_DISTORTON);
		else
			g_cds->CameraCmd(SETTINGS_DISTORTOFF);		
	}

	void Camera::CameraCmd(unsigned char cmd)
	{
		g_cds->CameraCmd(cmd);
	}
	void Camera::SetCameraMode(CameraMode cameraMode)
	{
		if(m_mode!=cameraMode)
		{
			m_mode = cameraMode;
			int w = 752,h = 480;
			CAMERA_FPS fps = CAMERA_FPS_30;
			switch(m_mode)
			{
			case CAM_STEREO_752X480_LR_60FPS:
				w = 752;
				h = 480;
				fps = CAMERA_FPS_60;
				break;
			case CAM_STEREO_752X480_LR_30FPS:
				w = 752;
				h = 480;
				fps = CAMERA_FPS_30;
				break;
			case CAM_STEREO_752X480_LR_15FPS:
				w = 752;
				h = 480;
				fps = CAMERA_FPS_15;
				break;
			case CAM_STEREO_752X480_LR_10FPS:
				w = 752;
				h = 480;
				fps = CAMERA_FPS_10;
				break;
			case CAM_STEREO_752X480_LD_60FPS:
				w = 752;
				h = 480;
				fps = CAMERA_FPS_60;
				break;
			case CAM_STEREO_752X480_LD_30FPS:
				w = 752;
				h = 480;
				fps = CAMERA_FPS_30;
				break;
			case CAM_STEREO_752X480_LD_15FPS:
				w = 752;
				h = 480;
				fps = CAMERA_FPS_15;
				break;
			case CAM_STEREO_752X480_LD_10FPS:
				w = 752;
				h = 480;
						fps = CAMERA_FPS_10;
				break;
			case CAM_STEREO_752X480_LRD_60FPS:
				w = 752;
				h = 720;
				fps = CAMERA_FPS_60;
				break;
			case CAM_STEREO_752X480_LRD_30FPS:
				w = 752;
				h = 720;
				fps = CAMERA_FPS_30;
				break;
			case CAM_STEREO_752X480_LRD_15FPS:
				w = 752;
				h = 720;
				fps = CAMERA_FPS_15;
				break;
			case CAM_STEREO_752X480_LRD_10FPS:
				w = 752;
				h = 720;
				fps = CAMERA_FPS_10;
				break;
			default:break;
			}
			delete g_cds;
			g_cds = new VL4_IF(w,h,fps);
		}
	}
	CameraMode Camera::GetCameraMode()
	{
		return m_mode;
	}

	void Camera::SetDesireBin(int value)
	{
		g_cds->SetDesireBin(value);
	}
}
