/*
 * CameraCtrl.cpp
 *
 *  Created on: May 17, 2016
 *      Author: renyin
 */

#include "CameraCtrl.h"
#include "CameraCmd.h"
#include "pid_vid.h"
#include <iostream>
//#include "cv.h"
namespace movesense {

Ctrl_Inf *g_ctrlInf = NULL;
CameraCtrl::CameraCtrl():m_opened(false) {
	g_ctrlInf = new Ctrl_Inf();
}

CameraCtrl::~CameraCtrl() {
	delete g_ctrlInf;
}

bool CameraCtrl::Open()
{
	std::string name = g_ctrlInf->ScanDev(g_vid_usb,g_pid_usb2);
	if(name.empty())
	{
		return false;
	}
	std::cout << "device name is "<< name <<std::endl;
	bool ret = g_ctrlInf->Open(name);
	m_opened =ret;
	return ret;

}

void CameraCtrl::Close()
{
	g_ctrlInf->Close();
}

void CameraCtrl::SetStereoCamParas(float p[41])
{
	char open[5] = "STAR";
	if(m_opened)
	{
		int res1 = g_ctrlInf->write_port(open,4);
		unsigned char data[41*sizeof(float) + 2];
		data[0] = SETTINGS_CAMERA_PARAS; 	// type
		data[1] = sizeof(float)*41;      	// len
		memcpy(data + 2 ,p,sizeof(float)*41);
		for(int i = 0; i < 41*sizeof(float)+2; i++)
		{
			res1 = g_ctrlInf->write_port((char*)data+i,1);
			usleep(20);
		}
		printf("len is %d.\r\n",res1);
	}
}


//void CameraCtrl::SetStereoCamParasFromDir(std::string dir)
//{
//		//string dir = "/media/renyin/Work/Robots/DesignCode_1/calib_data/2016_2_28_09_39_red";
//		CvMat *M1 =     (CvMat *)cvLoad((dir+"/M1.xml").c_str());
//	    CvMat *M2 =     (CvMat *)cvLoad((dir+"/M2.xml").c_str());
//	    CvMat *D1 =     (CvMat *)cvLoad((dir+"/D1.xml").c_str());
//	    CvMat *D2 = 	(CvMat *)cvLoad((dir+"/D2.xml").c_str());
//	    CvMat *IR_1 =   (CvMat *)cvLoad((dir+"/iR_1.xml").c_str());
//	    CvMat *IR_2 =   (CvMat *)cvLoad((dir+"/iR_2.xml").c_str());
//	    CvMat *t_P2 =   (CvMat *)cvLoad((dir+"/t_P2.xml").c_str());
//	    float data[41];
//
//	    data[0] = cvmGet(M1,0,0); //fx
//	    data[1] = cvmGet(M1,1,1); //fy
//	    data[2] = cvmGet(M1,0,2); //cx
//	    data[3] = cvmGet(M1,1,2); //cy
//
//	    for(int i = 0; i < 3 ; i++)
//	    for(int j = 0; j < 3; j++)
//	     {
//	    	data[4 + i*3+j] = cvmGet(IR_1,i,j);
//	     }
//
//	    for(int i = 0; i < 5 ; i++)
//	    {
//	    	data[13 + i] = cvmGet(D1,i,0);//D1->data.db[i];
//	    }
//
//	    data[18] = cvmGet(M2,0,0); //fx
//	    data[19] = cvmGet(M2,1,1); //fy
//	    data[20] = cvmGet(M2,0,2); //cx
//	    data[21] = cvmGet(M2,1,2); //cy
//
//	    for(int i = 0; i < 3 ; i++)
//	    for(int j = 0; j < 3; j++)
//	     {
//	    	data[22 + i*3+j] = cvmGet(IR_2,i,j);
//	     }
//
//	    for(int i = 0; i < 5 ; i++)
//	    {
//	    	data[31 + i] = cvmGet(D2,i,0);//D1->data.db[i];
//	    }
//	    data[36] = cvmGet(t_P2,0,0);
//	    data[37] = cvmGet(t_P2,1,1);
//	    data[38] = cvmGet(t_P2,0,2);
//	    data[39] = cvmGet(t_P2,1,2);
//	    data[40] = cvmGet(t_P2,0,3)/data[36];
//	    SetStereoCamParas(data);
//
//	    cvReleaseMat(&M1);
//	    cvReleaseMat(&M2);
//	    cvReleaseMat(&D1);
//	    cvReleaseMat(&D2);
//	    cvReleaseMat(&IR_1);
//	    cvReleaseMat(&IR_2);
//}
void CameraCtrl::GetParas(MsgPkg& p)
{
	char open[5] = "STAR";
	if(m_opened)
	{
		//first send
		g_ctrlInf->write_port((char*)open,4);
		unsigned char data[2];
		data[0] = p.type;
		data[1] = p.len;
		g_ctrlInf->write_port((char*)data,2);
		p.data= (unsigned char*)malloc(p.len);
		// then receieve
		MsgPkg mp;
		RecvPkg(mp);
		if(mp.len == p.len && mp.type == p.type)
		{
			p.data = new unsigned char[mp.len];
			memcpy(p.data,mp.data,mp.len);
		}else
		{
			p.len = 0;
			p.data = 0;
		}
		delete mp.data;

	}
}

void CameraCtrl::RecvPkg(MsgPkg &p)
{

}

} /* namespace movesense */
