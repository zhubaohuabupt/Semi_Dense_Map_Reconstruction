/*
 * CameraCtrl.h
 *
 *  Created on: May 17, 2016
 *      Author: renyin
 */

#ifndef SRC_CAMERACTRL_H_
#define SRC_CAMERACTRL_H_

#include "CtrlInf.h"

namespace movesense {
struct MsgPkg{
	unsigned char type;
	unsigned char len;
	unsigned char *data;
};

class CameraCtrl {
public:
	CameraCtrl();
	virtual ~CameraCtrl();

	bool Open();
	void Close();

	void SetStereoCamParas(float p[36]);
	//void SetStereoCamParasFromDir(std::string dir);
	void GetParas(MsgPkg &p);
	void RecvPkg(MsgPkg &p);
	bool m_opened;
};

} /* namespace movesense */

#endif /* SRC_CAMERACTRL_H_ */
