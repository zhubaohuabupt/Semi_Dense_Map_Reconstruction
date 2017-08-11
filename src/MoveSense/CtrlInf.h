/*
 * CtrlInf.h
 *
 *  Created on: May 17, 2016
 *      Author: renyin
 */

#ifndef SRC_CTRLINF_H_
#define SRC_CTRLINF_H_
#include <string>
#include <stdio.h>
#include <string.h>
#include <string>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>

namespace movesense {

class Ctrl_Inf {
public:
	Ctrl_Inf();
	virtual ~Ctrl_Inf();

	std::string ScanDev(const std::string &vid, const std::string &pid);
	bool Open(std::string name);

	void Close();

	int set_opt(int fd);

	// read port
	int read_port(char buf[], int len);

	//write port
	int write_port(char buff[],int len);
	int m_fd;
};

} /* namespace movesense */

#endif /* SRC_CTRLINF_H_ */
