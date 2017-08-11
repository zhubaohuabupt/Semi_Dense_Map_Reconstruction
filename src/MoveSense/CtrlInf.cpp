/*
 * CtrlInf.cpp
 *
 *  Created on: May 17, 2016
 *      Author: renyin
 */
#include <iostream>
#include "CtrlInf.h"
#include <libudev.h>

namespace movesense {

Ctrl_Inf::Ctrl_Inf() {


}

Ctrl_Inf::~Ctrl_Inf() {

}


std::string Ctrl_Inf::ScanDev(const std::string &vid, const std::string &pid)
{
	std::cout << "ScanDev" << std::endl;
	struct udev *udev;
	struct udev_enumerate *enumerate;
	struct udev_list_entry *devices, *dev_list_entry;
	struct udev_device *dev;
	std::string dev_name;
	char * device[] = {"ttyUSB0","ttyUSB1","ttyUSB2","ttyUSB3","ttyUSB4","ttyUSB5","ttyUSB6","ttyUSB7"};
	std::cout << "ScanDev1" << std::endl;
	for(int index=0;index<8;index++)
	{
		udev = udev_new();
		if (!udev) {
			printf("Can't create udev\n");
			return "";
		}
		std::cout << "ScanDev2" << std::endl;
		/* Create a list of the devices in the 'hidraw' subsystem. */
		enumerate = udev_enumerate_new(udev);
		//udev_enumerate_add_match_subsystem(enumerate, "block");
		udev_enumerate_add_match_sysname(enumerate,device[index]);
		udev_enumerate_scan_devices(enumerate);
		devices = udev_enumerate_get_list_entry(enumerate);
		std::cout << "ScanDev3"<< std::endl;
		udev_list_entry_foreach(dev_list_entry, devices)
		{

			const char *path;
			/* Get the filename of the /sys entry for the device
								 and create a udev_device object (dev) representing it */
			path = udev_list_entry_get_name(dev_list_entry);

			dev = udev_device_new_from_syspath(udev, path);
			/* usb_device_get_devnode() returns the path to the device node
								 itself in /dev. */

			char *temp_path = (char *)udev_device_get_devnode(dev);
			dev = udev_device_get_parent_with_subsystem_devtype(dev, "usb","usb_device");
			if (!dev) {
				printf("Unable to find parent usb device.");
				exit(1);
			}
			char *vvid = (char*)udev_device_get_sysattr_value(dev,"idVendor");
			char *ppid =(char*)udev_device_get_sysattr_value(dev, "idProduct");
			//printf("  VID/PID: %s %s\n",vvid,ppid);

			if((strcasecmp(vvid,vid.c_str())==0) && (strcasecmp(ppid,pid.c_str()) == 0)){
				//if(index<4){
				if(temp_path!=NULL)
				{
					dev_name=(temp_path);
					printf("qqq...%s.\r\n", temp_path);
				}
				//}
			}
			udev_device_unref(dev);
		}
		/* Free the enumerator object */
		udev_enumerate_unref(enumerate);
		udev_unref(udev);

	}
	printf("%s.\r\n", dev_name.c_str());
	return dev_name;
}
bool Ctrl_Inf::Open(std::string name)
{
    int fd = open(name.c_str(),O_RDWR|O_NOCTTY); //|O_NDELAY
	if(fd == -1){
	    perror("Can't Open Serial Port--lw");
	    return -1;
        }
    if(fcntl(fd,F_SETFL,0) < 0){
  	printf("fcntl failed/n");
    }
    else{
	printf("fcntl=%d/n",fcntl(fd,F_SETFL,0));
    }
    if(isatty(STDIN_FILENO) == 0){
	printf("standard input is not a terminal device/n");
    }
    else{
     	printf("isatty sucess!/n");
    }
    printf("fd-open=%d/n",fd);
    m_fd = fd;
    if(m_fd) set_opt(m_fd);
    if(fd>0) return true;
    return false;
}

void Ctrl_Inf::Close()
{
	close(m_fd);
}

int Ctrl_Inf::set_opt(int fd)
{
	int nSpeed = B9600;
	int nBits = 8;
	char nEvent = 'N';
	int nStop = 1;
    struct termios newtio;
    struct termios oldtio;

    if(tcgetattr(fd,&oldtio) != 0)
    {
        perror("SetupSerial 1");
        return -1;
    }

    bzero(&newtio,sizeof(newtio));
    newtio.c_cflag |= CLOCAL |CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch(nBits)
    {
        case 7:
            newtio.c_cflag |= CS7;
 	    break;
        case 8:
	    newtio.c_cflag |= CS8;
   	    break;
    }

    switch(nEvent)
    {
        case 'O':
            newtio.c_cflag |= PARENB;
   	    newtio.c_cflag |= PARODD;
   	    newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'E':
 	    newtio.c_iflag |= (INPCK |ISTRIP);
   	    newtio.c_cflag |= PARENB;
   	    newtio.c_cflag &= ~PARODD;
	    break;
        case 'N':
 	    newtio.c_cflag &= ~PARENB;
	    break;
    }

    switch(nSpeed)
    {
        case 2400:
	    cfsetispeed(&newtio,B2400);
	    cfsetospeed(&newtio,B2400);
            break;
        case 4800:
			cfsetispeed(&newtio,B4800);
			cfsetospeed(&newtio,B4800);
			break;
        case 9600:
			cfsetispeed(&newtio,B9600);
			cfsetospeed(&newtio,B9600);
			break;
        case 115200:
			cfsetispeed(&newtio,B115200);
			cfsetospeed(&newtio,B115200);
			break;
        case 460800:
			cfsetispeed(&newtio,B460800);
			cfsetospeed(&newtio,B460800);
			break;
        default:
            cfsetispeed(&newtio,B9600);
            cfsetospeed(&newtio,B9600);
            break;
    }

    if(nStop == 1){
    	newtio.c_cflag &= ~CSTOPB;
    }
    else if(nStop ==2){
        newtio.c_cflag |= CSTOPB;
    }
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;

    tcflush(fd,TCIFLUSH);
    if((tcsetattr(fd,TCSANOW,&newtio)) != 0)
    {
        perror("com set error");
        return -1;
    }
    printf("set done!/n");
    return 0;
}

// read port
int Ctrl_Inf::read_port(char buf[], int len)
{
	int n = read(m_fd, buf, len);
	return n;
}

//write port
int Ctrl_Inf::write_port(char buff[],int len)
{
	int n = write(m_fd,buff,len);
	return n;
}
} /* namespace movesense */
