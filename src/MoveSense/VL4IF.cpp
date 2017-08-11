/*
 * VL4IF.cpp
 *
 *  Created on: May 5, 2016
 *      Author: renyin
 */

#include "VL4IF.h"
#include "CameraCmd.h"
#include <libudev.h>
#include <string.h>

VL4_IF::VL4_IF(int w,int h, CAMERA_FPS fps):m_camera_opened(false),nWidth(w),nHeight(h),m_fps(fps),m_dev_fd(-1),m_buf_cnt(4),buffers(NULL)
{

}

VL4_IF::~VL4_IF() {

}

string VL4_IF::ScanCamera(const string &vid, const string &pid)
{
	struct udev *udev;
	struct udev_enumerate *enumerate;
	struct udev_list_entry *devices, *dev_list_entry;
	struct udev_device *dev;
	string dev_name;
	char * device[] = {"video0","video1","video2","video3","ttyUSB0","ttyUSB1","ttyUSB2","ttyUSB3"};

	for(int index=0;index<8;index++)
	{
		udev = udev_new();
		if (!udev) {
			printf("Can't create udev\n");
			return "";
		}

		/* Create a list of the devices in the 'hidraw' subsystem. */
		enumerate = udev_enumerate_new(udev);
		//udev_enumerate_add_match_subsystem(enumerate, "block");
		udev_enumerate_add_match_sysname(enumerate,device[index]);
		udev_enumerate_scan_devices(enumerate);
		devices = udev_enumerate_get_list_entry(enumerate);
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
				if(index<4){
					dev_name=temp_path;
				}
			}
			udev_device_unref(dev);
		}
		/* Free the enumerator object */
		udev_enumerate_unref(enumerate);
		udev_unref(udev);

	}

	return dev_name;
}

bool VL4_IF::OpenCamera(string dev_name)
{
	if(!open_device(dev_name))
		return false;
	if(!init_uvcdev())
		return false;
	if(!start_capturing())
		return false;
	m_dev_name = dev_name;
	return true;
}
void VL4_IF::CloseCamera()
{
	stop_capturing();
	uninit_device();
	close_device();
}

bool VL4_IF::SetCameraParas(int w, int h, CAMERA_FPS fps)
{
	return true;
}

int VL4_IF::GetImageData(unsigned char * &data, int &len)
{
	v4l2_buffer buf;
	int ret = read_frame(buf);
	if(ret)
	{
		len = buffers[buf.index].length;
		//data = (unsigned char *)buffers[buf.index].start;
		memcpy(data, buffers[buf.index].start, len);
	}
	else
	{
		//data = NULL;
		len  =0 ;
	}
	return ret;
}

void VL4_IF::SetGainValue(int value)
{
	v4l2_control c;
	c.id 	= V4L2_CID_GAIN;
	c.value = value;
	xioctl(m_dev_fd, VIDIOC_S_CTRL, &c);
}


void VL4_IF::SetDesireBin(int value)
{
	v4l2_control c;
	c.id 	= V4L2_CID_GAMMA;
	c.value = value;
	xioctl(m_dev_fd, VIDIOC_S_CTRL, &c);
}

void VL4_IF::SetExposureValue(int value)
{
	v4l2_control c;
	c.id 	= V4L2_CID_BRIGHTNESS;
	c.value = value;
	xioctl(m_dev_fd, VIDIOC_S_CTRL, &c);
}

void VL4_IF::SetAutoExposure(bool value)
{
	if(value)
		CameraCmd(SETTINGS_AUTOEXP_ON);
	else
		CameraCmd(SETTINGS_AUTOEXP_OFF);
}

void VL4_IF::SetAutoGain(bool value)
{
	if(value)
		CameraCmd(SETTINGS_AUTOGAIN_ON);
	else
		CameraCmd(SETTINGS_AUTOGAIN_OFF);
}

void VL4_IF::SetSM_P1(unsigned int value)
{
	v4l2_control c;
	c.id 	= V4L2_CID_HUE;
	c.value = value;
	xioctl(m_dev_fd, VIDIOC_S_CTRL, &c);
}

void VL4_IF::SetSM_P2(unsigned int value)
{
	v4l2_control c;
	c.id 	= V4L2_CID_SATURATION;
	c.value = value;
	xioctl(m_dev_fd, VIDIOC_S_CTRL, &c);
}

void VL4_IF::SetSM_P3(unsigned int value)
{
	v4l2_control c;
	c.id 	= V4L2_CID_SHARPNESS;
	c.value = value;
	xioctl(m_dev_fd, VIDIOC_S_CTRL, &c);
}

void VL4_IF::CameraCmd(unsigned char value)
{
	v4l2_control c;
	c.id 	= V4L2_CID_CONTRAST;
	c.value = value;
	xioctl(m_dev_fd, VIDIOC_S_CTRL, &c);
}

bool VL4_IF::open_device(string dev_name)
{
  struct stat st;

  if (-1 == stat(dev_name.c_str(), &st))
    {
      fprintf(stderr, "Cannot identify '%s': %d, %s\n",
	      dev_name.c_str(), errno, strerror(errno));
      return false;
    }

  if (!S_ISCHR(st.st_mode))
    {
      fprintf(stderr, "%s is no device\n", dev_name.c_str());
      return false;
    }
  //open device
  m_dev_fd = open(dev_name.c_str(), O_RDWR /* required */  | O_NONBLOCK, 0);

  if (-1 == m_dev_fd)
    {
      fprintf(stderr, "Cannot open '%s': %d, %s\n",
	      dev_name.c_str(), errno, strerror(errno));
      return false;
    }
  return true;
}

bool VL4_IF::init_uvcdev()
{
  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;
  struct v4l2_format fmt;

  if (-1 == xioctl(m_dev_fd, VIDIOC_QUERYCAP, &cap))
    {
      if (EINVAL == errno)
	{
	  fprintf(stderr, "%s is no V4L2 device\n", m_dev_name.c_str());
	  return false;
	}
      else
	{
      return false;
	}
    }

  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
      fprintf(stderr, "%s is no video capture device\n", m_dev_name.c_str());
      return false;
    }


  if (!(cap.capabilities & V4L2_CAP_STREAMING))
  {
	  fprintf(stderr, "%s does not support streaming i/o\n", m_dev_name.c_str());
	  return false;
}


  /* Select video input, video standard and tune here. */

  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (-1 == xioctl(m_dev_fd, VIDIOC_CROPCAP, &cropcap))
    {
      /* Errors ignored. */
    }

  crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  crop.c = cropcap.defrect;	/* reset to default */

  if (-1 == xioctl(m_dev_fd, VIDIOC_S_CROP, &crop))
    {
      switch (errno)
	{
	case EINVAL:
	  /* Cropping not supported. */
	  break;
	default:
	  /* Errors ignored. */
	  break;
	}
    }

  CLEAR(fmt);

  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = nWidth;
  fmt.fmt.pix.height = nHeight;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;


  if (-1 == xioctl(m_dev_fd, VIDIOC_S_FMT, &fmt))
	  return false;

  /* Note VIDIOC_S_FMT may change width and height. */
  nWidth =  fmt.fmt.pix.width;
  nHeight = fmt.fmt.pix.height;

  struct v4l2_streamparm* setfps;
     setfps=(struct v4l2_streamparm *) calloc(1, sizeof(struct v4l2_streamparm));
     memset(setfps, 0, sizeof(struct v4l2_streamparm));
     setfps->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
     setfps->parm.capture.timeperframe.numerator = 1;
     setfps->parm.capture.timeperframe.denominator = GetFPSCount(m_fps);
     if(ioctl(m_dev_fd, VIDIOC_S_PARM, setfps) < 0){
                 printf("set fps fail\n");
                 return -1;
     }

 init_mmap();
 return true;
}

int VL4_IF::xioctl(int fd, int request, void *arg)
{
  int r;

  do
    r = ioctl(fd, request, arg);
  while (-1 == r && EINTR == errno);

  return r;
}

bool VL4_IF::init_mmap(void)
{
  struct v4l2_requestbuffers req;

  CLEAR(req);

  req.count = m_buf_cnt;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (-1 == xioctl(m_dev_fd, VIDIOC_REQBUFS, &req))
    {
      if (EINVAL == errno)
	{
	  fprintf(stderr, "%s does not support "
		  "memory mapping\n", m_dev_name.c_str());
	  return false;
	}
      else
	{
    	return false;
	}
    }

  if (req.count < 2)
    {
      fprintf(stderr, "Insufficient buffer memory on %s\n", m_dev_name.c_str());
      return false;
    }

  buffers = (struct buffer*)calloc(req.count, sizeof(*buffers));

  if (!buffers)
    {
      fprintf(stderr, "Out of memory\n");
      exit(EXIT_FAILURE);
    }

  for (int n_buffers = 0; n_buffers < req.count; ++n_buffers)
    {
      struct v4l2_buffer buf;

      CLEAR(buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = n_buffers;

      if (-1 == xioctl(m_dev_fd, VIDIOC_QUERYBUF, &buf))
    	  return false;

      buffers[n_buffers].length = buf.length;
      buffers[n_buffers].start 	= mmap(NULL /* start anywhere */ ,
				      	  	  	  	  buf.length,
									  PROT_READ | PROT_WRITE /* required */ ,
									  MAP_SHARED /* recommended */ ,
									  m_dev_fd, buf.m.offset);

    if (MAP_FAILED == buffers[n_buffers].start)
    	  return false;
    }
  return true;
}

bool VL4_IF::start_capturing(void)
{
  unsigned int i;
  enum v4l2_buf_type type;

  for (i = 0; i < m_buf_cnt; ++i)
  {
  	  struct v4l2_buffer buf;

  	  CLEAR(buf);

  	  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  	  buf.memory = V4L2_MEMORY_MMAP;
  	  buf.index = i;

  	  if (-1 == xioctl(m_dev_fd, VIDIOC_QBUF, &buf))
  	    return false;
  }

  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (-1 == xioctl(m_dev_fd, VIDIOC_STREAMON, &type))
	  return false;
  return true;
}


bool VL4_IF::stop_capturing(void)
{
  enum v4l2_buf_type type;
  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (-1 == xioctl(m_dev_fd, VIDIOC_STREAMOFF, &type))
	return false;
  return true;
}

bool VL4_IF::uninit_device(void)
{
  unsigned int i;

      for (i = 0; i < m_buf_cnt; ++i)
	if (-1 == munmap(buffers[i].start, buffers[i].length))
	  return false;

  free(buffers);
  return true;
}

bool VL4_IF::close_device(void)
{
	if (-1 == close(m_dev_fd))
	   return false;
	m_dev_fd = -1;
	return true;
}

int VL4_IF::read_frame(struct v4l2_buffer &buf)
{
	unsigned int i;
	ssize_t read_bytes;
	unsigned int total_read_bytes;
    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(m_dev_fd, VIDIOC_DQBUF, &buf))
	{
	  return 0;
	}

    assert(buf.index < m_buf_cnt);

    if (-1 == xioctl(m_dev_fd, VIDIOC_QBUF, &buf))
		return 0;
    return 1;
}
