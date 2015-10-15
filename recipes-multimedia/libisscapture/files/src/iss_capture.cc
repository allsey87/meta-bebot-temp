
#include "iss_capture.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdlib.h>
#include <stdarg.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include "opencv2/opencv.hpp"

namespace jafp {

   const OvVideoMode OvVideoCapture::OV_MODE_320_240_30 = { 320, 240, 30, 1 };
   const OvVideoMode OvVideoCapture::OV_MODE_640_480_30 = { 640, 480, 30, 0 };
   const OvVideoMode OvVideoCapture::OV_MODE_1280_720_30 = { 1280, 720, 30, 0 };

   static void to_gray(const unsigned char* input, unsigned char* output, int length) {
      int i = 0, j = 0;
      for (; i < length; i += 2) {
         output[j] = input[i + 1];
         j += 1;
      }
   }

   OvVideoCapture::OvVideoCapture(const char* pch_device, const OvVideoMode& mode) :
      m_pchDevice(pch_device),
      mode_(mode),
      m_unScale(2) {}

   OvVideoCapture::~OvVideoCapture() {
      if (buffer_) {
         delete[] buffer_;
      }
      release();
   }

   bool OvVideoCapture::open() {
      if(!open_internal()) {
         return false;
      }
      if (!start_capturing()) {
         return false;
      }
      is_opened_ = true;
      return true;
   }

   bool OvVideoCapture::release() {
      if (isOpened()) {
         int i;
         enum v4l2_buf_type type;
		
         type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
         ioctl (fd_, VIDIOC_STREAMOFF, &type);

         for (i = 0; i < NumBuffers; i++) {
            munmap(buffers_[i].start, buffers_[i].length);
         }	
         close (fd_);
      }
      return true;
   }

   bool OvVideoCapture::grab() {
      struct v4l2_buffer capture_buf;

      // Give it a shot if the device hasn't been opened
      // at this point
      if (!is_opened_) {
         if (!open()) {
            return false;
         }
      }
      memset (&capture_buf, 0, sizeof(capture_buf));
      capture_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      capture_buf.memory = V4L2_MEMORY_MMAP;
	
      if (ioctl(fd_, VIDIOC_DQBUF, &capture_buf) < 0) {
         return false;
      }
      // Do not copy anything here, but save the index of the current
      // frame buffer for later use (e.g. in retrieve)
      current_buffer_index_ = capture_buf.index;
      memcpy(buffer_, buffers_[capture_buf.index].start, frame_size_);

      if (ioctl (fd_, VIDIOC_QBUF, &capture_buf) < 0) {
         return false;
      }
      return true;
   }

   double OvVideoCapture::get(int propId) {
      switch(propId) {
      case CV_CAP_PROP_FRAME_WIDTH:
         return mode_.width / m_unScale;
         break;
      case CV_CAP_PROP_FRAME_HEIGHT:
         return mode_.height / m_unScale;
         break;
      default:
         return -1;
         break;
      }

   }

   bool OvVideoCapture::retrieve(cv::Mat& image) {
		
      if (!grab()) {
         return false;
      }	

      image.create(mode_.height / m_unScale,
                   mode_.width / m_unScale,
                   CV_8UC1);
		
      for (int i = 0; i < mode_.height; i += m_unScale) {
         for (int j = 0; j < mode_.width; j += m_unScale) {
            image.data[(i / m_unScale) * (mode_.width / m_unScale) + (j / m_unScale)] = buffer_[i * (mode_.width * 2) + (j*2 + 1)];
         }
      }
		

      return true;
   }

   bool OvVideoCapture::read(cv::Mat& image) {
      return retrieve(image);
   }

   bool OvVideoCapture::start_capturing() {
      unsigned int i;
      struct v4l2_buffer buf;
      enum v4l2_buf_type type;

      for (i = 0; i < NumBuffers; i++) {
         memset (&buf, 0, sizeof (buf));
         buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
         buf.memory = V4L2_MEMORY_MMAP;
         buf.index = i;
	 fprintf(stderr, "%s: VIDIOC_QUERYBUF %d\n", m_pchDevice, i);
         if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
            return false;
         }
         buffers_[i].length = buf.length;
         buffers_[i].offset = (size_t) buf.m.offset;
         buffers_[i].start = static_cast<unsigned char*>(mmap (NULL, buffers_[i].length, 
                                                               PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buffers_[i].offset));

         memset (buffers_[i].start, 0xFF, buffers_[i].length);
      }

      for (i = 0; i < NumBuffers; i++) {
         memset (&buf, 0, sizeof (buf));
         buf.index = i;
         buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
         buf.memory = V4L2_MEMORY_MMAP;
         buf.m.offset = buffers_[i].offset;

	 fprintf(stderr, "%s: VIDIOC_QBUF %d\n", m_pchDevice, i);
         if (ioctl (fd_, VIDIOC_QBUF, &buf) < 0) {
            return false;
         }
      }

      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      fprintf(stderr, "%s: VIDIOC_STREAMON\n", m_pchDevice);
      if (ioctl (fd_, VIDIOC_STREAMON, &type) < 0) {
	 fprintf(stderr, "%s: VIDIOC_STREAMON errno %d\n", m_pchDevice, errno);
         return false;
      }

      return true;
   }

   bool OvVideoCapture::open_internal() {
      struct v4l2_format fmt;
      struct v4l2_requestbuffers req;
      struct v4l2_streamparm parm;	
      struct v4l2_crop crop;
      v4l2_std_id id;
      int input = DefaultInputNo;

      // Mode's size combined with default format's number of channels
      frame_size_ = mode_.width * mode_.height * DefaultFormatChannels;
      buffer_ = new unsigned char[frame_size_];
      fprintf(stderr, "%s: open\n", m_pchDevice);
      if ((fd_ = ::open(m_pchDevice, O_RDWR, 0)) < 0) {
         return false;
      }
      fprintf(stderr, "%s: VIDIOC_S_INPUT\n", m_pchDevice);
      if (ioctl(fd_, VIDIOC_S_INPUT, &input) < 0) {
         close(fd_);
         return false;
      }

      fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      fmt.fmt.pix.width = mode_.width;
      fmt.fmt.pix.height = mode_.height;
      fmt.fmt.pix.sizeimage = frame_size_;
      fmt.fmt.pix.bytesperline = mode_.width * DefaultFormatChannels;
      fmt.fmt.pix.pixelformat = DefaultFormat;
      fmt.fmt.pix.field = V4L2_FIELD_NONE;

      fprintf(stderr, "%s: VIDIOC_S_FMT\n", m_pchDevice);
      if (ioctl (fd_, VIDIOC_S_FMT, &fmt) < 0){
         close(fd_);
         return false;
      }

      fprintf(stderr, "%s: VIDIOC_G_FMT\n", m_pchDevice);
      if (ioctl(fd_, VIDIOC_G_FMT, &fmt) < 0) {
         close(fd_);
         return false;
      }

      memset(&req, 0, sizeof (req));
      req.count = NumBuffers;
      req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      req.memory= V4L2_MEMORY_MMAP;

      fprintf(stderr, "%s: VIDIOC_REQBUFS\n", m_pchDevice);
      if (ioctl (fd_, VIDIOC_REQBUFS, &req) < 0) {
         return false;
      }
      fprintf(stderr, "%s: COUNT\n", m_pchDevice);
      if (req.count < 2) {
         return false;
      }
      return true;
   }

}

