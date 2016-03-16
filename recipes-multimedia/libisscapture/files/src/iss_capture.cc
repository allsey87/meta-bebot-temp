
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

#include <fstream>

const CISSCaptureDevice::OvVideoMode CISSCaptureDevice::OV_MODE_640_480_30 =
   { 640, 480, 30, 0 };
const CISSCaptureDevice::OvVideoMode CISSCaptureDevice::OV_MODE_320_240_30 =
   { 320, 240, 30, 1 };
const CISSCaptureDevice::OvVideoMode CISSCaptureDevice::OV_MODE_1280_720_30 = 
   { 1280, 720, 30, 0 };

static void to_gray(const unsigned char* input, unsigned char* output, int length) {
   int i = 0, j = 0;
   for (; i < length; i += 2) {
      output[j] = input[i + 1];
      j += 1;
   }
}

CISSCaptureDevice::CISSCaptureDevice(const char* pch_device, const OvVideoMode& mode) :
   m_pchDevice(pch_device),
   mode_(mode),
   m_unScale(2) {}

CISSCaptureDevice::~CISSCaptureDevice() {
   if (buffer_) {
      delete[] buffer_;
   }
   Release();
}

bool CISSCaptureDevice::Open() {
   if(!OpenInternal()) {
      return false;
   }
   if (!StartCapturing()) {
      return false;
   }
   is_opened_ = true;
   return true;
}

bool CISSCaptureDevice::Release() {
   if (IsOpened()) {
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

bool CISSCaptureDevice::Grab() {
   struct v4l2_buffer capture_buf;

   // Give it a shot if the device hasn't been opened
   // at this point
   if (!is_opened_) {
      if (!Open()) {
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

bool CISSCaptureDevice::GetFrame(image_u8_t* pt_y_channel, image_u8_t* pt_u_channel, image_u8_t* pt_v_channel) {
   if (!Grab()) {
      return false;
   }
 
   for (unsigned int unHeightIdx = 0; unHeightIdx < mode_.height; unHeightIdx++) {
      for (int unWidthIdx = 0; unWidthIdx < mode_.width; unWidthIdx++) {
         // i and j are the horizontal and vertical indices into the image, ingnoring format and scaling 
         
         if((unHeightIdx % m_unScale == 0) && (unWidthIdx % m_unScale == 0)) {
            unsigned int unPixelOffset = unHeightIdx * (mode_.width * 2) + (unWidthIdx * 2);
            unsigned int unDestIdx = (unHeightIdx / m_unScale) * (pt_y_channel->stride) + (unWidthIdx / m_unScale);

            SPixelData* psPixelData = reinterpret_cast<SPixelData*>(buffer_ + unPixelOffset);
            
            /* split into seperate planes */
            pt_u_channel->buf[unDestIdx] = psPixelData->U0;
            pt_y_channel->buf[unDestIdx] = psPixelData->Y0;
            pt_v_channel->buf[unDestIdx] = psPixelData->V0;
         }
      }
   }

   return true;
}

bool CISSCaptureDevice::StartCapturing() {
   unsigned int i;
   struct v4l2_buffer buf;
   enum v4l2_buf_type type;

   for (i = 0; i < NumBuffers; i++) {
      memset (&buf, 0, sizeof (buf));
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;

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

      if (ioctl (fd_, VIDIOC_QBUF, &buf) < 0) {
         return false;
      }
   }
   type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
   if (ioctl (fd_, VIDIOC_STREAMON, &type) < 0) {
      return false;
   }

   return true;
}

bool CISSCaptureDevice::OpenInternal() {
   struct v4l2_format fmt;
   struct v4l2_requestbuffers req;
   struct v4l2_streamparm parm;	
   struct v4l2_crop crop;
   v4l2_std_id id;
   int input = DefaultInputNo;

   // Mode's size combined with default format's number of channels
   frame_size_ = mode_.width * mode_.height * DefaultFormatChannels;
   buffer_ = new unsigned char[frame_size_];
   if ((fd_ = ::open(m_pchDevice, O_RDWR, 0)) < 0) {
      return false;
   }
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

   if (ioctl (fd_, VIDIOC_S_FMT, &fmt) < 0){
      close(fd_);
      return false;
   }

   if (ioctl(fd_, VIDIOC_G_FMT, &fmt) < 0) {
      close(fd_);
      return false;
   }

   memset(&req, 0, sizeof (req));
   req.count = NumBuffers;
   req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
   req.memory= V4L2_MEMORY_MMAP;

   if (ioctl (fd_, VIDIOC_REQBUFS, &req) < 0) {
      return false;
   }

   if (req.count < 2) {
      return false;
   }
   return true;
}

