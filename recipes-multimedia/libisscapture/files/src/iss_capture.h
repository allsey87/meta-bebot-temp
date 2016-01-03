/*
 * Copyright 2014 Jacob Aslund <jacob@itbuster.dk>
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

#ifndef _OV_VIDEO_CAPTURE_H_
#define _OV_VIDEO_CAPTURE_H_

#include <opencv2/core/core.hpp>
#include <linux/videodev2.h>

class CISSCaptureDevice {
public:

   struct OvVideoMode {
      int width;
      int height;
      int framerate;
      int capture_mode;
   };

   struct OvFrameBuffer {
      unsigned char* start;
      unsigned int length;
      unsigned int offset;
   };

   struct SPixelData {
      unsigned char U0;
      unsigned char Y0;
      unsigned char V0;
      unsigned char Y1;
   };
   
public:

   // Various constants
   static const int NumBuffers = 2;
   static const int DefaultInputNo = 0;
   static const int DefaultFormat = V4L2_PIX_FMT_UYVY;
   static const int DefaultFormatChannels = 2;

   // Modes
   static const OvVideoMode OV_MODE_640_480_30;
   static const OvVideoMode OV_MODE_320_240_30;
   static const OvVideoMode OV_MODE_1280_720_30;
   
   CISSCaptureDevice(const char* pch_device, const OvVideoMode& mode = OV_MODE_1280_720_30);
   virtual ~CISSCaptureDevice();

   // Opens the device (OV5640 sensor connected to the MIPI CSI2 channel).
   // No parameters are given, as we expect the device to have a fixed device id
   bool Open();

   // Relase the device. 
   // Free'es the internal frame buffer.
   bool Release();

   // Returns true if the device already has been opened. 
   inline bool IsOpened() const { return is_opened_; }

   bool WriteFrameToDisk(std::string s_path_to_file);
   
   // Grabs a single frame from the image sensor
   bool Grab();

   // Grabs, decodes and returns the grabbed image.
   bool GetFrame(cv::Mat& c_y_channel, cv::Mat& c_u_channel, cv::Mat& c_v_channel);


   

private:
   int fd_;
   int current_buffer_index_;
   int frame_size_;
   bool is_opened_;
   unsigned char* buffer_;

   OvFrameBuffer buffers_[NumBuffers];
   const OvVideoMode& mode_;

   bool OpenInternal();
   bool StartCapturing();

   const unsigned int m_unScale;

   const char* m_pchDevice;

};

#endif
