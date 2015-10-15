/**
 * @file april_tags.cpp
 * @brief Example application for April tags library
 * @author: Michael Kaess
 *
 * Opens the first available camera (typically a built in camera in a
 * laptop) and continuously detects April tags in the incoming
 * images. Detections are both visualized in the live image and shown
 * in the text console. Optionally allows selecting of a specific
 * camera in case multiple ones are present and specifying image
 * resolution as long as supported by the camera. Also includes the
 * option to send tag detections via a serial port, for example when
 * running on a Raspberry Pi that is connected to an Arduino.
 */

using namespace std;

#include <iostream>
#include <iomanip>
#include <cstring>
#include <vector>
#include <list>
#include <sys/time.h>
#include <signal.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iss_capture.h>
#include "tcp_image_socket.h"

/* override the interrupt signal for clean shutdown */
volatile bool bInterruptEvent = false;

void InteruptSignalHandler(int n_unused) {
   bInterruptEvent = true;
}

CTCPImageSocket& operator<<(CTCPImageSocket& m_cTCPImageSocket, const cv::Mat& m_cImage) {
   if(m_cImage.type() == CV_8UC1) {
      m_cTCPImageSocket.Write(m_cImage.data, m_cImage.size().width, m_cImage.size().height);
   } else {
      cv::Mat m_cGrayscale;
      cv::cvtColor(m_cImage, m_cGrayscale, CV_RGB2GRAY);
      m_cTCPImageSocket.Write(m_cGrayscale.data, m_cGrayscale.size().width, m_cGrayscale.size().height);
   }
   return m_cTCPImageSocket;
}

class Demo {
   CTCPImageSocket m_cTCPImageSocket[2];
   jafp::OvVideoCapture m_cLeftCamera;
   jafp::OvVideoCapture m_cRightCamera;

   int m_width; // image size in pixels
   int m_height;

public:

   // default constructor
   Demo() :
      m_width(640),
      m_height(360),
      
      m_cLeftCamera("/dev/video0"),
      m_cRightCamera("/dev/video1") {}
      
   ~Demo() {
      m_cLeftCamera.release();
      m_cRightCamera.release();
   }

   void setup() {
      cerr << "Connecting to dev machine for image streaming:";
      m_cTCPImageSocket[0].Open("192.168.1.174", 23268);
      m_cTCPImageSocket[1].Open("192.168.1.2", 23269);
      cerr << "done" << endl;
   }

   void setupVideo() {
      if (!m_cLeftCamera.open()) {
         cerr << "ERROR: Can't find video device 0\n";
         exit(1);
      }
      if (!m_cRightCamera.open()) {
         cerr << "ERROR: Can't find video device 1\n";
         exit(1);
      }
   }

   void loop() {
      cv::Mat cCameraImage[2];

      uint32_t unFrames = 0;
      double fLastTime = 0;
      for(;;) {
         /* sample camera sensors */
         m_cLeftCamera >> cCameraImage[0];
         m_cRightCamera >> cCameraImage[1];
         /* send the images to the host */
         m_cTCPImageSocket[0] << cCameraImage[0];
         m_cTCPImageSocket[1] << cCameraImage[1];

         unFrames++;
         if (unFrames % 10 == 0) {
            struct timeval sTime;
            gettimeofday(&sTime, NULL);
            double fTime = double(sTime.tv_sec) + double(sTime.tv_usec)/1000000.;
            if(fLastTime != 0) {
               cout << 10./(fTime - fLastTime) << " fps" << endl;
            }          
            fLastTime = fTime;
         }

         if(bInterruptEvent) {
            break;
         }
      }
   }
}; // Demo


// here is were everything begins
int main(int argc, char* argv[]) {
   ::signal(SIGINT, InteruptSignalHandler);

   /* Set up ISS */
   ::system("media-ctl -r");
   ::system("media-ctl -l '\"OMAP4 ISS CSI2a\":1 -> \"OMAP4 ISS CSI2a output\":0 [1]'");
   ::system("media-ctl -V '\"ov5640 3-003c\":0 [UYVY 1280x720]','\"OMAP4 ISS CSI2a\":0 [UYVY 1280x720]'");

   ::system("media-ctl -l '\"OMAP4 ISS CSI2b\":1 -> \"OMAP4 ISS CSI2b output\":0 [1]'");
   ::system("media-ctl -V '\"ov5640 4-003c\":0 [UYVY 1280x720]','\"OMAP4 ISS CSI2b\":0 [UYVY 1280x720]'");

   Demo demo;

   demo.setup();
   demo.setupVideo();

   demo.loop();
   return 0;
}
