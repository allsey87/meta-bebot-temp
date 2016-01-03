/**
 * @file block_demo.cc
 * @brief Block demo applcation
 * @author: Michael Allwright
 */

#include <cstring>
#include <iostream>
#include <iomanip>
#include <fstream>

#include <error.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <iss_capture.h>

#include "block_demo.h"
#include "block_tracker.h"
#include "block_sensor.h"
#include "frame_annotator.h"
#include "packet_control_interface.h"
#include "tcp_image_socket.h"

/****************************************/
/****************************************/

volatile bool bShutdownSignal = false;

void InteruptSignalHandler(int n_unused) {
   bShutdownSignal = true;
}

/****************************************/
/****************************************/

int main(int n_arg_count, char* ppch_args[]) {
   /* Register the SIGINT handler to shut down system cleanly on abort */
   ::signal(SIGINT, InteruptSignalHandler);
   /* Print intro message */
   std::cerr << CBlockDemo::GetIntroString() << std::endl;
   /* Create application */
   CBlockDemo cBlockDemo;
   /* Init application */
   int nInitStatus = cBlockDemo.Init(n_arg_count, ppch_args);
   /* Run application if init was successful */
   if(nInitStatus == 0) {
      cBlockDemo.Exec();
   }
   return nInitStatus;
}

/****************************************/
/****************************************/

double GetTime() {
   struct timeval sT;
   ::gettimeofday(&sT, NULL);
   return (static_cast<double>(sT.tv_sec) + static_cast<double>(sT.tv_usec)/1000000.);
}

/****************************************/
/****************************************/

CTCPImageSocket& operator<<(CTCPImageSocket& m_cTCPImageSocket, const cv::Mat& m_cImage) {
   if(m_cImage.type() == CV_8UC1) {
      m_cTCPImageSocket.Write(m_cImage.data, m_cImage.size().width, m_cImage.size().height);
   } else {
      std::cerr << "Error" << std::endl
                << "CTCPImageSocket requires a grayscale image" << std::endl;
      bShutdownSignal = true;
   }
   return m_cTCPImageSocket;
}

/****************************************/
/****************************************/

const std::string CBlockDemo::m_strIntro =
   "Blocktracker Test Application\n"
   "2015 University of Paderborn\n"
   "Michael Allwright\n";

/****************************************/
/****************************************/

const std::string CBlockDemo::m_strUsage =
   "Usage:\n"
   "  blocktracker [OPTION...]\n"
   "\n"
   "Options:\n"
   "  -h -?              Show help options\n"
   "  -a                 Annotate images\n"
   "  -r <host:port>     Send images via TCP to remote host\n"
   "  -s <path/prefix>   Save images to folder with prefix\n";

/****************************************/
/****************************************/

int CBlockDemo::Init(int n_arg_count, char* ppch_args[]) {  
   /* Parse command line arguments */
   int c;
   while ((c = getopt(n_arg_count, ppch_args, "ar:s:")) != -1) {     
      switch (c) {
      case 'a':
         m_bAnnotateImages = true;
         break;
      case 'r':
         m_strRemoteHost = optarg;
         break;
      case 's':
         m_strImageSavePath = optarg;
         break;
      default:
         std::cerr << CBlockDemo::GetUsageString() << std::endl;
         return -EINVAL;
         break;
      }
   }

   m_pcPowerManagementInterface =
      new CPacketControlInterface("power management", "/dev/ttySC0", 57600);
   m_pcSensorActuatorInterface =
      new CPacketControlInterface("sensor/actuator", "/dev/ttySC1", 57600);
   m_pcManipulatorInterface =
      new CPacketControlInterface("manipulator", "/dev/ttySC2", 57600);

   /* Connect and check remote interfaces are responding */
   for(CPacketControlInterface* pcInterface : { m_pcPowerManagementInterface, m_pcSensorActuatorInterface}) { //, m_pcManipulatorInterface}) {
      std::cerr << "Connecting to the " << pcInterface->GetId() << " microcontroller: ";
      if(!pcInterface->Open()) {
         std::cerr << "Error" << std::endl << "Could not open the device" << std::endl;
         return -ENODEV;
      }
      uint32_t unUptime = 0;
      pcInterface->SendPacket(CPacketControlInterface::CPacket::EType::GET_UPTIME);
      uint8_t unAttempts = 3;
      while(unAttempts > 0) {     
         if(pcInterface->WaitForPacket(1000, 5)) {
            const CPacketControlInterface::CPacket& cPacket = pcInterface->GetPacket();
            if(cPacket.GetType() == CPacketControlInterface::CPacket::EType::GET_UPTIME) {
               break;
            }
         }
         unAttempts--;
      }
      if(unAttempts == 0) {
         std::cerr << "Error" << std::endl << "Device is not responding" << std::endl;
         return -EIO;
      }
      else {
         std::cerr << "OK" << std::endl;
      }
   }

   /* check the battery level of the manipulator module */
   m_pcPowerManagementInterface->SendPacket(CPacketControlInterface::CPacket::EType::GET_BATT_LVL);
   if(m_pcPowerManagementInterface->WaitForPacket(1000, 5)) {
      const CPacketControlInterface::CPacket& cPacket = m_pcPowerManagementInterface->GetPacket();
      if(cPacket.GetType() == CPacketControlInterface::CPacket::EType::GET_BATT_LVL &&
         cPacket.GetDataLength() == 2) {
         std::cerr << "System battery: "
                   << 17u * cPacket.GetDataPointer()[0]
                   << "mV" << std::endl;
         std::cerr << "Actuator battery: "
                   << 17u * cPacket.GetDataPointer()[1]
                   << "mV" << std::endl;
      }
   }
   else {
      std::cerr << "Warning: Could not read the system/actuator battery levels" << std::endl;
   }

   /* check the battery levels for the system and actuator circuits */
   /*
   m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::GET_BATT_LVL);
   if(m_pcManipulatorInterface->WaitForPacket(1000, 5)) {
      const CPacketControlInterface::CPacket& cPacket = m_pcManipulatorInterface->GetPacket();
      if(cPacket.GetType() == CPacketControlInterface::CPacket::EType::GET_BATT_LVL &&
         cPacket.GetDataLength() == 1) {
         std::cerr << "Manipulator battery: "
                   << 17u * cPacket.GetDataPointer()[0]
                   << "mV" << std::endl;
      }
   }
   else {
      std::cerr << "Warning: Could not read manipulator battery level" << std::endl;
   }
   */
 
   /* connect to the remote host for image streaming */
   if(!m_strRemoteHost.empty()) {
      /* find the hostname port number seperator */
      std::size_t unSepIdx = m_strRemoteHost.find_last_of(':');
      /* pass the hostname and port number to the TCP image socket */
      std::string strHostname(m_strRemoteHost.substr(0, unSepIdx));
      std::string strPortNumber(m_strRemoteHost.substr(unSepIdx + 1));
      std::cerr << "Connecting to " << strHostname << " on " << strPortNumber << ": ";
      m_pcTCPImageSocket = new CTCPImageSocket;
      int nStatus = m_pcTCPImageSocket->Open(strHostname.c_str(), atoi(strPortNumber.c_str()));
      if(nStatus != 0) {
         std::cerr << "Error" << std::endl << "Connection refused" << std::endl;
         return -ECONNREFUSED;
      }
      else {
         std::cerr << "OK" << std::endl;
      }
   }

   /* Check if /dev/media0 exists - indicates init during boot was successful */
   struct stat buf;
   std::cerr << "Opening video device: ";
   if (stat("/dev/media0", &buf) != 0) {
      std::cerr << "Error" << std::endl << "/dev/media0 does not exist" << std::endl;
      return -ENODEV;
   }  
   /* TODO: use the actual media-ctl API and integrate these commands into the ISS capture class */
   ::system("media-ctl -r -l '\"OMAP4 ISS CSI2a\":1 -> \"OMAP4 ISS CSI2a output\":0 [1]'");
   ::system("media-ctl -V '\"ov5640 3-003c\":0 [UYVY 1280x720]','\"OMAP4 ISS CSI2a\":0 [UYVY 1280x720]'");
   /* Open the video device */
   m_pcISSCaptureDevice = new CISSCaptureDevice(
      "/dev/video0", CISSCaptureDevice::OV_MODE_1280_720_30);
   if(!m_pcISSCaptureDevice->Open()) {
      std::cerr << "Error" << std::endl << "Could not open /dev/video0" << std::endl;
      return -ENODEV;
   }
   std::cerr << "OK" << std::endl;

   /* Create the block sensor/tracker instances */
   m_pcBlockSensor = new CBlockSensor;
   m_pcBlockTracker = new CBlockTracker(360u, 640u, 3u, 0.5f, 50.0f);

   /* Initialisation was successful */
   return 0;
}

/****************************************/
/****************************************/

void CBlockDemo::Exec() {
/*
   int8_t pnliftSpeeds[] = {-64, 0, 64};
   uint8_t* punData = reinterpret_cast<uint8_t*>(pnliftSpeeds);
   m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_LIFT_ACTUATOR_SPEED, &punData[0], 1);
   m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_LIFT_ACTUATOR_SPEED, &punData[1], 1);
*/    

   struct {
      cv::Mat Y;
      cv::Mat U;
      cv::Mat V;
   } sCurrentFrame;

   std::list<CBlockSensor::SBlock> lstDetectedBlocks;
   std::list<CBlockTracker::STarget> lstTrackedTargets;

   unsigned int unBlockId = 0;
   unsigned int unFrameIdx = 0;

   /* grab a couple frames to allow the sensor to adjust to the lighting conditions */
   for(unsigned int unNumberFrames = 3; unNumberFrames > 0; unNumberFrames--) {
      m_pcISSCaptureDevice->Grab();
   }
 
   for(;;) {
     
      m_pcISSCaptureDevice->GetFrame(sCurrentFrame.Y, sCurrentFrame.U, sCurrentFrame.V);

      if(!m_strImageSavePath.empty()) {
         std::ostringstream cStream;
         cStream << m_strImageSavePath << "_" << std::setfill('0') << std::setw(5) << unFrameIdx << ".y";
         std::ofstream cFrameOutputY(cStream.str().c_str());
         cFrameOutputY.write(reinterpret_cast<char*>(sCurrentFrame.Y.data),
                             sCurrentFrame.Y.rows * sCurrentFrame.Y.cols);
         unFrameIdx++;
      }

      //m_pcBlockSensor->SetCameraPosition();
      /*
      m_pcBlockSensor->DetectBlocks(sCurrentFrame.Y, lstDetectedBlocks);

      unBlockId = 0;
      for(const CBlockSensor::SBlock& s_block : lstDetectedBlocks) {
         std::ostringstream cStream;
         cStream << '[' << unBlockId << ']';
         for(const CBlockSensor::STag& s_tag : s_block.Tags) {
            CFrameAnnotator::Annotate(sCurrentFrame.Y, s_tag, cStream.str());
         }
         unBlockId++;
      }
      */

      // pass the time in miliseconds to track targets to allow for protectile based matching
      
      //m_pcBlockTracker->AssociateAndTrackTargets(lstDetectedBlocks, lstTrackedTargets);
      /*
      // Draws line connecting block centres of previous matches and outline of the last observation
      for(const CBlockTracker::STarget& s_target : vecTrackedTargets) {
         CFrameAnnotator::Annotate(cCurrentFrame, s_target);
      }
      */

      
      /*
      // Cluster targets into structures
      CStructureDetectionAlgorithm::GenerateStructures();
      for(const CMicroRule& c_rule : vecMicroRules) {
         if(c_rule.Matches(set_of_detected_structures)) {
            m_pcActiveRule = &c_rule;
            break;
         }
      }
      */

      /* stream frame to host if connected */
      if(m_pcTCPImageSocket != nullptr) {
         *m_pcTCPImageSocket << sCurrentFrame.Y;
      }

      if(bShutdownSignal) {
         std::cerr << "Shutdown: request acknowledged" << std::endl;
         break;
      }
   }  
   std::cerr << "Shutdown: terminating now" << std::endl;
}

