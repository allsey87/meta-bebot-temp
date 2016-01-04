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
   for(CPacketControlInterface* pcInterface : {
         m_pcPowerManagementInterface,
         m_pcSensorActuatorInterface,
         m_pcManipulatorInterface
      }) {
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
   m_pcBlockTracker = new CBlockTracker(640u, 360u, 10u, 10u, 0.5f, 50.0f);

   /* Initialisation was successful */
   return 0;
}

/****************************************/
/****************************************/

void CBlockDemo::Exec() {
   /* useful values for sending over packet control interface */
   const int8_t nLiftStop = 0;
   const int8_t nLiftLowSpeedUp = -64;
   const int8_t nLiftLowSpeedDown = 64;
   
   const uint8_t unElectromagnetDischargeConstructive = 0;
   const uint8_t unElectromagnetDischargeDestructive = 1;
   const uint8_t unElectromagnetDischargeDisabled = 2;

   const uint8_t unActuatorInputLimitAuto = 0;
   const uint8_t unActuatorInputLimit100mA = 1;
   const uint8_t unActuatorInputLimit150mA = 2;
   const uint8_t unActuatorInputLimit500mA = 3;
   const uint8_t unActuatorInputLimit900mA = 4;

   const int8_t pnDriveSystemStop[] = {0, 0};

   const uint8_t unDisabled = 0;
   const uint8_t unEnabled = 1;



   struct {
      cv::Mat Y;
      cv::Mat U;
      cv::Mat V;
   } sCurrentFrame;

   /* for storing remote sensor data */
   bool bLiftAtBottom = false, 
      bLiftAtTop = false;
   uint16_t unRfRangeLeft = 0, 
      unRfRangeRight = 0, 
      unRfRangeFront = 0,
      unRfRangeUnderneath = 0;

   std::list<SBlock> lstDetectedBlocks;
   std::list<STarget> lstTrackedTargets;

   unsigned int unTargetId = -1;
   unsigned int unFrameIdx = -1;

   /* grab a couple frames to allow the sensor to adjust to the lighting conditions */
   for(unsigned int unNumberFrames = 3; unNumberFrames > 0; unNumberFrames--) {
      m_pcISSCaptureDevice->Grab();
   }

   /* Override actuator input limit to 100mA */
   m_pcPowerManagementInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_ACTUATOR_INPUT_LIMIT_OVERRIDE,
                                            &unActuatorInputLimit100mA,
                                            1);
   
   /* Enable the actuator power domain */
   m_pcPowerManagementInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_ACTUATOR_POWER_ENABLE,
                                            &unEnabled,
                                            1);
   
   /* Power up the differential drive system */
   m_pcSensorActuatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_DDS_ENABLE,
                                           &unEnabled,
                                           1);

   /* Initialize the differential drive system */
   m_pcSensorActuatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_DDS_SPEED,
                                           reinterpret_cast<const uint8_t*>(pnDriveSystemStop),
                                           2);
   
   for(;;) {
      /* send requests to sample sensors on the manipulator microcontroller */
      m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::GET_LIMIT_SWITCH_STATE);
      m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::GET_RF_RANGE);

      /* capture a frame  from the camera interface */
      m_pcISSCaptureDevice->GetFrame(sCurrentFrame.Y, sCurrentFrame.U, sCurrentFrame.V);
      unFrameIdx++;

      /* wait for responses from the manipulator interface */
      bool bWaitingForRfResponse = true;
      bool bWaitingForSwitchResponse = true;

      do {
         m_pcManipulatorInterface->ProcessInput();
         if(m_pcManipulatorInterface->GetState() == CPacketControlInterface::EState::RECV_COMMAND) {
            const CPacketControlInterface::CPacket& cPacket = m_pcManipulatorInterface->GetPacket();
            switch(cPacket.GetType()) {
            case CPacketControlInterface::CPacket::EType::GET_LIMIT_SWITCH_STATE:
               if(cPacket.GetDataLength() == 2) {
                  const uint8_t* punPacketData = cPacket.GetDataPointer();
                  bLiftAtTop = (punPacketData[0] != 0);
                  bLiftAtBottom = (punPacketData[1] != 0);		
                  bWaitingForSwitchResponse = false;
               }
               break;
            case CPacketControlInterface::CPacket::EType::GET_RF_RANGE:
               if(cPacket.GetDataLength() == 8) {
                  const uint8_t* punPacketData = cPacket.GetDataPointer();
                  unRfRangeLeft = (punPacketData[0] << 8) | punPacketData[1];
                  unRfRangeRight = (punPacketData[2] << 8) | punPacketData[3];
                  unRfRangeFront = (punPacketData[4] << 8) | punPacketData[5];
                  unRfRangeUnderneath = (punPacketData[6] << 8) | punPacketData[7];
                  bWaitingForRfResponse = false;
               }
               break;
            default:
               continue;
            }
         }
      } while((bWaitingForRfResponse || bWaitingForSwitchResponse) && !bShutdownSignal);

      std::cout << "[Range Finders]" << std::endl;
      std::cout << "R(L) = " << unRfRangeLeft << ", "
                << "R(R) = " << unRfRangeRight << ", "
                << "R(F) = " << unRfRangeFront << ", "
                << "R(U) = " << unRfRangeUnderneath << std::endl;
      std::cout << "[Limit Switches]" << std::endl;
      std::cout << "S(T) = " << (bLiftAtTop ? '1' : '0') << ", "
                << "S(B) = " << (bLiftAtBottom ? '1' : '0') << std::endl;

      if(!m_strImageSavePath.empty()) {
         std::ostringstream cStream;
         cStream << m_strImageSavePath << "_" << std::setfill('0') << std::setw(5) << unFrameIdx << ".y";
         std::ofstream cFrameOutputY(cStream.str().c_str());
         cFrameOutputY.write(reinterpret_cast<char*>(sCurrentFrame.Y.data),
                             sCurrentFrame.Y.rows * sCurrentFrame.Y.cols);
      }

      //m_pcBlockSensor->SetCameraPosition();  
      m_pcBlockSensor->DetectBlocks(sCurrentFrame.Y, lstDetectedBlocks);

      // pass the time in miliseconds to track targets to allow for protectile based matching        
      m_pcBlockTracker->AssociateAndTrackTargets(lstDetectedBlocks, lstTrackedTargets);

      for(const STarget& s_target : lstTrackedTargets) {
         ostringstream cText;
         cText << '[' << s_target.Id << ']';
         
         CFrameAnnotator::Annotate(sCurrentFrame.Y,
                                   s_target,
                                   m_pcBlockSensor->GetCameraMatrix(),
                                   m_pcBlockSensor->GetDistortionParameters(),
                                   cText.str());
      }

      if(!lstTrackedTargets.empty()) {        
         const std::list<SBlock>& lstObservations = std::begin(lstTrackedTargets)->Observations;
         const SBlock& sBlock = *std::begin(lstObservations);

         float fDist = std::sqrt(std::pow(sBlock.X, 2) +
                                 std::pow(sBlock.Y, 2) +
                                 std::pow(sBlock.Z, 2));

         std::cout << "[Block] " << fDist << "m" << std::endl;



         /* Saturate fDist between 0.0f and 1.0f */
         fDist = (fDist > 1.0f) ? 1.0f : (fDist < 0.0f) ? 0.0f : fDist; 
         
         float fTargetSpeed = std::abs(fDist - 0.3) * 100;
         bool fReverse = std::signbit(fDist - 0.3);
         
         int8_t pnSpeeds[] = {
            static_cast<int8_t>(std::floor(fTargetSpeed) * (fReverse ? -1.0f : 1.0f)),
            static_cast<int8_t>(std::floor(fTargetSpeed) * (fReverse ? -1.0f : 1.0f))
         };
         
         m_pcSensorActuatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_DDS_SPEED,
                                                 reinterpret_cast<const uint8_t*>(pnSpeeds),
                                                 2);
      }
      else {
         m_pcSensorActuatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_DDS_SPEED,
                                                 reinterpret_cast<const uint8_t*>(pnDriveSystemStop),
                                                 2);

      }
      
      // if unTarget = -1 && target list is not empty, select first target
      // else if unTarget doesn't exists then select first target
      // get distance to first observation and use DDS to maintain distance

      

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

   /* Disable the lift actuator */
   m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_LIFT_ACTUATOR_SPEED,
                                        reinterpret_cast<const uint8_t*>(&nLiftStop),
                                        1);

   /* Disable electromagnet discharge */
   m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_EM_DISCHARGE_MODE,
                                        &unElectromagnetDischargeDisabled,
                                        1);

   /* Disable electromagnet charging */
   m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_EM_CHARGE_ENABLE,
                                        &unDisabled,
                                        1);

   /* Disable the differential drive system */
   m_pcSensorActuatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_DDS_SPEED,
                                           reinterpret_cast<const uint8_t*>(pnDriveSystemStop),
                                           2);

   /* Power down the differential drive system */
   m_pcSensorActuatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_DDS_ENABLE,
                                           &unDisabled,
                                           1);

   /* Switch off LEDs */
   

   /* Disable the actuator power domain */
   m_pcPowerManagementInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_ACTUATOR_POWER_ENABLE,
                                            &unDisabled,
                                            1);

   /* Disable actuator input limit override */
   m_pcPowerManagementInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_ACTUATOR_INPUT_LIMIT_OVERRIDE,
                                            &unActuatorInputLimitAuto,
                                            1);



   /* Power down range finder LEDs */
   
   std::cerr << "Shutdown: terminating now" << std::endl;
}

