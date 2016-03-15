/**
 * @file block_demo.cc
 * @brief Block demo applcation
 * @author: Michael Allwright
 */

#include <cstring>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <thread>
#include <chrono>

#include <error.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>

#include <argos3/core/utility/math/angles.h>
#include <iss_capture.h>
#include <opencv2/core/core.hpp>

#include "block_demo.h"
#include "block_tracker.h"
#include "block_sensor.h"
#include "frame_annotator.h"
#include "leds.h"
#include "packet_control_interface.h"
#include "tcp_image_socket.h"

#include "qualitative_stigmergy_test.h"

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
   "2016 University of Paderborn\n"
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
   while ((c = ::getopt(n_arg_count, ppch_args, "avr:s:")) != -1) {     
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
      case 'v':
         m_bVerboseOutput = true;
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
   
   /* Link to the LEDs */
   for(unsigned int un_dev_idx = 0; un_dev_idx < NUM_LEDS; un_dev_idx++) {
      m_vecLEDs.emplace_back("/sys/class/leds/", "pca963x:led_deck", un_dev_idx);
   }

   /* Create the block sensor/tracker instances */
   m_pcBlockSensor = new CBlockSensor;
   m_pcBlockTracker = new CBlockTracker(640u, 360u, 10u, 5u, 0.5f, 50.0f);

   /* Create structures for communicating sensor/actuator data with the task */
   m_psSensorData = new SSensorData;
   m_psActuatorData = new SActuatorData;

   /* Create the task */
   m_pcManipulatorTestingTask = new CManipulatorTestingTask(m_psSensorData, m_psActuatorData);

   /* Initialisation was successful */
   return 0;
}

/****************************************/
/****************************************/

void CBlockDemo::Exec() {
   /* useful values for sending over packet control interface */
   enum class EActuatorInputLimit : uint8_t {
      LAUTO = 0, L100 = 1, L150 = 2, L500 = 3, L900 = 4
   };

   uint8_t pnStopDriveSystemData[] = {0, 0, 0, 0};

   unsigned int unTargetId = -1;
   unsigned int unControlTick = -1;

   /* Override actuator input limit to 100mA */
   m_pcPowerManagementInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_ACTUATOR_INPUT_LIMIT_OVERRIDE,
                                            static_cast<const uint8_t>(EActuatorInputLimit::L500));

   /* Enable the actuator power domain */
   m_pcPowerManagementInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_ACTUATOR_POWER_ENABLE, true);

   /* Power up the differential drive system */
   m_pcSensorActuatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_DDS_ENABLE, true);
   
   /* Initialize the differential drive system */
   m_pcSensorActuatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_DDS_SPEED,
                                           pnStopDriveSystemData,
                                           sizeof(pnStopDriveSystemData));


   std::cerr << "Charging the electromagnet capacitors: ";
   /* Disable electromagnet discharge */
   m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_EM_DISCHARGE_MODE,
                                        static_cast<const uint8_t>(EGripperFieldMode::DISABLED));
   /* Enable charging for the electromagnetic capacitors */
   m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_EM_CHARGE_ENABLE, true);
   
   /* Allow the electromagnet capacitors to charge to 50% so that we don't brown out the remote power supply */
   while(!bShutdownSignal) {        
      m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::GET_EM_ACCUM_VOLTAGE);
      if(m_pcManipulatorInterface->WaitForPacket(1000, 5)) {
         const CPacketControlInterface::CPacket& cPacket = m_pcManipulatorInterface->GetPacket();
         if(cPacket.GetType() == CPacketControlInterface::CPacket::EType::GET_EM_ACCUM_VOLTAGE &&
            cPacket.GetDataLength() == 1) {
            const uint8_t* punPacketData = cPacket.GetDataPointer();
            if(punPacketData[0] > 0x80) {
               std::cerr << "OK" << std::endl;
               break;
            }
            else {
               std::cerr << static_cast<int>((punPacketData[0] * 100.0f) / (0x80 * 1.0f)) << "% ";
            }
         }
      }
      sleep(1);
   }
   
   /* Start calibration of the lift actuator */
   std::cerr << "Calibrating the lift actuator: ";
   m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::CALIBRATE_LIFT_ACTUATOR);

   while(!bShutdownSignal) {
      m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::GET_LIFT_ACTUATOR_STATE);
      if(m_pcManipulatorInterface->WaitForPacket(1000, 5)) {
         const CPacketControlInterface::CPacket& cPacket = m_pcManipulatorInterface->GetPacket();
         if(cPacket.GetType() == CPacketControlInterface::CPacket::EType::GET_LIFT_ACTUATOR_STATE &&
            cPacket.GetDataLength() == 1) {
            const uint8_t* punPacketData = cPacket.GetDataPointer();
            if(punPacketData[0] == static_cast<uint8_t>(ELiftActuatorSystemState::INACTIVE)) {
               std::cerr << "OK" << std::endl;
               break;
            }
         }
      }
      sleep(1);
   }
   
   /* grab a couple frames to allow the sensor to adjust to the lighting conditions */
   for(unsigned int unNumberFrames = 5; unNumberFrames > 0; unNumberFrames--) {
      m_pcISSCaptureDevice->Grab();
   }
   
   
   
   std::chrono::time_point<std::chrono::system_clock> tExperimentStart, tLastTick, tNow;
   
   tExperimentStart = std::chrono::system_clock::now();
   
   for(;;) {
      /* regulate the control tick rate to 200ms */
      for(;;) {
         tNow = std::chrono::system_clock::now();
         if(std::chrono::duration<float>(tNow - tLastTick).count() > 0.2f) {
            tLastTick = tNow;
            break;
         }
      }
   
      unControlTick++;
      
      /******** DO IMAGE PROCESSING ********/   
      std::thread tImageProcessing([this]() {
         if(m_psSensorData->ImageSensor.Enable) {
            /* capture a frame  from the camera interface */
            m_pcISSCaptureDevice->GetFrame(m_psSensorData->ImageSensor.Y,
                                           m_psSensorData->ImageSensor.U,
                                           m_psSensorData->ImageSensor.V);                                 
            /* Reset the list of detected blocks */
            m_psSensorData->ImageSensor.Detections.Blocks.clear();
            /* Populate that list with new detections */
            m_pcBlockSensor->DetectBlocks(m_psSensorData->ImageSensor.Y,
                                          m_psSensorData->ImageSensor.U,
                                          m_psSensorData->ImageSensor.V,
                                          m_psSensorData->ImageSensor.Detections.Blocks);
            /* Associate detections to a set of targets */
            m_pcBlockTracker->AssociateAndTrackTargets(m_psSensorData->ImageSensor.Detections.Blocks,
                                                       m_psSensorData->ImageSensor.Detections.Targets);
                                                       
            //m_pcStructureAnalyser->DetectStructures(m_psSensorData->ImageSensor.Detections.Targets);
         }
      });

      /* wait for responses from the manipulator interface */
      bool bWaitingForRfResponse = true;
      bool bWaitingForLiftActuatorPositionResponse = true;
      bool bWaitingForLiftActuatorStateResponse = true;
      bool bWaitingForElectromagnetCharge = true;
      
      std::chrono::time_point<std::chrono::system_clock> tCmdDispatch; 
      
      /******** SAMPLE SENSORS ********/
      /* send requests to sample sensors on the manipulator microcontroller */

      do {
         if(std::chrono::duration<float>(std::chrono::system_clock::now() - tCmdDispatch).count() > 0.5) {
            tCmdDispatch = std::chrono::system_clock::now();
            if(bWaitingForRfResponse) {
               m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::GET_RF_RANGE);
            }
            if(bWaitingForLiftActuatorPositionResponse) {
               m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::GET_LIFT_ACTUATOR_POSITION);
            }
            if(bWaitingForLiftActuatorStateResponse) {
               m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::GET_LIFT_ACTUATOR_STATE);
            }
            if(bWaitingForElectromagnetCharge) {
               m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::GET_EM_ACCUM_VOLTAGE);
            }
         }
      
         m_pcManipulatorInterface->ProcessInput();
         if(m_pcManipulatorInterface->GetState() == CPacketControlInterface::EState::RECV_COMMAND) {
            const CPacketControlInterface::CPacket& cPacket = m_pcManipulatorInterface->GetPacket();
            switch(cPacket.GetType()) {
            case CPacketControlInterface::CPacket::EType::GET_EM_ACCUM_VOLTAGE:
               if(cPacket.GetDataLength() == 1) {
                  const uint8_t* punPacketData = cPacket.GetDataPointer();                
                  m_psSensorData->ManipulatorModule.LiftActuator.Electromagnets.Charge.push_back(punPacketData[0]);
                  if(m_psSensorData->ManipulatorModule.LiftActuator.Electromagnets.Charge.size() > 3) {
                     m_psSensorData->ManipulatorModule.LiftActuator.Electromagnets.Charge.pop_front();
                  }
                  bWaitingForElectromagnetCharge = false;
               }
               break;
            case CPacketControlInterface::CPacket::EType::GET_LIFT_ACTUATOR_STATE:
               if(cPacket.GetDataLength() == 1) {
                  const uint8_t* punPacketData = cPacket.GetDataPointer();
                  switch(punPacketData[0]) {
                  case 0:
                     m_psSensorData->ManipulatorModule.LiftActuator.State =
                        ELiftActuatorSystemState::INACTIVE;
                     break;
                  case 1:
                     m_psSensorData->ManipulatorModule.LiftActuator.State =
                        ELiftActuatorSystemState::ACTIVE_POSITION_CTRL;
                     break;
                  case 2:
                     m_psSensorData->ManipulatorModule.LiftActuator.State =
                        ELiftActuatorSystemState::ACTIVE_SPEED_CTRL;
                     break;
                  case 3:
                     m_psSensorData->ManipulatorModule.LiftActuator.State =
                        ELiftActuatorSystemState::CALIBRATION_SRCH_TOP;
                     break;
                  case 4:
                     m_psSensorData->ManipulatorModule.LiftActuator.State =
                        ELiftActuatorSystemState::CALIBRATION_SRCH_BTM;
                     break;
                  default:
                     m_psSensorData->ManipulatorModule.LiftActuator.State =
                        ELiftActuatorSystemState::UNDEFINED;
                  }
                  bWaitingForLiftActuatorStateResponse = false;
               }
               break;
            case CPacketControlInterface::CPacket::EType::GET_RF_RANGE:
               if(cPacket.GetDataLength() == 10) {
                  const uint8_t* punPacketData = cPacket.GetDataPointer();
                  m_psSensorData->ManipulatorModule.RangeFinders.EndEffector =
                     (punPacketData[0] << 8) | punPacketData[1];
                  m_psSensorData->ManipulatorModule.RangeFinders.Left =
                     (punPacketData[2] << 8) | punPacketData[3];
                  m_psSensorData->ManipulatorModule.RangeFinders.Right =
                     (punPacketData[4] << 8) | punPacketData[5];
                  m_psSensorData->ManipulatorModule.RangeFinders.Front =
                     (punPacketData[6] << 8) | punPacketData[7];
                  m_psSensorData->ManipulatorModule.RangeFinders.Underneath =
                     (punPacketData[8] << 8) | punPacketData[9];
                  bWaitingForRfResponse = false;
               }
               break;
            case CPacketControlInterface::CPacket::EType::GET_LIFT_ACTUATOR_POSITION:
               if(cPacket.GetDataLength() == 1) {
                  const uint8_t* punPacketData = cPacket.GetDataPointer();
                  m_psSensorData->ManipulatorModule.LiftActuator.EndEffector.Position =
                     punPacketData[0];
                  bWaitingForLiftActuatorPositionResponse = false;
               }
               break;
            default:
               continue;
            }
         }
      } while((bWaitingForRfResponse || 
               bWaitingForLiftActuatorStateResponse || 
               bWaitingForLiftActuatorPositionResponse ||
               bWaitingForElectromagnetCharge) && !bShutdownSignal);
      
      /* Store the system time and the control tick counter */
      m_psSensorData->Clock.Time = GetTime();
      m_psSensorData->Clock.Ticks = unControlTick;

      if(m_bVerboseOutput) {
         std::cerr << "[Sensors] "
                   << "S(T) = "
                   << (m_psSensorData->ManipulatorModule.LiftActuator.LimitSwitches.Top ? '1' : '0') << ", "
                   << "S(B) = "
                   << (m_psSensorData->ManipulatorModule.LiftActuator.LimitSwitches.Bottom ? '1' : '0') << ", "
                   << "R(E) = "
                   << m_psSensorData->ManipulatorModule.RangeFinders.EndEffector << ", "
                   << "R(L) = "
                   << m_psSensorData->ManipulatorModule.RangeFinders.Left << ", "
                   << "R(R) = "
                   << m_psSensorData->ManipulatorModule.RangeFinders.Right << ", "
                   << "R(F) = "
                   << m_psSensorData->ManipulatorModule.RangeFinders.Front << ", "
                   << "R(U) = "
                   << m_psSensorData->ManipulatorModule.RangeFinders.Underneath << ", " 
                   << "Chg = ";
          
         for(uint8_t un_charge : m_psSensorData->ManipulatorModule.LiftActuator.Electromagnets.Charge) {
            std::cerr << static_cast<int>(un_charge) << " ";
         }
         std::cerr << std::endl;
      }
      
      /* wait for image processing to complete */
      tImageProcessing.join();
       
      /*
      for(const STarget& s_target : m_psSensorData->ImageSensor.Detections.Targets) {
         std::cerr << "Target #" << s_target.Id << ":" << std::endl
                   << "T(X, Y, Z): "
                   << s_target.Observations.front().Translation.X << ", "
                   << s_target.Observations.front().Translation.Y << ", "
                   << s_target.Observations.front().Translation.Z << std::endl
                   << "R(Z, Y, X): "
                   << argos::ToDegrees(argos::CRadians(s_target.Observations.front().Rotation.Z)).GetValue() << ", "
                   << argos::ToDegrees(argos::CRadians(s_target.Observations.front().Rotation.Y)).GetValue() << ", "
                   << argos::ToDegrees(argos::CRadians(s_target.Observations.front().Rotation.X)).GetValue() << std::endl
                   << "LEDs: ";
                   
         for(ELedState e_led_state : s_target.Observations.front().Tags.front().DetectedLeds) {
            std::cerr << e_led_state << " ";
         }
         std::cerr << std::endl;
      }
      */
      
      /******** STEP THE TASK ********/
      bool bTaskIsComplete = m_pcManipulatorTestingTask->Step();

      /* Output the current state of the state machine */
      std::cerr << "[Task] " << *m_pcManipulatorTestingTask << std::endl;
       
      /******** UPDATE ACTUATORS ********/
      /* Differential Drive System */
      if(m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq || 
         m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq) {
         /* Data for transfer */
         /* TODO this should be swapped on the microcontroller */
         int16_t pnLeftSpeed = -m_psActuatorData->DifferentialDriveSystem.Left.Velocity;
         int16_t pnRightSpeed = -m_psActuatorData->DifferentialDriveSystem.Right.Velocity;
         
         uint8_t punData[] = {
            reinterpret_cast<const uint8_t*>(&pnLeftSpeed)[1],
            reinterpret_cast<const uint8_t*>(&pnLeftSpeed)[0],
            reinterpret_cast<const uint8_t*>(&pnRightSpeed)[1],
            reinterpret_cast<const uint8_t*>(&pnRightSpeed)[0]
         };
         /* Send data in a packet to the sensor/actuator interface */
         m_pcSensorActuatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_DDS_SPEED,
                                                 punData,
                                                 sizeof(punData));
         
         /* clear the update request */
         m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = false;
         m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = false;
      }
      
      if(m_psActuatorData->DifferentialDriveSystem.Power.UpdateReq) {
            m_pcSensorActuatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_DDS_ENABLE, 
                                                    m_psActuatorData->DifferentialDriveSystem.Power.Enable);
         m_psActuatorData->DifferentialDriveSystem.Power.UpdateReq = false;
      }

      /* LED Deck */
      for(unsigned int unLEDIdx = 0; unLEDIdx < NUM_LEDS; unLEDIdx++) {
         if(m_psActuatorData->LEDDeck.UpdateReq[unLEDIdx]) {
            switch(m_psActuatorData->LEDDeck.Color[unLEDIdx]) {
            case EColor::RED:
               m_vecLEDs.at(unLEDIdx).SetRed(0x15);
               m_vecLEDs.at(unLEDIdx).SetGreen(0x00);
               m_vecLEDs.at(unLEDIdx).SetBlue(0x00);               
               break;
            case EColor::GREEN:
               m_vecLEDs.at(unLEDIdx).SetRed(0x00);
               m_vecLEDs.at(unLEDIdx).SetGreen(0x15);
               m_vecLEDs.at(unLEDIdx).SetBlue(0x00);               
               break;
            case EColor::BLUE:
               m_vecLEDs.at(unLEDIdx).SetRed(0x00);
               m_vecLEDs.at(unLEDIdx).SetGreen(0x00);
               m_vecLEDs.at(unLEDIdx).SetBlue(0x15);               
               break;
            }
            m_psActuatorData->LEDDeck.UpdateReq[unLEDIdx] = false;
         }     
      }

      /* Lift Actuator */
      if(m_psActuatorData->ManipulatorModule.LiftActuator.Velocity.UpdateReq) {
         /* Data for transfer */
         int8_t pnSpeed[] = {
            m_psActuatorData->ManipulatorModule.LiftActuator.Velocity.Value,
         };
         /* Send data in a packet to the manipulator interface */
         m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_LIFT_ACTUATOR_SPEED,
                                              reinterpret_cast<const uint8_t*>(pnSpeed),
                                              sizeof(pnSpeed));
         /* clear the update requests */
         m_psActuatorData->ManipulatorModule.LiftActuator.Velocity.UpdateReq = false;
      }
      if(m_psActuatorData->ManipulatorModule.LiftActuator.Position.UpdateReq) {
         /* Data for transfer */
         uint8_t pnPosition[] = {
            m_psActuatorData->ManipulatorModule.LiftActuator.Position.Value,
         };
         /* Send data in a packet to the manipulator interface */
         m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_LIFT_ACTUATOR_POSITION,
                                              pnPosition, sizeof(pnPosition));
         /* clear the update requests */
         m_psActuatorData->ManipulatorModule.LiftActuator.Position.UpdateReq = false;
      }

      /* NFC Interface */
      if(m_psActuatorData->ManipulatorModule.NFCInterface.UpdateReq) {
         /* Data for transfer */
         const std::string& strData = m_psActuatorData->ManipulatorModule.NFCInterface.OutboundMessage;
         /* Send data in a packet to the sensor/actuator interface */
         m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::SEND_NFC_MESSAGE,
                                              reinterpret_cast<const uint8_t*>(strData.c_str()),
                                              strData.size());
         /* clear the update request */
         m_psActuatorData->ManipulatorModule.NFCInterface.UpdateReq = false;
      }

      /* Gripper */
      if(m_psActuatorData->ManipulatorModule.EndEffector.UpdateReq) {
         /* Data for transfer */
         const EGripperFieldMode& eGripperFieldMode = m_psActuatorData->ManipulatorModule.EndEffector.FieldMode;
         /* Send data in a packet to the sensor/actuator interface */
         m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_EM_DISCHARGE_MODE,
                                              static_cast<const uint8_t>(eGripperFieldMode));
         /* clear the update request */
         m_psActuatorData->ManipulatorModule.EndEffector.UpdateReq = false;
      }
     
      /* Annotate the frame if requested */
      if(m_bAnnotateImages) {
         for(const STarget& s_target : m_psSensorData->ImageSensor.Detections.Targets) {
            std::ostringstream cText;
            cText << '[' << s_target.Id << ']';
         
            CFrameAnnotator::Annotate(m_psSensorData->ImageSensor.Y,
                                      s_target,
                                      m_pcBlockSensor->GetCameraMatrix(),
                                      m_pcBlockSensor->GetDistortionParameters(),
                                      cText.str());
         }
      }

      /* save frame to disk if a path was given */
      if(!m_strImageSavePath.empty()) {
         std::ostringstream cStream;
         cStream << m_strImageSavePath << "_" << std::setfill('0') << std::setw(5) << unControlTick << ".y";
         std::ofstream cFrameOutputY(cStream.str().c_str());
         cFrameOutputY.write(reinterpret_cast<char*>(m_psSensorData->ImageSensor.Y.data),
                             m_psSensorData->ImageSensor.Y.rows *
                             m_psSensorData->ImageSensor.Y.cols);
      }
      
      /* stream frame to host if connected */
      if(m_pcTCPImageSocket != nullptr && m_psSensorData->ImageSensor.Enable) {
         *m_pcTCPImageSocket << m_psSensorData->ImageSensor.Y;
      }

      /* Exit if the shutdown signal was recieved or the state machine performed an exit transition */
      if(bShutdownSignal || bTaskIsComplete) {
         std::cerr << "Shutdown: request acknowledged" << std::endl;
         break;
      }
   }
   
   std::cerr << "Shutdown: experiment run time was " 
             << std::chrono::duration<float>(std::chrono::system_clock::now() - tExperimentStart).count() 
             << " seconds" << std::endl;

   /******** SHUTDOWN ROUTINE ********/
   /* Disable the lift actuator */
   m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::EMER_STOP_LIFT_ACTUATOR);
   /* Disable electromagnet discharge */
   m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_EM_DISCHARGE_MODE,
                                        static_cast<const uint8_t>(EGripperFieldMode::DISABLED));
   /* Disable electromagnet charging */
   m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_EM_CHARGE_ENABLE, false);
   /* Disable the differential drive system */
   m_pcSensorActuatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_DDS_SPEED,
                                           pnStopDriveSystemData,
                                           sizeof(pnStopDriveSystemData));
   /* Power down the differential drive system */
   m_pcSensorActuatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_DDS_ENABLE, false);

   /* Switch off LEDs */
   for(unsigned int unLEDIdx = 0; unLEDIdx < NUM_LEDS; unLEDIdx++) {    
      m_vecLEDs.at(unLEDIdx).SetRed(0x00);
      m_vecLEDs.at(unLEDIdx).SetGreen(0x00);
      m_vecLEDs.at(unLEDIdx).SetBlue(0x00);
   }
   
   /* Disable the actuator power domain */
   m_pcPowerManagementInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_ACTUATOR_POWER_ENABLE, false);
   /* Disable actuator input limit override */
   m_pcPowerManagementInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_ACTUATOR_INPUT_LIMIT_OVERRIDE,
                                            static_cast<const uint8_t>(EActuatorInputLimit::LAUTO));
   /* Power down range finder LEDs */
   
   std::cerr << "Shutdown: terminating now" << std::endl;
}

