/**
 * @file block_demo.cc
 * @brief Block demo application
 * @author: Michael Allwright
 */

#include <csignal>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <thread>
#include <chrono>

#include <error.h>
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
#include "image_processing_pipeline.h"
#include "leds.h"
#include "packet_control_interface.h"
#include "rfs.h"
#include "structure_analyser.h"
#include "tcp_image_socket.h"

//#include "qualitative_stigmergy_experiment.h"
#include "quantitative_stigmergy_experiment.h"
//#include "pyramid_experiment.h"

unsigned int ManipulatorErrors = 0;

const std::string INDENT = "   ";

/****************************************/
/****************************************/

volatile bool bShutdownSignal = false;

void InterruptSignalHandler(int n_unused) {
   bShutdownSignal = true;
}

/****************************************/
/****************************************/

int main(int n_arg_count, char* ppch_args[]) {
   /* Register the SIGINT handler to shut down system cleanly on abort */
   std::signal(SIGINT, InterruptSignalHandler);
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

const std::string CBlockDemo::m_strIntro =
   "Blocktracker Test Application (build: " __TIME__ ", " __DATE__ ")\n"
   "2016 University of Paderborn\n"
   "Michael Allwright\n";

/****************************************/
/****************************************/

const std::string CBlockDemo::m_strUsage =
   "Usage:\n"
   "  blocktracker [OPTION]...\n"
   "\n"
   "Options:\n"
   "  -h -?              Show help options\n"
   "  -a                 Annotate images\n"
   "  -r <host:port>     Send images via TCP to remote host\n"
   "  -s <path/prefix>   Save images to folder with prefix\n"
   "  -v                 Verbose mode\n";

/****************************************/
/****************************************/

int CBlockDemo::Init(int n_arg_count, char* ppch_args[]) {
   /* Parse command line arguments */
   int c;
   while ((c = ::getopt(n_arg_count, ppch_args, "avr:s:")) != -1) {
      switch (c) {
      case 'a':
         m_bAnnotateEnable = true;
         break;
      case 'r':
         m_bStreamEnable = true;
         m_strRemoteHost = optarg;
         break;
      case 's':
         m_bSaveEnable = true;
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

   /* check the battery levels for the system and actuator circuits */
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

   /* check the battery level of the manipulator module */
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
   if(m_bStreamEnable) {
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
   else {
      std::cerr << "OK" << std::endl;
   }

   /* Link to the LEDs */
   for(unsigned int un_dev_idx = 0; un_dev_idx < NUM_CHASSIS_DEVICES; un_dev_idx++) {
      m_vecLEDs.emplace_back("/sys/class/leds/", "pca963x:led_deck", un_dev_idx);
   }

   /* Link to the rangefinders */
   for(unsigned int un_dev_idx = 0; un_dev_idx < NUM_CHASSIS_DEVICES; un_dev_idx++) {
      m_vecRangeFinders.emplace_back("/sys/class/rfs/", "bebot:rf_chassis", un_dev_idx);
   }

   /* Create the block sensor/tracker instances */
   m_pcBlockSensor = new CBlockSensor(m_cCameraMatrix, m_cDistortionParameters);
   m_pcFrameAnnotator = new CFrameAnnotator(m_cCameraMatrix, m_cDistortionParameters);
   m_pcBlockTracker = new CBlockTracker(3u, 0.05f);
   m_pcStructureAnalyser = new CStructureAnalyser;

   /* Create the image processing pipeline operations */
   /* Capture */
   m_ptrCaptureOp = std::make_shared<CAsyncCaptureOp>(m_pcISSCaptureDevice);
   /* Inject some empty buffers into the capture operation */
   std::list<SBuffer> lstBuffers;
   lstBuffers.emplace_back(640u, 360u);
   lstBuffers.emplace_back(640u, 360u);
   lstBuffers.emplace_back(640u, 360u);
   lstBuffers.emplace_back(640u, 360u);
   m_ptrCaptureOp->Enqueue(lstBuffers);
   /* Detect */
   m_ptrDetectOp = std::make_shared<CAsyncDetectOp>(m_pcBlockSensor);
   m_ptrDetectOp->SetEnable(true);
   /* Save (optional) */
   if(m_bSaveEnable) {
      m_ptrSaveOp = std::make_shared<CAsyncSaveOp>(m_strImageSavePath, 1u, 0u, 0u, m_tpExperimentStart);
      m_ptrSaveOp->SetEnable(true);
   }
   /* Annotate (optional) */
   if(m_bAnnotateEnable) {
      m_ptrAnnotateOp = std::make_shared<CAsyncAnnotateOp>(2u);
      m_ptrAnnotateOp->SetEnable(true);
   }
   /* Stream (optional) */
   if(m_bStreamEnable) {
      m_ptrStreamOp = std::make_shared<CAsyncStreamOp>(m_pcTCPImageSocket, 2u);
      m_ptrStreamOp->SetEnable(true);
   }

   std::list<std::shared_ptr<CAsyncPipelineOp>> lstPipelineOps = {
      m_ptrCaptureOp, m_ptrDetectOp, m_ptrSaveOp, m_ptrAnnotateOp, m_ptrStreamOp
   };
   /* remove unused operations */
   lstPipelineOps.remove(nullptr);
   /* connect the remaining operations together */
   for(auto it_op = std::begin(lstPipelineOps);
       it_op != std::end(lstPipelineOps);
       it_op++) {
      auto itNextOp = std::next(it_op);
      if(itNextOp != std::end(lstPipelineOps)) {
         (*it_op)->SetNextOp(*itNextOp);
      }
      else {
         /* connect the last element back to the front operation */
         (*it_op)->SetNextOp(lstPipelineOps.front());
      }
   }

   /* Create structures for communicating sensor/actuator data with the task */
   m_psSensorData = new SSensorData;
   m_psActuatorData = new SActuatorData;
   /* Update the global data struct */
   Data.Sensors = m_psSensorData;
   Data.Actuators = m_psActuatorData;

   /* Create the finite state machine */
   m_pcFiniteStateMachine = new CFiniteStateMachine();

   /* Initialisation was successful */
   return 0;
}

/****************************************/
/****************************************/

std::ostream& operator<<(std::ostream& c_stream, CBlockDemo::ELiftActuatorSystemState e_state) {
   switch(e_state) {
      case CBlockDemo::ELiftActuatorSystemState::INACTIVE:
         c_stream << "INACTIVE";
      break;
      case CBlockDemo::ELiftActuatorSystemState::ACTIVE_POSITION_CTRL:
         c_stream << "ACTIVE_POSITION_CTRL";
      break;
      case CBlockDemo::ELiftActuatorSystemState::ACTIVE_SPEED_CTRL:
         c_stream << "ACTIVE_SPEED_CTRL";
      break;
      case CBlockDemo::ELiftActuatorSystemState::CALIBRATION_SRCH_TOP:
         c_stream << "CALIBRATION_SRCH_TOP";
      break;
      case CBlockDemo::ELiftActuatorSystemState::CALIBRATION_SRCH_BTM:
         c_stream << "CALIBRATION_SRCH_BTM";
      break;
      default:
         c_stream << "UNDEFINED";
      break;
   }
   return c_stream;
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
                                            static_cast<const uint8_t>(EActuatorInputLimit::L100));
   /* Enable the actuator power domain */
   m_pcPowerManagementInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_ACTUATOR_POWER_ENABLE, true);
   /* Power up the differential drive system */
   m_pcSensorActuatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_DDS_ENABLE, true);
   /* Initialize the differential drive system */
   m_pcSensorActuatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_DDS_SPEED,
                                           pnStopDriveSystemData,
                                           sizeof(pnStopDriveSystemData));
   /* Disable electromagnet discharge */
   m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_EM_DISCHARGE_MODE,
                                        static_cast<const uint8_t>(EGripperFieldMode::DISABLED));
   /* Enable charging for the electromagnetic capacitors */
   m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_EM_CHARGE_ENABLE, true);
   /* Allow the electromagnet capacitors to charge to 50% so that we don't brown out the remote power supply */
   std::cerr << "Charging the electromagnet capacitors: ";
   while(!bShutdownSignal) {
      m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::GET_EM_ACCUM_VOLTAGE);
      if(m_pcManipulatorInterface->WaitForPacket(1000, 5)) {
         const CPacketControlInterface::CPacket& cPacket = m_pcManipulatorInterface->GetPacket();
         if(cPacket.GetType() == CPacketControlInterface::CPacket::EType::GET_EM_ACCUM_VOLTAGE &&
            cPacket.GetDataLength() == 1) {
            const uint8_t* punPacketData = cPacket.GetDataPointer();
            if(punPacketData[0] > 0xC0) {
               std::cerr << "OK" << std::endl;
               break;
            }
            else {
               std::cerr << static_cast<int>((punPacketData[0] * 100.0f) / (0xC0 * 1.0f)) << "% ";
            }
         }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
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
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
   }

   /* grab a couple frames to allow the sensor to adjust to the lighting conditions */
   for(unsigned int unNumberFrames = 5; unNumberFrames > 0; unNumberFrames--) {
      m_pcISSCaptureDevice->Grab();
   }

   /* lists for storing the detected blocks, targets and stuctures */
   SBlock::TList tDetectedBlockList;
   STarget::TList& tTrackedTargetList = m_psSensorData->ImageSensor.Detections.Targets;
   SStructure::TList& tStructureList = m_psSensorData->ImageSensor.Detections.Structures;

   /* debugging strings */
   std::string strLastStateInfo = "top_level_state";
   std::string strLastTrackingInfo = "()";

   /* create time point for the last tick */
   std::chrono::time_point<std::chrono::steady_clock> tLastTick;
   /* mark the start time of the experiment */
   m_tpExperimentStart = std::chrono::steady_clock::now();
   /* Start the async image processing pipeline by enabling the capture operation */
   m_ptrCaptureOp->SetEnable(true);

   for(;;) {
      /* regulate the control tick rate to 150ms */
      for(;;) {
         std::chrono::time_point<std::chrono::steady_clock> tNow = std::chrono::steady_clock::now();
         if(std::chrono::duration<double>(tNow - tLastTick) > std::chrono::milliseconds(250)) {
            tLastTick = tNow;
            break;
         }
         std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      unControlTick++;

      /******** SAMPLE SENSORS ********/
      std::chrono::time_point<std::chrono::steady_clock> tpDetectionTimestamp;

      /* wait for detected blocks */
      while(!m_ptrDetectOp->HasDetectedBlocks()) {
         std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      /* consume all detected blocks */
      while(m_ptrDetectOp->HasDetectedBlocks()) {
         /* Get the detected blocks from the pipeline */
         m_ptrDetectOp->GetDetectedBlocks(tDetectedBlockList, tpDetectionTimestamp);
         auto ts = std::chrono::duration_cast<std::chrono::milliseconds>(tpDetectionTimestamp - m_tpExperimentStart).count();
         /* Associate detections to a set of targets */
         m_pcBlockTracker->AssociateAndTrackTargets(tpDetectionTimestamp, tDetectedBlockList, tTrackedTargetList);
         /* Detect structures */
         m_pcStructureAnalyser->DetectStructures(tTrackedTargetList, tStructureList);
         /* annotate images if enabled */
         if(m_bAnnotateEnable) {
            std::shared_ptr<image_u8_t> ptrBuffer;
            std::chrono::time_point<std::chrono::steady_clock> tpTimestamp;
            m_ptrAnnotateOp->GetBuffer(ptrBuffer, tpTimestamp);
            if(tpTimestamp == tpDetectionTimestamp) {
               for(const STarget& s_target : tTrackedTargetList) {
                  cv::Scalar cColor(0.0f,0.0f,0.0f);
                  m_pcFrameAnnotator->Annotate(s_target, cColor);
               }
               /* create an opencv header for the image data */
               cv::Mat cFrame(ptrBuffer->height, ptrBuffer->width, CV_8UC1, ptrBuffer->buf, ptrBuffer->stride);
               /* annotate it */
               m_pcFrameAnnotator->WriteToFrame(cFrame);
               m_pcFrameAnnotator->Clear();
               /* reset the shared ptr before releasing buffer */
               ptrBuffer = nullptr;
               m_ptrAnnotateOp->ReleaseBuffer();
            }
            else {
               ptrBuffer = nullptr;
            }
         }
      }

      /* wait for responses from the manipulator interface */
      bool bWaitingForRfResponse = true;
      bool bWaitingForLiftActuatorPositionResponse = true;
      bool bWaitingForLiftActuatorStateResponse = true;
      bool bWaitingForElectromagnetCharge = true;

      /* send requests to sample sensors on the manipulator microcontroller */
      m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::GET_RF_RANGE);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::GET_LIFT_ACTUATOR_POSITION);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::GET_LIFT_ACTUATOR_STATE);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::GET_EM_ACCUM_VOLTAGE);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));

      std::chrono::time_point<std::chrono::steady_clock> tManipQueryStart = std::chrono::steady_clock::now();

      while((bWaitingForRfResponse || bWaitingForLiftActuatorStateResponse ||
             bWaitingForLiftActuatorPositionResponse || bWaitingForElectromagnetCharge) && !bShutdownSignal) {
         /* process input on the manipulator interface */
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
                  m_psSensorData->ManipulatorModule.RangeFinders.Underneath =
                     (punPacketData[6] << 8) | punPacketData[7];
                  m_psSensorData->ManipulatorModule.RangeFinders.Front =
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
         if(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - tManipQueryStart).count() > 250) {
            /* TODO remove test code */
            tManipQueryStart = std::chrono::steady_clock::now();
            ManipulatorErrors++;
            if(bWaitingForRfResponse) {
               m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::GET_RF_RANGE);
               std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            if(bWaitingForLiftActuatorPositionResponse) {
               m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::GET_LIFT_ACTUATOR_POSITION);
               std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            if(bWaitingForLiftActuatorStateResponse) {
               m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::GET_LIFT_ACTUATOR_STATE);
               std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            if(bWaitingForElectromagnetCharge) {
               m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::GET_EM_ACCUM_VOLTAGE);
               std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
         }
      }
   
      /* sample chassis range finders */
      for(unsigned int unRfIdx = 0; unRfIdx < NUM_CHASSIS_DEVICES; unRfIdx++) {
         m_psSensorData->RangeFinders[unRfIdx].push_front(m_vecRangeFinders[unRfIdx].Read());
         if(m_psSensorData->RangeFinders[unRfIdx].size() > RF_HISTORY_LEN) {
            m_psSensorData->RangeFinders[unRfIdx].pop_back();
         }
      }

      /* store the experiment timer and the control tick counter */
      m_psSensorData->Clock.ExperimentStart = m_tpExperimentStart;
      m_psSensorData->Clock.Time = tpDetectionTimestamp;
      m_psSensorData->Clock.Ticks = unControlTick;

      auto nFrameIdx = std::chrono::duration_cast<std::chrono::milliseconds>(tpDetectionTimestamp - m_tpExperimentStart).count();

      std::ostringstream cTrackingInfo;
      for(STarget& s_target : tTrackedTargetList) {
         if(s_target.Id == Data.TrackedTargetId) {
            cTrackingInfo << ('(' + std::to_string(s_target.Id) + ')');
         }
         else {
            cTrackingInfo << std::to_string(s_target.Id);
         }
         cTrackingInfo << ' ';
      }
      if(cTrackingInfo.str() != strLastTrackingInfo) {
         std::cerr << '[' << std::setfill('0') << std::setw(8) << nFrameIdx << ']' << " tracking status:" << std::endl
                   << INDENT << ((cTrackingInfo.str() == "") ? std::string("()") : cTrackingInfo.str()) << std::endl;
         strLastTrackingInfo = cTrackingInfo.str();
      }

      /*
      auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
      if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {       
         std::cerr << "selected LED state: " << GetBlockLedState(itTarget->Observations.front()) << std::endl;
      
         const SBlock& s_block = itTarget->Observations.front();
         std::cerr << "x = " << s_block.Translation.GetX() << " (" << PREAPPROACH_BLOCK_X_TARGET << ")" << std::endl;
         std::cerr << "z = " << s_block.Translation.GetZ() << " (" << PREAPPROACH_BLOCK_Z_TARGET << ")" << std::endl;
      }
      */

      /*
      for(STarget& s_target : tTrackedTargetList) {
         std::cerr << "Target[" << s_target.Id;
         const SBlock& s_block = s_target.Observations.front();
         double fStructureDistanceYZ = 
            std::hypot(s_block.Translation.GetY(), s_block.Translation.GetZ()) * 
            argos::Sin(-argos::ATan2(s_block.Translation.GetY(), s_block.Translation.GetZ()) + argos::CRadians::PI_OVER_FOUR);        
         std::cerr << "] fDist = " << fStructureDistanceYZ << std::endl;
      }
      */

      /*
      auto itPyramidTarget = FindPyramidTarget(tTrackedTargetList);
      if(itPyramidTarget != std::end(tTrackedTargetList)) {
         std::cerr << "Pyramid target " << itPyramidTarget->Id << " (" << GetBlockLedState(itPyramidTarget->Observations.front()) << ")" << std::endl;
      }
      */

      /******** STEP THE TASK ********/
      bool bTaskIsComplete = m_pcFiniteStateMachine->Step();

      /* Output the current state of the state machine if a transition occurred */
      std::ostringstream cStateInfo;
      cStateInfo << *m_pcFiniteStateMachine;
      if(cStateInfo.str() != strLastStateInfo) {
         std::cerr << '[' << std::setfill('0') << std::setw(8) << nFrameIdx << ']' << " transition:" << std::endl
                   << INDENT << strLastStateInfo << " =>" << std::endl
                   << INDENT << cStateInfo.str() << std::endl;
         strLastStateInfo = cStateInfo.str();
      }

      /******** UPDATE ACTUATORS ********/
      /* Differential Drive System */
      /*
      if(m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq ||
         m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq) {
      */
      // note: this hack is put in place as occasionally the microcontrollers seem to not receive a message
      if(true) {
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

      /* LED Deck */
      for(unsigned int unLEDIdx = 0; unLEDIdx < NUM_CHASSIS_DEVICES; unLEDIdx++) {
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
         EGripperFieldMode eGripperFieldMode = m_psActuatorData->ManipulatorModule.EndEffector.FieldMode;
         /* Send data in a packet to the sensor/actuator interface */
         m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_EM_DISCHARGE_MODE,
                                              static_cast<const uint8_t>(eGripperFieldMode));
         /* clear the update request */
         m_psActuatorData->ManipulatorModule.EndEffector.UpdateReq = false;
      }

      /* Exit if the shutdown signal was recieved or the state machine performed an exit transition */
      if(bShutdownSignal || bTaskIsComplete) {
         std::cerr << '\r' << '[' << std::setfill('0') << std::setw(8) << nFrameIdx << ']' << " shutdown requested:" << std::endl;
         break;
      }
   }

   double fExperimentRuntime = std::chrono::duration<double>(std::chrono::steady_clock::now() - m_tpExperimentStart).count();


   std::cerr << INDENT << "experiment run time: " << fExperimentRuntime << 's' << std::endl;
   std::cerr << INDENT << "average control tick length: " << fExperimentRuntime / static_cast<double>(unControlTick) << 's' << std::endl;

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
   /* shutdown async image processing pipeline */
   m_ptrCaptureOp->SetEnable(false);
   /* Switch off LEDs */
   for(unsigned int unLEDIdx = 0; unLEDIdx < NUM_CHASSIS_DEVICES; unLEDIdx++) {
      m_vecLEDs.at(unLEDIdx).SetRed(0x00); m_vecLEDs.at(unLEDIdx).SetGreen(0x00); m_vecLEDs.at(unLEDIdx).SetBlue(0x00);
   }
   /* Disable the actuator power domain */
   m_pcPowerManagementInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_ACTUATOR_POWER_ENABLE, false);
   /* Disable actuator input limit override */
   m_pcPowerManagementInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_ACTUATOR_INPUT_LIMIT_OVERRIDE,
                                            static_cast<const uint8_t>(EActuatorInputLimit::LAUTO));

   if(ManipulatorErrors > 0) {
      std::cerr << INDENT << "manipulator errors: " << ManipulatorErrors << std::endl;
   }
   
   std::cerr << INDENT << "shutdown" << std::endl;
}

