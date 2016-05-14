#ifndef BLOCK_DEMO_H
#define BLOCK_DEMO_H

#include <string>
#include <list>
#include <deque>

#include "target.h"
#include "image_processing_pipeline.h"

#include <apriltag/image_u8.h>

#define NUM_LEDS 12

class CPacketControlInterface;
class CTCPImageSocket;
class CBlockSensor;
class CBlockTracker;
class CFrameAnnotator;
class CStructureAnalyser;
class CISSCaptureDevice;
class CManipulatorTestingTask;
class CLED;

class CBlockDemo {
public:
   
   CBlockDemo() :
      m_bAnnotateEnable(false),
      m_bSaveEnable(false),
      m_bStreamEnable(false),

      m_bVerboseOutput(false),

      /* remote interfaces */
      m_pcPowerManagementInterface(nullptr),
      m_pcSensorActuatorInterface(nullptr),
      m_pcManipulatorInterface(nullptr),
      /* local devices */ 
      m_pcISSCaptureDevice(nullptr),
      m_pcTCPImageSocket(nullptr),
      /* async pipeline */ 
      m_ptrAnnotateOp(nullptr),
      m_ptrCaptureOp(nullptr),
      m_ptrDetectOp(nullptr),
      m_ptrSaveOp(nullptr),
      m_ptrStreamOp(nullptr),
      /* local algorithms */ 
      m_pcBlockSensor(nullptr),
      m_pcBlockTracker(nullptr),
      m_pcStructureAnalyser(nullptr),
      m_pcFrameAnnotator(nullptr) {}
   
   static const std::string& GetUsageString() {
      return m_strUsage;
   }

   static const std::string& GetIntroString() {
      return m_strIntro;
   } 
   
   int Init(int n_arg_count, char* pch_args[]); 
   
   void Exec();

   /* to implement */
   void Destroy() {}

public:
   enum class EGripperFieldMode : uint8_t {
      CONSTRUCTIVE = 0,
      DESTRUCTIVE = 1,
      DISABLED = 2
   };
   
   enum class ELiftActuatorSystemState : uint8_t {
      /* Inactive means the stepper motor is disabled */
      INACTIVE = 0,
      /* Active means the stepper motor is running */
      ACTIVE_POSITION_CTRL = 1,
      ACTIVE_SPEED_CTRL = 2,
      /* Calibration search bottom/top */
      CALIBRATION_SRCH_TOP = 3,
      CALIBRATION_SRCH_BTM = 4,
      /* Not actually a state */ 
      UNDEFINED = 5,
   };
   
   enum class EColor {
      RED, GREEN, BLUE
   };

   struct SSensorData {
      struct {
         struct {
            std::list<STarget> Targets;
         } Detections;
      } ImageSensor;
      struct {
         struct {
            uint16_t EndEffector = 0, Left = 0, Right = 0,
               Front = 0, Underneath = 0;
         } RangeFinders;
         struct {
            struct {
               bool Top = false, Bottom = false;
            } LimitSwitches;
            struct {
               std::deque<uint8_t> Charge;
            } Electromagnets;
            struct {
               uint8_t Position = 0;
            } EndEffector;
            ELiftActuatorSystemState State = ELiftActuatorSystemState::UNDEFINED;
         } LiftActuator;
      } ManipulatorModule;
      struct {
         float Time = 0;
         unsigned int Ticks = 0;
      } Clock;
   };

   struct SActuatorData {
      struct {
         struct {
            int16_t Velocity = 0;
            bool UpdateReq = false;
         } Left, Right;
         struct {
            bool Enable = false;
            bool UpdateReq = false;
         } Power;
      } DifferentialDriveSystem;
      struct {
         std::array<EColor, 12> Color;
         std::array<bool, 12> UpdateReq = {};
      } LEDDeck;
      struct {
         struct {
            struct {
               int8_t Value = 0;
               bool UpdateReq = false;
            } Velocity;
            struct {
               uint8_t Value = 0;
               bool UpdateReq = false;
            } Position;
         } LiftActuator;
         struct {
            std::string OutboundMessage;
            bool UpdateReq = false;
         } NFCInterface;
         struct {
            EGripperFieldMode FieldMode =
               EGripperFieldMode::DISABLED;
            bool UpdateReq = false;
         } EndEffector;
      } ManipulatorModule;
   };

private:
   bool m_bAnnotateEnable;
   bool m_bSaveEnable;
   bool m_bStreamEnable;

   bool m_bVerboseOutput;
   
   std::string m_strImageSavePath;
   std::string m_strRemoteHost;

   CPacketControlInterface* m_pcPowerManagementInterface;
   CPacketControlInterface* m_pcSensorActuatorInterface;
   CPacketControlInterface* m_pcManipulatorInterface;
   
   std::vector<CLED> m_vecLEDs;

   CISSCaptureDevice* m_pcISSCaptureDevice;
   CTCPImageSocket* m_pcTCPImageSocket;
   
   std::shared_ptr<CAsyncAnnotateOp> m_ptrAnnotateOp;
   std::shared_ptr<CAsyncCaptureOp> m_ptrCaptureOp;
   std::shared_ptr<CAsyncDetectOp> m_ptrDetectOp;
   std::shared_ptr<CAsyncSaveOp> m_ptrSaveOp;
   std::shared_ptr<CAsyncStreamOp> m_ptrStreamOp;

   std::chrono::time_point<std::chrono::steady_clock> m_tExperimentStart;
   
   CBlockSensor* m_pcBlockSensor;
   CBlockTracker* m_pcBlockTracker;
   CStructureAnalyser* m_pcStructureAnalyser;
   CFrameAnnotator* m_pcFrameAnnotator;
   
   SSensorData* m_psSensorData;
   SActuatorData* m_psActuatorData;
   CManipulatorTestingTask* m_pcManipulatorTestingTask;

   const static std::string m_strIntro;
   const static std::string m_strUsage;

private:
   /* camera focal length in pixels */
   const double m_fFx =  8.8396142504070610e+02; 
   const double m_fFy =  8.8396142504070610e+02; 
   /* camera principal point */
   const double m_fPx =  3.1950000000000000e+02;
   const double m_fPy =  1.7950000000000000e+02;
   /* camera distortion coefficients */
   const double m_fK1 =  1.8433447851104852e-02;
   const double m_fK2 =  1.6727474183089033e-01;
   const double m_fK3 = -1.5480889084966631e+00;
   /* camera matrix */
   const cv::Matx<double, 3, 3> m_cCameraMatrix = 
      cv::Matx<double, 3, 3>(m_fFx, 0.0f, m_fPx,
                             0.0f, m_fFy, m_fPy,
                             0.0f,  0.0f,  1.0f);
   /* camera distortion parameters */
   const cv::Matx<double, 5, 1> m_cDistortionParameters =
      cv::Matx<double, 5, 1>(m_fK1, m_fK2, 0.0f, 0.0f, m_fK3);

};

#endif
