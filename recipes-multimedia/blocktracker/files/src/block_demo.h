#ifndef BLOCK_DEMO_H
#define BLOCK_DEMO_H

#include <string>

#define NUM_LEDS 12

class CPacketControlInterface;
class CTCPImageSocket;
class CBlockSensor;
class CBlockTracker;
class CISSCaptureDevice;
class CManipulatorTestingTask;
class CLED;

class CBlockDemo {
public:
   
   CBlockDemo() :
      m_bAnnotateImages(false),
      m_bVerboseOutput(false),
      m_pcPowerManagementInterface(nullptr),
      m_pcSensorActuatorInterface(nullptr),
      m_pcManipulatorInterface(nullptr),
      m_pcISSCaptureDevice(nullptr),
      m_pcTCPImageSocket(nullptr),
      m_pcBlockSensor(nullptr),
      m_pcBlockTracker(nullptr) {}
   
   static const std::string& GetUsageString() {
      return m_strUsage;
   }

   static const std::string& GetIntroString() {
      return m_strIntro;
   } 
   
   int Init(int n_arg_count, char* pch_args[]); 
   
   void Exec();

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
      ACTIVE = 1,
      /* Calibration search bottom/top */
      CAL_SRCH_BTM = 2,
      CAL_SRCH_TOP = 3,
   };
   
   enum class EColor {
      RED, GREEN, BLUE
   };
   
   struct SSensorData {
      struct {
         cv::Mat Y, U, V;
         bool Enable = false;
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
               uint8_t Charge = 0;
            } Electromagnets;
            struct {
               uint8_t Position = 0;
            } EndEffector;
            ELiftActuatorSystemState State = ELiftActuatorSystemState::INACTIVE;
         } LiftActuator;
      } ManipulatorModule;
      struct {
         double Time = 0;
         unsigned int Ticks = 0;
      } Clock;
   };

   struct SActuatorData {
      struct {
         struct {
            int8_t Velocity = 0;
            bool UpdateReq = false;
         } Left, Right;
         struct {
            bool Enable = false;
            bool UpdateReq = false;
         } Power;
      } DifferentialDriveSystem;
      struct {
         std::array<EColor, 12> Color;
         std::array<bool, 12> UpdateReq;
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
   bool m_bAnnotateImages;
   bool m_bVerboseOutput;
   
   std::string m_strImageSavePath;
   std::string m_strRemoteHost;

   CPacketControlInterface* m_pcPowerManagementInterface;
   CPacketControlInterface* m_pcSensorActuatorInterface;
   CPacketControlInterface* m_pcManipulatorInterface;
   
   std::vector<CLED> m_vecLEDs;

   CISSCaptureDevice* m_pcISSCaptureDevice;
   CTCPImageSocket* m_pcTCPImageSocket;
   
   CBlockSensor* m_pcBlockSensor;
   CBlockTracker* m_pcBlockTracker;

   SSensorData* m_psSensorData;
   SActuatorData* m_psActuatorData;
   CManipulatorTestingTask* m_pcManipulatorTestingTask;

   const static std::string m_strIntro;
   const static std::string m_strUsage;
};

#endif
