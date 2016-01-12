#ifndef BLOCK_DEMO_H
#define BLOCK_DEMO_H

#include <string>

class CPacketControlInterface;
class CTCPImageSocket;
class CBlockSensor;
class CBlockTracker;
class CISSCaptureDevice;
class CManipulatorTestingTask;

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
   } DischargeMode;
   
   struct SSensorData {
      struct {
         cv::Mat Y, U, V;
      } ImageSensor;
      struct {
         struct {
            uint16_t Left = 0, Right = 0, Front = 0, Underneath = 0;
         } RangeFinders;
         struct {
            bool Top = false, Bottom = false;
         } LimitSwitches;
         struct {
            uint8_t StoredCharge = 0;
         } Gripper;
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
      } DifferentialDriveSystem;
      struct {
         struct {
            int8_t Velocity = 0;
            bool UpdateReq = false;
         } LiftActuator;
         struct {
            std::string OutboundMessage;
            bool UpdateReq = false;
         } NFCInterface;
         struct {
            EGripperFieldMode FieldMode =
               EGripperFieldMode::DISABLED;
            bool UpdateReq = false;
         } Gripper;
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
