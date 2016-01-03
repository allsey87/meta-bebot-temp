#ifndef BLOCK_DEMO_H
#define BLOCK_DEMO_H

#include <string>

class CPacketControlInterface;
class CTCPImageSocket;
class CBlockSensor;
class CBlockTracker;
class CISSCaptureDevice;

class CBlockDemo {
public:
   CBlockDemo() :
      m_bAnnotateImages(false),
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

private:
   bool m_bAnnotateImages;
   
   std::string m_strImageSavePath;
   std::string m_strRemoteHost;

   CPacketControlInterface* m_pcPowerManagementInterface;
   CPacketControlInterface* m_pcSensorActuatorInterface;
   CPacketControlInterface* m_pcManipulatorInterface;

   CISSCaptureDevice* m_pcISSCaptureDevice;
   CTCPImageSocket* m_pcTCPImageSocket;
   
   CBlockSensor* m_pcBlockSensor;
   CBlockTracker* m_pcBlockTracker;

   const static std::string m_strIntro;
   const static std::string m_strUsage;
};

#endif
