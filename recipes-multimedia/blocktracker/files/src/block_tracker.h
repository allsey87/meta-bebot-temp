#ifndef BLOCK_TRACKER_H
#define BLOCK_TRACKER_H

#include <string>

class CPacketControlInterface;
class CTCPImageSocket;
class CBlockSensor;
class CISSCaptureDevice;

class CBlocktracker {
public:
   CBlocktracker() :
      m_bAnnotateImages(false),
      m_pcPowerManagementInterface(nullptr),
      m_pcSensorActuatorInterface(nullptr),
      m_pcManipulatorInterface(nullptr),
      m_pcISSCaptureDevice(nullptr),
      m_pcTCPImageSocket(nullptr),
      m_pcBlockSensor(nullptr) {}
   
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

   const static std::string m_strIntro;
   const static std::string m_strUsage;
};

#endif
