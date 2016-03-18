#ifndef BLOCK_DETECTION_PIPELINE_H
#define BLOCK_DETECTION_PIPELINE_H

#include <thread>
#include <mutex>
#include <iostream>
#include <sstream>

#include <iss_capture.h>

#include "block_sensor.h"

class CBlockDetectionPipeline {

public:
   CBlockDetectionPipeline(CISSCaptureDevice* pc_iss_capture_device,
                           CBlockSensor* pc_block_sensor);

   ~CBlockDetectionPipeline();

   void Enable();

   void Disable();

   void WriteDebugToScreen();

   void GetDetectedBlocks(std::list<SBlock>& lst_detected_blocks);

public: //private
   std::function<void()> m_fnCaptureLoop;
   std::function<void()> m_fnDetectLoop;

   std::mutex DebugMutex;
   std::ostringstream Debug;

   CISSCaptureDevice* m_pcIssCaptureDevice;
   CBlockSensor* m_pcBlockSensor;

   std::thread* m_pcCaptureThread;
   std::thread* m_pcDetectThread;

   bool m_bPipelineEnable;

   struct SImageBuffer {
      SImageBuffer();
      ~SImageBuffer();
      std::string Id;
      image_u8_t* Y;
      image_u8_t* U;
      image_u8_t* V;
      std::mutex Mutex;
      enum class EState {
         IDLE,
         READY,   
      } State = EState::IDLE;
      unsigned int Index = -1;
   } m_psImageBuffers[2];

   struct SBlockListBuffer {
      std::string Id;
      std::list<SBlock> Blocks;
      std::mutex Mutex;
      enum class EState {
         IDLE,
         READY,   
      } State = EState::IDLE;
      unsigned int Index = -1;
   } m_psBlockListBuffers[2];
};

#endif
