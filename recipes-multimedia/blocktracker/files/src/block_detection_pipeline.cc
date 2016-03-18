
#include "block_detection_pipeline.h"

#include <chrono>

CBlockDetectionPipeline::CBlockDetectionPipeline(CISSCaptureDevice* pc_iss_capture_device,
                                                 CBlockSensor* pc_block_sensor) :
   m_pcIssCaptureDevice(pc_iss_capture_device),
   m_pcBlockSensor(pc_block_sensor),
   m_pcCaptureThread(nullptr),
   m_pcDetectThread(nullptr),
   m_bPipelineEnable(false) {

   /* loop function for capture images from the camera */
   m_fnCaptureLoop = [this] {
      static unsigned int unCaptureIndex = -1;
      while(m_bPipelineEnable) {
         std::this_thread::sleep_for(std::chrono::milliseconds(1));
         for(SImageBuffer& s_image_buffer : m_psImageBuffers) {
            if(s_image_buffer.Mutex.try_lock()) {
               if(s_image_buffer.State == SImageBuffer::EState::IDLE) {
                  if(m_pcIssCaptureDevice->GetFrame(s_image_buffer.Y,
                                                    s_image_buffer.U,
                                                    s_image_buffer.V)) {
                     s_image_buffer.State = SImageBuffer::EState::READY;
                     s_image_buffer.Index = ++unCaptureIndex;
                  }
               }
               s_image_buffer.Mutex.unlock();
            }
         }
      }
   };



   /* loop function for detection blocks from the captured image */
   m_fnDetectLoop = [this] {
      static unsigned int unDetectIndex = -1;
      while(m_bPipelineEnable) {
         std::this_thread::sleep_for(std::chrono::milliseconds(1));
         for(SImageBuffer& s_image_buffer : m_psImageBuffers) {
            if(s_image_buffer.Mutex.try_lock()) {
               if(s_image_buffer.State == SImageBuffer::EState::READY) {
                  for(SBlockListBuffer& s_block_list_buffer : m_psBlockListBuffers) {
                     if(s_block_list_buffer.Mutex.try_lock()) {
                        if(s_block_list_buffer.State == SBlockListBuffer::EState::IDLE) {
                           m_pcBlockSensor->DetectBlocks(s_image_buffer.Y,
                                                         s_image_buffer.U,
                                                         s_image_buffer.V,
                                                         s_block_list_buffer.Blocks);
                           s_block_list_buffer.State = SBlockListBuffer::EState::READY;
                           s_image_buffer.State = SImageBuffer::EState::IDLE;
                           s_block_list_buffer.Index = ++unDetectIndex;
                        }
                        s_block_list_buffer.Mutex.unlock();
                        break;
                     }
                  }
               }
               s_image_buffer.Mutex.unlock();
            }
         }
      }
   };
}

CBlockDetectionPipeline::~CBlockDetectionPipeline() {
   /* disable and clean up resources */
   Disable();
}

void CBlockDetectionPipeline::Enable() {
   if(m_bPipelineEnable == false &&
      m_pcCaptureThread == nullptr &&
      m_pcDetectThread == nullptr) {
      /* reset the buffers */
      for(SBlockListBuffer& s_block_list_buffer : m_psBlockListBuffers) {
         s_block_list_buffer.State = SBlockListBuffer::EState::IDLE;
         s_block_list_buffer.Blocks.clear();
      }
      for(SImageBuffer& s_image_buffer : m_psImageBuffers) {
         s_image_buffer.State = SImageBuffer::EState::IDLE;
      }
      /* enable the pipeline */
      m_bPipelineEnable = true;
      m_pcCaptureThread = new std::thread(m_fnCaptureLoop);
      m_pcDetectThread = new std::thread(m_fnDetectLoop);
   }
}

void CBlockDetectionPipeline::Disable() {
   m_bPipelineEnable = false;
   if(m_pcCaptureThread != nullptr) {
      m_pcCaptureThread->join();
      delete m_pcCaptureThread;
      m_pcCaptureThread = nullptr;
   }
   if(m_pcDetectThread != nullptr) {
      m_pcDetectThread->join();
      delete m_pcDetectThread;
      m_pcDetectThread = nullptr;
   }
}

void CBlockDetectionPipeline::GetDetectedBlocks(std::list<SBlock>& lst_detected_blocks) {
   for(;;) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      for(SBlockListBuffer& s_block_list_buffer : m_psBlockListBuffers) {
         if(s_block_list_buffer.Mutex.try_lock()) {
            if(s_block_list_buffer.State == SBlockListBuffer::EState::READY) {
               lst_detected_blocks.swap(s_block_list_buffer.Blocks);
               s_block_list_buffer.State = SBlockListBuffer::EState::IDLE;
               s_block_list_buffer.Blocks.clear();
               s_block_list_buffer.Mutex.unlock();              
               return;
            }
            else {
               s_block_list_buffer.Mutex.unlock();
            }
         }
      }            
   }
}

CBlockDetectionPipeline::SImageBuffer::SImageBuffer() :
   Y(image_u8_create(640u, 360u)),
   U(image_u8_create(640u, 360u)),
   V(image_u8_create(640u, 360u)) {}

CBlockDetectionPipeline::SImageBuffer::~SImageBuffer() {
   image_u8_destroy(Y);
   image_u8_destroy(U);
   image_u8_destroy(V);
}

