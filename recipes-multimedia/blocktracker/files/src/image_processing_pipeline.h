#ifndef IMAGE_PROCESSING_PIPELINE_H
#define IMAGE_PROCESSING_PIPELINE_H

#include <iss_capture.h>

#include <chrono>
#include <condition_variable>
#include <fstream>
#include <list>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>


#include "block_sensor.h"
#include "tcp_image_socket.h"

/****************************************/
/****************************************/

struct SBuffer {
   /* shared pointers to pixel data */
   std::shared_ptr<image_u8_t> Y, U, V;
   /* index (set by capture thread) */
   unsigned int Index = -1;
   /* capture time */
   std::chrono::time_point<std::chrono::steady_clock> Timestamp;
   /* default constructor */
   SBuffer() :
      Y(image_u8_create(0, 0), image_u8_destroy),
      U(image_u8_create(0, 0), image_u8_destroy),
      V(image_u8_create(0, 0), image_u8_destroy) {}
   /* argument constructor */
   SBuffer(unsigned int un_width, unsigned int un_height) :
      Y(image_u8_create(un_width, un_height), image_u8_destroy),
      U(image_u8_create(un_width, un_height), image_u8_destroy),
      V(image_u8_create(un_width, un_height), image_u8_destroy) {}
   /* copy constructor */
   SBuffer(const SBuffer& s_other_buffer) = delete; /*:
      Y(image_u8_copy(s_other_buffer.Y.get()), image_u8_destroy),
      U(image_u8_copy(s_other_buffer.U.get()), image_u8_destroy),
      V(image_u8_copy(s_other_buffer.V.get()), image_u8_destroy) {}*/
   /* move constructor */
   SBuffer(SBuffer&& s_other_buffer) = delete; /*:
      Y(image_u8_create(0, 0), image_u8_destroy),
      U(image_u8_create(0, 0), image_u8_destroy),
      V(image_u8_create(0, 0), image_u8_destroy) {
         Y.swap(s_other_buffer.Y);
         U.swap(s_other_buffer.U);
         V.swap(s_other_buffer.V);
         Index = s_other_buffer.Index;
         s_other_buffer.Index = -1;
      }
   */
   //}
};

/****************************************/
/****************************************/

class CAsyncPipelineOp {
public:
   ~CAsyncPipelineOp() {
      SetEnable(false);
   }
   
   void SetNextOp(std::shared_ptr<CAsyncPipelineOp> ptr_next_op) {
      m_ptrNextOp = ptr_next_op;
   }

   virtual void Execute(SBuffer& s_buffer) = 0;

   void Enqueue(std::list<SBuffer>& lst_buffers) {
      std::unique_lock<std::mutex> lck(m_mtxBuffersToProcess);
      /* move all buffers from list to our queue */
      m_lstBuffersToProcess.splice(std::end(m_lstBuffersToProcess), lst_buffers);
   }

   void SetEnable(bool b_set_enable) {
      std::unique_lock<std::mutex> lck(m_mtxEnable);
      if(b_set_enable != m_bEnable) {
         if(b_set_enable) {
            /* enable operation */
            m_bEnable = true;
            /* start thread */
            m_ptrThread.reset(new std::thread(&CAsyncPipelineOp::operator(), this));
         }
         else {
            /* disable operation */
            m_bEnable = false;
            /* wait until thread exits */
            m_ptrThread->join();
            /* delete thread */
            m_ptrThread.reset(nullptr);
         }
      }
   }

private:
   void operator()() {
      std::list<SBuffer> lstCurrentBuffer;
      while(m_bEnable) {
         /* wait for buffer */
         std::unique_lock<std::mutex> lckBuffersToProcess(m_mtxBuffersToProcess);
         if(m_lstBuffersToProcess.size() > 0) {
            lstCurrentBuffer.splice(std::begin(lstCurrentBuffer),
                                    m_lstBuffersToProcess,
                                    std::begin(m_lstBuffersToProcess));
         }
         lckBuffersToProcess.unlock();
         if(lstCurrentBuffer.size() > 0) {
            /* process buffer */
            Execute(lstCurrentBuffer.front());
            /* send buffer to next op */
            if(auto ptr_next_op = m_ptrNextOp.lock()) {
               ptr_next_op->Enqueue(lstCurrentBuffer);
            }
            lstCurrentBuffer.clear();
         }
         else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
         }
      }
   }

private:
   std::unique_ptr<std::thread> m_ptrThread;
   std::weak_ptr<CAsyncPipelineOp> m_ptrNextOp;
   std::list<SBuffer> m_lstBuffersToProcess;
   bool m_bEnable = false;
   std::mutex m_mtxEnable;
   std::mutex m_mtxBuffersToProcess;
};

/****************************************/
/****************************************/

class CAsyncCaptureOp : public CAsyncPipelineOp {
public:
   CAsyncCaptureOp(CISSCaptureDevice* pc_iss_capture_device) :
      m_pcISSCaptureDevice(pc_iss_capture_device) {}
private:
   void Execute(SBuffer& s_buffer) override {
      if(m_pcISSCaptureDevice->GetFrame(s_buffer.Y.get(), s_buffer.U.get(), s_buffer.V.get()) == false) {
         std::cerr << "ISS Capture Device Failure" << std::endl;
      }
      s_buffer.Index = ++m_unCaptureIndex;
      s_buffer.Timestamp = std::chrono::steady_clock::now();
   }
   CISSCaptureDevice* m_pcISSCaptureDevice;
   unsigned int m_unCaptureIndex = -1;
};

/****************************************/
/****************************************/

class CAsyncDetectOp : public CAsyncPipelineOp {
public:
   CAsyncDetectOp(CBlockSensor* pc_block_sensor) :
      m_pcBlockSensor(pc_block_sensor) {}
   bool HasDetectedBlocks() {
      std::unique_lock<std::mutex> lckDetectionLists(m_mtxDetectionLists);
      return ((m_lstDetectedBlocks.size() > 0) && (m_lstDetectionTimestamps.size() > 0));
   }   
   void GetDetectedBlocks(SBlock::TList& t_block_list, std::chrono::time_point<std::chrono::steady_clock>& t_timestamp) {
      std::unique_lock<std::mutex> lckDetectionLists(m_mtxDetectionLists);
      if((m_lstDetectedBlocks.size() > 0) && (m_lstDetectionTimestamps.size() > 0)) {
         t_block_list = std::move(m_lstDetectedBlocks.front());
         m_lstDetectedBlocks.pop_front();
         t_timestamp = m_lstDetectionTimestamps.front();
         m_lstDetectionTimestamps.pop_front();
      }
   }
private:
   void Execute(SBuffer& s_buffer) override {
      SBlock::TList lstBlocks;
      m_pcBlockSensor->DetectBlocks(s_buffer.Y.get(), s_buffer.U.get(), s_buffer.V.get(), lstBlocks);
      std::unique_lock<std::mutex> lckDetectionLists(m_mtxDetectionLists);
      m_lstDetectedBlocks.emplace_back(std::move(lstBlocks));
      m_lstDetectionTimestamps.emplace_back(s_buffer.Timestamp);
   }
   CBlockSensor* m_pcBlockSensor;
   std::mutex m_mtxDetectionLists;
   std::list<SBlock::TList> m_lstDetectedBlocks;
   std::list<std::chrono::time_point<std::chrono::steady_clock>> m_lstDetectionTimestamps;
};

/****************************************/
/****************************************/

class CAsyncAnnotateOp : public CAsyncPipelineOp {
public:
   CAsyncAnnotateOp(unsigned int un_annotate_period) :
      m_unAnnotatePeriod(un_annotate_period),
      m_ptrBuffer(nullptr) {}

   void GetBuffer(std::shared_ptr<image_u8_t>& ptr_buffer,
                  std::chrono::time_point<std::chrono::steady_clock>& tp_timestamp) {
      std::unique_lock<std::mutex> lckBuffer(m_mtxBuffer);
      /* wait until m_ptrBuffer has an associated buffer */
      m_cvBufferReady.wait(lckBuffer, [this] {
         return (m_ptrBuffer != nullptr);
      });
      ptr_buffer = m_ptrBuffer;
      tp_timestamp = m_tpTimestamp;
   }
 
   void ReleaseBuffer() {
      std::unique_lock<std::mutex> lckBuffer(m_mtxBuffer);    
      m_ptrBuffer = nullptr;
      lckBuffer.unlock();
      m_cvBufferReady.notify_one();
   }

private:
   void Execute(SBuffer& s_buffer) override {
      if((m_unAnnotatePeriod != 0) && (s_buffer.Index % m_unAnnotatePeriod == 0)) {
         std::unique_lock<std::mutex> lckBuffer(m_mtxBuffer);
         /* set the buffer pointer and timestamp */
         m_ptrBuffer = s_buffer.Y;
         m_tpTimestamp = s_buffer.Timestamp;        
         m_cvBufferReady.notify_one();
         m_cvBufferReady.wait(lckBuffer, [this] {
            return (m_ptrBuffer == nullptr);
         });
      }
   }

   std::chrono::time_point<std::chrono::steady_clock> m_tpTimestamp;
   std::condition_variable m_cvBufferReady;
   std::mutex m_mtxBuffer;
   std::shared_ptr<image_u8_t> m_ptrBuffer;

   unsigned int m_unAnnotatePeriod;
};

/****************************************/
/****************************************/

class CAsyncStreamOp : public CAsyncPipelineOp {
public:
   CAsyncStreamOp(CTCPImageSocket* pc_tcp_image_socket,
                  unsigned int un_stream_period) :
      m_pcTCPImageSocket(pc_tcp_image_socket),
      m_unStreamPeriod(un_stream_period) {}
private:
   void Execute(SBuffer& s_buffer) override {
      if((m_unStreamPeriod != 0) && (s_buffer.Index % m_unStreamPeriod == 0)) {
         m_pcTCPImageSocket->Write(s_buffer.Y->buf,
                                   s_buffer.Y->width,
                                   s_buffer.Y->height,
                                   s_buffer.Y->stride);
      }
   }
   CTCPImageSocket* m_pcTCPImageSocket;
   unsigned int m_unStreamPeriod;
};

/****************************************/
/****************************************/

class CAsyncSaveOp : public CAsyncPipelineOp {
public:
   CAsyncSaveOp(const std::string& str_image_save_path,
                unsigned int un_save_period_y,
                unsigned int un_save_period_u,
                unsigned int un_save_period_v,
                std::chrono::time_point<std::chrono::steady_clock>& t_experiment_start) :
      m_strImageSavePath(str_image_save_path),
      m_mapSavePeriods {
         std::make_pair('y', un_save_period_y),
         std::make_pair('u', un_save_period_u),
         std::make_pair('v', un_save_period_v)
      },
      m_tExperimentStart(t_experiment_start) {}
private:
   void Execute(SBuffer& s_buffer) override {
      for(const std::pair<const char, unsigned int>& c_save_period : m_mapSavePeriods) {
         if((c_save_period.second != 0) && (s_buffer.Index % c_save_period.second == 0)) {   
            std::shared_ptr<image_u8_t> ptrImageData;
            switch(c_save_period.first) {
               case 'y':
                  ptrImageData = s_buffer.Y;
                  break;
               case 'u':
                  ptrImageData = s_buffer.U;
                  break;
               case 'v':
                  ptrImageData = s_buffer.V;
                  break;
            }
            if(ptrImageData) {
               std::ostringstream cFilePath;
               cFilePath << m_strImageSavePath
                         << "_" 
                         << std::setfill('0') 
                         << std::setw(7) 
                         << std::chrono::duration_cast<std::chrono::milliseconds>(s_buffer.Timestamp - m_tExperimentStart).count()
                         << "." 
                         << c_save_period.first;
               std::ofstream cOutputFile(cFilePath.str().c_str());
               for(unsigned int un_row = 0; un_row < ptrImageData->height; un_row++) {
                  // take a pointer to the image row
                  uint8_t* pun_row = ptrImageData->buf + (un_row * ptrImageData->stride);
                  // write it to the output file
                  cOutputFile.write(reinterpret_cast<char*>(pun_row), ptrImageData->width);
               }
            }
         }
      }
   }
   std::string m_strImageSavePath;
   std::map<char, unsigned int> m_mapSavePeriods;
   /* this reference is to a variable initialized by the main thread once the experiment starts */
   std::chrono::time_point<std::chrono::steady_clock>& m_tExperimentStart;
};

/****************************************/
/****************************************/

#endif


