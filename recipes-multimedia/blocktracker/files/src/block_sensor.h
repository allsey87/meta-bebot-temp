#ifndef BLOCK_SENSOR_H
#define BLOCK_SENSOR_H

#include <apriltags/TagDetector.h>
#include <apriltags/Tag36h11.h>
#include <opencv2/core/core.hpp>

#include "block.h"

class CBlockSensor {
   
public:

   CBlockSensor();

   void DetectBlocks(const cv::Mat& c_grayscale_frame, std::list<SBlock>& lst_blocks);

   const cv::Matx33f& GetCameraMatrix() {
      return m_cCameraMatrix;
   }

   const cv::Vec4f& GetDistortionParameters() {
      return m_cDistortionParameters;
   }

private:

   /* Initialise select the AprilTag tag family and init the detector */
   AprilTags::TagCodes m_cTagCodes = AprilTags::TagCodes(AprilTags::tagCodes36h11);
   AprilTags::TagDetector m_cTagDetector = AprilTags::TagDetector(m_cTagCodes);

   /* April tag (w.r.t. black frame) and block side length in meters */
   const float m_fTagSize = 0.024;
   const float m_fBlockSideLength = 0.055;
   
   /* camera focal length in pixels */
   const float m_fFx = 555.0; 
   const float m_fFy = 555.0;
   
   /* camera principal point */
   const float m_fPx = 320.0; 
   const float m_fPy = 180.0;

   /* Tag to block translation and rotation constants */
   const cv::Matx31f m_cTagToBlockTranslation = cv::Matx31f(0, 0, -m_fBlockSideLength / 2);
   const cv::Matx31f m_cTagToBlockRotation = cv::Matx31f(0, 0, 0);

   /* corner locations of the april tag */
   const std::vector<cv::Point3f> m_vecTagPts = {
      cv::Point3f(-m_fTagSize/2., -m_fTagSize/2., 0),
      cv::Point3f( m_fTagSize/2., -m_fTagSize/2., 0),
      cv::Point3f( m_fTagSize/2.,  m_fTagSize/2., 0),
      cv::Point3f(-m_fTagSize/2.,  m_fTagSize/2., 0)
   };

   /* camera matrix */
   const cv::Matx33f m_cCameraMatrix = cv::Matx33f(m_fFx, 0, m_fPx,
                                                   0, m_fFy, m_fPy,
                                                   0,     0,    1);
   /* camera distortion parameters */
   const cv::Vec4f m_cDistortionParameters = cv::Vec4f(0, 0, 0, 0);

   
};

#endif


