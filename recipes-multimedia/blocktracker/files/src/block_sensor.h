#ifndef BLOCK_SENSOR_H
#define BLOCK_SENSOR_H

#include <apriltags/TagDetector.h>
#include <apriltags/Tag36h11.h>
#include <opencv2/core/core.hpp>

class CBlockSensor {
public:

   struct STag {
      std::vector<std::pair<float, float>> Corners;
      std::pair<float, float> Center;
      cv::Mat RotationVector;
      cv::Mat TranslationVector;
   };

   struct SBlock {
      /* Set of tags used to identify the block */
      std::vector<STag> Tags;
      /* Block coordinates and orientation */
      float X, Y, Z, Yaw, Pitch, Roll;
   };
   
public:

   CBlockSensor();

   void ProcessFrame(cv::Mat& c_grayscale_frame);

   const std::vector<SBlock>& GetBlocks() const;

   static void AnnotateFrame(cv::Mat& c_grayscale_frame, const STag& s_tag, const std::string& s_text);

   static void AnnotateFrame(cv::Mat& c_grayscale_frame, const SBlock& s_block);

private:
   std::vector<SBlock> m_vecBlocks;

   /* Initialise select the AprilTag tag family and init the detector */
   AprilTags::TagCodes m_cTagCodes = AprilTags::TagCodes(AprilTags::tagCodes36h11);
   AprilTags::TagDetector m_cTagDetector = AprilTags::TagDetector(m_cTagCodes);

   /* image size in pixels */
   const uint16_t m_unWidth = 640; 
   const uint16_t m_unHeight = 360;
   
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


