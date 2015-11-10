#ifndef BLOCK_SENSOR_H
#define BLOCK_SENSOR_H

#include <apriltags/TagDetector.h>
#include <apriltags/Tag36h11.h>
#include <opencv2/core/core.hpp>

class CBlockSensor {
public:

   struct STag {
      std::vector<std::pair<float, float>> Corners;
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

   void AnnotateFrame(cv::Mat& c_grayscale_frame, const STag& s_tag);

   void AnnotateFrame(cv::Mat& c_grayscale_frame, const SBlock& s_block);

private:
   std::vector<SBlock> m_vecBlocks;
   
   AprilTags::TagCodes m_cTagCodes;
   AprilTags::TagDetector m_cTagDetector;

   /* image size in pixels */
   uint16_t m_unWidth; 
   uint16_t m_unHeight;
   /* April tag side length in meters of square black frame */
   float m_fTagSize;
   /* camera focal length in pixels */
   float m_fFx; 
   float m_fFy;
   /* camera principal point */
   float m_fPx; 
   float m_fPy;
   /* camera matrix */
   cv::Matx33f m_cCameraMatrix;
   /* distortion parameters */
   cv::Vec4f m_cDistortionParameters;
};

#endif


