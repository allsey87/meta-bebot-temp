
#include "block_sensor.h"

#include <iostream>
#include <iomanip>

/****************************************/
/****************************************/

float StandardRad(double t) {
   if (t >= 0.) {
      t = fmod(t+M_PI, 2*M_PI) - M_PI;
   } else {
      t = fmod(t-M_PI, -2*M_PI) + M_PI;
   }
   return t;
}

/****************************************/
/****************************************/

CBlockSensor::CBlockSensor() {}

/****************************************/
/****************************************/

void CBlockSensor::ProcessFrame(cv::Mat& c_grayscale_frame) {
   /* Clear the vector of existing blocks */
   m_vecBlocks.clear();

   /* GetTags extract tags from frame */
   std::vector<AprilTags::TagDetection> vecDetections =
      m_cTagDetector.extractTags(c_grayscale_frame);

   for(const AprilTags::TagDetection& cDetection : vecDetections) {
      STag sTag;
      /* Copy the corners of the tags into an STag for future use */
      sTag.Corners.assign(cDetection.p, cDetection.p + 4);
      sTag.Center = cDetection.cxy;
      /* Create a vector of OpenCV 2D points representing the tag */
      std::vector<cv::Point2f> vecImagePts = {
         cv::Point2f(cDetection.p[0].first, cDetection.p[0].second),
         cv::Point2f(cDetection.p[1].first, cDetection.p[1].second),
         cv::Point2f(cDetection.p[2].first, cDetection.p[2].second),
         cv::Point2f(cDetection.p[3].first, cDetection.p[3].second)
      };
      /* OpenCV SolvePnP - detect the translation between the camera 
         plane and the tag plane */
      cv::solvePnP(m_vecTagPts,
                   vecImagePts,
                   m_cCameraMatrix,
                   m_cDistortionParameters,
                   sTag.RotationVector,
                   sTag.TranslationVector);

      cv::Mat cCamToBlockRotation, cCamToBlockTranslation;    
      /* Compose the tag-to-block and camera-to-tag transformations to get
         the camera-to-block transformation */
      cv::composeRT(m_cTagToBlockRotation,
                    m_cTagToBlockTranslation,
                    sTag.RotationVector,
                    sTag.TranslationVector,
                    cCamToBlockRotation,
                    cCamToBlockTranslation);

      /* extract the position and rotation of the block, relative to the camera */
      cv::Matx33f cR;
      cv::Rodrigues(cCamToBlockRotation, cR);
      cv::Matx44f cT(cR(0,0), cR(0,1), cR(0,2), cCamToBlockTranslation.at<float>(0),
                     cR(1,0), cR(1,1), cR(1,2), cCamToBlockTranslation.at<float>(1),
                     cR(2,0), cR(2,1), cR(2,2), cCamToBlockTranslation.at<float>(2),
                     0,       0,       0,       1);
      cv::Matx44f cM( 0,  0,  1,  0,
                     -1,  0,  0,  0,
                      0, -1,  0,  0,
                      0,  0,  0,  1);
      cv::Matx44f cMT(cM, cT, cv::Matx_MatMulOp());

      cv::Matx33f cF( 1,  0,  0, 
                      0, -1,  0,
                      0,  0,  1);
      cv::Matx33f cFR(cF, cR, cv::Matx_MatMulOp());
      float fYaw, fPitch, fRoll;
      fYaw = StandardRad(atan2(cFR(1,0), cFR(0,0)));
      fPitch = StandardRad(atan2(-cFR(2,0), cFR(0,0) * cos(fYaw) + cFR(1,0) * sin(fYaw)));
      fRoll  = StandardRad(atan2(cFR(0,2) * sin(fYaw) - cFR(1,2) * cos(fYaw),
                                 -cFR(0,1) * sin(fYaw) + cFR(1,1) * cos(fYaw)));

      /* allocate tag to existing block or create a new block */
      bool bBelongsToExistingBlock = false;
      for(SBlock& s_existing_block : m_vecBlocks) {
         float fInterblockDist = sqrt(pow(cMT(0,3) - s_existing_block.X, 2) +
                                      pow(cMT(1,3) - s_existing_block.Y, 2) +
                                      pow(cMT(2,3) - s_existing_block.Z, 2));
         if(fInterblockDist < (m_fBlockSideLength / 2)) {
            s_existing_block.Tags.push_back(sTag);
            bBelongsToExistingBlock = true;
            break;
         }
      }

      /* if the detected block doesn't already exist add it to the collection */
      if(!bBelongsToExistingBlock) {
         m_vecBlocks.push_back(SBlock{{sTag}, cMT(0,3), cMT(1,3), cMT(02,3), fYaw, fPitch, fRoll});
      }
   }

   unsigned int unCount = 0;
   for(const SBlock& s_existing_block : m_vecBlocks) {
      for(const STag& s_tag : s_existing_block.Tags) {
         std::ostringstream cStream;
         cStream << "[" << unCount << "]";
         AnnotateFrame(c_grayscale_frame, s_tag, cStream.str());
      }
      unCount++;
   }
}

/****************************************/
/****************************************/

const std::vector<CBlockSensor::SBlock>& CBlockSensor::GetBlocks() const {
   return m_vecBlocks;
}

/****************************************/
/****************************************/


void CBlockSensor::AnnotateFrame(cv::Mat& c_grayscale_frame, const SBlock& s_block) {
   /* project points is checking correctness */
   /*
   std::vector<cv::Point3f> TgtPts;
   std::vector<cv::Point2f> outImgPoints;
      
   TgtPts.push_back(cv::Point3f( 0.0275,  0.0275, 0.0275));
   TgtPts.push_back(cv::Point3f( 0.0275, -0.0275, 0.0275));
   TgtPts.push_back(cv::Point3f(-0.0275, -0.0275, 0.0275));
   TgtPts.push_back(cv::Point3f(-0.0275,  0.0275, 0.0275));
   TgtPts.push_back(cv::Point3f( 0.0275,  0.0275, -0.0275));
   TgtPts.push_back(cv::Point3f( 0.0275, -0.0275, -0.0275));
   TgtPts.push_back(cv::Point3f(-0.0275, -0.0275, -0.0275));
   TgtPts.push_back(cv::Point3f(-0.0275,  0.0275, -0.0275));

   cv::projectPoints(TgtPts, cb_rvec, cb_tvec, cameraMatrix, distParam, outImgPoints);

   cv::line(image_gray, outImgPoints[0], outImgPoints[1], cv::Scalar(255,255,255,0), 2);
   cv::line(image_gray, outImgPoints[1], outImgPoints[2], cv::Scalar(255,255,255,0), 2);
   cv::line(image_gray, outImgPoints[2], outImgPoints[3], cv::Scalar(255,255,255,0), 2);
   cv::line(image_gray, outImgPoints[3], outImgPoints[0], cv::Scalar(255,255,255,0), 2);

   cv::line(image_gray, outImgPoints[4], outImgPoints[5], cv::Scalar(255,255,255,0), 2);
   cv::line(image_gray, outImgPoints[5], outImgPoints[6], cv::Scalar(255,255,255,0), 2);
   cv::line(image_gray, outImgPoints[6], outImgPoints[7], cv::Scalar(255,255,255,0), 2);
   cv::line(image_gray, outImgPoints[7], outImgPoints[4], cv::Scalar(255,255,255,0), 2);

   cv::line(image_gray, outImgPoints[0], outImgPoints[4], cv::Scalar(255,255,255,0), 2);
   cv::line(image_gray, outImgPoints[1], outImgPoints[5], cv::Scalar(255,255,255,0), 2);
   cv::line(image_gray, outImgPoints[2], outImgPoints[6], cv::Scalar(255,255,255,0), 2);
   cv::line(image_gray, outImgPoints[3], outImgPoints[7], cv::Scalar(255,255,255,0), 2);
   */
}

/****************************************/
/****************************************/

void CBlockSensor::AnnotateFrame(cv::Mat& c_grayscale_frame, const STag& s_tag, const std::string& s_text = "") {
   for(uint8_t un_corner_idx = 0; un_corner_idx < 4; un_corner_idx++) {
      cv::line(c_grayscale_frame,
               cv::Point2f(s_tag.Corners[un_corner_idx].first, s_tag.Corners[un_corner_idx].second),
               cv::Point2f(s_tag.Corners[(un_corner_idx + 1) % 4].first, s_tag.Corners[(un_corner_idx + 1) % 4].second),
               cv::Scalar(255,255,255,0));
   }
   cv::putText(c_grayscale_frame,
               s_text,
               cv::Point2f(s_tag.Center.first + 10, s_tag.Center.second + 10),
               cv::FONT_HERSHEY_SIMPLEX,
               1,
               cv::Scalar(255,255,255),
               2);
}
