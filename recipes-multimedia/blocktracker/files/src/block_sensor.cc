
#include "block_sensor.h"

#include "tag.h"
#include "block.h"

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

void CBlockSensor::DetectBlocks(const cv::Mat& c_grayscale_frame,
                                std::list<SBlock>& lst_blocks) {
   /* clear the list of existing blocks */
   lst_blocks.clear();

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
      for(SBlock& s_existing_block : lst_blocks) {
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
         /* compute the 2D coordinates of the block */
         std::vector<cv::Point3f> vecCentrePoint = {cv::Point3f(0,0,0)};
         std::vector<cv::Point2f> vecCentrePixel;
         cv::projectPoints(vecCentrePoint,
                           cCamToBlockRotation,
                           cCamToBlockTranslation,
                           m_cCameraMatrix,
                           m_cDistortionParameters,
                           vecCentrePixel);
         /* Create and initialise a new block */
         lst_blocks.emplace_back(cMT(0,3), cMT(1,3), cMT(2,3), fYaw, fPitch, fRoll);
         lst_blocks.back().Tags = {sTag};
         lst_blocks.back().Coordinates = std::pair<float, float>(vecCentrePixel[0].x,
                                                                 vecCentrePixel[0].y);
         lst_blocks.back().TranslationMatrix = cCamToBlockTranslation;
         lst_blocks.back().RotationMatrix = cCamToBlockRotation;  
      }
   }
}
