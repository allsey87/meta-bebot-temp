
#include "block_sensor.h"

#include "tag.h"
#include "block.h"

#include <argos3/core/utility/math/matrix/rotationmatrix3.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/angles.h>

/****************************************/
/****************************************/

CBlockSensor::CBlockSensor() {}

/****************************************/
/****************************************/

void CBlockSensor::DetectBlocks(cv::Mat& c_frame_y,
                                cv::Mat& c_frame_u,
                                cv::Mat& c_frame_v,
                                std::list<SBlock>& lst_blocks) {

   /* Create a list for the detections */
   std::list<SBlock> lst_detections;
   /* extract tags from frame */
   std::vector<AprilTags::TagDetection> vecDetections =
      m_cTagDetector.extractTags(c_frame_y);

   for(const AprilTags::TagDetection& c_detection : vecDetections) {
      /* Create a block for the detection */
      SBlock sBlock;
      /* Add an empty tag to the block and make a reference to it */
      std::vector<STag>& vecBlockTags = sBlock.Tags;
      vecBlockTags.emplace_back();
      STag& sTag = vecBlockTags.back();
      /* Copy the corners of the tags into an STag for future use */
      sTag.Corners.assign(c_detection.p, c_detection.p + 4);
      sTag.Center = c_detection.cxy;
      /* Create a vector of OpenCV 2D points representing the tag */
      std::vector<cv::Point2f> vecImagePts = {
         cv::Point2f(c_detection.p[0].first, c_detection.p[0].second),
         cv::Point2f(c_detection.p[1].first, c_detection.p[1].second),
         cv::Point2f(c_detection.p[2].first, c_detection.p[2].second),
         cv::Point2f(c_detection.p[3].first, c_detection.p[3].second)
      };
      /* OpenCV SolvePnP - detect the translation between the camera 
         plane and the tag plane */
      cv::solvePnP(m_vecTagPts,
                   vecImagePts,
                   m_cCameraMatrix,
                   m_cDistortionParameters,
                   sTag.RotationVector,
                   sTag.TranslationVector);

      /* Detect the LEDs surrounding the tag */
      DetectLeds(sTag, c_frame_y, c_frame_u, c_frame_v);
   
      /* Compose the tag-to-block and camera-to-tag transformations to get
         the camera-to-block transformation, storing the result directly 
         inside sBlock */
      cv::composeRT(m_cTagToBlockRotation,
                    m_cTagToBlockTranslation,
                    sTag.RotationVector,
                    sTag.TranslationVector,
                    sBlock.RotationVector,
                    sBlock.TranslationVector);
      
      /* create opencv and ARGoS rotation matrices for conversion */
      /* Note: it should be possible to convert directly using the angle-axis format (for which a quaternion constructor exists) */
      cv::Matx33f cRotationMatrixCV;
      argos::CRotationMatrix3 cRotationMatrix;
      
      cv::Rodrigues(sBlock.RotationVector, cRotationMatrixCV);
      cRotationMatrix.Set(&cRotationMatrixCV(0,0));
      argos::CQuaternion cRotationQuaternion = cRotationMatrix.ToQuaternion();
      
      /* extract euler angles and normalize */
      argos::CRadians cBlockEulerAngles[3];
      cRotationQuaternion.ToEulerAngles(cBlockEulerAngles[0], cBlockEulerAngles[1], cBlockEulerAngles[2]);
      argos::CRange<argos::CRadians> cBlockRotationRange(-argos::CRadians::PI_OVER_FOUR, argos::CRadians::PI_OVER_FOUR);
      cBlockRotationRange.WrapValue(cBlockEulerAngles[0]);

      sBlock.Rotation.Z = cBlockEulerAngles[0].GetValue();
      sBlock.Rotation.Y = cBlockEulerAngles[1].GetValue();
      sBlock.Rotation.X = cBlockEulerAngles[2].GetValue();
      
      sBlock.Translation.X = sBlock.TranslationVector.at<float>(0, 0);
      sBlock.Translation.Y = sBlock.TranslationVector.at<float>(1, 0);
      sBlock.Translation.Z = sBlock.TranslationVector.at<float>(2, 0);
      
      /* compute the 2D coordinates of the block */
      std::vector<cv::Point3f> vecCentrePoint = {
         cv::Point3f(0,0,0)
      };
      std::vector<cv::Point2f> vecCentrePixel;
      cv::projectPoints(vecCentrePoint,
                        sBlock.RotationVector,
                        sBlock.TranslationVector,
                        m_cCameraMatrix,
                        m_cDistortionParameters,
                        vecCentrePixel);
      sBlock.Coordinates = std::pair<float, float>(vecCentrePixel[0].x, vecCentrePixel[0].y);

      /* store the block into our block list */
      lst_detections.push_back(sBlock);
   }
   /* cluster the blocks */
   ClusterDetections(lst_detections, lst_blocks);
}

/****************************************/
/****************************************/

void CBlockSensor::DetectLeds(STag& s_tag, cv::Mat& c_y_frame, cv::Mat& c_u_frame, cv::Mat& c_v_frame) {
   std::vector<cv::Point3f> vecLedPoints = {
      cv::Point3f(-m_fInterLedLength * 0.5f, 0, 0),
      cv::Point3f( m_fInterLedLength * 0.5f, 0, 0),
      cv::Point3f( 0, -m_fInterLedLength * 0.5f, 0),
      cv::Point3f( 0,  m_fInterLedLength * 0.5f, 0)
   };
   std::vector<cv::Point2f> vecLedCentrePixels;
   cv::projectPoints(vecLedPoints,
                     s_tag.RotationVector,
                     s_tag.TranslationVector,
                     m_cCameraMatrix,
                     m_cDistortionParameters,
                     vecLedCentrePixels);
                    
   for(cv::Point2f& c_led_point : vecLedCentrePixels) {
      /* skip this LED if the coordinates are out of range */
      if(c_led_point.x < (m_unLedRegionOfInterestLength / 2u) ||
         c_led_point.x > (c_u_frame.cols - 1) - (m_unLedRegionOfInterestLength / 2u)) {
         continue;   
      }
      if(c_led_point.y < (m_unLedRegionOfInterestLength / 2u) ||
         c_led_point.y > (c_u_frame.rows - 1) - (m_unLedRegionOfInterestLength / 2u)) {
         continue;   
      }
      
      cv::Rect cLedRectangle(c_led_point.x - (m_unLedRegionOfInterestLength / 2u),
                             c_led_point.y - (m_unLedRegionOfInterestLength / 2u),
                             m_unLedRegionOfInterestLength,
                             m_unLedRegionOfInterestLength);
      
      struct {
         cv::Mat Y, U, V;
      } sRegionOfInterest = {
         c_y_frame(cLedRectangle),
         c_u_frame(cLedRectangle),
         c_v_frame(cLedRectangle),
      };
      
      float fSumY = 0.0f, fWeightedAverageU = 0.0f, fWeightedAverageV = 0.0f;
      
      for(unsigned int un_row = 0; un_row < sRegionOfInterest.Y.rows; un_row++) {
         for(unsigned int un_col = 0; un_col < sRegionOfInterest.Y.cols; un_col++) {
            float fPixelY = sRegionOfInterest.Y.at<uint8_t>(un_row, un_col);
            fSumY += fPixelY;
            fWeightedAverageU += fPixelY * sRegionOfInterest.U.at<uint8_t>(un_row, un_col);
            fWeightedAverageV += fPixelY * sRegionOfInterest.V.at<uint8_t>(un_row, un_col);
         }
      }
      
      //cv::rectangle(c_y_frame, cLedRectangle, cv::Scalar(0, 0, 0));
            
      fWeightedAverageU /= fSumY;
      fWeightedAverageV /= fSumY;
      
      if(fSumY / (m_unLedRegionOfInterestLength * m_unLedRegionOfInterestLength) > 
         m_unLedLuminanceOnThreshold) {
         if(fWeightedAverageU > 128.0f) {
            if(fWeightedAverageV > 128.0f) {
               s_tag.DetectedLeds.push_back(ELedState::Q1);
            }
            else {
               s_tag.DetectedLeds.push_back(ELedState::Q4);
            }
         }
         else {
            if(fWeightedAverageV > 128.0f) {
               s_tag.DetectedLeds.push_back(ELedState::Q2);
            }
            else {
               s_tag.DetectedLeds.push_back(ELedState::Q3);
            }
         }
      }
      else {
         s_tag.DetectedLeds.push_back(ELedState::OFF);
      }
   }
}

/****************************************/
/****************************************/

void CBlockSensor::ClusterDetections(std::list<SBlock>& lst_detections,
                                     std::list<SBlock>& lst_blocks) {
   
   /* some typedefs to avoid going insane */                                  
   typedef std::list<SBlock> TCluster;
   typedef std::list<TCluster> TClusterList;
   /* a working list of clusters */
   TClusterList lstClusters;
   /* loop until we have allocated all of our detections into clusters */
   while(!lst_detections.empty()) {
      /* take a reference to the first block in the detections list */
      std::list<SBlock>::iterator itDetectedBlock = std::begin(lst_detections);
      /* keep a list of interators into the matching clusters */
      std::list<TClusterList::iterator> lstBlockToClusterAssignments;
      /* for each cluster */
      for(TClusterList::iterator it_cluster = std::begin(lstClusters);
          it_cluster != std::end(lstClusters);
          it_cluster++) {
         /* for each block in the cluster */
         for(TCluster::iterator it_block = it_cluster->begin();
             it_block != it_cluster->end();
             it_block++) {
            float fInterblockDist = 
               sqrt(pow(it_block->Translation.X - itDetectedBlock->Translation.X, 2) +
                    pow(it_block->Translation.Y - itDetectedBlock->Translation.Y, 2) +
                    pow(it_block->Translation.Z - itDetectedBlock->Translation.Z, 2));
            /* if the given block in this cluster is within a distance of 
               (m_fBlockSideLength / 2) of the detected block, they belong 
               to the same cluster */
            if(fInterblockDist < (m_fBlockSideLength / 2)) {
               lstBlockToClusterAssignments.push_back(it_cluster);
               /* at this point we know that this cluster is a 
                  candidate for the block and we can stop */
               break;
            }
         }
      }
      /* At this point we have searched all the clusters */
      if(lstBlockToClusterAssignments.size() == 0) {
         /* no matches - create a new cluster in the cluster list */
         lstClusters.emplace_back();
         /* take a reference to the newly created cluster */
         TCluster& tCluster = lstClusters.back();
         /* move our detected block into the cluster */
         tCluster.splice(std::begin(tCluster),
                         lst_detections,
                         itDetectedBlock);
      }
      else {        
         /* move the detected block into the first matching clusters */
         TClusterList::iterator itCluster = lstBlockToClusterAssignments.front();
         /* add the detected block into the first matching cluster */
         itCluster->splice(std::begin(*itCluster),
                           lst_detections,
                           itDetectedBlock);
         /* if there was more than one matching cluster, merge them */
         if(lstBlockToClusterAssignments.size() > 1) {
            /* take an iterator to the first (destination) cluster */
            std::list<TClusterList::iterator>::iterator itDestinationCluster =
               std::begin(lstBlockToClusterAssignments);
            /* move the blocks from the other (source) clusters into the destination cluster */
            for(std::list<TClusterList::iterator>::iterator itSourceCluster = std::next(itDestinationCluster);
                itSourceCluster != std::end(lstBlockToClusterAssignments);
                itSourceCluster++) {
               /* move all the blocks from the source cluster to the destination cluster */
               (*itDestinationCluster)->splice(std::begin(**itDestinationCluster), **itSourceCluster);
               /* remove the empty source cluster */
               lstClusters.erase(*itSourceCluster);
            }
         }
      }
   }

   /*
   unsigned int i = 0;
   for(TCluster& t_cluster : lstClusters) {
      std::cerr << "cluster " << i << " contains:" << std::endl;
      unsigned int j = 0;
      for(SBlock& s_block : t_cluster) {
         std::cout << "Block " << j << std::endl
                   << "Translation (X, Y, Z): " << s_block.Translation.X << ", " << s_block.Translation.Y << ", " << s_block.Translation.Z << std::endl
                   << "Rotation (Z, Y, X): " << argos::ToDegrees(argos::CRadians(s_block.Rotation.Z)).GetValue() << ", " 
                                             << argos::ToDegrees(argos::CRadians(s_block.Rotation.Y)).GetValue() << ", "
                                             << argos::ToDegrees(argos::CRadians(s_block.Rotation.X)).GetValue() << std::endl;
         j++;
      }
      i++;
   }
   */

   /* find the average location of the block */
   for(TCluster& t_cluster : lstClusters) {
      float fX = 0.0f, fY = 0.0f, fZ = 0.0f;
      for(SBlock& s_block : t_cluster) {
         fX += s_block.Translation.X;
         fY += s_block.Translation.Y;
         fZ += s_block.Translation.Z;
      }
      fX /= t_cluster.size();
      fY /= t_cluster.size();
      fZ /= t_cluster.size();
      
      /* Find the block that has the highest tag (lowest Y coordinate in 2D) */
      TCluster::iterator itTopTagBlock = std::min_element(std::begin(t_cluster), 
                                         std::end(t_cluster),
                                         [] (const SBlock& s_block_a, 
                                             const SBlock& s_block_b) {
         return (s_block_a.Tags[0].Center.second 
               < s_block_b.Tags[0].Center.second);
      });
            
      /* update it's location with the average location */
      itTopTagBlock->Translation.X = fX;
      itTopTagBlock->Translation.Y = fY;
      itTopTagBlock->Translation.Z = fZ;
      
      /* Move itTopTagBlock into our list of blocks */
      /* TODO: move the other tags into the block structure */
      lst_blocks.splice(std::begin(lst_blocks), t_cluster, itTopTagBlock);
   }
   
}

/****************************************/
/****************************************/


