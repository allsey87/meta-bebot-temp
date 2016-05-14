
#include "block_sensor.h"

#include <cstring>

#include <argos3/core/utility/math/matrix/rotationmatrix3.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/angles.h>

#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/image_u8.h>
#include <apriltag/zarray.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "tag.h"
#include "block.h"

/****************************************/
/****************************************/

CBlockSensor::CBlockSensor(const cv::Matx<double, 3, 3>& c_camera_matrix,
                           const cv::Matx<double, 5, 1>& c_distortion_parameters) :
   m_cCameraMatrix(c_camera_matrix),
   m_cDistortionParameters(c_distortion_parameters) {
   /* create the tag family */
   m_psTagFamily = tag36h11_create();
   m_psTagFamily->black_border = 1;
   /* create the tag detector */
   m_psTagDetector = apriltag_detector_create();
   /* add the tag family to the tag detector */
   apriltag_detector_add_family(m_psTagDetector, m_psTagFamily);
   /* configure the tag detector */
   m_psTagDetector->quad_decimate = 1.0f;
   m_psTagDetector->quad_sigma = 0.0f;
   m_psTagDetector->nthreads = 1;
   m_psTagDetector->debug = 0;
   m_psTagDetector->refine_edges = 1;
   m_psTagDetector->refine_decode = 0;
   m_psTagDetector->refine_pose = 0;
}

/****************************************/
/****************************************/

CBlockSensor::~CBlockSensor() {
   /* remove the tag family to the tag detector */
   apriltag_detector_remove_family(m_psTagDetector, m_psTagFamily);
   /* destroy the tag detector */
   apriltag_detector_destroy(m_psTagDetector);
   /* destroy the tag family */
   tag36h11_destroy(m_psTagFamily);
}

/****************************************/
/****************************************/


void CBlockSensor::DetectBlocks(image_u8_t* pt_image_y,
                                image_u8_t* pt_image_u,
                                image_u8_t* pt_image_v,
                                SBlock::TList& t_block_list) {

   /* Create a list for the detections */
   SBlock::TList t_detections_list;

   zarray_t* psDetections = apriltag_detector_detect(m_psTagDetector, pt_image_y);
   
   for(unsigned int un_det_index = 0; un_det_index < zarray_size(psDetections); un_det_index++) {
      apriltag_detection_t *psDetection;
      zarray_get(psDetections, un_det_index, &psDetection);
      /* Create a block for the detection */
      SBlock sBlock;
      /* Add an empty tag to the block and make a reference to it */
      std::vector<STag>& vecBlockTags = sBlock.Tags;
      vecBlockTags.emplace_back();
      STag& sTag = vecBlockTags.back();
      /* Copy the corners of the tags into an STag for later use */
      sTag.Corners = {
         std::pair<double, double>(psDetection->p[0][0], psDetection->p[0][1]),
         std::pair<double, double>(psDetection->p[1][0], psDetection->p[1][1]),
         std::pair<double, double>(psDetection->p[2][0], psDetection->p[2][1]),
         std::pair<double, double>(psDetection->p[3][0], psDetection->p[3][1]),
      };
      /* Copy the tag center coordinate */
      sTag.Center = std::pair<double, double>(psDetection->c[0], psDetection->c[1]);
      /* Create a vector of OpenCV 2D points representing the tag */
      std::vector<cv::Point2d> vecImagePts = {
         cv::Point2d(psDetection->p[0][0], psDetection->p[0][1]),
         cv::Point2d(psDetection->p[1][0], psDetection->p[1][1]),
         cv::Point2d(psDetection->p[2][0], psDetection->p[2][1]),
         cv::Point2d(psDetection->p[3][0], psDetection->p[3][1]),
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
      DetectLeds(sTag, pt_image_y, pt_image_u, pt_image_v);
      /* Compose the tag-to-block and camera-to-tag transformations to get the camera-to-block transformation */
      cv::composeRT(m_cTagToBlockRotationCV,
                    m_cTagToBlockTranslationCV,
                    sTag.RotationVector,
                    sTag.TranslationVector,
                    sBlock.RotationVector,
                    sBlock.TranslationVector);
      /* calculate the center coordinate the the block */
      std::vector<cv::Point2d> vecBlockCentrePixel;
      cv::projectPoints(m_vecOriginPts,
                        sBlock.RotationVector,
                        sBlock.TranslationVector,
                        m_cCameraMatrix,
                        m_cDistortionParameters,
                        vecBlockCentrePixel);
      sBlock.Coordinates.Set(vecBlockCentrePixel[0].x, vecBlockCentrePixel[0].y);
      /* normalise the Z angle of the tag and convert to argos::CVector3 and argos::CQuaternion format */
      argos::CRotationMatrix3 cRotationMatrix;
      argos::CRadians cBlockEulerAngles[3];
      cv::Rodrigues(sBlock.RotationVector, cv::Mat(3, 3, CV_64F, &cRotationMatrix(0,0)));
      cRotationMatrix.ToQuaternion().ToEulerAngles(cBlockEulerAngles[0], cBlockEulerAngles[1], cBlockEulerAngles[2]);
      m_cBlockZRotationRange.WrapValue(cBlockEulerAngles[0]);
      /* set the argos::CVector3 and argos::CQuaternion */
      sBlock.Rotation.FromEulerAngles(cBlockEulerAngles[0], cBlockEulerAngles[1], cBlockEulerAngles[2]);
      sBlock.Translation.Set(sBlock.TranslationVector(0), sBlock.TranslationVector(1), sBlock.TranslationVector(2));
      /* store the block into our block list */
      t_detections_list.push_back(sBlock);
   }
   
   /* clean up */
   apriltag_detections_destroy(psDetections);
   
   /* cluster the blocks */
   ClusterDetections(t_detections_list, t_block_list);
}

/****************************************/
/****************************************/

void CBlockSensor::DetectLeds(STag& s_tag, image_u8_t* pt_y_frame, image_u8_t* pt_u_frame, image_u8_t* pt_v_frame) {
   /* create opencv headers (no copying happens here) */
   cv::Mat c_y_frame(pt_y_frame->height, pt_y_frame->width, CV_8UC1, pt_y_frame->buf, pt_y_frame->stride);
   cv::Mat c_u_frame(pt_u_frame->height, pt_u_frame->width, CV_8UC1, pt_u_frame->buf, pt_u_frame->stride);
   cv::Mat c_v_frame(pt_v_frame->height, pt_v_frame->width, CV_8UC1, pt_v_frame->buf, pt_v_frame->stride);

   std::vector<cv::Point2d> vecLedCentrePixels;
   cv::projectPoints(m_vecLedPoints,
                     s_tag.RotationVector,
                     s_tag.TranslationVector,
                     m_cCameraMatrix,
                     m_cDistortionParameters,
                     vecLedCentrePixels);
                    
   for(cv::Point2d& c_led_point : vecLedCentrePixels) {
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
      
      double fSumY = 0.0f, fWeightedAverageU = 0.0f, fWeightedAverageV = 0.0f;
      
      for(unsigned int un_row = 0; un_row < sRegionOfInterest.Y.rows; un_row++) {
         for(unsigned int un_col = 0; un_col < sRegionOfInterest.Y.cols; un_col++) {
            double fPixelY = sRegionOfInterest.Y.at<uint8_t>(un_row, un_col);
            fSumY += fPixelY;
            fWeightedAverageU += fPixelY * sRegionOfInterest.U.at<uint8_t>(un_row, un_col);
            fWeightedAverageV += fPixelY * sRegionOfInterest.V.at<uint8_t>(un_row, un_col);
         }
      }
      
      //cv::rectangle(c_y_frame, cLedRectangle, cv::Scalar(255, 255, 255));
            
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

void CBlockSensor::ClusterDetections(SBlock::TList& t_detections_list,
                                     SBlock::TList& t_block_list) {
   
   /* some typedefs to avoid going insane */                                  
   using TCluster = SBlock::TList;
   using TClusterList = std::list<TCluster>;
   /* a working list of clusters */
   TClusterList lstClusters;
   /* loop until we have allocated all of our detections into clusters */
   while(!t_detections_list.empty()) {
      /* take a reference to the first block in the detections list */
      SBlock::TList::iterator itDetectedBlock = std::begin(t_detections_list);
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
            double fInterblockDist = argos::Distance(it_block->Translation, itDetectedBlock->Translation);
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
                         t_detections_list,
                         itDetectedBlock);
      }
      else {        
         /* move the detected block into the first matching clusters */
         TClusterList::iterator itCluster = lstBlockToClusterAssignments.front();
         /* add the detected block into the first matching cluster */
         itCluster->splice(std::begin(*itCluster),
                           t_detections_list,
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

   /* find the average location of the block */
   for(TCluster& t_cluster : lstClusters) {
      argos::CVector3 cAvgLocation;
      for(SBlock& s_block : t_cluster) {
         cAvgLocation += s_block.Translation;
      }
      cAvgLocation /= t_cluster.size();
      
      /* Find the block that has the highest tag (lowest Y coordinate in 2D) */
      TCluster::iterator itTopTagBlock = std::min_element(std::begin(t_cluster),
                                         std::end(t_cluster),
                                         [] (const SBlock& s_block_a, 
                                             const SBlock& s_block_b) {
         return (s_block_a.Tags[0].Center.second 
               < s_block_b.Tags[0].Center.second);
      });
            
      /* update it's location with the average location */
      /* perhaps it is better to turn this off - top tag will likely be the best measurement */
      //itTopTagBlock->Translation = cAvgLocation;
      
      /* This is a hack to conserve the other detected tags */
      for(TCluster::iterator it_block = std::begin(t_cluster);
         it_block != std::end(t_cluster);
         it_block++) {
         if(itTopTagBlock != it_block) {
            itTopTagBlock->HackTags.push_back(it_block->Tags[0]);
         }
      }
      
      /* Move itTopTagBlock into our list of blocks */
      /* TODO: move the other tags into the block structure */
      t_block_list.splice(std::begin(t_block_list), t_cluster, itTopTagBlock);
   }
   
}

/****************************************/
/****************************************/


