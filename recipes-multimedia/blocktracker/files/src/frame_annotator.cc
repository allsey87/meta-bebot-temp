
#include "frame_annotator.h"

#include "tag.h"
#include "block.h"
#include "target.h"

/* for cv::projectPoints */
#include <opencv2/calib3d/calib3d.hpp>

/****************************************/
/****************************************/

void CFrameAnnotator::Annotate(cv::Mat& c_frame,
                               const SBlock& s_block,
                               const cv::Matx33f& c_camera_matrix,
                               const cv::Vec4f& c_distortion_parameters,
                               const std::string& s_text) {
   /* project points is checking correctness */
   std::vector<cv::Point3f> vecInputTargetPoints;
   std::vector<cv::Point2f> vecOutputImagePoints;
      
   vecInputTargetPoints.push_back(cv::Point3f( 0.0275,  0.0275, 0.0275));
   vecInputTargetPoints.push_back(cv::Point3f( 0.0275, -0.0275, 0.0275));
   vecInputTargetPoints.push_back(cv::Point3f(-0.0275, -0.0275, 0.0275));
   vecInputTargetPoints.push_back(cv::Point3f(-0.0275,  0.0275, 0.0275));
   vecInputTargetPoints.push_back(cv::Point3f( 0.0275,  0.0275, -0.0275));
   vecInputTargetPoints.push_back(cv::Point3f( 0.0275, -0.0275, -0.0275));
   vecInputTargetPoints.push_back(cv::Point3f(-0.0275, -0.0275, -0.0275));
   vecInputTargetPoints.push_back(cv::Point3f(-0.0275,  0.0275, -0.0275));

   cv::projectPoints(vecInputTargetPoints,
                     s_block.RotationMatrix,
                     s_block.TranslationMatrix,
                     c_camera_matrix,
                     c_distortion_parameters,
                     vecOutputImagePoints);

   cv::line(c_frame, vecOutputImagePoints[0], vecOutputImagePoints[1], cv::Scalar(255,0,0), 2);
   cv::line(c_frame, vecOutputImagePoints[1], vecOutputImagePoints[2], cv::Scalar(255,0,0), 2);
   cv::line(c_frame, vecOutputImagePoints[2], vecOutputImagePoints[3], cv::Scalar(255,0,0), 2);
   cv::line(c_frame, vecOutputImagePoints[3], vecOutputImagePoints[0], cv::Scalar(255,0,0), 2);

   cv::line(c_frame, vecOutputImagePoints[4], vecOutputImagePoints[5], cv::Scalar(255,0,0), 2);
   cv::line(c_frame, vecOutputImagePoints[5], vecOutputImagePoints[6], cv::Scalar(255,0,0), 2);
   cv::line(c_frame, vecOutputImagePoints[6], vecOutputImagePoints[7], cv::Scalar(255,0,0), 2);
   cv::line(c_frame, vecOutputImagePoints[7], vecOutputImagePoints[4], cv::Scalar(255,0,0), 2);

   cv::line(c_frame, vecOutputImagePoints[0], vecOutputImagePoints[4], cv::Scalar(255,0,0), 2);
   cv::line(c_frame, vecOutputImagePoints[1], vecOutputImagePoints[5], cv::Scalar(255,0,0), 2);
   cv::line(c_frame, vecOutputImagePoints[2], vecOutputImagePoints[6], cv::Scalar(255,0,0), 2);
   cv::line(c_frame, vecOutputImagePoints[3], vecOutputImagePoints[7], cv::Scalar(255,0,0), 2);
}

/****************************************/
/****************************************/

void CFrameAnnotator::Annotate(cv::Mat& c_frame, const STag& s_tag, const std::string& s_text) {
   for(uint8_t un_corner_idx = 0; un_corner_idx < 4; un_corner_idx++) {
      cv::line(c_frame,
               cv::Point2f(s_tag.Corners[un_corner_idx].first, s_tag.Corners[un_corner_idx].second),
               cv::Point2f(s_tag.Corners[(un_corner_idx + 1) % 4].first, s_tag.Corners[(un_corner_idx + 1) % 4].second),
               cv::Scalar(0,0,255),
               2);
   }
   if(!s_text.empty()) {
      cv::putText(c_frame,
                  s_text,
                  cv::Point2f(s_tag.Center.first + 10, s_tag.Center.second + 10),
                  cv::FONT_HERSHEY_SIMPLEX,
                  1,
                  cv::Scalar(255,0,0),
                  2);
   }
}

/****************************************/
/****************************************/

void CFrameAnnotator::Annotate(cv::Mat& c_frame,
                               const STarget& s_target,
                               const cv::Matx33f& c_camera_matrix,
                               const cv::Vec4f& c_distortion_parameters,
                               const std::string& s_text) {

   Annotate(c_frame,
            s_target.Observations.front(),
            c_camera_matrix,
            c_distortion_parameters);
            
   for(std::list<SBlock>::const_iterator it_block = std::begin(s_target.Observations);
       it_block != std::end(s_target.Observations);
       it_block++) {

      std::list<SBlock>::const_iterator itNextBlock = std::next(it_block);
               
      if(itNextBlock != std::end(s_target.Observations)) {
         cv::line(c_frame,
                  cv::Point2f(it_block->Coordinates.first, it_block->Coordinates.second),
                  cv::Point2f(itNextBlock->Coordinates.first, itNextBlock->Coordinates.second),
                  cv::Scalar(0,0,255),
                  2);
      }
   }
   
   if(!s_text.empty()) {
      cv::putText(c_frame,
                  s_text,
                  cv::Point2f(std::begin(s_target.Observations)->Coordinates.first + 10,
                              std::begin(s_target.Observations)->Coordinates.second + 10),
                  cv::FONT_HERSHEY_SIMPLEX,
                  1,
                  cv::Scalar(255,255,255),
                  2);
   }
}

/****************************************/
/****************************************/


