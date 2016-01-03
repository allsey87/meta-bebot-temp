
#include "frame_annotator.h"

/****************************************/
/****************************************/

void CFrameAnnotator::Annotate(cv::Mat& c_grayscale_frame, const CBlockSensor::SBlock& s_block, const std::string& s_text) {
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

void CFrameAnnotator::Annotate(cv::Mat& c_grayscale_frame, const CBlockSensor::STag& s_tag, const std::string& s_text) {
   for(uint8_t un_corner_idx = 0; un_corner_idx < 4; un_corner_idx++) {
      cv::line(c_grayscale_frame,
               cv::Point2f(s_tag.Corners[un_corner_idx].first, s_tag.Corners[un_corner_idx].second),
               cv::Point2f(s_tag.Corners[(un_corner_idx + 1) % 4].first, s_tag.Corners[(un_corner_idx + 1) % 4].second),
               cv::Scalar(255,255,255,0));
   }
   if(!s_text.empty()) {
      cv::putText(c_grayscale_frame,
                  s_text,
                  cv::Point2f(s_tag.Center.first + 10, s_tag.Center.second + 10),
                  cv::FONT_HERSHEY_SIMPLEX,
                  1,
                  cv::Scalar(255,255,255),
                  2);
   }
}

