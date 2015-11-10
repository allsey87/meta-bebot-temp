
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

CBlockSensor::CBlockSensor() :
   m_cTagCodes(AprilTags::tagCodes36h11),
   m_cTagDetector(m_cTagCodes),

   m_unWidth(640),
   m_unHeight(360),
   m_fTagSize(0.024),
   m_fFx(554.502453),
   m_fFy(555.298755),
   m_fPx(317.056149),
   m_fPy(181.126526),
   m_cCameraMatrix(m_fFx, 0, m_fPx,
                   0, m_fFy, m_fPy,
                   0,     0,    1),
   m_cDistortionParameters(0, 0, 0, 0) {
}

/****************************************/
/****************************************/

void CBlockSensor::ProcessFrame(cv::Mat& c_grayscale_frame) {

   std::vector<cv::Point3f> vecObjectPts = {
      cv::Point3f(-m_fTagSize/2., -m_fTagSize/2., 0),
      cv::Point3f( m_fTagSize/2., -m_fTagSize/2., 0),
      cv::Point3f( m_fTagSize/2.,  m_fTagSize/2., 0),
      cv::Point3f(-m_fTagSize/2.,  m_fTagSize/2., 0)
   };

   /* GetTags extract tags from frame */
   std::vector<AprilTags::TagDetection> vecDetections =
      m_cTagDetector.extractTags(c_grayscale_frame);

   for(const AprilTags::TagDetection& cDetection : vecDetections) {
      STag sTag;
      /* Copy the corners of the tags into an STag for future use */
      sTag.Corners.assign(cDetection.p, cDetection.p + 4);
      /* Create a vector of OpenCV 2D points representing the tag */
      std::vector<cv::Point2f> vecImagePts = {
         cv::Point2f(cDetection.p[0].first, cDetection.p[0].second),
         cv::Point2f(cDetection.p[1].first, cDetection.p[1].second),
         cv::Point2f(cDetection.p[2].first, cDetection.p[2].second),
         cv::Point2f(cDetection.p[3].first, cDetection.p[3].second)
      };
      /* OpenCV matrices for storing the result of cv::solvePnP */
      cv::Mat ct_rvec, ct_tvec;
      cv::solvePnP(vecObjectPts, vecImagePts, m_cCameraMatrix, m_cDistortionParameters, ct_rvec, ct_tvec);
      cv::Mat 
         cb_rvec, 
         cb_tvec, 
         tb_rvec,
         tb_tvec = cv::Mat::zeros(3,1,CV_32F);
      /* initialization of zero rotation vector */
      cv::Rodrigues(cv::Mat::eye(3,3,CV_32F), tb_rvec);
      /* tb_tvec projects from the tag into the center of the block */
      tb_tvec.at<float>(2) = -0.0275;

      /* Compose the tag-to-block and camera-to-tag transformations to get
         the camera-to-block transformation */
      cv::composeRT(tb_rvec, tb_tvec, ct_rvec, ct_tvec, cb_rvec, cb_tvec);

      /* extract the position and rotation of the block, relative to the camera */
      cv::Matx33f cR;
      cv::Rodrigues(cb_rvec, cR);
      cv::Matx44f cT(cR(0,0), cR(0,1), cR(0,2), cb_tvec.at<float>(0),
                     cR(1,0), cR(1,1), cR(1,2), cb_tvec.at<float>(1),
                     cR(2,0), cR(2,1), cR(2,2), cb_tvec.at<float>(2),
                     0,       0,       0,       1);
      cv::Matx44f cM( 0,  0,  1,  0,
                     -1,  0,  0,  0,
                      0, -1,  0,  0,
                      0,  0,  0,  1);
      cv::Matx44f cMT(cM, cT, cv::Matx_MatMulOp());
      /* Dump the output to the console */
      std::cout << fixed << std::setprecision(3) << "trans = [" << cMT(0,3) << ", " << cMT(1,3) << ", " << cMT(2,3) << "]" << std::endl;

      cv::Matx33f cF( 1,  0,  0, 
                      0, -1,  0,
                      0,  0,  1);
      cv::Matx33f cFR(cF, cR, cv::Matx_MatMulOp());
      float fYaw, fPitch, fRoll;
      fYaw = StandardRad(atan2(cFR(1,0), cFR(0,0)));
      fPitch = StandardRad(atan2(-cFR(2,0), cFR(0,0) * cos(fYaw) + cFR(1,0) * sin(fYaw)));
      fRoll  = StandardRad(atan2(cFR(0,2) * sin(fYaw) - cFR(1,2) * cos(fYaw), -cFR(0,1) * sin(fYaw) + cFR(1,1) * cos(fYaw)));
      /* Dump the output to the console */
      std::cout << fixed << std::setprecision(3) << "rot = [" << fYaw << ", " << fPitch << ", " << fRoll << "]" << endl;

      /* Get the distance */
      float fDist = sqrt(cMT(0,3) * cMT(0,3) + cMT(1,3) * cMT(1,3) + cMT(2,3) * cMT(2,3));
      std::cout << fixed << std::setprecision(3) << "dist = " << fDist << std::endl;

      /*
      if(true) {
         vec_blocks.push_back(SBlock{cMT(0,3), cMT(1,3), cMT(2,3), fYaw, fPitch, fRoll});
      }
      */
   }
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

void CBlockSensor::AnnotateFrame(cv::Mat& c_grayscale_frame, const STag& s_tag) {

}
