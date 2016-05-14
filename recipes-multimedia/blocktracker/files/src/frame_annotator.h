#ifndef FRAME_ANNOTATOR_H
#define FRAME_ANNOTATOR_H

#include <opencv2/core/core.hpp>
#include <functional>

struct STag;
struct SBlock;
struct STarget;
struct SStructure;

class CFrameAnnotator {
public:
   CFrameAnnotator(const cv::Matx<double, 3, 3>& c_camera_matrix,
                   const cv::Matx<double, 5, 1>& c_distortion_parameters) :
      m_cCameraMatrix(c_camera_matrix),
      m_cDistortionParameters(c_distortion_parameters) {}

   void Annotate(const STag& s_tag,
                 const cv::Scalar& c_color,
                 int n_thickness = 1,
                 const std::string& str_text = "");

   void Annotate(const SBlock& s_block,
                 const cv::Scalar& c_color,
                 const std::string& str_text = "");

   void Annotate(const STarget& s_target,
                 const cv::Scalar& c_color,
                 const std::string& str_text = "");

   void Annotate(const SStructure& s_structure,
                 const cv::Scalar& c_color,
                 const std::string& s_text = "");

   void SetLineThickness(int n_line_thickness) {
      m_nLineThickness = n_line_thickness;
   }

   void Label(const cv::Point2d& c_origin,
              const std::string& str_text);

   void WriteToFrame(cv::Mat& c_frame);

   void Clear();

private:
   /* camera matrix */
   const cv::Matx<double, 3, 3>& m_cCameraMatrix;
   /* camera distortion parameters */
   const cv::Matx<double, 5, 1>& m_cDistortionParameters;

   int m_nLineThickness = 1;

   std::vector<std::function<void(cv::Mat&)>> m_vecLabelBackgrounds;
   std::vector<std::function<void(cv::Mat&)>> m_vecLabels;
   std::vector<std::function<void(cv::Mat&)>> m_vecLines;

   const std::vector<cv::Point3d> m_vecBlockVertices = {     
      cv::Point3d( 0.0275,  0.0275, 0.0275),
      cv::Point3d( 0.0275, -0.0275, 0.0275),
      cv::Point3d(-0.0275, -0.0275, 0.0275),
      cv::Point3d(-0.0275,  0.0275, 0.0275),
      cv::Point3d( 0.0275,  0.0275, -0.0275),
      cv::Point3d( 0.0275, -0.0275, -0.0275),
      cv::Point3d(-0.0275, -0.0275, -0.0275),
      cv::Point3d(-0.0275,  0.0275, -0.0275)
   };
};


#endif
