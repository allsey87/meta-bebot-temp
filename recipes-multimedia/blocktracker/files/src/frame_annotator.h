#ifndef FRAME_ANNOTATOR_H
#define FRAME_ANNOTATOR_H

#include <opencv2/core/core.hpp>

struct STag;
struct SBlock;
struct STarget;

class CFrameAnnotator {

public:

   static void Annotate(cv::Mat& c_frame,
                        const STag& s_tag,
                        const std::string& s_text = "");

   static void Annotate(cv::Mat& c_frame,
                        const SBlock& s_block,
                        const cv::Matx33f& c_camera_matrix,
                        const cv::Vec4f& c_distortion_parameters,
                        const std::string& s_text = "");

   static void Annotate(cv::Mat& c_frame,
                        const STarget& s_target,
                        const cv::Matx33f& c_camera_matrix,
                        const cv::Vec4f& c_distortion_parameters,
                        const std::string& s_text = "");

};


#endif
