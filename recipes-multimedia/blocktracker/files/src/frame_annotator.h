#ifndef FRAME_ANNOTATOR_H
#define FRAME_ANNOTATOR_H

#include <opencv2/core/core.hpp>
#include "block_sensor.h"

class CFrameAnnotator {

public:

   static void Annotate(cv::Mat& c_grayscale_frame, const CBlockSensor::STag& s_tag, const std::string& s_text = "");

   static void Annotate(cv::Mat& c_grayscale_frame, const CBlockSensor::SBlock& s_block, const std::string& s_text = "");
   
};


#endif
