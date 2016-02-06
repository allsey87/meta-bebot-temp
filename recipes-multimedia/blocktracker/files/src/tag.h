#ifndef TAG_H
#define TAG_H

#include <iostream>
#include <opencv2/core/core.hpp>

enum class ELedState {
   OFF, Q1, Q2, Q3, Q4,
};

struct STag {
   std::vector<std::pair<float, float>> Corners;
   std::pair<float, float> Center;
   cv::Mat RotationVector;
   cv::Mat TranslationVector;
   std::vector<ELedState> DetectedLeds;
};

std::ostream& operator<<(std::ostream& c_output_stream, ELedState e_led_state);

#endif
