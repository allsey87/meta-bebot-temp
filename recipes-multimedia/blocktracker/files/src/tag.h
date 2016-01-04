#ifndef TAG_H
#define TAG_H

#include <opencv2/core/core.hpp>

struct STag {
   std::vector<std::pair<float, float>> Corners;
   std::pair<float, float> Center;
   cv::Mat RotationVector;
   cv::Mat TranslationVector;
};

#endif
