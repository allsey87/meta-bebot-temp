#ifndef TAG_H
#define TAG_H

#include <iostream>
#include <opencv2/core/core.hpp>

enum class ELedState {
   OFF, Q1, Q2, Q3, Q4
};

struct STag {
   using TCoordinate = std::pair<double, double>;
   using TCoordinateConstIterator = std::vector<TCoordinate>::const_iterator;

   std::vector<TCoordinate> Corners;
   TCoordinate Center;
   std::vector<ELedState> DetectedLeds;

   cv::Matx31d RotationVector;
   cv::Matx31d TranslationVector;
};

std::ostream& operator<<(std::ostream& c_output_stream, ELedState e_led_state);

const STag::TCoordinate& FindTagCornerFurthestToTheLeft(const STag& s_tag);
const STag::TCoordinate& FindTagCornerFurthestToTheRight(const STag& s_tag);
const STag::TCoordinate& FindTagCornerFurthestToTheBottom(const STag& s_tag);
const STag::TCoordinate& FindTagCornerFurthestToTheTop(const STag& s_tag);

const STag::TCoordinate& GetTagCenter(const STag& s_tag);

#endif
