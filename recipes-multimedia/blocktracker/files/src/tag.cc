#include "tag.h"

std::ostream& operator<<(std::ostream& c_output_stream, ELedState e_led_state) {
   switch(e_led_state) {
      case ELedState::OFF:
         c_output_stream << "OFF";
         break;
      case ELedState::Q1:
         c_output_stream << "Q1";
         break;
      case ELedState::Q2:
         c_output_stream << "Q2";
         break;
      case ELedState::Q3:
         c_output_stream << "Q3";
         break;
      case ELedState::Q4:
         c_output_stream << "Q4";
         break;
   }
   return c_output_stream;
}

const STag::TCoordinate& FindTagCornerFurthestToTheRight(const STag& s_tag) {
   return *std::max_element(std::begin(s_tag.Corners),
                            std::end(s_tag.Corners),
                            [] (const STag::TCoordinate& c_lhs, const STag::TCoordinate& c_rhs) {
                               return c_lhs.first < c_rhs.first;
                            });
}

const STag::TCoordinate& FindTagCornerFurthestToTheLeft(const STag& s_tag) {
   return *std::min_element(std::begin(s_tag.Corners),
                            std::end(s_tag.Corners),
                            [] (const STag::TCoordinate& c_lhs, const STag::TCoordinate& c_rhs) {
                               return c_lhs.first < c_rhs.first;
                            });
}

const STag::TCoordinate& FindTagCornerFurthestToTheBottom(const STag& s_tag) {
   return *std::max_element(std::begin(s_tag.Corners),
                            std::end(s_tag.Corners),
                            [] (const STag::TCoordinate& c_lhs, const STag::TCoordinate& c_rhs) {
                               return c_lhs.second < c_rhs.second;
                            });
}

const STag::TCoordinate& FindTagCornerFurthestToTheTop(const STag& s_tag) {
   return *std::min_element(std::begin(s_tag.Corners),
                            std::end(s_tag.Corners),
                            [] (const STag::TCoordinate& c_lhs, const STag::TCoordinate& c_rhs) {
                               return c_lhs.second < c_rhs.second;
                            });
}

const STag::TCoordinate& GetTagCenter(const STag& s_tag) {
   return s_tag.Center;
}
