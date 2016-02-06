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

