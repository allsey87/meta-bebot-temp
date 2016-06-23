#include "block.h"

unsigned int GetLedCount(const SBlock& s_block, const std::initializer_list<ELedState>& lst_led_states) {
   unsigned int unCount = 0;
   for(ELedState e_led_state : s_block.Tags[0].DetectedLeds) {
      unCount += (std::find(std::begin(lst_led_states), std::end(lst_led_states), e_led_state) != std::end(lst_led_states)) ? 1 : 0;
   }
   for(const STag& s_tag : s_block.HackTags) {
      for(ELedState e_led_state : s_tag.DetectedLeds) {
         unCount += (std::find(std::begin(lst_led_states), std::end(lst_led_states), e_led_state) != std::end(lst_led_states)) ? 1 : 0;
      }
   }
   return unCount;
}

