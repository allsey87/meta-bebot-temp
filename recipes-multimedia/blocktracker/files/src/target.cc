#include "target.h"

STarget::TConstListIterator FindTargetFurthestToTheLeft(const STarget::TList& s_target_list) {
   STarget::TConstListIterator itTargetFurthestToTheLeft = std::begin(s_target_list);
   for(STarget::TConstListIterator it_target = std::begin(s_target_list);
      it_target != std::end(s_target_list);
      it_target++) {
      if(it_target->Observations.front().Translation.GetX() <
         itTargetFurthestToTheLeft->Observations.front().Translation.GetX()) {
         itTargetFurthestToTheLeft = it_target;
      }
   }
   return itTargetFurthestToTheLeft;
}

STarget::TConstListIterator FindTargetFurthestToTheRight(const STarget::TList& s_target_list) {
   STarget::TConstListIterator itTargetFurthestToTheRight = std::begin(s_target_list);
   for(STarget::TConstListIterator it_target = std::begin(s_target_list);
      it_target != std::end(s_target_list);
      it_target++) {
      if(it_target->Observations.front().Translation.GetX() >
         itTargetFurthestToTheRight->Observations.front().Translation.GetX()) {
         itTargetFurthestToTheRight = it_target;
      }
   }
   return itTargetFurthestToTheRight;
}

STarget::TConstListIterator FindTrackedTarget(unsigned int un_target_id, const STarget::TList& s_target_list) {
   return std::find_if(std::begin(s_target_list),
                       std::end(s_target_list),
                       [un_target_id] (const STarget& s_target) {
                          return (s_target.Id == un_target_id);
                       });
}

STarget::TConstListIterator FindMostRecentTarget(const STarget::TList& s_target_list) {
   return std::max_element(std::begin(s_target_list),
                           std::end(s_target_list),
                           [] (const STarget& s_target_lhs, const STarget& s_target_rhs) {
                              return (s_target_lhs.Id < s_target_rhs.Id);
                           });
}

/*
STarget::TConstListIterator FindTargetWithMostLeds(const STarget::TList& s_target_list, ELedState e_led_filter = ELedState::ANY) {
   STarget::TConstListIterator itTargetWithMostQ4Leds = std::end(s_target_list);
   unsigned int unTargetWithMostQ4LedsCount = 0;
   for(STarget::TConstListIterator it_target = std::begin(s_target_list);
      it_target != std::end(s_target_list);
      it_target++) {
      const SBlock& s_block = it_target->Observations.front();
      unsigned int unTargetQ4Leds = 0;
      for(ELedState e_led_state : s_block.Tags.front().DetectedLeds) {
         unTargetQ4Leds += (e_led_state == ELedState::Q4) ? 1 : 0;
      }
      for(const STag& s_tag : s_block.HackTags) {
         for(ELedState e_led_state : s_tag.DetectedLeds) {
            unTargetQ4Leds += (e_led_state == ELedState::Q4) ? 1 : 0;
         }
      }
      if(unTargetQ4Leds > unTargetWithMostQ4LedsCount) {
         unTargetWithMostQ4LedsCount = unTargetQ4Leds;
         itTargetWithMostQ4Leds = it_target;
      }
   }
   return itTargetWithMostQ4Leds;
}
*/
