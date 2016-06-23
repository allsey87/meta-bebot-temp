#ifndef TARGET_H
#define TARGET_H

#include <list>
#include "block.h"

struct STarget {
   using TList = std::list<STarget>;
   using TListIterator = TList::iterator;
   using TConstListIterator = TList::const_iterator;

   /* Lists of target observations */
   SBlock::TList Observations;
   SBlock::TList PseudoObservations;

   /* Identifier (0 => unassigned) */
   unsigned int Id = 0;
};

/* functions for working with a list of targets */
STarget::TConstListIterator FindTargetFurthestToTheLeft(const STarget::TList& s_target_list);
STarget::TConstListIterator FindTargetFurthestToTheRight(const STarget::TList& s_target_list);
STarget::TConstListIterator FindTrackedTarget(unsigned int un_target_id, const STarget::TList& s_target_list);
STarget::TConstListIterator FindMostRecentTarget(const STarget::TList& s_target_list);
STarget::TConstListIterator FindTargetWithMostQ4Leds(const STarget::TList& s_target_list);

#endif
