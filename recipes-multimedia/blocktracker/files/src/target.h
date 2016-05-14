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

#endif
