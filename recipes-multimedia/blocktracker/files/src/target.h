#ifndef TARGET_H
#define TARGET_H

#include <list>
#include <string>

#include "block.h"

struct STarget {     
   std::list<SBlock> Observations;
   unsigned int FramesSinceLastObservation = 0;

   /* -1 represents no assigned identifier */
   unsigned int Id = -1;
};

#endif
