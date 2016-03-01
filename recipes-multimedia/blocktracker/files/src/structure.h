#ifndef TARGET_H
#define TARGET_H

#include <list>
#include <string>

#include "target.h"

struct SStructure {     
   std::list<STarget> Members;
   
   /* -1 represents no assigned identifier */
   unsigned int Id = -1;
};

#endif
