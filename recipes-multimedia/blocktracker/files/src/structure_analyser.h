#ifndef STRUCTURE_ANALYSER_H
#define STRUCTURE_ANALYSER_H

#include <list>
#include <algorithm>

#include "block.h"
#include "target.h"
#include "structure.h"

class CStructureAnalyser {

public:
   CStructureAnalyser() {}

   void DetectStructures(std::list<STarget>& lst_targets);
private:

};

#endif
