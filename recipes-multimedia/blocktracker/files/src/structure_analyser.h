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

   void DetectStructures(STarget::TList& lst_targets,
                         SStructure::TList& lst_structures);

   //void CalculateTransform(To, From, Translation, Rotation)
   
   //void MatchesConstraint(const SStructure& s_structure, const SConstraint& s_constraint)
private:
   const double m_fBlockSideLength = 0.055f;
   const double m_fConnectivityThreshold = m_fBlockSideLength * 1.25f;
};

#endif
