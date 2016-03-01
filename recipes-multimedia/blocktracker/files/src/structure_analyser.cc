
#include "structure_analyser.h"

#include <iostream>

void CStructureAnalyser::DetectStructures(std::list<STarget>& lst_targets,
                                          std::list<SStructure> lst_structures) {
  
   /* some typedefs to avoid going insane */                                  
   typedef std::list<STarget> TStructure;
   typedef std::list<TStructure> TStructureList;
   /* a working list of structures */
   TStructureList lstStructures;
   /* loop until we have allocated all of our targets to structures */
   while(!lst_targets.empty()) {
      /* take a reference to the first target in the targets list */
      std::list<STarget>::iterator itTarget = std::begin(lst_targets);
      /* keep a list of interators into the matching structures */
      std::list<TStructureList::iterator> lstTargetToStructureAssignments;
      /* for each structure */
      for(TStructureList::iterator it_structure = std::begin(lstStructures);
          it_structure != std::end(lstStructures);
          it_structure++) {
         /* for each target in the structure */
         for(TStructure::iterator it_target = it_structure->begin();
             it_target != it_structure->end();
             it_target++) {
            float fIntertargetDist = 
               sqrt(pow(it_target->Translation.X - itDetectedTarget->Translation.X, 2) +
                    pow(it_target->Translation.Y - itDetectedTarget->Translation.Y, 2) +
                    pow(it_target->Translation.Z - itDetectedTarget->Translation.Z, 2));
            /* if the given target in this structure is within a distance of 
               (m_fTargetSideLength / 2) of the detected target, they belong 
               to the same structure */
            if(fIntertargetDist < (m_fBlockSideLength / 2)) {
               lstTargetToStructureAssignments.push_back(it_structure);
               /* at this point we know that this structure is a 
                  candidate for the target and we can stop */
               break;
            }
         }
      }
      /* At this point we have searched all the structures */
      if(lstTargetToStructureAssignments.size() == 0) {
         /* no matches - create a new structure in the structure list */
         lstStructures.emplace_back();
         /* take a reference to the newly created structure */
         TStructure& tStructure = lstStructures.back();
         /* move our detected target into the structure */
         tStructure.splice(std::begin(tStructure),
                         lst_targets,
                         itDetectedTarget);
      }
      else {        
         /* move the detected target into the first matching structures */
         TStructureList::iterator itStructure = lstTargetToStructureAssignments.front();
         /* add the detected target into the first matching structure */
         itStructure->splice(std::begin(*itStructure),
                           lst_targets,
                           itDetectedTarget);
         /* if there was more than one matching structure, merge them */
         if(lstTargetToStructureAssignments.size() > 1) {
            /* take an iterator to the first (destination) structure */
            std::list<TStructureList::iterator>::iterator itDestinationStructure =
               std::begin(lstTargetToStructureAssignments);
            /* move the targets from the other (source) structures into the destination structure */
            for(std::list<TStructureList::iterator>::iterator itSourceStructure = std::next(itDestinationStructure);
                itSourceStructure != std::end(lstTargetToStructureAssignments);
                itSourceStructure++) {
               /* move all the targets from the source structure to the destination structure */
               (*itDestinationStructure)->splice(std::begin(**itDestinationStructure), **itSourceStructure);
               /* remove the empty source structure */
               lstStructures.erase(*itSourceStructure);
            }
         }
      }
   }

   
}
