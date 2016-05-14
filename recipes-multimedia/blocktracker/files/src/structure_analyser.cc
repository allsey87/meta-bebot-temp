
#include "structure_analyser.h"

#include <iostream>

void CStructureAnalyser::DetectStructures(STarget::TList& lst_targets,
                                          SStructure::TList& lst_structures) {
   /***********************************************/
   /*         clear previous structures           */
   /***********************************************/
   lst_structures.clear();

   /***********************************************/
   /*       cluster targets into structures       */
   /***********************************************/
   for(auto it_target = std::begin(lst_targets);
       it_target != std::end(lst_targets);
       it_target++) {
      /* build a list of iterators to the matching structures */
      std::list<SStructure::TListIterator> lstStructuresMatchingTarget;
      /* for each structure */
      for(auto it_structure = std::begin(lst_structures); // std::list<SStructure>::iterator
          it_structure != std::end(lst_structures);
          it_structure++) {
         /* for each target in the structure */
         for(auto it_structure_target = std::begin(it_structure->Members);
             it_structure_target != std::end(it_structure->Members);
             it_structure_target++) {
            double fDist = argos::Distance((*it_structure_target)->Observations.front().Translation,
                                           it_target->Observations.front().Translation);
            /* if the given target in this structure is within a distance of m_fConnectivityThreshold
               of the other target, they belong to the same structure */
            if(fDist < m_fConnectivityThreshold) {
               lstStructuresMatchingTarget.push_back(it_structure);
               /* at this point we know that this structure is a candidate for the target and we can
                  stop comparing it against targets in the same structure */
               break;
            }
         }
      }
      /* Create, add or merge the structures */
      if(lstStructuresMatchingTarget.size() == 0) {
         /* no matches - create a new structure */
         lst_structures.emplace_back(it_target);
      }
      else {
         /* one or more matches, add target to the first and merge the rest into the first */ 
         /* take an iterator to the destination structure in the matching list */
         auto itDestinationStructure = std::begin(lstStructuresMatchingTarget);
         /* add our current target to the destination structure */
         (*itDestinationStructure)->Members.emplace_back(it_target);
         /* move the targets from the other structures (if any) into the destination structure */
         for(auto itSourceStructure = std::next(itDestinationStructure);
             itSourceStructure != std::end(lstStructuresMatchingTarget);
             itSourceStructure++) {
            /* move targets */
            std::list<STarget::TListIterator>& lstDestinationStructureMembers = (*itDestinationStructure)->Members;
            std::list<STarget::TListIterator>& lstSourceStructureMembers = (*itSourceStructure)->Members;
            lstDestinationStructureMembers.splice(std::end(lstDestinationStructureMembers),
                                                  lstSourceStructureMembers);
            /* remove the empty source structure */
            lst_structures.erase(*itSourceStructure);
         }
      }
   } // for(auto it_target...

   /***********************************************/
   /*  determine connectivity for all structures  */
   /***********************************************/
   for(SStructure& s_structure : lst_structures) {
      /* build the structure connectivity map */
      for(auto it_from_target = std::begin(s_structure.Members);
          it_from_target != std::end(s_structure.Members);
          it_from_target++) {
         for(auto it_to_target = std::begin(s_structure.Members);
             it_to_target != std::end(s_structure.Members);
             it_to_target++) {
            /* don't add links from a target to itself */               
            if(it_from_target != it_to_target) {
               double fDist = argos::Distance((*it_from_target)->Observations.front().Translation,
                                              (*it_to_target)->Observations.front().Translation);
               if(fDist < m_fConnectivityThreshold) {
                  /* populate the connectivity map */
                  s_structure.Connectivity.emplace(*it_from_target, *it_to_target);
               }
            }
         }
      }
   }
}

/*
void afunction() {
   // define the root target as the target with the highest connectivity
   auto itRootTarget = std::end(lstTrackedTargets);
   for(auto it_target = std::begin(lstTrackedTargets);
       it_target != std::end(lstTrackedTargets);
       it_target++) {
      if(mapTargetConnectivity.count(it_target) >= mapTargetConnectivity.count(itRootTarget)) {
         itRootTarget = it_target;
      }
   }

   if(itRootTarget != std::end(lstTrackedTargets)) {
      cFrameAnnotator.Annotate(*itRootTarget, cv::Scalar(0,255,255), "[R]");

      const SBlock& sRootObservation = (itRootTarget->PseudoObservations.size() != 0) ?
          itRootTarget->PseudoObservations.front() : itRootTarget->Observations.front();

      // display the connectivity 
      for(auto it_target = std::begin(lstTrackedTargets);
          it_target != std::end(lstTrackedTargets);
          it_target++) {

         if(it_target == itRootTarget) {
            continue;
         }

         const SBlock& sObservation = (it_target->PseudoObservations.size() != 0) ?
            it_target->PseudoObservations.front() : it_target->Observations.front();

         // calculate the relative transform 
         argos::CVector3 cTargetRelativeTranslation = sObservation.Translation;
         cTargetRelativeTranslation -= sRootObservation.Translation;
         cTargetRelativeTranslation.Rotate(sRootObservation.Rotation.Inverse());

         double fBlockWidth = 0.055f;

         std::ostringstream cCoords;

         cCoords << std::round(cTargetRelativeTranslation.GetX() / fBlockWidth) << ", ";
         cCoords << std::round(cTargetRelativeTranslation.GetY() / fBlockWidth) << ", ";
         cCoords << std::round(cTargetRelativeTranslation.GetZ() / fBlockWidth);

         //cCoords.str(std::to_string(it_target->Id));
         //std::cout << "Target #" << itRootTarget->Id << " => Target #" << it_target->Id << ": [" << cCoords.str() << "]" << std::endl;

         cv::Scalar cColor;
         switch(mapTargetConnectivity.count(it_target)) {
            case 0:
               cColor = cv::Scalar(255,0,0);
               break;
            case 1:
               cColor = cv::Scalar(0,255,0);
               break;
            default:
               cColor = cv::Scalar(0,0,255);
               break;
         }
         std::ostringstream cText;
         cText << '[' << cCoords.str() << ']';
         cFrameAnnotator.Annotate(*it_target, cColor, cText.str());
      }
   }
}
*/

