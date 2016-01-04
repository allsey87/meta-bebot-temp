
#include "block_tracker.h"

#include <iostream>

void CBlockTracker::AssociateAndTrackTargets(std::list<SBlock>& lst_unassociated_blocks,
                                             std::list<STarget>& lst_targets) {
   /* list for keeping track of associations between the blocks and the existing targets */
   std::list<SAssociation> lstAssociations, lstFinalAssociations;
   /* while we still have unassociated blocks or targets */
   while(!lst_targets.empty() || !lst_unassociated_blocks.empty()) {
      /* for each detected block */
      while(!lst_unassociated_blocks.empty()) {
         /* Take the first block in the list */
         std::list<SBlock>::iterator itUnassociatedBlock = std::begin(lst_unassociated_blocks);
         /* compare the block to each target, keeping track of the target with the
            minimum distance between itself and the block */
         std::list<STarget>::iterator itClosestTrackedTarget = std::end(lst_targets);
         float fClosestTrackedTargetDist = 0;
         /* select the closest target to the block */
         for(std::list<STarget>::iterator itTrackedTarget = std::begin(lst_targets);
             itTrackedTarget != std::end(lst_targets);
             itTrackedTarget++) {
            /* TODO: where there are two observations, use velocity matching approach */
            std::list<SBlock>::iterator itTrackedBlock = std::begin(itTrackedTarget->Observations);
            /* calculate the distance between this block and the tracked block */
            float fInterblockDist =
               sqrt(pow(itTrackedBlock->X - itUnassociatedBlock->X, 2) +
                    pow(itTrackedBlock->Y - itUnassociatedBlock->Y, 2) +
                    pow(itTrackedBlock->Z - itUnassociatedBlock->Z, 2));
            if(itClosestTrackedTarget == std::end(lst_targets) ||
               fInterblockDist < fClosestTrackedTargetDist) {
               itClosestTrackedTarget = itTrackedTarget;
               fClosestTrackedTargetDist = fInterblockDist;
            }
         }
         /* consider the case of a new block appearing (i.e. a new target) */
         float fMinDistToFrame = CalculateMinimumDistanceToFrame(itUnassociatedBlock->Coordinates);
         float fNewTrackedTargetDist = m_fPixelToDistanceCoefficient / (fMinDistToFrame + 1);
         if(itClosestTrackedTarget == std::end(lst_targets) ||
            fNewTrackedTargetDist < fClosestTrackedTargetDist) {
            /* create a new association with */
            lstAssociations.emplace_back(fNewTrackedTargetDist);
            /* reference the list for the candidate block */
            std::list<SBlock>& lstCandidateBlock = lstAssociations.back().CandidateBlock;
            /* move the block into this list */
            lstCandidateBlock.splice(std::begin(lstCandidateBlock),
                                     lst_unassociated_blocks,
                                     itUnassociatedBlock);
         }
         else {
            /* set the association distance for the closest tracked target */
            lstAssociations.emplace_back(fClosestTrackedTargetDist);
            /* reference the list for the candidate block and existing target */
            std::list<STarget>& lstExistingTarget = lstAssociations.back().ExistingTarget;
            std::list<SBlock>& lstCandidateBlock = lstAssociations.back().CandidateBlock;
            /* move the existing target and block into the association */
            lstExistingTarget.splice(std::begin(lstExistingTarget),
                                     lst_targets,
                                     itClosestTrackedTarget);
            lstCandidateBlock.splice(std::begin(lstCandidateBlock),
                                     lst_unassociated_blocks,
                                     itUnassociatedBlock);
         }
      } /* while(!lst_unassociated_blocks.empty()) */
      /* For each target that doesn't have an associated block */
      while(!lst_targets.empty()) {
         std::list<STarget>::iterator itLostTarget = std::begin(lst_targets);
         /* compute an effective tracking distance based on the distance to the frame */
         float fMinDistToFrame =
            CalculateMinimumDistanceToFrame(std::begin(itLostTarget->Observations)->Coordinates);
         lstAssociations.emplace_back(m_fPixelToDistanceCoefficient / (fMinDistToFrame + 1));
         /* reference the list for the existing target */
         std::list<STarget>& lstExistingTarget = lstAssociations.back().ExistingTarget;
         /* move the existing target into the association */
         lstExistingTarget.splice(std::begin(lstExistingTarget),
                                  lst_targets,
                                  itLostTarget);
      }
      /* sort the list of associations by distance */
      lstAssociations.sort([] (const SAssociation& s_association_first,
                               const SAssociation& s_association_second) {
                              return (s_association_first.AssociationDist <
                                      s_association_second.AssociationDist);
                           });
      /* determine the range of the X best associations */
      std::list<SAssociation>::iterator itFinalAssociationRangeEnd = std::begin(lstAssociations);
      std::advance(itFinalAssociationRangeEnd,
                   std::ceil(lstAssociations.size() * (1 - m_fAssociationRecursionRatio)));
      /* move the X best associations into the final associations list */
      lstFinalAssociations.splice(std::begin(lstFinalAssociations),
                                  lstAssociations,
                                  std::begin(lstAssociations),
                                  itFinalAssociationRangeEnd);
      /* undo the remaining associations */
      for(SAssociation& s_association : lstAssociations) {
         if(!s_association.ExistingTarget.empty()) {
            lst_targets.splice(lst_targets.begin(), s_association.ExistingTarget);
         }
         if(!s_association.CandidateBlock.empty()) {
            lst_unassociated_blocks.splice(lst_unassociated_blocks.begin(), s_association.CandidateBlock);
         }
      }
      lstAssociations.clear();
   } /* while(!lst_targets.empty() || !lst_unassociated_blocks.empty()) */
   /* move the final associations back into the target list */
   for(SAssociation& s_association : lstFinalAssociations) {
      /* if association doesn't have a target, create one */
      if(s_association.ExistingTarget.empty()) {
         s_association.ExistingTarget.emplace_back();
      }
      /* if a candidate block exists, move it into the target observations list */
      if(!s_association.CandidateBlock.empty()) {
         std::begin(s_association.ExistingTarget)->FramesSinceLastObservation = 0;
         std::list<SBlock>& lstObservations = std::begin(s_association.ExistingTarget)->Observations;
         lstObservations.splice(std::begin(lstObservations), s_association.CandidateBlock);
         /* limit the number of observations to m_unTrackingDepth */
         while(lstObservations.size() > m_unTrackingDepth) {
            lstObservations.pop_back();
         }
      }
      else {
         std::begin(s_association.ExistingTarget)->FramesSinceLastObservation++;   
      }
      /* if we haven't exceeded m_unLostTargetThreshold, move the target back into the targets list */
      if(std::begin(s_association.ExistingTarget)->FramesSinceLastObservation < m_unLostTargetThreshold) {
         lst_targets.splice(std::begin(lst_targets), s_association.ExistingTarget);
      }
   }
   AssignIdentifiers(lst_targets);
}

void CBlockTracker::AssignIdentifiers(std::list<STarget>& lst_targets) {
   /* create a list of the used target indentifiers */
   std::list<unsigned int> lstUsedIds;
   unsigned int unNextId = 0;
   /* populate that list */
   for(STarget& s_target : lst_targets) {
      if(s_target.Id != -1) {
         lstUsedIds.push_back(s_target.Id);
      }
   }
   /* assign identifiers */
   for(STarget& s_target : lst_targets) {
      if(s_target.Id == -1) {
         while(std::find(std::begin(lstUsedIds), std::end(lstUsedIds), unNextId) != std::end(lstUsedIds)) {
            unNextId++;
         }
         s_target.Id = unNextId;
         lstUsedIds.push_back(unNextId);
      }
   }
}


float CBlockTracker::CalculateMinimumDistanceToFrame(const std::pair<float, float>& c_coordinates) {
   /* init fMinDistToFrame as float max */
   float fMinDistToFrame = std::numeric_limits<float>::max();
   float fDistToFrames[] = {
      c_coordinates.second,
      c_coordinates.first,
      m_unFrameHeight - c_coordinates.second,
      m_unFrameWidth - c_coordinates.first
   };
   /* calculate the smallest distance from frame */
   for(float fDist : fDistToFrames) {
      /* saturate at zero, c_coordinates could be outside of the frame for velocity based matching */
      if(fDist < 0) {
         fMinDistToFrame = 0;
         break;
      }
      else if(fDist < fMinDistToFrame) {
         fMinDistToFrame = fDist;
      }
   }
   return fMinDistToFrame;
}
