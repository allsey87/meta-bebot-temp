
#include "block_tracker.h"

void CBlockTracker::AssociateAndTrackTargets(const std::list<CBlockSensor::SBlock>& lst_detected_blocks,
                                             std::list<STarget>& lst_targets) {

   /* TODO / TOCHECK */
   /*
     1. check the case that multiple targets could be assigned the
     same block, how should this be handled?

     2. implement block disappearance distance based on distance from
     frame

     3. check that the variables for closest distance etc are
     appropiately set when being moved back into the unassociated list

     4. capture sample video from camera as series of images and build
     test cases for calibrating and testing the algorithm

     5. note, when doing velocity based matching, it is possible for
     block positions to be predicted outside of the frame, probably
     breaking the CalculateMinimumDistanceToFrame code

    */
   
   
   /* Create a set of mappings by sequentially assigning pairs based on the best
      match (smallest distance between a block in frame n - 1 and frame n) */
   /* Select a random permutation */
   /* If there is more two existing observations, consider doing a projectile
      based match */

   std::list<SAssociatedTarget> lstUnassociatedTargets, lstAssociatedTargets;

   /* create a list of unassociated targets to be matched against the detected blocks */
   for(const STarget& s_target : lst_targets) {
      lstUnassociatedTargets.emplace_back(&s_target);
   }

   while(!lstUnassociatedTargets.empty()) {
      for(std::list<CBlockSensor::SBlock>::const_iterator itDetectedBlock = std::begin(lst_detected_blocks);
          itDetectedBlock != std::end(lst_detected_blocks);
          itDetectedBlock++) {

         /* randomize the order of unassociated targets */
         //std::random_shuffle(std::begin(lstUnassociatedTargets), std::end(lstUnassociatedTargets));

         /* warning: begin is defined, element is not for empty list */
         std::list<SAssociatedTarget>::iterator itClosestTrackedTarget = std::end(lstUnassociatedTargets);
         float fClosestTrackedTargetDist = 0;
         
         for(std::list<SAssociatedTarget>::iterator itTrackedTarget = std::begin(lstUnassociatedTargets);
             itTrackedTarget != std::end(lstUnassociatedTargets);
             itTrackedTarget++) {

            /* TODO: handle case where Observations[0] does not exist */
            /* where there are two observations, use velocity matching approach */
            const CBlockSensor::SBlock& sTrackedBlock = itTrackedTarget->Target->Observations.front();
            /* calculate the distance between this block and the tracked block */
            float fInterblockDist = sqrt(pow(sTrackedBlock.X - itDetectedBlock->X, 2) +
                                         pow(sTrackedBlock.Y - itDetectedBlock->Y, 2) +
                                         pow(sTrackedBlock.Z - itDetectedBlock->Z, 2));
            if(itClosestTrackedTarget == std::end(lstUnassociatedTargets) ||
               fInterblockDist < fClosestTrackedTargetDist) {
               itClosestTrackedTarget = itTrackedTarget;
               fClosestTrackedTargetDist = fInterblockDist;
            }
         }
         /* consider the case of a new block appearing */
         float fMinDistToFrame = CalculateMinimumDistanceToFrame(itDetectedBlock->Coordinates);
         float fNewTrackedTargetDist = m_fPixelToDistanceCoefficient / (fMinDistToFrame + 1);

         if(fNewTrackedTargetDist < fClosestTrackedTargetDist) {
            //lst_targets.emplace_back(*itDetectedBlock);
            //lstAssociatedTargets.emplace_back(&lst_targets.back(), itDetectedBlock, fNewTrackedTargetDist);
         }
         else {
            /* Move the closest match from the unassociated list to the associated list */
            //lstAssociatedTargets.splice(std::begin(lstAssociatedTargets),
            //                            lstUnassociatedTargets,
            //                            itClosestTrackedTarget);
         }
      } // end loop over each detected block
      
      /* For each target that doesn't have a matching block, we assume we have temporarily 
         lost tracking and add the effective distance based on the position in the frame*/
      for(SAssociatedTarget& s_unassociated_target : lstUnassociatedTargets) {
         float fMinDistToFrame = CalculateMinimumDistanceToFrame(s_unassociated_target.Target->Observations.front().Coordinates);
         s_unassociated_target.CandidateAssociationDist = m_fPixelToDistanceCoefficient /
            (fMinDistToFrame + 1);
      }

      /* move the unassociated targets that we assume we have lost to the associated list */
      lstAssociatedTargets.splice(std::begin(lstAssociatedTargets),
                                  lstUnassociatedTargets);

      /* sort the list of associated targets by distance */
      lstAssociatedTargets.sort([] (const SAssociatedTarget& s_associated_target_first,
                                    const SAssociatedTarget& s_associated_target_second) {
                                   return (s_associated_target_first.CandidateAssociationDist >
                                           s_associated_target_second.CandidateAssociationDist);
                                });

      /* Move the X worst associations back into the unassociated targets list and continue */
      std::list<SAssociatedTarget>::iterator itReassociationRangeEnd = std::begin(lstAssociatedTargets);
      std::advance(itReassociationRangeEnd, lstAssociatedTargets.size() * m_fAssociationRecursionRatio);
      
      lstUnassociatedTargets.splice(std::begin(lstUnassociatedTargets),
                                    lstAssociatedTargets,
                                    std::begin(lstAssociatedTargets),
                                    itReassociationRangeEnd);
   }
   /* clean out targets that haven't had an observation for X frames */
   /* check all targets have an unique id in the list */
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
