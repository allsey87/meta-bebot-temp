
#include "block_tracker.h"
#include "hungarian_solver.h"

#include <argos3/core/utility/math/vector3.h>

#include <iostream>

void CBlockTracker::AssociateAndTrackTargets(std::chrono::time_point<std::chrono::steady_clock> t_timestamp,
                                             SBlock::TList& t_unassociated_block_list,
                                             STarget::TList& t_target_list) {

   /* write the timestamps into each entry of t_unassociated_block_list */
   for(SBlock& s_block : t_unassociated_block_list) {
      s_block.Timestamp = t_timestamp;
   }
   /* if there are existing targets, estimate their position based */
   if(t_target_list.size() > 0u) {
      /* estimate the velocity of the camera based (weighted towards more recent observations) */
      argos::CVector3 cCameraVelocityEstimate(0.0f, 0.0f, 0.0f);
      double fCameraVelocityEstimateWeight = 0.0f;
      /* calculate the current camera velocity */
      unsigned int unUsedTargetCount = 0u;
      for(STarget& s_target : t_target_list) {
         if(s_target.Observations.size() >= 2u) {
            auto itMostRecentObservation = std::begin(s_target.Observations);
            auto itSecondMostRecentObservation = std::next(itMostRecentObservation);
            double fDelta = std::chrono::duration<double>(t_timestamp - itSecondMostRecentObservation->Timestamp).count();
            double fWeight = 1.0f / fDelta;
            unUsedTargetCount++;
            fCameraVelocityEstimateWeight += fWeight;
            cCameraVelocityEstimate +=
               ((itMostRecentObservation->Translation - itSecondMostRecentObservation->Translation) / fDelta) * fWeight;
         }
      }
      /* update the average camera velocity, if the weight is zero, we don't have enough readings to determine the camera velocity */
      cCameraVelocityEstimate = (fCameraVelocityEstimateWeight == 0.0f) ? 
         argos::CVector3::ZERO : (cCameraVelocityEstimate / fCameraVelocityEstimateWeight);

      /* calculate the camera velocity estimate average weight - indicates the quality (recentness) of the estimate */
      double fCameraVelocityEstimateAverageWeight = (unUsedTargetCount == 0) ?
         0.0f : (fCameraVelocityEstimateWeight / static_cast<double>(unUsedTargetCount));

      /* estimate velocities of existing targets, storing the result as a pseudo observation */
      for(STarget& s_target : t_target_list) {
         /* initialise the estimates to the camera velocity */
         argos::CVector3 cTargetVelocityEstimate = cCameraVelocityEstimate * fCameraVelocityEstimateAverageWeight;
         auto itMostRecentObservation = std::begin(s_target.Observations);
         double fTargetVelocityWeight = 0.0f;
         /* if there are two or more observations, compute an average velocity of the target */
         if(s_target.Observations.size() >= 2u) {
            auto itSecondMostRecentObservation = std::next(itMostRecentObservation);
            double fDelta = std::chrono::duration<double>(itMostRecentObservation->Timestamp - itSecondMostRecentObservation->Timestamp).count();
            /* calculate the weight from delta */
            fTargetVelocityWeight = 1.0f / fDelta;
            cTargetVelocityEstimate += 
               ((itMostRecentObservation->Translation - itSecondMostRecentObservation->Translation) / fDelta) * fTargetVelocityWeight;
         }
         /* calculate the target velocity estimate */
         double fTotalWeight = fCameraVelocityEstimateAverageWeight + fTargetVelocityWeight;

         cTargetVelocityEstimate = (fTotalWeight == 0.0f) ? argos::CVector3::ZERO : (cTargetVelocityEstimate / fTotalWeight);
         /* Calculate the expected position of the target */
         double fElapsedTime = std::chrono::duration<double>(t_timestamp - itMostRecentObservation->Timestamp).count();
         argos::CVector3 cTargetPseudoPosition = itMostRecentObservation->Translation + cTargetVelocityEstimate * fElapsedTime;
         /* Add a pseudo observation using the default constructor */
         s_target.PseudoObservations.emplace_front(t_timestamp, cTargetPseudoPosition, itMostRecentObservation->Rotation);
      }
   }

   /* list of blocks that don't correspond to any target */
   SBlock::TList tUnmatchedBlockList;
   /* list of targets that don't correspond to any block */
   STarget::TList tUnmatchedTargetList;
   /* new targets */
   STarget::TList tNewTargetList;
   /* build a cost matrix */
   CHungarianSolver::TCostMatrix tCostMatrix(t_unassociated_block_list.size(), std::vector<double>(t_target_list.size(), 0.0f));
   unsigned int n_target_idx = 0u;
   auto it_target = std::begin(t_target_list);
   for(; it_target != std::end(t_target_list); n_target_idx++, it_target++) {
      unsigned int n_block_idx = 0u;
      auto it_block = std::begin(t_unassociated_block_list);
      for(; it_block != std::end(t_unassociated_block_list); n_block_idx++, it_block++) {
         // Cost matrix is blocks x targets
         const SBlock& sTrackedBlock = it_target->PseudoObservations.front();

         tCostMatrix[n_block_idx][n_target_idx] = argos::Distance(sTrackedBlock.Translation, it_block->Translation);
      }
   }
   /* remove rows from the cost matrix that exceed the distance threshold */
   std::list<unsigned int> lstRowsIndicesToRemove;
   for(auto it_row = std::begin(tCostMatrix); it_row != std::end(tCostMatrix); it_row++) {
      bool bRowExceedsThreshold = true;
      for(auto it_el = std::begin(*it_row); it_el != std::end(*it_row); it_el++) {
         bRowExceedsThreshold = bRowExceedsThreshold && (*it_el > m_fDistanceThreshold);
      }
      if(bRowExceedsThreshold) {
         /* add this row to the list for removal */
         lstRowsIndicesToRemove.push_front(std::distance(std::begin(tCostMatrix), it_row));
      }
   }
   /* remove rows / move unmatched blocks to unmatched list  */
   for(unsigned int un_idx : lstRowsIndicesToRemove) {
      auto it_erase = std::begin(tCostMatrix);
      std::advance(it_erase, un_idx);
      tCostMatrix.erase(it_erase);
      /* move the unassociated block into the unmatched list */
      auto it_move = std::begin(t_unassociated_block_list);
      std::advance(it_move, un_idx);
      tUnmatchedBlockList.splice(std::begin(tUnmatchedBlockList), t_unassociated_block_list, it_move);
   }
   /* remove columns from the cost matrix that exceed the distance threshold */
   /* check if matrix has at least 1 row */
   if(tCostMatrix.size() > 0) {
      unsigned int unColumnCount = tCostMatrix[0].size();
      std::list<unsigned int> lstColumnIndicesToRemove;
      for(unsigned int un_col = 0; un_col < unColumnCount; un_col++) {
         bool bColExceedsThreshold = true;
         for(auto it_row = std::begin(tCostMatrix); it_row != std::end(tCostMatrix); it_row++) {
            auto it_el = std::begin(*it_row);     
            std::advance(it_el, un_col);
            bColExceedsThreshold = bColExceedsThreshold && (*it_el > m_fDistanceThreshold);
         }
         if(bColExceedsThreshold) {
            lstColumnIndicesToRemove.push_front(un_col);
         }
      }
      for(auto un_idx : lstColumnIndicesToRemove) {
         for(auto it_row = std::begin(tCostMatrix); it_row != std::end(tCostMatrix); it_row++) {
            auto it_col_el = std::begin(*it_row);
            std::advance(it_col_el, un_idx);
            it_row->erase(it_col_el);
         }
         /* move unassociated target into the unmatched list */
         auto it_move = std::begin(t_target_list);
         std::advance(it_move, un_idx);
         tUnmatchedTargetList.splice(std::begin(tUnmatchedTargetList), t_target_list, it_move);
      }
   }      
   /* adjust individual elements that exceed m_fDistanceThreshold */
   for(auto it_row = std::begin(tCostMatrix); it_row != std::end(tCostMatrix); it_row++) {
      for(auto it_el = std::begin(*it_row); it_el != std::end(*it_row); it_el++) {
         *it_el = (*it_el < m_fDistanceThreshold) ? *it_el : std::max(t_target_list.size(), t_unassociated_block_list.size()) * m_fDistanceThreshold;
      }
   }
   /* zero pad the cost matrix until it is square */
   unsigned int unRowCount = tCostMatrix.size();
   unsigned int unColCount = (unRowCount > 0) ? tCostMatrix[0].size() : 0;
   if(unRowCount > unColCount) {
      /* add columns */
      for(auto it_row = std::begin(tCostMatrix); it_row != std::end(tCostMatrix); it_row++) {
         it_row->insert(std::end(*it_row), unRowCount - unColCount, 0.0f);
      }
   }
   if(unColCount > unRowCount) {
      /* add rows */
      tCostMatrix.insert(std::end(tCostMatrix), unColCount - unRowCount, std::vector<double>(unColCount, 0.0f));
   }
   /* negate all elements such that we solve for the minimum */
   for(auto it_row = std::begin(tCostMatrix); it_row != std::end(tCostMatrix); it_row++) {
      for(auto it_el = std::begin(*it_row); it_el != std::end(*it_row); it_el++) {
         *it_el = -(*it_el);
      }
   }
   /* create instance of the solver */
   CHungarianSolver CHungarianSolver;
   /* run the solver on the cost matrix */   
   CHungarianSolver(tCostMatrix);
   /* get the indices of the blocks and targets to be assigned */
   std::vector<std::pair<int, int> > vecAssignmentIndices = CHungarianSolver.GetAssignments();
   /* schedule all target to block assignments */
   std::list<std::pair<SBlock::TListIterator, STarget::TListIterator> > lstScheduledAssignments;
   for(const auto& t_assignment : vecAssignmentIndices) {
      bool bBlockExists = (t_assignment.first < t_unassociated_block_list.size());
      bool bTargetExists = (t_assignment.second < t_target_list.size());
      if(bBlockExists) {
         auto itBlock = std::begin(t_unassociated_block_list);
         std::advance(itBlock, t_assignment.first);
         if(bTargetExists) {
            auto itTarget = std::begin(t_target_list);
            std::advance(itTarget, t_assignment.second);
            lstScheduledAssignments.push_front(std::make_pair(itBlock, itTarget));
         }
         else  {
            /* create a new target */
            tNewTargetList.emplace_front();
            /* schedule the assignment */
            lstScheduledAssignments.push_front(std::make_pair(itBlock, std::begin(tNewTargetList)));
         }
      }
   }
   /* do the assignments */
   for(auto& c_scheduled_assignment : lstScheduledAssignments) {
      SBlock::TList& tObservationList = c_scheduled_assignment.second->Observations;
      tObservationList.splice(std::begin(tObservationList), t_unassociated_block_list, c_scheduled_assignment.first);
      /* clear pseudo observations */
      c_scheduled_assignment.second->PseudoObservations.clear();
      /* limit the number of observations to m_unTrackingDepth */
      while(c_scheduled_assignment.second->Observations.size() > m_unTrackingDepth) {
         c_scheduled_assignment.second->Observations.pop_back();
      }
   }
   /* create targets for unmatched blocks and assign them */
   while(!tUnmatchedBlockList.empty()) {
      auto itUnmatchedBlock = std::begin(tUnmatchedBlockList);
      tNewTargetList.emplace_front();
      SBlock::TList& tObservationList = std::begin(tNewTargetList)->Observations;
      tObservationList.splice(std::begin(tObservationList), tUnmatchedBlockList, itUnmatchedBlock);  
   }
   /* move all targets back to main list */
   t_target_list.splice(std::begin(t_target_list), tNewTargetList);
   t_target_list.splice(std::begin(t_target_list), tUnmatchedTargetList);  
   /* clear targets with pseudo count higher than tracking threshold */
   t_target_list.remove_if([this] (const STarget& s_target) {
      return (s_target.PseudoObservations.size() > m_unTrackingDepth);
   });
   /* assign indentifiers */
   AssignIdentifiers(t_target_list);
}

void CBlockTracker::AssignIdentifiers(STarget::TList& t_target_list) {
   /* create a list of the used target indentifiers */
   std::list<unsigned int> lstUsedIds;
   /* populate that list */
   for(STarget& s_target : t_target_list) {
      if(s_target.Id != 0) {
         lstUsedIds.push_back(s_target.Id);
      }
   }
   /* assign identifiers */
   for(STarget& s_target : t_target_list) {
      if(s_target.Id == 0) {
         while((m_unNextId == 0) || std::find(std::begin(lstUsedIds), std::end(lstUsedIds), m_unNextId) != std::end(lstUsedIds)) {
            m_unNextId++;
         }
         s_target.Id = m_unNextId;
         lstUsedIds.push_back(m_unNextId);
         /* increment the Id after the assignment */
         m_unNextId++;
      }
   }
}
