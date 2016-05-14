#ifndef BLOCK_TRACKER_H
#define BLOCK_TRACKER_H

#include <list>
#include <algorithm>

#include "block.h"
#include "target.h"

class CBlockTracker {

public:
   CBlockTracker(unsigned int un_tracking_depth,
                 double f_distance_threshold) :
      m_unTrackingDepth(un_tracking_depth),
      m_fDistanceThreshold(f_distance_threshold) {}

   void AssociateAndTrackTargets(std::chrono::time_point<std::chrono::steady_clock> t_timestamp,
                                 SBlock::TList& t_unassociated_block_list,
                                 STarget::TList& t_target_list);
private:
   void AssignIdentifiers(STarget::TList& t_target_list);

   unsigned int m_unTrackingDepth;
   double m_fDistanceThreshold;
   unsigned int m_unNextId = 0;
};

#endif
