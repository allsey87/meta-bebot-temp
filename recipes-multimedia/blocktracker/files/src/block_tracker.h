#ifndef BLOCK_TRACKER_H
#define BLOCK_TRACKER_H

#include <list>
#include <algorithm>

#include "block.h"
#include "target.h"

class CBlockTracker {

public:
   CBlockTracker(unsigned int un_frame_width,
                 unsigned int un_frame_height,
                 unsigned int un_tracking_depth,
                 unsigned int un_lost_target_threshold,
                 float f_association_recursion_ratio,
                 float f_pixel_to_distance_coefficient) :

      m_unFrameWidth(un_frame_width),
      m_unFrameHeight(un_frame_height),
      m_unTrackingDepth(un_tracking_depth),
      m_unLostTargetThreshold(un_lost_target_threshold),
      m_fAssociationRecursionRatio(f_association_recursion_ratio),
      m_fPixelToDistanceCoefficient(f_pixel_to_distance_coefficient) {}

   void AssociateAndTrackTargets(std::list<SBlock>& lst_unassociated_blocks,
                                 std::list<STarget>& lst_targets);
private:

   struct SAssociation {
      SAssociation(float f_association_distance) :
         AssociationDist(f_association_distance) {}
      
      std::list<STarget> ExistingTarget;
      std::list<SBlock> CandidateBlock;
      float AssociationDist;
   };

   void AssignIdentifiers(std::list<STarget>& lst_targets);

   float CalculateMinimumDistanceToFrame(const std::pair<float, float>& c_coordinates);

   unsigned int m_unFrameWidth;
   unsigned int m_unFrameHeight;
   unsigned int m_unTrackingDepth;
   unsigned int m_unLostTargetThreshold;
   float m_fAssociationRecursionRatio;
   float m_fPixelToDistanceCoefficient;

};

#endif
