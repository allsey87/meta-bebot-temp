#ifndef BLOCK_TRACKER_H
#define BLOCK_TRACKER_H

#include "block_sensor.h"

#include <algorithm>

/* Parameters

   1. How many observations to hold
   2. Frame bounds
   3. Coefficient converting pixels to distance
   4. Resampling ratio
   5. Time to live

*/

class CBlockTracker {

public:
   CBlockTracker(unsigned int un_frame_height,
                 unsigned int un_frame_width,
                 unsigned int un_tracking_depth,
                 float f_association_recursion_ratio,
                 float f_pixel_to_distance_coefficient) :
      m_unFrameHeight(un_frame_height),
      m_unFrameWidth(un_frame_width),
      m_unTrackingDepth(un_tracking_depth),
      m_fAssociationRecursionRatio(f_association_recursion_ratio),
      m_fPixelToDistanceCoefficient(f_pixel_to_distance_coefficient) {}

   struct STarget {
      STarget(const CBlockSensor::SBlock& s_block) :
         FramesSinceLastObservation(0) {
         Observations.emplace_back(s_block);
      }
      
      std::list<CBlockSensor::SBlock> Observations;
      unsigned int FramesSinceLastObservation;
   };

   void AssociateAndTrackTargets(const std::list<CBlockSensor::SBlock>& lst_detected_blocks,
                                 std::list<STarget>& lst_targets);
private:

   struct SAssociatedTarget {
      SAssociatedTarget(const STarget* ps_target) :
         Target(ps_target) {}
      
      const STarget* Target;
      std::list<CBlockSensor::SBlock>::const_iterator CandidateAssociation;
      float CandidateAssociationDist;
   };

   float CalculateMinimumDistanceToFrame(const std::pair<float, float>& c_coordinates);


   unsigned int m_unFrameHeight;
   unsigned int m_unFrameWidth;
   unsigned int m_unTrackingDepth;
   float m_fAssociationRecursionRatio;
   float m_fPixelToDistanceCoefficient;

};

   

#endif
