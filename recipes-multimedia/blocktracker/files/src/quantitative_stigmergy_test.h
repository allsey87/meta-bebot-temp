#ifndef MANIPULATOR_TESTING_TASK_H
#define MANIPULATOR_TESTING_TASK_H

#include "state.h"
#include "block_demo.h"

#include <iostream>

#define MTT_LIFT_ACTUATOR_MAX_HEIGHT 140
#define MTT_LIFT_ACTUATOR_MIN_HEIGHT 3
#define MTT_LIFT_ACTUATOR_INC_HEIGHT 15

#define MTT_LIFT_ACTUATOR_ERR_THRES 0.375f


#define PREAPPROACH_BLOCK_Z_TARGET 0.325f

#define TARGET_Z_DIST_REVERSE_TO 0.25f

#define MTT_BLOCK_SEP_THRESHOLD 0.065f

#define BASE_VELOCITY 20
#define MIN_SPEED_FRAC 0.05f

#define IMAGE_SENSOR_WIDTH 640.0f
#define IMAGE_SENSOR_HEIGHT 360.0f

#define TAG_CENTERED_LOWER_THRES (0.40f * IMAGE_SENSOR_WIDTH)
#define TAG_CENTERED_UPPER_THRES (0.60f * IMAGE_SENSOR_WIDTH)

struct {
  CBlockDemo::SSensorData* Sensors = nullptr;
  CBlockDemo::SActuatorData* Actuators = nullptr;
  unsigned int TrackedTargetId = 0;
  unsigned int TrackedStructureId = 0;
} Data;

/* Common functions for all states */
STarget::TConstListIterator FindTargetFurthestToTheLeft(const STarget::TList& s_target_list) {
   STarget::TConstListIterator itTargetFurthestToTheLeft = std::begin(s_target_list);
   for(STarget::TConstListIterator it_target = std::begin(s_target_list);
      it_target != std::end(s_target_list);
      it_target++) {
      if(it_target->Observations.front().Translation.GetX() < 
         itTargetFurthestToTheLeft->Observations.front().Translation.GetX()) {
         itTargetFurthestToTheLeft = it_target;
      }
   }
   return itTargetFurthestToTheLeft;
}

STarget::TConstListIterator FindTargetFurthestToTheRight(const STarget::TList& s_target_list) {
   STarget::TConstListIterator itTargetFurthestToTheRight = std::begin(s_target_list);
   for(STarget::TConstListIterator it_target = std::begin(s_target_list);
      it_target != std::end(s_target_list);
      it_target++) {
      if(it_target->Observations.front().Translation.GetX() > 
         itTargetFurthestToTheRight->Observations.front().Translation.GetX()) {
         itTargetFurthestToTheRight = it_target;
      }
   }
   return itTargetFurthestToTheRight;
}

STarget::TConstListIterator FindTrackedTarget(unsigned int un_target_id, const STarget::TList& s_target_list) {
   return std::find_if(std::begin(s_target_list),
                       std::end(s_target_list),
                       [un_target_id] (const STarget& s_target) {
                          return (s_target.Id == un_target_id);
                       });
}

double TrackBlockViaLiftActuatorHeight(const SBlock& s_block, 
                                       uint8_t un_min_position = MTT_LIFT_ACTUATOR_MIN_HEIGHT,
                                       uint8_t un_max_position = MTT_LIFT_ACTUATOR_MAX_HEIGHT) {
   double fEndEffectorPos = Data.Sensors->ManipulatorModule.LiftActuator.EndEffector.Position;                
   double fLiftActuatorHeightError = (180.0f - s_block.Tags.front().Center.second) / 180.0f;
   fEndEffectorPos += (fLiftActuatorHeightError * MTT_LIFT_ACTUATOR_INC_HEIGHT);
   uint8_t unEndEffectorPos = 
      (fEndEffectorPos > un_max_position) ? un_max_position : 
      (fEndEffectorPos < un_min_position) ? un_min_position : static_cast<uint8_t>(fEndEffectorPos); 
   Data.Actuators->ManipulatorModule.LiftActuator.Position.Value = unEndEffectorPos;
   Data.Actuators->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
   return fLiftActuatorHeightError;
}

/* Common state definitions */
class CStateTurnTowardsTarget : public CState {
public:
   CStateTurnTowardsTarget() :
      CState("turn_towards_target", [this] {
         /* default velocities, overwritten if target is detected */
         double fLeft = 0.0f, fRight = 0.0f, fLiftActuatorHeightError = 0.0f;
         /* select tracked target */
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            /* track the target using lift actuator position */          
            fLiftActuatorHeightError = TrackBlockViaLiftActuatorHeight(s_block, MTT_LIFT_ACTUATOR_MIN_HEIGHT + 2u);
            /* calculate the approach velocity */
            double fTagXOffset = (s_block.Tags.front().Center.first - 320.0f) / 320.0f;
            if(fTagXOffset < 0) {
               fLeft = -BASE_VELOCITY * (std::abs(fTagXOffset) + MIN_SPEED_FRAC);
               fRight = BASE_VELOCITY * (std::abs(fTagXOffset) + MIN_SPEED_FRAC);
            }
            else {
               fLeft = BASE_VELOCITY * (std::abs(fTagXOffset) + MIN_SPEED_FRAC);
               fRight = -BASE_VELOCITY * (std::abs(fTagXOffset) + MIN_SPEED_FRAC);
            }
         }
         /* scale the speed by the lift actuator height error */
         //std::cerr << "|fLiftActuatorHeightError| = " << std::abs(fLiftActuatorHeightError) << std::endl;       
         fLeft *= (1.0f - std::abs(fLiftActuatorHeightError));
         fRight *= (1.0f - std::abs(fLiftActuatorHeightError));
         /* apply the approach velocity */
         Data.Actuators->DifferentialDriveSystem.Left.Velocity = std::round(fLeft);
         Data.Actuators->DifferentialDriveSystem.Right.Velocity = std::round(fRight);
         Data.Actuators->DifferentialDriveSystem.Left.UpdateReq = true;
         Data.Actuators->DifferentialDriveSystem.Right.UpdateReq = true;
      }) {}
};

class CStateApproachTarget : public CState {
public:
   CStateApproachTarget(const std::string str_id, double f_offset_target) :
      CState(str_id, [this, f_offset_target] {
         /* default velocities, overwritten if target is detected */
         double fLeft = 0.0f, fRight = 0.0f, fLiftActuatorHeightError = 0.0f;
         /* select tracked target */
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);                         
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            /* track the target using lift actuator position */
            fLiftActuatorHeightError = TrackBlockViaLiftActuatorHeight(s_block, MTT_LIFT_ACTUATOR_MIN_HEIGHT + 2u);
            /* calculate the approach velocity */
            double fTagXOffset = (s_block.Tags.front().Center.first - 320.0f) / 320.0f;
            double fTagXOffsetTarget = f_offset_target * std::abs((s_block.Translation.GetZ() - 0.1f) / 0.25f);
            fRight = (1 + fTagXOffsetTarget - fTagXOffset) * BASE_VELOCITY;
            fLeft  = (1 + fTagXOffset - fTagXOffsetTarget) * BASE_VELOCITY;
         }
         /* scale the speed by the lift actuator height error */
         //std::cerr << "|fLiftActuatorHeightError| = " << std::abs(fLiftActuatorHeightError) << std::endl;
         fLeft *= (1.0f - std::abs(fLiftActuatorHeightError));
         fRight *= (1.0f - std::abs(fLiftActuatorHeightError));
         /* apply the approach velocity */
         Data.Actuators->DifferentialDriveSystem.Left.Velocity = std::round(fLeft);
         Data.Actuators->DifferentialDriveSystem.Right.Velocity = std::round(fRight);
         Data.Actuators->DifferentialDriveSystem.Left.UpdateReq = true;
         Data.Actuators->DifferentialDriveSystem.Right.UpdateReq = true;
      }) {}
};

class CStateSetLiftActuatorPosition : public CState {
public: 
   CStateSetLiftActuatorPosition(const std::string str_id, uint8_t un_position) :
      CState(str_id, nullptr, nullptr, {
         CState("set_position", [un_position] {
            Data.Actuators->ManipulatorModule.LiftActuator.Position.Value = un_position;
            Data.Actuators->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
         }),
         CState("wait_for_lift_actuator"),
      }) {
      AddTransition("set_position","wait_for_lift_actuator");
      AddExitTransition("wait_for_lift_actuator", [] {
         return (Data.Sensors->ManipulatorModule.LiftActuator.State == 
                 CBlockDemo::ELiftActuatorSystemState::INACTIVE);
      });
   }
};

/* Main state machine definition */
class CManipulatorTestingTask : public CState {
   
public:
   CManipulatorTestingTask() :
      CState("top_level_state", nullptr, nullptr, {
         CStateSetLiftActuatorPosition("raise_lift_actuator", MTT_LIFT_ACTUATOR_MAX_HEIGHT - 5u),
         CState("search_for_unused_block", nullptr, nullptr, {
            CState("set_search_velocity", [this] {
               Data.Actuators->DifferentialDriveSystem.Left.Velocity = BASE_VELOCITY;
               Data.Actuators->DifferentialDriveSystem.Right.Velocity = -BASE_VELOCITY;
               Data.Actuators->DifferentialDriveSystem.Right.UpdateReq = true;
               Data.Actuators->DifferentialDriveSystem.Left.UpdateReq = true;
               for(CBlockDemo::EColor& e_color : Data.Actuators->LEDDeck.Color) 
                  e_color = CBlockDemo::EColor::RED;
               for(bool& b_update : Data.Actuators->LEDDeck.UpdateReq)
                  b_update = true;
            }),
            CState("wait_for_next_target"),
            CStateTurnTowardsTarget(),
            CState("reverse_until_distance", [this] {
               /* default velocities, overwritten if target is detected */
               double fLeft = 0.0f, fRight = 0.0f, fLiftActuatorHeightError = 0.0f;
               /* select tracked target */
               auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
               if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
                  const SBlock& s_block = itTarget->Observations.front();
                  /* track the target using lift actuator position */
                  fLiftActuatorHeightError = TrackBlockViaLiftActuatorHeight(s_block, MTT_LIFT_ACTUATOR_MIN_HEIGHT + 2u);
                  /* calculate the approach velocity */
                  double fTagXOffset = (s_block.Tags.front().Center.first - 320.0f) / 320.0f;
                  if(fTagXOffset < 0) {
                     fLeft = -BASE_VELOCITY * (1.0f + std::abs(fTagXOffset));
                     fRight = -BASE_VELOCITY;
                  }
                  else {
                     fLeft = -BASE_VELOCITY;
                     fRight = -BASE_VELOCITY * (1.0f + std::abs(fTagXOffset));
                  }
               }
               /* scale the speed by the lift actuator height error */
               std::cerr << "|fLiftActuatorHeightError| = " << std::abs(fLiftActuatorHeightError) << std::endl;
               fLeft *= (1.0f - std::abs(fLiftActuatorHeightError));
               fRight *= (1.0f - std::abs(fLiftActuatorHeightError));
               /* apply the approach velocity */
               Data.Actuators->DifferentialDriveSystem.Left.Velocity = std::round(fLeft);
               Data.Actuators->DifferentialDriveSystem.Right.Velocity = std::round(fRight);
               Data.Actuators->DifferentialDriveSystem.Left.UpdateReq = true;
               Data.Actuators->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
         }),
         CState("far_block_approach", nullptr, nullptr, {
            CStateTurnTowardsTarget(),
            CStateApproachTarget("approach_block_left", 0.975f),
            CStateApproachTarget("approach_block_straight", 0.0f),
            CStateApproachTarget("approach_block_right", -0.975f),
            CState("align_end_effector", nullptr, nullptr, {
               CState("lower_manipulator", [this] {              
                  Data.Actuators->ManipulatorModule.LiftActuator.Position.Value = (MTT_LIFT_ACTUATOR_MIN_HEIGHT + 2u);
                  Data.Actuators->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
               }),
               CState("wait_for_manipulator"),
               CState("set_reverse_velocity", [this] {              
                  double fRight = -BASE_VELOCITY * 0.5;
                  double fLeft  = -BASE_VELOCITY * 0.5;
                  Data.Actuators->DifferentialDriveSystem.Left.Velocity = std::round(fLeft);
                  Data.Actuators->DifferentialDriveSystem.Right.Velocity = std::round(fRight);
                  Data.Actuators->DifferentialDriveSystem.Left.UpdateReq = true;
                  Data.Actuators->DifferentialDriveSystem.Right.UpdateReq = true;
               }),
               CState("wait_for_target"),
               CStateTurnTowardsTarget(),
            }),
         }),
         CState("near_block_approach", nullptr, nullptr, {
            CState("init_near_block_approach", [this] {
               double fLeft = 0.0f, fRight = 0.0f;
               switch(m_eApproachDirection) {
                  case EApproachDirection::LEFT:
                     fLeft =  1.5f * BASE_VELOCITY;
                     fRight = 0.5f * BASE_VELOCITY;
                     break;
                  case EApproachDirection::RIGHT:
                     fLeft =  0.5f * BASE_VELOCITY;
                     fRight = 1.5f * BASE_VELOCITY;
                     break;
                  case EApproachDirection::STRAIGHT:
                     fLeft =  1.0f * BASE_VELOCITY;
                     fRight = 1.0f * BASE_VELOCITY;                    
                     break;
               }
               /* configure drive system */
               Data.Actuators->DifferentialDriveSystem.Left.Velocity = std::round(fLeft);
               Data.Actuators->DifferentialDriveSystem.Right.Velocity = std::round(fRight);
               Data.Actuators->DifferentialDriveSystem.Right.UpdateReq = true;
               Data.Actuators->DifferentialDriveSystem.Left.UpdateReq = true;
               /* lower the manipulator */
               Data.Actuators->ManipulatorModule.LiftActuator.Position.Value = (MTT_LIFT_ACTUATOR_MIN_HEIGHT + 3u);
               Data.Actuators->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
            }),
            CState("wait_for_underneath_rf"),
            CState("set_approach_velocity", [this] {
               double fLeft = Data.Sensors->ManipulatorModule.RangeFinders.Left / 
                  static_cast<double>(Data.Sensors->ManipulatorModule.RangeFinders.Right + 
                                     Data.Sensors->ManipulatorModule.RangeFinders.Left) * 120;
               double fRight = Data.Sensors->ManipulatorModule.RangeFinders.Right / 
                  static_cast<double>(Data.Sensors->ManipulatorModule.RangeFinders.Right + 
                                     Data.Sensors->ManipulatorModule.RangeFinders.Left) * 120;
               
               Data.Actuators->DifferentialDriveSystem.Left.Velocity = std::round(fLeft);
               Data.Actuators->DifferentialDriveSystem.Right.Velocity = std::round(fRight);
               
               std::cerr << "Output Velocity" << " Left = " << fLeft << ", Right = " << fRight << std::endl;
               
               Data.Actuators->DifferentialDriveSystem.Left.UpdateReq = true;
               Data.Actuators->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
            CState("set_zero_velocity", [this] {
               Data.Actuators->DifferentialDriveSystem.Left.Velocity = 0;
               Data.Actuators->DifferentialDriveSystem.Right.Velocity = 0;
               Data.Actuators->DifferentialDriveSystem.Left.UpdateReq = true;
               Data.Actuators->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
         }),
         CState("pick_up_block", nullptr, nullptr, {
            CState("lower_lift_actuator", [this] {
               Data.Actuators->ManipulatorModule.LiftActuator.Position.Value = MTT_LIFT_ACTUATOR_MIN_HEIGHT;
               Data.Actuators->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
            }),
            CState("wait_for_lift_actuator"),
            CState("align_block_magnets", nullptr, nullptr, {
               CState("enable_attachment_field", [this] {
                  for(CBlockDemo::EColor& e_color : Data.Actuators->LEDDeck.Color)
                     e_color = CBlockDemo::EColor::BLUE;
                  for(bool& b_update : Data.Actuators->LEDDeck.UpdateReq)
                     b_update = true;
                  Data.Actuators->ManipulatorModule.EndEffector.FieldMode = 
                     CBlockDemo::EGripperFieldMode::CONSTRUCTIVE;
                  Data.Actuators->ManipulatorModule.EndEffector.UpdateReq = true;
               }),
               CState("disable_attachment_field", [this] {
                  Data.Actuators->ManipulatorModule.EndEffector.FieldMode = 
                     CBlockDemo::EGripperFieldMode::DISABLED;
                  Data.Actuators->ManipulatorModule.EndEffector.UpdateReq = true;
               }),
               CState("wait_for_recharge"),
            }),
            CState("attach_block", nullptr, nullptr, {
               CState("enable_attachment_field", [this] {
                  Data.Actuators->ManipulatorModule.EndEffector.FieldMode = 
                     CBlockDemo::EGripperFieldMode::CONSTRUCTIVE;
                  Data.Actuators->ManipulatorModule.EndEffector.UpdateReq = true;
               }),
               CState("wait_for_discharge"),
               CState("disable_attachment_field", [this] {
                  Data.Actuators->ManipulatorModule.EndEffector.FieldMode = 
                     CBlockDemo::EGripperFieldMode::DISABLED;
                  Data.Actuators->ManipulatorModule.EndEffector.UpdateReq = true;
               }),
            }),
            CState("raise_lift_actuator", nullptr, nullptr, {
               CState("set_position", [this] {
                  for(CBlockDemo::EColor& e_color : Data.Actuators->LEDDeck.Color)
                     e_color = CBlockDemo::EColor::BLUE;
                  for(bool& b_update : Data.Actuators->LEDDeck.UpdateReq)
                     b_update = true;
                  Data.Actuators->ManipulatorModule.LiftActuator.Position.Value = (MTT_LIFT_ACTUATOR_MAX_HEIGHT - 5u);
                  Data.Actuators->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
                  Data.Actuators->ManipulatorModule.EndEffector.UpdateReq = true;
                  Data.Actuators->ManipulatorModule.NFCInterface.OutboundMessage = "3";
                  Data.Actuators->ManipulatorModule.NFCInterface.UpdateReq = true;
               }),
               CState("wait_for_lift_actuator"),
            }), // raise_lift_actuator
         }), // pick up block
         CState("reattempt_block_pick_up", nullptr, nullptr, {
            CState("set_reverse_velocity", [this] {              
               double fRight = -BASE_VELOCITY * 0.5;
               double fLeft  = -BASE_VELOCITY * 0.5;
               Data.Actuators->DifferentialDriveSystem.Left.Velocity = std::round(fLeft);
               Data.Actuators->DifferentialDriveSystem.Right.Velocity = std::round(fRight);
               Data.Actuators->DifferentialDriveSystem.Left.UpdateReq = true;
               Data.Actuators->DifferentialDriveSystem.Right.UpdateReq = true;
               Data.Actuators->ManipulatorModule.LiftActuator.Position.Value = (MTT_LIFT_ACTUATOR_MIN_HEIGHT + 10u);
               Data.Actuators->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
            }),
            CState("wait_for_target"),
         }),
         CState("search_for_structures", nullptr, nullptr, {
            CState("set_search_velocity", [this] {
               Data.Actuators->DifferentialDriveSystem.Left.Velocity = BASE_VELOCITY;
               Data.Actuators->DifferentialDriveSystem.Left.UpdateReq = true;
               Data.Actuators->DifferentialDriveSystem.Right.Velocity = -BASE_VELOCITY;
               Data.Actuators->DifferentialDriveSystem.Right.UpdateReq = true;
               for(CBlockDemo::EColor& e_color : Data.Actuators->LEDDeck.Color) 
                  e_color = CBlockDemo::EColor::RED;
               for(bool& b_update : Data.Actuators->LEDDeck.UpdateReq)
                  b_update = true;
            }),
            CState("wait_for_zero_targets"),
            CState("wait_for_next_target"),
            CStateTurnTowardsTarget(),
         }),
         CState("search_for_previous_structure", nullptr, nullptr, {
            CState("set_search_velocity", [this] {
               Data.Actuators->DifferentialDriveSystem.Left.Velocity = -BASE_VELOCITY;
               Data.Actuators->DifferentialDriveSystem.Left.UpdateReq = true;
               Data.Actuators->DifferentialDriveSystem.Right.Velocity = BASE_VELOCITY;
               Data.Actuators->DifferentialDriveSystem.Right.UpdateReq = true;
               for(CBlockDemo::EColor& e_color : Data.Actuators->LEDDeck.Color) 
                  e_color = CBlockDemo::EColor::RED;
               for(bool& b_update : Data.Actuators->LEDDeck.UpdateReq)
                  b_update = true;
            }),
            CState("wait_for_zero_targets"),
            CState("wait_for_next_target"),
         }),
         CState("far_structure_approach", nullptr, nullptr, {
            CStateTurnTowardsTarget(),
            CStateApproachTarget("approach_block_left", 0.975f),
            CStateApproachTarget("approach_block_straight", 0.0f),
            CStateApproachTarget("approach_block_right", -0.975f),
         }),
         CState("near_structure_approach", nullptr, nullptr, {
            CState("set_manipulator_height", [this] {
               Data.Actuators->ManipulatorModule.LiftActuator.Position.Value = 28;
               Data.Actuators->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
            }),
            CState("wait_for_manipulator"),
            CState("set_approach_velocity", [this] {
               double fLeft = 0.0f, fRight = 0.0f;
               switch(m_eApproachDirection) {
                  case EApproachDirection::LEFT:
                     fLeft =  1.25f * BASE_VELOCITY;
                     fRight = 0.75f * BASE_VELOCITY;
                     break;
                  case EApproachDirection::RIGHT:
                     fLeft =  0.75f * BASE_VELOCITY;
                     fRight = 1.25f * BASE_VELOCITY;
                     break;
                  case EApproachDirection::STRAIGHT:
                     fLeft =  1.0f * BASE_VELOCITY;
                     fRight = 1.0f * BASE_VELOCITY;                    
                     break;
               }
               Data.Actuators->DifferentialDriveSystem.Left.Velocity = std::round(fLeft);
               Data.Actuators->DifferentialDriveSystem.Right.Velocity = std::round(fRight);
               Data.Actuators->DifferentialDriveSystem.Left.UpdateReq = true;
               Data.Actuators->DifferentialDriveSystem.Right.UpdateReq = true;
               m_fNearApproachStartTime = Data.Sensors->Clock.Time;
            }),
            CState("wait_until_approach_timer_expired"),
            CState("set_reverse_velocity", [this] {
               Data.Actuators->DifferentialDriveSystem.Left.Velocity = -BASE_VELOCITY;
               Data.Actuators->DifferentialDriveSystem.Right.Velocity = -BASE_VELOCITY;
               m_fReverseStartTime = Data.Sensors->Clock.Time;
               Data.Actuators->DifferentialDriveSystem.Left.UpdateReq = true;
               Data.Actuators->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
            CState("wait_until_reverse_timer_expired"),
            CState("set_zero_velocity", [this] {
               Data.Actuators->DifferentialDriveSystem.Left.Velocity = 0;
               Data.Actuators->DifferentialDriveSystem.Right.Velocity = 0;
               Data.Actuators->DifferentialDriveSystem.Left.UpdateReq = true;
               Data.Actuators->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
         }),
         CState("place_block", nullptr, nullptr, {
            CState("wait_until_charged"),
            CState("enable_detachment_field", [this] {
               Data.Actuators->ManipulatorModule.EndEffector.FieldMode = 
                  CBlockDemo::EGripperFieldMode::DESTRUCTIVE;
               Data.Actuators->ManipulatorModule.EndEffector.UpdateReq = true;   
            }),
            CState("disable_detachment_field", [this] {
               Data.Actuators->ManipulatorModule.EndEffector.FieldMode = 
                  CBlockDemo::EGripperFieldMode::DISABLED;
               Data.Actuators->ManipulatorModule.EndEffector.UpdateReq = true;   
            }),
         }),
         CState("reverse", nullptr, nullptr, {
            CState("set_reverse_velocity", [this] {
               Data.Actuators->DifferentialDriveSystem.Left.Velocity = -BASE_VELOCITY;
               Data.Actuators->DifferentialDriveSystem.Right.Velocity = -BASE_VELOCITY;
               m_fReverseStartTime = Data.Sensors->Clock.Time;
               Data.Actuators->DifferentialDriveSystem.Left.UpdateReq = true;
               Data.Actuators->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
            CState("wait_until_reverse_timer_expired"),
         }),        
      }) {
        
      (*this).AddTransition("raise_lift_actuator", "search_for_unused_block");
      //(*this).AddTransition("raise_lift_actuator", "search_for_structures");
      
      /* search_for_target transitions */
      (*this)["search_for_unused_block"].AddTransition("set_search_velocity","wait_for_next_target");
      (*this)["search_for_unused_block"].AddTransition("wait_for_next_target", "turn_towards_target", [this] {
          auto itTrackedTarget = std::find_if(std::begin(Data.Sensors->ImageSensor.Detections.Targets),
                                              std::end(Data.Sensors->ImageSensor.Detections.Targets),
                                              [this] (const STarget& s_target) {
                                                 return (s_target.Id != Data.TrackedTargetId);
                                              });
      
         if(itTrackedTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            Data.TrackedTargetId = itTrackedTarget->Id;
            std::cerr << "Tracking target = " << Data.TrackedTargetId << std::endl;
            return true;
         }
         return false;
      });
      
      (*this)["search_for_unused_block"].AddTransition("turn_towards_target", "set_search_velocity", [this] {
         STarget::TConstListIterator itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget == std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            std::cerr << "turn_towards_target: lost target " << Data.TrackedTargetId << std::endl;
            return true;
         }             
         for(const STarget& s_target : Data.Sensors->ImageSensor.Detections.Targets) {
            if(s_target.Id == itTarget->Id) {
               // don't compare target with itself
               continue;
            }
            else {
               // if the two blocks are with MTT_BLOCK_SEP_THRESHOLD of each other, this is not an unused block
               const SBlock& s_block_a = itTarget->Observations.front();
               const SBlock& s_block_b = s_target.Observations.front();
               double fNeighborBlockDist = argos::Distance(s_block_a.Translation, s_block_b.Translation);
               if(fNeighborBlockDist < MTT_BLOCK_SEP_THRESHOLD) {
                  std::cerr << "turn_towards_target: " << Data.TrackedTargetId << " and " << s_target.Id << " form a structure (" 
                            << fNeighborBlockDist << " < " << MTT_BLOCK_SEP_THRESHOLD << ")" << std::endl;
                  return true;
               }
            }
         }
         return false;
      });


      (*this)["search_for_unused_block"].AddTransition("turn_towards_target", "reverse_until_distance", [this] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            
            if(s_block.Tags.front().Center.first > TAG_CENTERED_LOWER_THRES && 
               s_block.Tags.front().Center.first < TAG_CENTERED_UPPER_THRES) {
               return true;
            }                
         }
         else {
            std::cerr << "turn_towards_target: lost target " << Data.TrackedTargetId << std::endl;
         }
         return false;
      });

      (*this)["search_for_unused_block"].AddTransition("reverse_until_distance", "set_search_velocity", [this] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget == std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            std::cerr << "reverse_until_distance: lost target " << Data.TrackedTargetId << std::endl;
            return true;
         }             
         for(const STarget& s_target : Data.Sensors->ImageSensor.Detections.Targets) {
            if(s_target.Id == itTarget->Id) {
               // don't compare target with itself
               continue;
            }
            else {
               // if the two blocks are with MTT_BLOCK_SEP_THRESHOLD of each other, this is not an unused block
               const SBlock& s_block_a = itTarget->Observations.front();
               const SBlock& s_block_b = s_target.Observations.front();
               double fNeighborBlockDist = argos::Distance(s_block_a.Translation, s_block_b.Translation);
               if(fNeighborBlockDist < MTT_BLOCK_SEP_THRESHOLD) {
                  std::cerr << "reverse_until_distance: " << Data.TrackedTargetId << " belonged to a structure" << std::endl;
                  return true;
               }
            }
         }
         return false;
      });

      (*this)["search_for_unused_block"].AddExitTransition("reverse_until_distance", [this] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
      
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            std::cerr << "reverse_until_distance: target " << Data.TrackedTargetId << " is at Z = " << s_block.Translation.GetZ() << "m" << std::endl;
            return (s_block.Translation.GetZ() > TARGET_Z_DIST_REVERSE_TO);
         }
         return false; 
      });
      

      (*this).AddTransition("search_for_unused_block","far_block_approach");
      
      /*************************************************************************************************************************/
      /*************************************************************************************************************************/

      (*this)["far_block_approach"].AddTransition("turn_towards_target", "approach_block_straight", [this] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if(s_block.Tags.front().Center.first > TAG_CENTERED_LOWER_THRES && 
               s_block.Tags.front().Center.first < TAG_CENTERED_UPPER_THRES) {
               argos::CRadians cEulerAngleZ, cEulerAngleY, cEulerAngleX;
               s_block.Rotation.ToEulerAngles(cEulerAngleZ, cEulerAngleY, cEulerAngleX);
               if(cEulerAngleZ.GetValue() > -(M_PI / 18.0f) &&
                  cEulerAngleZ.GetValue() <  (M_PI / 18.0f)) {
                  m_eApproachDirection = EApproachDirection::STRAIGHT;
                  std::cerr << "Target angle = " << cEulerAngleZ.GetValue() << ": Using straight approach" << std::endl;
                  return true;
               }
            }                
         }
         return false;
      });

      (*this)["far_block_approach"].AddTransition("turn_towards_target", "approach_block_right", [this] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if(s_block.Tags.front().Center.first > TAG_CENTERED_LOWER_THRES && 
               s_block.Tags.front().Center.first < TAG_CENTERED_UPPER_THRES) {
               argos::CRadians cEulerAngleZ, cEulerAngleY, cEulerAngleX;
               s_block.Rotation.ToEulerAngles(cEulerAngleZ, cEulerAngleY, cEulerAngleX);               
               if(cEulerAngleZ.GetValue() <= -(M_PI / 18.0f)) {
                  m_eApproachDirection = EApproachDirection::RIGHT;
                  std::cerr << "Target angle = " << cEulerAngleZ.GetValue() << ": Using right approach" << std::endl;
                  return true;
               }
            }                
         }
         return false;
      });
      
      (*this)["far_block_approach"].AddTransition("turn_towards_target", "approach_block_left", [this] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);                                         
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if(s_block.Tags.front().Center.first > TAG_CENTERED_LOWER_THRES && 
               s_block.Tags.front().Center.first < TAG_CENTERED_UPPER_THRES) {
               argos::CRadians cEulerAngleZ, cEulerAngleY, cEulerAngleX;
               s_block.Rotation.ToEulerAngles(cEulerAngleZ, cEulerAngleY, cEulerAngleX);              
               if(cEulerAngleZ.GetValue() >= (M_PI / 18.0f)) {
                  m_eApproachDirection = EApproachDirection::LEFT;
                  std::cerr << "Target angle = " << cEulerAngleZ.GetValue() << ": Using left approach" << std::endl;
                  return true;
               }
            }
         }
         return false;
      });

/*    
      (*this)["far_block_approach"].AddExitTransition("approach_block_left", [this] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         for(const STarget& s_target : Data.Sensors->ImageSensor.Detections.Targets) {
            if(s_target.Id == itTarget->Id) {
               // don't compare target with itself
               continue;
            }
            else {
               // if the two blocks are with MTT_BLOCK_SEP_THRESHOLD of each other, this is not an unused block
               const SBlock& s_block_a = itTarget->Observations.front();
               const SBlock& s_block_b = s_target.Observations.front();
               double fNeighborBlockDist = argos::Distance(s_block_a.Translation, s_block_b.Translation);
               if(fNeighborBlockDist < MTT_BLOCK_SEP_THRESHOLD) {
                  std::cerr << "approach_block_left: " << Data.TrackedTargetId << " belonged to a structure" << std::endl;
                  Data.TrackedTargetId = 0;
                  return true;
               }
            }
         }
         return false;
      });
      
      (*this)["far_block_approach"].AddExitTransition("approach_block_right", [this] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         for(const STarget& s_target : Data.Sensors->ImageSensor.Detections.Targets) {
            if(s_target.Id == itTarget->Id) {
               // don't compare target with itself
               continue;
            }
            else {
               // if the two blocks are with MTT_BLOCK_SEP_THRESHOLD of each other, this is not an unused block
               const SBlock& s_block_a = itTarget->Observations.front();
               const SBlock& s_block_b = s_target.Observations.front();
               double fNeighborBlockDist = argos::Distance(s_block_a.Translation, s_block_b.Translation);
               if(fNeighborBlockDist < MTT_BLOCK_SEP_THRESHOLD) {
                  std::cerr << "approach_block_right: " << Data.TrackedTargetId << " belonged to a structure" << std::endl;
                  Data.TrackedTargetId = 0;
                  return true;
               }
            }
         }
         return false;
      });
      
      (*this)["far_block_approach"].AddExitTransition("approach_block_straight", [this] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         for(const STarget& s_target : Data.Sensors->ImageSensor.Detections.Targets) {
            if(s_target.Id == itTarget->Id) {
               // don't compare target with itself
               continue;
            }
            else {
               // if the two blocks are with MTT_BLOCK_SEP_THRESHOLD of each other, this is not an unused block
               const SBlock& s_block_a = itTarget->Observations.front();
               const SBlock& s_block_b = s_target.Observations.front();
               double fNeighborBlockDist = argos::Distance(s_block_a.Translation, s_block_b.Translation);
               if(fNeighborBlockDist < MTT_BLOCK_SEP_THRESHOLD) {
                  std::cerr << "approach_block_straight: " << Data.TrackedTargetId << " belonged to a structure" << std::endl;
                  Data.TrackedTargetId = 0;
                  return true;
               }
            }
         }
         return false;
      });

     
      (*this)["far_block_approach"].AddTransition("approach_block_right", "align_end_effector", [this] {
         return (Data.Sensors->ImageSensor.Detections.Targets.size() == 0);
      });
      
      (*this)["far_block_approach"].AddTransition("approach_block_left", "align_end_effector", [this] {
         return (Data.Sensors->ImageSensor.Detections.Targets.size() == 0);
      });

      (*this)["far_block_approach"].AddTransition("approach_block_straight", "align_end_effector", [this] {
         return (Data.Sensors->ImageSensor.Detections.Targets.size() == 0);
      });

*/

      (*this)["far_block_approach"]["align_end_effector"].AddTransition("lower_manipulator", "wait_for_manipulator");
      
      (*this)["far_block_approach"]["align_end_effector"].AddTransition("wait_for_manipulator", "set_reverse_velocity", [this] {
         return (Data.Sensors->ManipulatorModule.LiftActuator.State == CBlockDemo::ELiftActuatorSystemState::INACTIVE);
      });
      
      //(*this)["far_block_approach"]["align_end_effector"].AddTransition("set_reverse_velocity", "wait_for_target");
      
      (*this)["far_block_approach"]["align_end_effector"].AddExitTransition("set_reverse_velocity");
            
      (*this)["far_block_approach"]["align_end_effector"].AddTransition("wait_for_target", "turn_towards_target", [this] {
         if(Data.Sensors->ImageSensor.Detections.Targets.size() > 0) {
            Data.TrackedTargetId = Data.Sensors->ImageSensor.Detections.Targets.front().Id;
            return true;
         }
         return false;
      });
      
      (*this)["far_block_approach"]["align_end_effector"].AddExitTransition("turn_towards_target", [this] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
                                         
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            
            if(s_block.Tags.front().Center.first > (0.475f * 640.0f) && 
               s_block.Tags.front().Center.first < (0.525f * 640.0f)) {
               return true;
            }                
         }
         return false;
      });
      
      (*this)["far_block_approach"].AddExitTransition("align_end_effector");
      
     
      (*this).AddTransition("far_block_approach", "near_block_approach", [this] {
         return (Data.TrackedTargetId != 0);
      });
      
      (*this).AddTransition("far_block_approach", "raise_lift_actuator", [this] {
         return (Data.TrackedTargetId == 0);
      });

      /*************************************************************************************************************************
      *************************************************************************************************************************/

      
      /* near_block_approach transitions */
      (*this)["near_block_approach"].AddTransition("init_near_block_approach","wait_for_underneath_rf");
      (*this)["near_block_approach"].AddTransition("wait_for_underneath_rf","set_approach_velocity", [this] {
         return (Data.Sensors->ManipulatorModule.RangeFinders.Underneath > 2300);
      });
      (*this)["near_block_approach"].AddTransition("set_approach_velocity", "set_zero_velocity", [this] {
         return (Data.Sensors->ManipulatorModule.RangeFinders.Left > 3000) &&
                (Data.Sensors->ManipulatorModule.RangeFinders.Right > 3000);
      });
      (*this)["near_block_approach"].AddExitTransition("set_zero_velocity");
           
      (*this).AddTransition("near_block_approach", "pick_up_block");

      /*************************************************************************************************************************
      *************************************************************************************************************************/

      /* pickup_block transistions */
      (*this)["pick_up_block"].AddTransition("lower_lift_actuator","wait_for_lift_actuator");
      (*this)["pick_up_block"].AddTransition("wait_for_lift_actuator", "align_block_magnets", [this] {
         return (Data.Sensors->ManipulatorModule.LiftActuator.State == 
                 CBlockDemo::ELiftActuatorSystemState::INACTIVE);
      });
      /* pickup_block.align_block_magnets */
      (*this)["pick_up_block"]["align_block_magnets"].AddTransition("enable_attachment_field","disable_attachment_field");
      (*this)["pick_up_block"]["align_block_magnets"].AddTransition("disable_attachment_field","wait_for_recharge");
      (*this)["pick_up_block"]["align_block_magnets"].AddExitTransition("wait_for_recharge", [this] {
         bool bChargeIsStable = true;
         for(unsigned int idx = 1; idx < Data.Sensors->ManipulatorModule.LiftActuator.Electromagnets.Charge.size(); idx++) {
            bChargeIsStable = bChargeIsStable &&
               (Data.Sensors->ManipulatorModule.LiftActuator.Electromagnets.Charge[idx] ==
                Data.Sensors->ManipulatorModule.LiftActuator.Electromagnets.Charge[idx - 1]);
         }
         return bChargeIsStable;
      });

      (*this)["pick_up_block"].AddTransition("align_block_magnets","attach_block");
      /* pickup_block.attach_block */
      (*this)["pick_up_block"]["attach_block"].AddTransition("enable_attachment_field","wait_for_discharge");
      (*this)["pick_up_block"]["attach_block"].AddTransition("wait_for_discharge","disable_attachment_field", [this] {
         return ((Data.Sensors->ManipulatorModule.LiftActuator.Electromagnets.Charge.size() > 0) &&
                 (Data.Sensors->ManipulatorModule.LiftActuator.Electromagnets.Charge.front() < 0x80));
      });
      (*this)["pick_up_block"]["attach_block"].AddExitTransition("disable_attachment_field");

      (*this)["pick_up_block"].AddTransition("attach_block", "raise_lift_actuator");
      /* pickup_block.raise_lift_actuator */
         
      (*this)["pick_up_block"]["raise_lift_actuator"].AddTransition("set_position","wait_for_lift_actuator");
      (*this)["pick_up_block"]["raise_lift_actuator"].AddExitTransition("wait_for_lift_actuator", [this] {
         if(Data.Sensors->ManipulatorModule.LiftActuator.State == CBlockDemo::ELiftActuatorSystemState::INACTIVE) {
            if(Data.Sensors->ManipulatorModule.RangeFinders.Underneath < 2300) {
               Data.TrackedTargetId = 0;
            }
            return true;
         }       
         return false;
      });

      (*this)["pick_up_block"].AddExitTransition("raise_lift_actuator");
      
      (*this).AddTransition("pick_up_block", "reattempt_block_pick_up", [this] {
         return (Data.TrackedTargetId == 0);
      });
      
      (*this).AddTransition("pick_up_block", "search_for_structures", [this] {
         return (Data.TrackedTargetId != 0);
      });
      
      /*************************************************************************************************************************
      *************************************************************************************************************************/
     
      
      (*this)["reattempt_block_pick_up"].AddTransition("set_reverse_velocity", "wait_for_target");
      
      (*this)["reattempt_block_pick_up"].AddExitTransition("wait_for_target", [this] {     
         if(Data.Sensors->ImageSensor.Detections.Targets.size() > 0) {
            Data.TrackedTargetId = Data.Sensors->ImageSensor.Detections.Targets.front().Id;
            return true;
         }
         return false;
      });
      
      (*this).AddTransition("reattempt_block_pick_up", "far_block_approach");
    
      /*************************************************************************************************************************
                                 _        __                  _                   _                  
        ___  ___  __ _ _ __ ___| |__    / _| ___  _ __   ___| |_ _ __ _   _  ___| |_ _   _ _ __ ___ 
       / __|/ _ \/ _` | '__/ __| '_ \  | |_ / _ \| '__| / __| __| '__| | | |/ __| __| | | | '__/ _ \
       \__ \  __/ (_| | | | (__| | | | |  _| (_) | |    \__ \ |_| |  | |_| | (__| |_| |_| | | |  __/
       |___/\___|\__,_|_|  \___|_| |_| |_|  \___/|_|    |___/\__|_|   \__,_|\___|\__|\__,_|_|  \___|
                                                                                                    
      *************************************************************************************************************************/
      
      (*this)["search_for_structures"].AddTransition("set_search_velocity","wait_for_zero_targets");
      
      (*this)["search_for_structures"].AddTransition("wait_for_zero_targets","wait_for_next_target", [this] {
         return (Data.Sensors->ImageSensor.Detections.Targets.size() == 0);
      });
      
      (*this)["search_for_structures"].AddTransition("wait_for_next_target", "turn_towards_target", [this] {
         if(Data.Sensors->ImageSensor.Detections.Targets.size() != 0) {
            Data.TrackedTargetId = Data.Sensors->ImageSensor.Detections.Targets.front().Id;
            std::cerr << "Tracking target = " << Data.TrackedTargetId << std::endl;
            return true;
         }         
         return false;
      });
      

      (*this)["search_for_structures"].AddTransition("turn_towards_target", "set_search_velocity", [this] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if(s_block.Tags.front().Center.first > (0.475f * 640.0f) && 
               s_block.Tags.front().Center.first < (0.525f * 640.0f) && 
               m_unNumTargetsInStructA == 0) {
               m_unNumTargetsInStructA = Data.Sensors->ImageSensor.Detections.Targets.size();
               std::cerr << "Structure A has " << m_unNumTargetsInStructA << " targets" << std::endl;
               return true;
            }
         }
         return false;
      });
      
      (*this)["search_for_structures"].AddExitTransition("turn_towards_target", [this] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();            
            if(s_block.Tags.front().Center.first > (0.475f * 640.0f) && 
               s_block.Tags.front().Center.first < (0.525f * 640.0f) && 
               m_unNumTargetsInStructA != 0) {
               m_unNumTargetsInStructB = Data.Sensors->ImageSensor.Detections.Targets.size();
               std::cerr << "Structure B has " << m_unNumTargetsInStructB << " targets" << std::endl;
               return true;
            }
         }
         return false;
      });
         
      /**********************************************************************************************************/
      
      (*this).AddTransition("search_for_structures", "far_structure_approach", [this] {
         return (m_unNumTargetsInStructA <= m_unNumTargetsInStructB);
      });
      
      (*this).AddTransition("search_for_structures", "search_for_previous_structure", [this] {
         return (m_unNumTargetsInStructA > m_unNumTargetsInStructB);
      });
      
      (*this)["search_for_previous_structure"].AddTransition("set_search_velocity", "wait_for_zero_targets");
      
      
      (*this)["search_for_previous_structure"].AddTransition("wait_for_zero_targets","wait_for_next_target", [this] {
         return (Data.Sensors->ImageSensor.Detections.Targets.size() == 0);
      });
      
      (*this)["search_for_previous_structure"].AddExitTransition("wait_for_next_target", [this] {
         if(Data.Sensors->ImageSensor.Detections.Targets.size() != 0) {
            Data.TrackedTargetId = Data.Sensors->ImageSensor.Detections.Targets.front().Id;
            std::cerr << "Tracking target = " << Data.TrackedTargetId << std::endl;
            return true;
         }         
         return false;
      });
      
      /**********************************************************************************************************/
      
      (*this).AddTransition("search_for_previous_structure", "far_structure_approach");      
      
      (*this)["far_structure_approach"].AddTransition("turn_towards_target", "approach_block_straight", [this] {
         auto itTargetFurthestToTheRight = FindTargetFurthestToTheRight(Data.Sensors->ImageSensor.Detections.Targets);
         if(itTargetFurthestToTheRight != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            if(Data.TrackedTargetId != itTargetFurthestToTheRight->Id) {
               std::cerr << "swapping target from " << Data.TrackedTargetId << " to " << itTargetFurthestToTheRight->Id << " (further to the right)" << std::endl;
               Data.TrackedTargetId = itTargetFurthestToTheRight->Id;
            }
         }     
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if(s_block.Tags.front().Center.first > (0.475f * 640.0f) && 
               s_block.Tags.front().Center.first < (0.525f * 640.0f)) {
               argos::CRadians cEulerAngleZ, cEulerAngleY, cEulerAngleX;
               s_block.Rotation.ToEulerAngles(cEulerAngleZ, cEulerAngleY, cEulerAngleX);
               if(cEulerAngleZ.GetValue() > -(M_PI / 18.0f) &&
                  cEulerAngleZ.GetValue() <  (M_PI / 18.0f)) {
                  m_eApproachDirection = EApproachDirection::STRAIGHT;
                  return true;
               }
            }                
         }
         return false;
      });

      (*this)["far_structure_approach"].AddTransition("turn_towards_target", "approach_block_right", [this] {
         auto itTargetFurthestToTheRight = FindTargetFurthestToTheRight(Data.Sensors->ImageSensor.Detections.Targets);
         if(itTargetFurthestToTheRight != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            if(Data.TrackedTargetId != itTargetFurthestToTheRight->Id) {
               std::cerr << "swapping target from " << Data.TrackedTargetId << " to " << itTargetFurthestToTheRight->Id << " (further to the right)" << std::endl;
               Data.TrackedTargetId = itTargetFurthestToTheRight->Id;
            }
         }
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if(s_block.Tags.front().Center.first > (0.475f * 640.0f) && 
               s_block.Tags.front().Center.first < (0.525f * 640.0f)) {
               argos::CRadians cEulerAngleZ, cEulerAngleY, cEulerAngleX;
               s_block.Rotation.ToEulerAngles(cEulerAngleZ, cEulerAngleY, cEulerAngleX);
               if(cEulerAngleZ.GetValue() <= -(M_PI / 18.0f)) {
                  m_eApproachDirection = EApproachDirection::RIGHT;
                  std::cerr << "turn_towards_target: selected right approach, Rot(Z) is " << cEulerAngleZ.GetValue() << std::endl;
                  return true;
               }
            }                
         }
         return false;
      });
      
      (*this)["far_structure_approach"].AddTransition("turn_towards_target", "approach_block_left", [this] {
         auto itTargetFurthestToTheRight = FindTargetFurthestToTheRight(Data.Sensors->ImageSensor.Detections.Targets);
         if(itTargetFurthestToTheRight != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            if(Data.TrackedTargetId != itTargetFurthestToTheRight->Id) {
               std::cerr << "swapping target from " << Data.TrackedTargetId << " to " << itTargetFurthestToTheRight->Id << " (further to the right)" << std::endl;
               Data.TrackedTargetId = itTargetFurthestToTheRight->Id;
            }
         }
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);                                         
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if(s_block.Tags.front().Center.first > (0.475f * 640.0f) && 
               s_block.Tags.front().Center.first < (0.525f * 640.0f)) {
               argos::CRadians cEulerAngleZ, cEulerAngleY, cEulerAngleX;
               s_block.Rotation.ToEulerAngles(cEulerAngleZ, cEulerAngleY, cEulerAngleX);
               if(cEulerAngleZ.GetValue() >= (M_PI / 18.0f)) {
                  m_eApproachDirection = EApproachDirection::LEFT;
                  std::cerr << "turn_towards_target: selected left approach, Rot(Z) is " << cEulerAngleZ.GetValue() << std::endl;
                  return true;
               }
            }
         }
         return false;
      });
      
      (*this)["far_structure_approach"].AddExitTransition("approach_block_right", [this] {
         auto itTargetFurthestToTheRight = FindTargetFurthestToTheRight(Data.Sensors->ImageSensor.Detections.Targets);
         if(itTargetFurthestToTheRight != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            if(Data.TrackedTargetId != itTargetFurthestToTheRight->Id) {
               std::cerr << "swapping target from " << Data.TrackedTargetId << " to " << itTargetFurthestToTheRight->Id << " (further to the right)" << std::endl;
               Data.TrackedTargetId = itTargetFurthestToTheRight->Id;
            }
         }     
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if(s_block.Translation.GetZ() < 0.08) {
               return true;
            }
         }
         else {
            /* lost tracking, drop the payload */
            return true;
         }
         return false;
      });
      
      (*this)["far_structure_approach"].AddExitTransition("approach_block_left", [this] {
         auto itTargetFurthestToTheRight = FindTargetFurthestToTheRight(Data.Sensors->ImageSensor.Detections.Targets);
         if(itTargetFurthestToTheRight != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            if(Data.TrackedTargetId != itTargetFurthestToTheRight->Id) {
               std::cerr << "swapping target from " << Data.TrackedTargetId << " to " << itTargetFurthestToTheRight->Id << " (further to the right)" << std::endl;
               Data.TrackedTargetId = itTargetFurthestToTheRight->Id;
            }
         }     
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if(s_block.Translation.GetZ() < 0.08) {
               return true;
            }
         }
         else {
            /* lost tracking, drop the payload */
            return true;
         }
         return false;
      });

      (*this)["far_structure_approach"].AddExitTransition("approach_block_straight", [this] {
         auto itTargetFurthestToTheRight = FindTargetFurthestToTheRight(Data.Sensors->ImageSensor.Detections.Targets);
         if(itTargetFurthestToTheRight != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            if(Data.TrackedTargetId != itTargetFurthestToTheRight->Id) {
               std::cerr << "swapping target from " << Data.TrackedTargetId << " to " << itTargetFurthestToTheRight->Id << " (further to the right)" << std::endl;
               Data.TrackedTargetId = itTargetFurthestToTheRight->Id;
            }
         }     
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if(s_block.Translation.GetZ() < 0.08) {
               return true;
            }
         }
         else {
            /* lost tracking, drop the payload */
            return true;
         }
         return false;
      });


      (*this).AddTransition("far_structure_approach", "near_structure_approach");
      
      (*this)["near_structure_approach"].AddTransition("set_manipulator_height", "wait_for_manipulator");
      
      (*this)["near_structure_approach"].AddTransition("wait_for_manipulator", "set_approach_velocity", [this] {
         return (Data.Sensors->ManipulatorModule.LiftActuator.State == CBlockDemo::ELiftActuatorSystemState::INACTIVE);
      });
      
      (*this)["near_structure_approach"].AddTransition("set_approach_velocity", "wait_until_approach_timer_expired");
      
      (*this)["near_structure_approach"].AddTransition("wait_until_approach_timer_expired", "set_reverse_velocity", [this] {
         return (Data.Sensors->Clock.Time - m_fNearApproachStartTime > 0.5f);
      });
      
      (*this)["near_structure_approach"].AddTransition("set_reverse_velocity", "wait_until_reverse_timer_expired");
      
      (*this)["near_structure_approach"].AddTransition("wait_until_reverse_timer_expired", "set_zero_velocity", [this] {
         return (Data.Sensors->Clock.Time - m_fNearApproachStartTime > 1.0f);
      });

      (*this)["near_structure_approach"].AddExitTransition("set_zero_velocity");
      
      (*this).AddTransition("near_structure_approach", "place_block");
      
      (*this)["place_block"].AddTransition("wait_until_charged", "enable_detachment_field", [this] {
         bool bChargeIsStable = true;
         for(unsigned int idx = 1; idx < Data.Sensors->ManipulatorModule.LiftActuator.Electromagnets.Charge.size(); idx++) {
            bChargeIsStable = bChargeIsStable &&
               (Data.Sensors->ManipulatorModule.LiftActuator.Electromagnets.Charge[idx] ==
                Data.Sensors->ManipulatorModule.LiftActuator.Electromagnets.Charge[idx - 1]);
         }
         return bChargeIsStable;
      });
      
      (*this)["place_block"].AddTransition("enable_detachment_field", "disable_detachment_field");
      (*this)["place_block"].AddExitTransition("disable_detachment_field");
      
      
      (*this).AddTransition("place_block", "reverse");
      
      (*this)["reverse"].AddTransition("set_reverse_velocity", "wait_until_reverse_timer_expired");
      
      (*this)["reverse"].AddExitTransition("wait_until_reverse_timer_expired", [this] {
         return (Data.Sensors->Clock.Time - m_fReverseStartTime > 10.0f);
      });

      
      (*this).AddExitTransition("reverse");

   }

private:  
   STarget::TConstListIterator FindTargetWithMostQ4Leds(const STarget::TList& s_target_list) {
      STarget::TConstListIterator itTargetWithMostQ4Leds = std::end(s_target_list);
      unsigned int unTargetWithMostQ4LedsCount = 0;      
      for(STarget::TConstListIterator it_target = std::begin(s_target_list);
         it_target != std::end(s_target_list);
         it_target++) {
         const SBlock& s_block = it_target->Observations.front();
         unsigned int unTargetQ4Leds = 0;
         for(ELedState e_led_state : s_block.Tags.front().DetectedLeds) {
            unTargetQ4Leds += (e_led_state == ELedState::Q4) ? 1 : 0;
         }
         for(const STag& s_tag : s_block.HackTags) {
            for(ELedState e_led_state : s_tag.DetectedLeds) {
               unTargetQ4Leds += (e_led_state == ELedState::Q4) ? 1 : 0;
            }        
         }
         if(unTargetQ4Leds > unTargetWithMostQ4LedsCount) {
            unTargetWithMostQ4LedsCount = unTargetQ4Leds;
            itTargetWithMostQ4Leds = it_target;
         }
      }
      return itTargetWithMostQ4Leds;
   }
   
   enum class EApproachDirection {
      LEFT, RIGHT, STRAIGHT
   } m_eApproachDirection;
   
   double m_fNearApproachStartTime, m_fReverseStartTime;
   
   unsigned int m_unNumTargetsInStructA = 0;
   unsigned int m_unNumTargetsInStructB = 0;
};

#endif
