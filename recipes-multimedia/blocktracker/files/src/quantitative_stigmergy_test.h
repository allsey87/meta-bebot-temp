#ifndef MANIPULATOR_TESTING_TASK_H
#define MANIPULATOR_TESTING_TASK_H

#include "state.h"
#include "block_demo.h"

#include <iostream>

#define IMAGE_SENSOR_WIDTH 640.0f
#define IMAGE_SENSOR_HALF_WIDTH (IMAGE_SENSOR_WIDTH / 2.0f)
#define IMAGE_SENSOR_HEIGHT 360.0f
#define IMAGE_SENSOR_HALF_HEIGHT (IMAGE_SENSOR_HEIGHT / 2.0f)

#define LIFT_ACTUATOR_MAX_HEIGHT 140
#define LIFT_ACTUATOR_MIN_HEIGHT 3
#define LIFT_ACTUATOR_INC_HEIGHT 15

#define PREAPPROACH_BLOCK_X_TARGET 0.000f
#define PREAPPROACH_BLOCK_Z_TARGET 0.325f
#define PREAPPROACH_BLOCK_XZ_THRES 0.010f

#define TAG_CENTERED_LOWER_THRES (0.40f * IMAGE_SENSOR_WIDTH)
#define TAG_CENTERED_UPPER_THRES (0.60f * IMAGE_SENSOR_WIDTH)

#define BASE_VELOCITY 30

#define MTT_BLOCK_SEP_THRESHOLD 0.065f

/************************************************************/
/*               Shared data for all states                 */
/************************************************************/
enum class EApproachDirection {
      LEFT, RIGHT, STRAIGHT
} m_eApproachDirection;

struct {
  CBlockDemo::SSensorData* Sensors = nullptr;
  CBlockDemo::SActuatorData* Actuators = nullptr;
  unsigned int TrackedTargetId = 0;
  unsigned int TrackedStructureId = 0;
  EApproachDirection ApproachDirection = EApproachDirection::STRAIGHT;
  std::chrono::time_point<std::chrono::steady_clock> ElectromagnetSwitchOnTime;
} Data;


/************************************************************/
/*            Common functions for all states               */
/************************************************************/

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

using TCornerConstIterator = std::vector<std::pair<double, double> >::const_iterator;

TCornerConstIterator FindTagCornerFurthestToTheRight(const SBlock& s_block) {
   return std::max_element(std::begin(s_block.Tags.front().Corners),
                           std::end(s_block.Tags.front().Corners),
                           [] (const std::pair<double, double>& c_pair_lhs, const std::pair<double, double>& c_pair_rhs) {
                              return c_pair_lhs.first < c_pair_rhs.first;
                           });
}

TCornerConstIterator FindTagCornerFurthestToTheLeft(const SBlock& s_block) {
   return std::min_element(std::begin(s_block.Tags.front().Corners),
                           std::end(s_block.Tags.front().Corners),
                           [] (const std::pair<double, double>& c_pair_lhs, const std::pair<double, double>& c_pair_rhs) {
                              return c_pair_lhs.first < c_pair_rhs.first;
                           });
}

double TrackBlockViaLiftActuatorHeight(const SBlock& s_block, 
                                       uint8_t un_min_position = LIFT_ACTUATOR_MIN_HEIGHT,
                                       uint8_t un_max_position = LIFT_ACTUATOR_MAX_HEIGHT) {
   double fEndEffectorPos = Data.Sensors->ManipulatorModule.LiftActuator.EndEffector.Position;                
   double fLiftActuatorHeightError = (IMAGE_SENSOR_HALF_HEIGHT - s_block.Tags.front().Center.second) / IMAGE_SENSOR_HALF_HEIGHT;
   fEndEffectorPos += (fLiftActuatorHeightError * LIFT_ACTUATOR_INC_HEIGHT);
   uint8_t unEndEffectorPos = 
      (fEndEffectorPos > un_max_position) ? un_max_position : 
      (fEndEffectorPos < un_min_position) ? un_min_position : static_cast<uint8_t>(fEndEffectorPos); 
   Data.Actuators->ManipulatorModule.LiftActuator.Position.Value = unEndEffectorPos;
   Data.Actuators->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
   return fLiftActuatorHeightError;
}

/************************************************************/
/*               Common state definitions                   */
/************************************************************/

class CStateSendNFCMessage : public CState {
public:
   CStateSendNFCMessage(const std::string& str_id, const std::string& str_data) :
      CState(str_id, [&str_data] {
         Data.Actuators->ManipulatorModule.NFCInterface.OutboundMessage = str_data;
         Data.Actuators->ManipulatorModule.NFCInterface.UpdateReq = true;
   }) {}
};

class CStateSetLedColors : public CState {
public:
   CStateSetLedColors(const std::string& str_id, CBlockDemo::EColor e_new_color) :
      CState(str_id, [e_new_color] {
         for(CBlockDemo::EColor& e_color : Data.Actuators->LEDDeck.Color)
            e_color = e_new_color;
         for(bool& b_update : Data.Actuators->LEDDeck.UpdateReq)
            b_update = true;
   }) {}
};

class CStateSetVelocity : public CState {
public:
   CStateSetVelocity(const std::string& str_id, double f_left, double f_right) :
      CState(str_id, [f_left, f_right] {
         /* apply the approach velocity */
         Data.Actuators->DifferentialDriveSystem.Left.Velocity = std::round(f_left);
         Data.Actuators->DifferentialDriveSystem.Right.Velocity = std::round(f_right);
         Data.Actuators->DifferentialDriveSystem.Left.UpdateReq = true;
         Data.Actuators->DifferentialDriveSystem.Right.UpdateReq = true;
      }) {}
};

class CStateMoveToTargetXZ : public CState {
public:
   CStateMoveToTargetXZ(const std::string& str_id, double f_x_target, double f_z_target) :
      CState(str_id, [f_x_target, f_z_target] {
         /* default velocities, overwritten if target is detected */
         double fLeft = 0.0f, fRight = 0.0f;
         /* select tracked target */
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            /* calculate the approach velocity */
            double fBlockXOffset = s_block.Translation.GetX() - f_x_target;
            double fBlockZOffset = s_block.Translation.GetZ() - f_z_target;
            fLeft = BASE_VELOCITY * (fBlockXOffset + fBlockZOffset) * 5.0f;
            fRight = BASE_VELOCITY * (-fBlockXOffset + fBlockZOffset) * 5.0f;
         }
         /* apply the approach velocity */
         Data.Actuators->DifferentialDriveSystem.Left.Velocity = std::round(fLeft);
         Data.Actuators->DifferentialDriveSystem.Right.Velocity = std::round(fRight);
         Data.Actuators->DifferentialDriveSystem.Left.UpdateReq = true;
         Data.Actuators->DifferentialDriveSystem.Right.UpdateReq = true;
      }) {}
};

class CStateMoveToTargetX : public CState {
public:
   CStateMoveToTargetX(const std::string& str_id, double f_x_target) :
      CState(str_id, [f_x_target] {
         /* default velocities, overwritten if target is detected */
         double fLeft = 0.0f, fRight = 0.0f;
         /* select tracked target */
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            /* calculate the approach velocity */
            double fBlockXOffset = s_block.Translation.GetX() - f_x_target;
            fLeft = BASE_VELOCITY * (fBlockXOffset) * 5.0f;
            fRight = BASE_VELOCITY * (-fBlockXOffset) * 5.0f;
         }
         /* apply the approach velocity */
         Data.Actuators->DifferentialDriveSystem.Left.Velocity = std::round(fLeft);
         Data.Actuators->DifferentialDriveSystem.Right.Velocity = std::round(fRight);
         Data.Actuators->DifferentialDriveSystem.Left.UpdateReq = true;
         Data.Actuators->DifferentialDriveSystem.Right.UpdateReq = true;
      }) {}
};

class CStateMoveToTargetZ : public CState {
public:
   CStateMoveToTargetZ(const std::string& str_id, double f_z_target) :
      CState(str_id, [f_z_target] {
         /* default velocities, overwritten if target is detected */
         double fLeft = 0.0f, fRight = 0.0f;
         /* select tracked target */
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            /* calculate the approach velocity */
            double fBlockZOffset = s_block.Translation.GetZ() - f_z_target;
            fLeft = BASE_VELOCITY * (fBlockZOffset) * 5.0f;
            fRight = BASE_VELOCITY * (fBlockZOffset) * 5.0f;
         }
         /* apply the approach velocity */
         Data.Actuators->DifferentialDriveSystem.Left.Velocity = std::round(fLeft);
         Data.Actuators->DifferentialDriveSystem.Right.Velocity = std::round(fRight);
         Data.Actuators->DifferentialDriveSystem.Left.UpdateReq = true;
         Data.Actuators->DifferentialDriveSystem.Right.UpdateReq = true;
      }) {}
};

class CStateSetLiftActuatorPosition : public CState {
public: 
   CStateSetLiftActuatorPosition(const std::string& str_id, uint8_t un_position) :
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

class CStatePulseElectromagnets : public CState {
public: 
   CStatePulseElectromagnets(const std::string& str_id, const std::chrono::milliseconds& t_duration, CBlockDemo::EGripperFieldMode e_field_mode) :
      CState(str_id, nullptr, nullptr, {
         CState("init_precharge", [] {
            Data.Actuators->ManipulatorModule.EndEffector.FieldMode = CBlockDemo::EGripperFieldMode::DISABLED;
            Data.Actuators->ManipulatorModule.EndEffector.UpdateReq = true;
         }),
         CState("wait_for_precharge"),
         CState("switch_field_on", [e_field_mode] {
            Data.Actuators->ManipulatorModule.EndEffector.FieldMode = e_field_mode;
            Data.Actuators->ManipulatorModule.EndEffector.UpdateReq = true;
            Data.ElectromagnetSwitchOnTime = std::chrono::steady_clock::now();
         }),
         CState("wait_for_duration"),
         CState("switch_field_off", [] {
            Data.Actuators->ManipulatorModule.EndEffector.FieldMode = CBlockDemo::EGripperFieldMode::DISABLED;
            Data.Actuators->ManipulatorModule.EndEffector.UpdateReq = true;
         }),
      }) {
      AddTransition("init_precharge","wait_for_precharge");
      AddTransition("wait_for_precharge","switch_field_on", [] {
         auto tMinMaxPair = std::minmax(std::begin(Data.Sensors->ManipulatorModule.LiftActuator.Electromagnets.Charge),
                                        std::end(Data.Sensors->ManipulatorModule.LiftActuator.Electromagnets.Charge));
         return *(tMinMaxPair.first) == *(tMinMaxPair.second);
      });
      AddTransition("switch_field_on","wait_for_duration");
      AddTransition("wait_for_duration", "switch_field_off", [t_duration] {
         return (Data.ElectromagnetSwitchOnTime + t_duration) < std::chrono::steady_clock::now();
      });
      AddExitTransition("switch_field_off");
   }
};

class CStateAttachBlock : public CState {
public:
   CStateAttachBlock(const std::string& str_id) :
      CState(str_id, nullptr, nullptr, {
         CStateSetLiftActuatorPosition("init_lift_actuator_position", LIFT_ACTUATOR_MIN_HEIGHT + 5u),
         CStatePulseElectromagnets("pre_alignment_pulse", std::chrono::milliseconds(500), CBlockDemo::EGripperFieldMode::CONSTRUCTIVE),
         CStateSetLiftActuatorPosition("lower_lift_actuator", LIFT_ACTUATOR_MIN_HEIGHT),
         CStatePulseElectromagnets("attachment_pulse", std::chrono::milliseconds(1500), CBlockDemo::EGripperFieldMode::CONSTRUCTIVE),
      }) {
         AddTransition("init_lift_actuator_position","pre_alignment_pulse");
         AddTransition("pre_alignment_pulse", "lower_lift_actuator");
         AddTransition("lower_lift_actuator","attachment_pulse");
         AddExitTransition("attachment_pulse");
      }
};

class CStateApproachTargetStraight : public CState {
public:
   CStateApproachTargetStraight(const std::string& str_id) :
      CState(str_id, [] {
         
      }) {}
};

class CStateApproachTargetFromSide : public CState {
public:
   CStateApproachTargetFromSide(const std::string& str_id) :
      CState(str_id, [] {
         /* default velocities, overwritten if target is detected */
         double fLeft = 0.0f, fRight = 0.0f, fLiftActuatorHeightError = 0.0f;
         /* select tracked target */
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);                         
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            /* track the target using lift actuator position */
            fLiftActuatorHeightError = TrackBlockViaLiftActuatorHeight(s_block, LIFT_ACTUATOR_MIN_HEIGHT + 2u);
            /* calculate the approach velocity */
            argos::CRadians cEulerAngles[3]; // ZYX
            s_block.Rotation.ToEulerAngles(cEulerAngles[0], cEulerAngles[1], cEulerAngles[2]);
            if(cEulerAngles[0] > argos::CRadians::PI_OVER_FOUR) {}

            double f_offset_target = 0.0f; //?????
            double fTagXOffset = 0.0f;
            
            //fTagXOffset = (FindTagCornerFurthestToTheRight(s_block)->first - IMAGE_SENSOR_HALF_WIDTH) / IMAGE_SENSOR_HALF_WIDTH;
            //fTagXOffset = (FindTagCornerFurthestToTheLeft(s_block)->first - IMAGE_SENSOR_HALF_WIDTH) / IMAGE_SENSOR_HALF_WIDTH;

            // scale fTagXOffset towards zero as we straighten up with the target
            
            double fScaleWrtZ = std::abs(cEulerAngles[0].GetValue() / argos::CRadians::PI_OVER_FOUR.GetValue());

            std::cerr << "1. fTagXOffset = " << fTagXOffset << std::endl; 
            fTagXOffset *= fScaleWrtZ;
            std::cerr << "2. fTagXOffset = " << fTagXOffset << std::endl; 

            /* fTagXOffsetTarget - quadratic scaling of f_offset_target to [0 f_offset_target] wrt. Z-translation [0.15 0.35] */
            double fTagXOffsetTarget = f_offset_target * (25.0f * std::pow(s_block.Translation.GetZ() - 0.15f, 2));
            std::cerr << "s_block.Translation.GetZ() = " << s_block.Translation.GetZ() << std::endl;
            std::cerr << "fTagXOffsetTarget = " << fTagXOffsetTarget << std::endl;

            fRight = (1 + fTagXOffsetTarget - fTagXOffset) * BASE_VELOCITY;
            fLeft  = (1 + fTagXOffset - fTagXOffsetTarget) * BASE_VELOCITY;
            std::cerr << "fLeft = " << fLeft << std::endl;
            std::cerr << "fRight = " << fRight << std::endl;
         }
         /* scale the speed by the lift actuator height error */
         fLeft *= (1.0f - std::abs(fLiftActuatorHeightError));
         fRight *= (1.0f - std::abs(fLiftActuatorHeightError));
         /* apply the approach velocity */
         Data.Actuators->DifferentialDriveSystem.Left.Velocity = std::round(fLeft);
         Data.Actuators->DifferentialDriveSystem.Right.Velocity = std::round(fRight);
         Data.Actuators->DifferentialDriveSystem.Left.UpdateReq = true;
         Data.Actuators->DifferentialDriveSystem.Right.UpdateReq = true;
      }) {}
};

/************************************************************/
/*             Main state machine definition                */
/************************************************************/
class CManipulatorTestingTask : public CState {
public:
   CManipulatorTestingTask() :
      CState("top_level_state", nullptr, nullptr, {
         CState("search_for_unused_block", nullptr, nullptr, {
            CStateSetLedColors("set_deck_color", CBlockDemo::EColor::BLUE),            
            CStateSetLiftActuatorPosition("set_lift_actuator_search_height", LIFT_ACTUATOR_MAX_HEIGHT - 5u),           
            CStateSetVelocity("set_search_velocity", BASE_VELOCITY * 0.5f, -BASE_VELOCITY * 0.5f),
            CState("wait_for_next_target"),
         }),
         CState("approach_block_far", nullptr, nullptr, {
            CStateSetLedColors("set_deck_color", CBlockDemo::EColor::GREEN),
            CStateMoveToTargetXZ("align_with_target", 0.0f, PREAPPROACH_BLOCK_Z_TARGET),
            //CStateApproachTargetFromSide("approach_target_from_side"),
            //CStateMoveToTargetX("turn_towards_target", 0.0f),
            //CStateApproachTargetStraight("approach_target_straight"),
            /* the above states are exited once the target disappears, we then reverse until the target is detected again */
            //CStateSetVelocity("set_reverse_velocity", -BASE_VELOCITY * 0.5f, -BASE_VELOCITY * 0.5f),
            //CStateSetLiftActuatorPosition("lower_lift_actuator", LIFT_ACTUATOR_MIN_HEIGHT + 5u),
            //CState("wait_for_target"),
            //CStateTurnTowardsTarget(), //CStateSetVelocity(0)
         }),
         CState("approach_block_near", nullptr, nullptr, {
            CStateSetLedColors("set_deck_color", CBlockDemo::EColor::RED),
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
               Data.Actuators->ManipulatorModule.LiftActuator.Position.Value = (LIFT_ACTUATOR_MIN_HEIGHT + 3u);
               Data.Actuators->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
            }),
            CState("wait_for_underneath_rf"),
            CState("set_approach_velocity", [this] {
               double fLeft = Data.Sensors->ManipulatorModule.RangeFinders.Left / 
                  static_cast<double>(Data.Sensors->ManipulatorModule.RangeFinders.Right + 
                                     Data.Sensors->ManipulatorModule.RangeFinders.Left) * BASE_VELOCITY;
               double fRight = Data.Sensors->ManipulatorModule.RangeFinders.Right / 
                  static_cast<double>(Data.Sensors->ManipulatorModule.RangeFinders.Right + 
                                     Data.Sensors->ManipulatorModule.RangeFinders.Left) * BASE_VELOCITY;
               
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
         CState("attach_block", nullptr, nullptr, {
            CStateAttachBlock("attempt_block_pickup"),
            CStateSetLiftActuatorPosition("raise_lift_actuator", 20u),
            // test rf to see if block is attached
            // reattempt attachment ?
            CStateSendNFCMessage("write_message", "3"),
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
            //CStateTurnTowardsTarget(),
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
            //CStateTurnTowardsTarget(),
            //CStateApproachTarget("approach_block_left", EApproachDirection::LEFT),
            //CStateApproachTarget("approach_block_straight", EApproachDirection::STRAIGHT),
            //CStateApproachTarget("approach_block_right", EApproachDirection::RIGHT),
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
      

      /*************************************************************************************************************************/
      /*************************************************************************************************************************/  


      CState("top_level_state", nullptr, nullptr, {
         CState("search_for_unused_block", nullptr, nullptr, {
            CStateSetLedColors("set_deck_color", CBlockDemo::EColor::BLUE),            
            CStateSetLiftActuatorPosition("set_lift_actuator_search_height", LIFT_ACTUATOR_MAX_HEIGHT - 5u),           
            CStateSetVelocity("set_search_velocity", BASE_VELOCITY * 0.5f, -BASE_VELOCITY * 0.5f),
            CState("wait_for_next_target"),
         }),

      /* search_for_unused_block transitions */
      (*this)["search_for_unused_block"].AddTransition("set_deck_color", "set_lift_actuator_search_height");
      (*this)["search_for_unused_block"].AddTransition("set_lift_actuator_search_height", "set_search_velocity");
      (*this)["search_for_unused_block"].AddTransition("set_search_velocity","wait_for_next_target");
      (*this)["search_for_unused_block"].AddExitTransition("wait_for_next_target", [this] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);                         
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {        
            Data.TrackedTargetId = itTrackedTarget->Id;
            return true;
         }
         return false;
      });

      (*this).AddTransition("search_for_unused_block","far_block_approach");
      
      /*************************************************************************************************************************/
      /*************************************************************************************************************************/

      (*this)["far_block_approach"].AddTransition("pre_approach_target", "approach_block_straight", [this] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if((std::abs(s_block.Translation.GetX() - PREAPPROACH_BLOCK_X_TARGET) < PREAPPROACH_BLOCK_XZ_THRES) &&
               (std::abs(s_block.Translation.GetZ() - PREAPPROACH_BLOCK_Z_TARGET) < PREAPPROACH_BLOCK_XZ_THRES)) {
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

      (*this)["far_block_approach"].AddTransition("pre_approach_target", "approach_block_right", [this] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if((std::abs(s_block.Translation.GetX() - PREAPPROACH_BLOCK_X_TARGET) < PREAPPROACH_BLOCK_XZ_THRES) &&
               (std::abs(s_block.Translation.GetZ() - PREAPPROACH_BLOCK_Z_TARGET) < PREAPPROACH_BLOCK_XZ_THRES)) {
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
      
      (*this)["far_block_approach"].AddTransition("pre_approach_target", "approach_block_left", [this] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);                                         
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if((std::abs(s_block.Translation.GetX() - PREAPPROACH_BLOCK_X_TARGET) < PREAPPROACH_BLOCK_XZ_THRES) &&
               (std::abs(s_block.Translation.GetZ() - PREAPPROACH_BLOCK_Z_TARGET) < PREAPPROACH_BLOCK_XZ_THRES)) {
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

      /*      
      (*this)["far_block_approach"]["align_end_effector"].AddTransition("wait_for_target", "turn_towards_target", [this] {
         if(Data.Sensors->ImageSensor.Detections.Targets.size() > 0) {
            Data.TrackedTargetId = Data.Sensors->ImageSensor.Detections.Targets.front().Id;
            return true;
         }
         return false;
      });
      */

      /*
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
      */
      /*
      (*this)["far_block_approach"].AddExitTransition("align_end_effector");
      
     
      (*this).AddTransition("far_block_approach", "near_block_approach", [this] {
         return (Data.TrackedTargetId != 0);
      });
      
      (*this).AddTransition("far_block_approach", "raise_lift_actuator", [this] {
         return (Data.TrackedTargetId == 0);
      });
      */

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
      
      /*
      (*this)["search_for_structures"].AddTransition("wait_for_next_target", "turn_towards_target", [this] {
         if(Data.Sensors->ImageSensor.Detections.Targets.size() != 0) {
            Data.TrackedTargetId = Data.Sensors->ImageSensor.Detections.Targets.front().Id;
            std::cerr << "Tracking target = " << Data.TrackedTargetId << std::endl;
            return true;
         }         
         return false;
      });
      */

      /*
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
      */
 
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
      /*
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
      */
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
   
   double m_fNearApproachStartTime, m_fReverseStartTime;
   
   unsigned int m_unNumTargetsInStructA = 0;
   unsigned int m_unNumTargetsInStructB = 0;
};

#endif
