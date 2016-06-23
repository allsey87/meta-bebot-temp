#ifndef PYRAMID_EXPERIMENT_H
#define PYRAMID_EXPERIMENT_H

#include "state.h"
#include "block_demo.h"
#include "pid_controller.h"

#include <iostream>

#define IMAGE_SENSOR_WIDTH 640.0
#define IMAGE_SENSOR_HALF_WIDTH (IMAGE_SENSOR_WIDTH / 2.0)
#define IMAGE_SENSOR_HEIGHT 360.0
#define IMAGE_SENSOR_HALF_HEIGHT (IMAGE_SENSOR_HEIGHT / 2.0)

#define LIFT_ACTUATOR_MAX_HEIGHT 135
#define LIFT_ACTUATOR_MIN_HEIGHT 4
#define LIFT_ACTUATOR_INC_HEIGHT 15
#define LIFT_ACTUATOR_BLOCK_HEIGHT 55
#define LIFT_ACTUATOR_BLOCK_PREATTACH_OFFSET 3

#define RF_UN_BLOCK_DETECT_THRES 2250
#define RF_LR_BLOCK_DETECT_THRES 3500

#define OBSERVE_BLOCK_X_TARGET 0.000
#define OBSERVE_BLOCK_Z_TARGET 0.275
#define OBSERVE_BLOCK_XZ_THRES 0.050

#define PREAPPROACH_BLOCK_X_TARGET 0.000
#define PREAPPROACH_BLOCK_Z_TARGET 0.325
#define PREAPPROACH_BLOCK_XZ_THRES 0.015

#define APPROACH_BLOCK_X_FAIL_THRES 0.025

#define NEAR_APPROACH_TIMEOUT std::chrono::milliseconds(7500)

#define BASE_VELOCITY 30.0
#define BASE_XZ_GAIN 5.0

/************************************************************/
/*               Shared data for all states                 */
/************************************************************/

struct {
   /* pointers to sensors and actuators */
   CBlockDemo::SSensorData* Sensors = nullptr;
   CBlockDemo::SActuatorData* Actuators = nullptr;
   /* controllers */
   CPIDController TagApproachController = CPIDController(1.250, 0.250, 0.375);
   /* generic local data */
   unsigned int TrackedTargetId = 0;
   struct {
      argos::CVector3 Translation;
      argos::CQuaternion Rotation;
   } TrackedTargetLastObservation;
   unsigned int TrackedStructureId = 0;
   std::chrono::time_point<std::chrono::steady_clock> ElectromagnetSwitchOnTime;
   std::chrono::time_point<std::chrono::steady_clock> NearApproachStartTime;
   /* data specific to quantitative stigmergy demo */
   std::list<unsigned int> FoundStructureSizes;
   std::list<unsigned int> InspectedTargets;
} Data;


/************************************************************/
/*            Common functions for all states               */
/************************************************************/

/**************** functions for transitions ****************/
bool IsTargetLost() {
   auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
   if(itTarget == std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
      return true;
   }
   return false;
}

bool IsNextTargetAcquired() {
   std::vector<unsigned int> vecDetectedTargetIds;
   for(const STarget& s_target : Data.Sensors->ImageSensor.Detections.Targets) {
      if(s_target.Id > Data.TrackedTargetId) {
         vecDetectedTargetIds.push_back(s_target.Id);
      }
   }
   if(!vecDetectedTargetIds.empty()) {
      std::sort(std::begin(vecDetectedTargetIds), std::end(vecDetectedTargetIds));
      Data.TrackedTargetId = vecDetectedTargetIds[0];
      return true;
   }
   return false;
}

/**************** functions for control loops ****************/
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

/**************** functions for updating actuators ****************/
void SetVelocity(double f_left, double f_right) {
   Data.Actuators->DifferentialDriveSystem.Left.Velocity = std::round(f_left);
   Data.Actuators->DifferentialDriveSystem.Right.Velocity = std::round(f_right);
   Data.Actuators->DifferentialDriveSystem.Left.UpdateReq = true;
   Data.Actuators->DifferentialDriveSystem.Right.UpdateReq = true;
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
         SetVelocity(f_left, f_right);
      }) {}
};

class CStateMoveToTargetXZ : public CState {
public:
   CStateMoveToTargetXZ(const std::string& str_id, double f_x_target, double f_z_target, bool b_track_via_lift_actuator) :
      CState(str_id, [f_x_target, f_z_target, b_track_via_lift_actuator] {
         /* default velocities, overwritten if target is detected */
         double fLeft = 0.000, fRight = 0.000;
         /* select tracked target */
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            /* calculate the approach velocity */
            double fBlockXOffset = s_block.Translation.GetX() - f_x_target;
            double fBlockZOffset = s_block.Translation.GetZ() - f_z_target;
            fLeft = BASE_VELOCITY * (fBlockXOffset + fBlockZOffset) * BASE_XZ_GAIN;
            fRight = BASE_VELOCITY * (-fBlockXOffset + fBlockZOffset) * BASE_XZ_GAIN;
            /* track target via the lift actuator if enabled */
            if(b_track_via_lift_actuator) {
               TrackBlockViaLiftActuatorHeight(s_block, LIFT_ACTUATOR_MIN_HEIGHT + LIFT_ACTUATOR_BLOCK_PREATTACH_OFFSET);
               /* adjust speed if the tag is falling out of the frame */
               double fTagOffsetTop =
                  std::abs(FindTagCornerFurthestToTheTop(s_block.Tags[0])->second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;
               double fTagOffsetBottom =
                  std::abs(FindTagCornerFurthestToTheBottom(s_block.Tags[0])->second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;
               fLeft *= (1.000 - std::max(fTagOffsetTop, fTagOffsetBottom));
               fRight *= (1.000 - std::max(fTagOffsetTop, fTagOffsetBottom));
            }
         }
         /* apply the approach velocity */
         SetVelocity(fLeft, fRight);
      }) {}
};

class CStateMoveToTargetX : public CState {
public:
   CStateMoveToTargetX(const std::string& str_id, double f_x_target, bool b_track_via_lift_actuator) :
      CState(str_id, [f_x_target, b_track_via_lift_actuator] {
         /* default velocities, overwritten if target is detected */
         double fLeft = 0.000, fRight = 0.000;
         /* select tracked target */
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            /* calculate the approach velocity */
            double fBlockXOffset = s_block.Translation.GetX() - f_x_target;
            fLeft = BASE_VELOCITY * (fBlockXOffset) * BASE_XZ_GAIN;
            fRight = BASE_VELOCITY * (-fBlockXOffset) * BASE_XZ_GAIN;
            /* track target via the lift actuator if enabled */
            if(b_track_via_lift_actuator) {
               TrackBlockViaLiftActuatorHeight(s_block, LIFT_ACTUATOR_MIN_HEIGHT + LIFT_ACTUATOR_BLOCK_PREATTACH_OFFSET);
               /* adjust speed if the tag is falling out of the frame */
               double fTagOffsetTop =
                  std::abs(FindTagCornerFurthestToTheTop(s_block.Tags[0])->second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;
               double fTagOffsetBottom =
                  std::abs(FindTagCornerFurthestToTheBottom(s_block.Tags[0])->second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;
               fLeft *= (1.000 - std::max(fTagOffsetTop, fTagOffsetBottom));
               fRight *= (1.000 - std::max(fTagOffsetTop, fTagOffsetBottom));
            }
         }
         /* apply the approach velocity */
         SetVelocity(fLeft, fRight);
      }) {}
};

class CStateMoveToTargetZ : public CState {
public:
   CStateMoveToTargetZ(const std::string& str_id, double f_z_target, bool b_track_via_lift_actuator) :
      CState(str_id, [f_z_target, b_track_via_lift_actuator] {
         /* default velocities, overwritten if target is detected */
         double fLeft = 0.000, fRight = 0.000;
         /* select tracked target */
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            /* calculate the approach velocity */
            double fBlockZOffset = s_block.Translation.GetZ() - f_z_target;
            fLeft = BASE_VELOCITY * (fBlockZOffset) * BASE_XZ_GAIN;
            fRight = BASE_VELOCITY * (fBlockZOffset) * BASE_XZ_GAIN;
            /* track target via the lift actuator if enabled */
            if(b_track_via_lift_actuator) {
               TrackBlockViaLiftActuatorHeight(s_block, LIFT_ACTUATOR_MIN_HEIGHT + LIFT_ACTUATOR_BLOCK_PREATTACH_OFFSET);
               /* adjust speed if the tag is falling out of the frame */
               double fTagOffsetTop =
                  std::abs(FindTagCornerFurthestToTheTop(s_block.Tags[0])->second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;
               double fTagOffsetBottom =
                  std::abs(FindTagCornerFurthestToTheBottom(s_block.Tags[0])->second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;
               fLeft *= (1.000 - std::max(fTagOffsetTop, fTagOffsetBottom));
               fRight *= (1.000 - std::max(fTagOffsetTop, fTagOffsetBottom));
            }
         }
         /* apply the approach velocity */
         SetVelocity(fLeft, fRight);
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
         CStateSetLiftActuatorPosition("init_lift_actuator_position", LIFT_ACTUATOR_MIN_HEIGHT + LIFT_ACTUATOR_BLOCK_PREATTACH_OFFSET),
         CStatePulseElectromagnets("generate_pre_alignment_pulse", std::chrono::milliseconds(1000), CBlockDemo::EGripperFieldMode::CONSTRUCTIVE),
         CStateSetLiftActuatorPosition("lower_lift_actuator", LIFT_ACTUATOR_MIN_HEIGHT),
         CStatePulseElectromagnets("generate_attachment_pulse", std::chrono::milliseconds(2000), CBlockDemo::EGripperFieldMode::CONSTRUCTIVE),
      }) {
         AddTransition("init_lift_actuator_position","generate_pre_alignment_pulse");
         AddTransition("generate_pre_alignment_pulse", "lower_lift_actuator");
         AddTransition("lower_lift_actuator","generate_attachment_pulse");
         AddExitTransition("generate_attachment_pulse");
      }
};

class CStateApproachTarget : public CState {
public:
   CStateApproachTarget(const std::string& str_id, double f_tag_offset_target, std::function<double(const STag&)> fn_get_tag_offset) :
      CState(str_id, nullptr, nullptr, {
         CState("reset_pid_controller", [] {
            Data.TagApproachController.Reset();
         }),
         CState("approach_target", [f_tag_offset_target, &fn_get_tag_offset] {
            /* default velocities, overwritten if target is detected */
            double fLeft = 0.000, fRight = 0.000;
            /* select tracked target */
            auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
            if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
               const SBlock& s_block = itTarget->Observations.front();
               /* update the last observation data */
               Data.TrackedTargetLastObservation.Rotation = s_block.Rotation;
               Data.TrackedTargetLastObservation.Translation = s_block.Translation;
               /* track the target by lowering the lift actuator position */
               TrackBlockViaLiftActuatorHeight(s_block,
                                               LIFT_ACTUATOR_MIN_HEIGHT + LIFT_ACTUATOR_BLOCK_PREATTACH_OFFSET,
                                               Data.Actuators->ManipulatorModule.LiftActuator.Position.Value);
               /* calculate the approach velocity */
               double fTagOffset = fn_get_tag_offset(s_block.Tags[0]);
               double fOutput = Data.TagApproachController.Step(fTagOffset, f_tag_offset_target, Data.Sensors->Clock.Time);
               fRight = (1.000 + fOutput) * BASE_VELOCITY;
               fLeft  = (1.000 - fOutput) * BASE_VELOCITY;
               /* scale the speed by the lift actuator height error */
               double fTagOffsetTop =
                  std::abs(FindTagCornerFurthestToTheTop(s_block.Tags[0])->second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;
               double fTagOffsetBottom =
                  std::abs(FindTagCornerFurthestToTheBottom(s_block.Tags[0])->second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;
               fLeft *= std::max(1.000 - std::max(fTagOffsetTop, fTagOffsetBottom), 0.250);
               fRight *= std::max(1.000 - std::max(fTagOffsetTop, fTagOffsetBottom), 0.250);
            }
            /* apply the approach velocity */
            SetVelocity(fLeft, fRight);
         }),
         CState("reacquire_target", nullptr, nullptr, {
            CState("adjust_lift_actuator_height", [] {
               if(Data.Actuators->ManipulatorModule.LiftActuator.Position.Value < (LIFT_ACTUATOR_MAX_HEIGHT - LIFT_ACTUATOR_BLOCK_PREATTACH_OFFSET)) {
                  Data.Actuators->ManipulatorModule.LiftActuator.Position.Value += LIFT_ACTUATOR_BLOCK_PREATTACH_OFFSET;
                  Data.Actuators->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
               }
            }),
            CStateSetVelocity("set_reverse_velocity", -0.250 * BASE_VELOCITY, -0.250 * BASE_VELOCITY),
            CState("wait_for_target"),
            CStateSetVelocity("set_zero_velocity", 0.000, 0.000),
         }),
      }) {
         AddTransition("reset_pid_controller", "approach_target");
         AddTransition("approach_target", "reacquire_target", [] {
            return IsTargetLost() &&
                   Data.Actuators->ManipulatorModule.LiftActuator.Position.Value >
                     (LIFT_ACTUATOR_MIN_HEIGHT + LIFT_ACTUATOR_BLOCK_PREATTACH_OFFSET);
         });
         AddExitTransition("approach_target", IsTargetLost);
         /* back off until target is reacquired*/
         GetSubState("reacquire_target").AddTransition("adjust_lift_actuator_height", "set_reverse_velocity");
         GetSubState("reacquire_target").AddTransition("set_reverse_velocity", "wait_for_target");
         GetSubState("reacquire_target").AddTransition("wait_for_target", "set_zero_velocity", IsNextTargetAcquired);
         GetSubState("reacquire_target").AddExitTransition("set_zero_velocity");
         /* try again */
         AddTransition("reacquire_target", "reset_pid_controller");
      }
};

/************************************************************/
/*             Main state machine definition                */
/************************************************************/
class CFiniteStateMachine : public CState {
public:
   CFiniteStateMachine() :
      CState("top_level_state", nullptr, nullptr, {
         CState("test"),
         CState("search_for_unused_block", nullptr, nullptr, {
            CStateSetLedColors("set_deck_color", CBlockDemo::EColor::BLUE),
            CStateSetLiftActuatorPosition("raise_lift_actuator", LIFT_ACTUATOR_MAX_HEIGHT),
            CStateSetVelocity("set_search_velocity", BASE_VELOCITY * 0.500, -BASE_VELOCITY * 0.500),
            CState("wait_for_next_target"),
            CStateMoveToTargetXZ("align_with_target", 0.000, PREAPPROACH_BLOCK_Z_TARGET, false),
         }),
         CState("approach_block_far", nullptr, nullptr, {
            CStateSetLedColors("set_deck_color", CBlockDemo::EColor::GREEN),
            CStateMoveToTargetXZ("align_with_target", 0.000, PREAPPROACH_BLOCK_Z_TARGET, false),
            CStateApproachTarget("approach_target_from_left", 0.650, [] (const STag& s_tag) {
               return (FindTagCornerFurthestToTheRight(s_tag)->first - IMAGE_SENSOR_HALF_WIDTH) / IMAGE_SENSOR_HALF_WIDTH;
            }),
            CStateApproachTarget("approach_target_from_right", -0.650, [] (const STag& s_tag) {
               return (FindTagCornerFurthestToTheLeft(s_tag)->first - IMAGE_SENSOR_HALF_WIDTH) / IMAGE_SENSOR_HALF_WIDTH;
            }),
            CStateApproachTarget("approach_target_straight", 0.000, [] (const STag& s_tag) {
               return (s_tag.Center.first - IMAGE_SENSOR_HALF_WIDTH) / IMAGE_SENSOR_HALF_WIDTH;
            }),
            // failure states
            CStateSetLiftActuatorPosition("raise_lift_actuator", LIFT_ACTUATOR_MAX_HEIGHT),
            CStateSetVelocity("set_reverse_velocity", -BASE_VELOCITY * 0.500, -BASE_VELOCITY * 0.500),
            CState("wait_for_next_target"),
         }),
         CState("approach_block_near", nullptr, nullptr, {
            CStateSetLedColors("set_deck_color", CBlockDemo::EColor::RED),
            CStateSetLiftActuatorPosition("lower_lift_actuator", LIFT_ACTUATOR_MIN_HEIGHT + LIFT_ACTUATOR_BLOCK_PREATTACH_OFFSET),
            CState("set_approach_velocity", [] {
               double fLastObservationX = Data.TrackedTargetLastObservation.Translation.GetX();
               double fLeft = BASE_VELOCITY * (1.000 + (fLastObservationX * BASE_XZ_GAIN));
               double fRight = BASE_VELOCITY * (1.000 - (fLastObservationX * BASE_XZ_GAIN));
               SetVelocity(fLeft, fRight);
            }),
            CState("wait_for_underneath_rf_or_timeout"),
            CState("wait_for_either_left_right_rf_or_timeout"),
            CState("set_pivot_velocity", [] {
               bool bRfBlockDetectedLeft = (Data.Sensors->ManipulatorModule.RangeFinders.Left > RF_LR_BLOCK_DETECT_THRES);
               bool bRfBlockDetectedRight = (Data.Sensors->ManipulatorModule.RangeFinders.Right > RF_LR_BLOCK_DETECT_THRES);
               // pivot the robot towards the other sensor
               double fLeft = (bRfBlockDetectedLeft ? 0.250 : 0.500) * BASE_VELOCITY;
               double fRight = (bRfBlockDetectedRight ? 0.250 : 0.500) * BASE_VELOCITY;
               // apply the velocity
               SetVelocity(fLeft, fRight);
            }),
            CState("wait_for_both_left_right_rf_or_timeout"),
            CStateSetVelocity("set_zero_velocity", 0.000, 0.000),
         }),
         CState("pick_up_block", nullptr, nullptr, {
            CStateAttachBlock("attach_block"),
            CStateSetLiftActuatorPosition("raise_lift_actuator", LIFT_ACTUATOR_MAX_HEIGHT),
            // failure states
            CStateSetVelocity("set_reverse_velocity", -BASE_VELOCITY * 0.500, -BASE_VELOCITY * 0.500),
            CState("wait_for_next_target"),
         }),
         CStateSendNFCMessage("configure_block_for_transport", "4"),
         CState("search_for_structure", nullptr, nullptr, {
            CStateSetLedColors("set_deck_color", CBlockDemo::EColor::BLUE),
            CStateSetLiftActuatorPosition("raise_lift_actuator", LIFT_ACTUATOR_MAX_HEIGHT),
            CStateSetVelocity("set_search_velocity", BASE_VELOCITY * 0.500, -BASE_VELOCITY * 0.500),
            CState("wait_for_next_target"),
            CStateMoveToTargetXZ("align_with_target", 0.000, PREAPPROACH_BLOCK_Z_TARGET, false),
         }),
         CState("count_visible_targets", nullptr, nullptr, {
            CState("wait_for_next_target"),
            CStateSetLedColors("set_alignment_color", CBlockDemo::EColor::BLUE),
            CStateMoveToTargetXZ("align_with_target", OBSERVE_BLOCK_X_TARGET, OBSERVE_BLOCK_Z_TARGET, true),
            CStateSetLedColors("set_waiting_color", CBlockDemo::EColor::GREEN),
         }),
         CState("search_for_previous_structure", nullptr, nullptr, {
            CStateSetLedColors("set_deck_color", CBlockDemo::EColor::BLUE),
            CStateSetVelocity("set_reverse_search_velocity", -BASE_VELOCITY * 0.500, BASE_VELOCITY * 0.500),
            CState("wait_for_zero_targets"),
            CState("wait_for_next_target"),
         }),
         CStateMoveToTargetXZ("select_placement_target", OBSERVE_BLOCK_X_TARGET, OBSERVE_BLOCK_Z_TARGET, true),
         CState("approach_structure_far", nullptr, nullptr, {
            CStateSetLedColors("set_deck_color", CBlockDemo::EColor::GREEN),
            CStateMoveToTargetXZ("align_with_target", 0.000, PREAPPROACH_BLOCK_Z_TARGET, false),
            CStateApproachTarget("approach_target_from_left", 0.650, [] (const STag& s_tag) {
               return (FindTagCornerFurthestToTheRight(s_tag)->first - IMAGE_SENSOR_HALF_WIDTH) / IMAGE_SENSOR_HALF_WIDTH;
            }),
            CStateApproachTarget("approach_target_from_right", -0.650, [] (const STag& s_tag) {
               return (FindTagCornerFurthestToTheLeft(s_tag)->first - IMAGE_SENSOR_HALF_WIDTH) / IMAGE_SENSOR_HALF_WIDTH;
            }),
            CStateApproachTarget("approach_target_straight", 0.000, [] (const STag& s_tag) {
               return (s_tag.Center.first - IMAGE_SENSOR_HALF_WIDTH) / IMAGE_SENSOR_HALF_WIDTH;
            }),
         }),
      }) {

      AddTransition("test", "search_for_structure");

      /**************** search_for_unused_block transitions ****************/
      GetSubState("search_for_unused_block").AddTransition("set_deck_color", "raise_lift_actuator");
      GetSubState("search_for_unused_block").AddTransition("raise_lift_actuator", "set_search_velocity");
      GetSubState("search_for_unused_block").AddTransition("set_search_velocity","wait_for_next_target");
      GetSubState("search_for_unused_block").AddTransition("wait_for_next_target", "align_with_target", IsNextTargetAcquired);
      /* keep searching if the target was lost */
      GetSubState("search_for_unused_block").AddTransition("align_with_target", "set_search_velocity", IsTargetLost);
      /* unused block found - exit */
      GetSubState("search_for_unused_block").AddExitTransition("align_with_target", [] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if((std::abs(s_block.Translation.GetX() - PREAPPROACH_BLOCK_X_TARGET) < PREAPPROACH_BLOCK_XZ_THRES) &&
               (std::abs(s_block.Translation.GetZ() - PREAPPROACH_BLOCK_Z_TARGET) < PREAPPROACH_BLOCK_XZ_THRES)) {
               /* search all structures, does it belong to any */
               for(const SStructure& s_structure : Data.Sensors->ImageSensor.Detections.Structures) {
                  if(std::find(std::begin(s_structure.Members), std::end(s_structure.Members), itTarget) != std::end(s_structure.Members)) {
                     /* is the size of the structure 1? i.e. a unused block */
                     return (s_structure.Members.size() == 1);
                  }
               }
            }
         }
         return false;
      });
      /* keep searching if block belongs to a structure */
      GetSubState("search_for_unused_block").AddTransition("align_with_target", "set_search_velocity", [] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if((std::abs(s_block.Translation.GetX() - PREAPPROACH_BLOCK_X_TARGET) < PREAPPROACH_BLOCK_XZ_THRES) &&
               (std::abs(s_block.Translation.GetZ() - PREAPPROACH_BLOCK_Z_TARGET) < PREAPPROACH_BLOCK_XZ_THRES)) {
               /* search all structures, does it belong to any */
               for(const SStructure& s_structure : Data.Sensors->ImageSensor.Detections.Structures) {
                  if(std::find(std::begin(s_structure.Members), std::end(s_structure.Members), itTarget) != std::end(s_structure.Members)) {
                     /* is the size of the structure not 1? */
                     return (s_structure.Members.size() != 1);
                  }
               }
            }
         }
         return false;
      });

      /// Top level transitions ///
      AddTransition("search_for_unused_block","approach_block_far");

      /**************** approach_block_far transitions ****************/
      /* lost tracking transitions */
      GetSubState("approach_block_far").AddExitTransition("set_deck_color", IsTargetLost);
      GetSubState("approach_block_far").AddExitTransition("align_with_target", IsTargetLost);
      /* select approach direction transitions */
      GetSubState("approach_block_far").AddTransition("set_deck_color", "align_with_target");
      GetSubState("approach_block_far").AddTransition("align_with_target", "approach_target_from_left", [] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if((std::abs(s_block.Translation.GetX() - PREAPPROACH_BLOCK_X_TARGET) < PREAPPROACH_BLOCK_XZ_THRES) &&
               (std::abs(s_block.Translation.GetZ() - PREAPPROACH_BLOCK_Z_TARGET) < PREAPPROACH_BLOCK_XZ_THRES)) {
               argos::CRadians cEulerAngleZ, cEulerAngleY, cEulerAngleX;
               s_block.Rotation.ToEulerAngles(cEulerAngleZ, cEulerAngleY, cEulerAngleX);
               return (cEulerAngleZ.GetValue() >= (M_PI / 18.0));
            }
         }
         return false;
      });
      GetSubState("approach_block_far").AddTransition("align_with_target", "approach_target_from_right", [] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if((std::abs(s_block.Translation.GetX() - PREAPPROACH_BLOCK_X_TARGET) < PREAPPROACH_BLOCK_XZ_THRES) &&
               (std::abs(s_block.Translation.GetZ() - PREAPPROACH_BLOCK_Z_TARGET) < PREAPPROACH_BLOCK_XZ_THRES)) {
               argos::CRadians cEulerAngleZ, cEulerAngleY, cEulerAngleX;
               s_block.Rotation.ToEulerAngles(cEulerAngleZ, cEulerAngleY, cEulerAngleX);
               return (cEulerAngleZ.GetValue() <= -(M_PI / 18.0));
            }
         }
         return false;
      });
      GetSubState("approach_block_far").AddTransition("align_with_target", "approach_target_straight", [] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if((std::abs(s_block.Translation.GetX() - PREAPPROACH_BLOCK_X_TARGET) < PREAPPROACH_BLOCK_XZ_THRES) &&
               (std::abs(s_block.Translation.GetZ() - PREAPPROACH_BLOCK_Z_TARGET) < PREAPPROACH_BLOCK_XZ_THRES)) {
               argos::CRadians cEulerAngleZ, cEulerAngleY, cEulerAngleX;
               s_block.Rotation.ToEulerAngles(cEulerAngleZ, cEulerAngleY, cEulerAngleX);
               return ((cEulerAngleZ.GetValue() > -(M_PI / 18.0)) && (cEulerAngleZ.GetValue() < (M_PI / 18.0)));
            }
         }
         return false;
      });
      /* approach failed - retry */
      GetSubState("approach_block_far").AddTransition("approach_target_from_left", "raise_lift_actuator", [] {
         return (std::abs(Data.TrackedTargetLastObservation.Translation.GetX()) > APPROACH_BLOCK_X_FAIL_THRES);
      });
      GetSubState("approach_block_far").AddTransition("approach_target_from_right", "raise_lift_actuator", [] {
         return (std::abs(Data.TrackedTargetLastObservation.Translation.GetX()) > APPROACH_BLOCK_X_FAIL_THRES);
      });
      GetSubState("approach_block_far").AddTransition("approach_target_straight", "raise_lift_actuator", [] {
         return (std::abs(Data.TrackedTargetLastObservation.Translation.GetX()) > APPROACH_BLOCK_X_FAIL_THRES);
      });
      GetSubState("approach_block_far").AddTransition("raise_lift_actuator", "set_reverse_velocity");
      GetSubState("approach_block_far").AddTransition("set_reverse_velocity", "wait_for_next_target");
      GetSubState("approach_block_far").AddTransition("wait_for_next_target", "align_with_target", IsNextTargetAcquired);
      /* approach success - exit state */
      GetSubState("approach_block_far").AddExitTransition("approach_target_from_left");
      GetSubState("approach_block_far").AddExitTransition("approach_target_from_right");
      GetSubState("approach_block_far").AddExitTransition("approach_target_straight");

      /// Top level transitions ///
      AddTransition("approach_block_far","approach_block_near");

      /**************** approach_block_near transitions ****************/
      GetSubState("approach_block_near").AddTransition("set_deck_color", "lower_lift_actuator");
      GetSubState("approach_block_near").AddTransition("lower_lift_actuator", "set_approach_velocity");
      GetSubState("approach_block_near").AddTransition("set_approach_velocity", "wait_for_underneath_rf_or_timeout", [] {
         /* reset timer for "wait_for_underneath_rf_or_timeout" */
         Data.NearApproachStartTime = std::chrono::steady_clock::now();
         return true;
      });
      GetSubState("approach_block_near").AddTransition("wait_for_underneath_rf_or_timeout", "wait_for_either_left_right_rf_or_timeout", [] {
         /* block detected on the underneath rf */
         if(Data.Sensors->ManipulatorModule.RangeFinders.Underneath > RF_UN_BLOCK_DETECT_THRES) {
            std::cerr << "rf.left = " << static_cast<int>(Data.Sensors->ManipulatorModule.RangeFinders.Left) << ", "
                      << "rf.right = " << static_cast<int>(Data.Sensors->ManipulatorModule.RangeFinders.Right) << ", "
                      << "rf.underneath = " << static_cast<int>(Data.Sensors->ManipulatorModule.RangeFinders.Underneath) << std::endl;
            /* reset timer for "wait_for_either_left_right_rf_or_timeout" */
            Data.NearApproachStartTime = std::chrono::steady_clock::now();
            return true;
         }
         return false;
      });
      GetSubState("approach_block_near").AddTransition("wait_for_either_left_right_rf_or_timeout", "set_zero_velocity", [] {
         /* block detected on both the left & right rf */
         if((Data.Sensors->ManipulatorModule.RangeFinders.Left > RF_LR_BLOCK_DETECT_THRES) &&
            (Data.Sensors->ManipulatorModule.RangeFinders.Right > RF_LR_BLOCK_DETECT_THRES)) {
            std::cerr << "rf.left = " << static_cast<int>(Data.Sensors->ManipulatorModule.RangeFinders.Left) << ", "
                      << "rf.right = " << static_cast<int>(Data.Sensors->ManipulatorModule.RangeFinders.Right) << ", "
                      << "rf.underneath = " << static_cast<int>(Data.Sensors->ManipulatorModule.RangeFinders.Underneath) << std::endl;
            return true;
         }
         return false;
      });
      GetSubState("approach_block_near").AddTransition("wait_for_either_left_right_rf_or_timeout", "set_pivot_velocity", [] {
         /* block detected on both the left & right rf */
         if((Data.Sensors->ManipulatorModule.RangeFinders.Left > RF_LR_BLOCK_DETECT_THRES) ||
            (Data.Sensors->ManipulatorModule.RangeFinders.Right > RF_LR_BLOCK_DETECT_THRES)) {
            std::cerr << "rf.left = " << static_cast<int>(Data.Sensors->ManipulatorModule.RangeFinders.Left) << ", "
                      << "rf.right = " << static_cast<int>(Data.Sensors->ManipulatorModule.RangeFinders.Right) << ", "
                      << "rf.underneath = " << static_cast<int>(Data.Sensors->ManipulatorModule.RangeFinders.Underneath) << std::endl;
            return true;
         }
         return false;
      });
      GetSubState("approach_block_near").AddTransition("set_pivot_velocity", "wait_for_both_left_right_rf_or_timeout", [] {
         /* reset timer for "wait_for_both_left_right_rf_or_timeout" */
         Data.NearApproachStartTime = std::chrono::steady_clock::now();
         return true;
      });
      GetSubState("approach_block_near").AddTransition("wait_for_both_left_right_rf_or_timeout", "set_zero_velocity", [] {
         /* block detected on both the left & right rf */
         if((Data.Sensors->ManipulatorModule.RangeFinders.Left > RF_LR_BLOCK_DETECT_THRES) &&
            (Data.Sensors->ManipulatorModule.RangeFinders.Right > RF_LR_BLOCK_DETECT_THRES)) {
            std::cerr << "rf.left = " << static_cast<int>(Data.Sensors->ManipulatorModule.RangeFinders.Left) << ", "
                      << "rf.right = " << static_cast<int>(Data.Sensors->ManipulatorModule.RangeFinders.Right) << ", "
                      << "rf.underneath = " << static_cast<int>(Data.Sensors->ManipulatorModule.RangeFinders.Underneath) << std::endl;
            return true;
         }
         return false;
      });
      GetSubState("approach_block_near").AddTransition("wait_for_underneath_rf_or_timeout", "set_zero_velocity", [] {
         return (Data.NearApproachStartTime + NEAR_APPROACH_TIMEOUT < std::chrono::steady_clock::now());
      });
      GetSubState("approach_block_near").AddTransition("wait_for_either_left_right_rf_or_timeout", "set_zero_velocity", [] {
         return (Data.NearApproachStartTime + NEAR_APPROACH_TIMEOUT < std::chrono::steady_clock::now());
      });
      GetSubState("approach_block_near").AddTransition("wait_for_both_left_right_rf_or_timeout", "set_zero_velocity", [] {
         return (Data.NearApproachStartTime + NEAR_APPROACH_TIMEOUT < std::chrono::steady_clock::now());
      });
      GetSubState("approach_block_near").AddExitTransition("set_zero_velocity");

      /// Top level transitions ///
      AddTransition("approach_block_near", "pick_up_block");

      /**************** pick_up_block transitions ****************/
      GetSubState("pick_up_block").AddTransition("attach_block", "raise_lift_actuator");
      GetSubState("pick_up_block").AddExitTransition("raise_lift_actuator", [] {
         return (Data.Sensors->ManipulatorModule.RangeFinders.Underneath > RF_UN_BLOCK_DETECT_THRES);
      });
      GetSubState("pick_up_block").AddTransition("raise_lift_actuator", "set_reverse_velocity");
      GetSubState("pick_up_block").AddTransition("set_reverse_velocity", "wait_for_next_target");
      GetSubState("pick_up_block").AddExitTransition("wait_for_next_target", IsNextTargetAcquired);

      /// Top level transitions ///
      AddTransition("pick_up_block", "approach_block_far", [] {
         return (Data.Sensors->ManipulatorModule.RangeFinders.Underneath < RF_UN_BLOCK_DETECT_THRES);
      });
      AddTransition("pick_up_block", "configure_block_for_transport");
      AddTransition("configure_block_for_transport", "search_for_structure");

      /**************** search_for_structure transitions ****************/
      GetSubState("search_for_structure").AddTransition("set_deck_color", "raise_lift_actuator");
      GetSubState("search_for_structure").AddTransition("raise_lift_actuator", "set_search_velocity");
      GetSubState("search_for_structure").AddTransition("set_search_velocity", "wait_for_next_target");
      GetSubState("search_for_structure").AddTransition("wait_for_next_target", "align_with_target", IsNextTargetAcquired);
      GetSubState("search_for_structure").AddTransition("align_with_target", "wait_for_next_target", IsTargetLost);
      GetSubState("search_for_structure").AddExitTransition("align_with_target", [] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if((std::abs(s_block.Translation.GetX() - PREAPPROACH_BLOCK_X_TARGET) < PREAPPROACH_BLOCK_XZ_THRES) &&
               (std::abs(s_block.Translation.GetZ() - PREAPPROACH_BLOCK_Z_TARGET) < PREAPPROACH_BLOCK_XZ_THRES)) {
               /* search all structures, does it belong to any */
               for(const SStructure& s_structure : Data.Sensors->ImageSensor.Detections.Structures) {
                  if(std::find(std::begin(s_structure.Members), std::end(s_structure.Members), itTarget) != std::end(s_structure.Members)) {
                     /* is the size of the structure greater than 1? (we are not looking at a unused block) */
                     return (s_structure.Members.size() > 1);
                  }
               }
            }
         }
         return false;
      });
      GetSubState("search_for_structure").AddTransition("align_with_target", "set_search_velocity", [] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if((std::abs(s_block.Translation.GetX() - PREAPPROACH_BLOCK_X_TARGET) < PREAPPROACH_BLOCK_XZ_THRES) &&
               (std::abs(s_block.Translation.GetZ() - PREAPPROACH_BLOCK_Z_TARGET) < PREAPPROACH_BLOCK_XZ_THRES)) {
               /* search all structures, does it belong to any */
               for(const SStructure& s_structure : Data.Sensors->ImageSensor.Detections.Structures) {
                  if(std::find(std::begin(s_structure.Members), std::end(s_structure.Members), itTarget) != std::end(s_structure.Members)) {
                     /* if the size of the structure equals 1, we are looking at a unused block */
                     return (s_structure.Members.size() == 1);
                  }
               }
            }
         }
         return false;
      });

      /// Top level transitions ///
      AddTransition("search_for_structure", "count_visible_targets", [] {
         /* add a counting slot for the structure in focus */
         Data.FoundStructureSizes.push_front(1u);
         Data.InspectedTargets.clear();
         return true;
      });

      /**************** count_visible_targets transitions ****************/
      GetSubState("count_visible_targets").AddTransition("set_alignment_color", "align_with_target");
      GetSubState("count_visible_targets").AddTransition("align_with_target", "set_waiting_color", [] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if((std::abs(s_block.Translation.GetX() - OBSERVE_BLOCK_X_TARGET) < OBSERVE_BLOCK_XZ_THRES) &&
               (std::abs(s_block.Translation.GetZ() - OBSERVE_BLOCK_Z_TARGET) < OBSERVE_BLOCK_XZ_THRES)) {
               /* increment the number of targets found in the current structure */
               Data.FoundStructureSizes.front()++;
               /* keep track of the targets we have already counted */
               Data.InspectedTargets.push_front(itTarget->Id);
               return true;
            }
         }
         return false;
      });
      GetSubState("count_visible_targets").AddTransition("align_with_target", "wait_for_next_target", IsTargetLost);
      GetSubState("count_visible_targets").AddTransition("set_waiting_color", "wait_for_next_target");
      GetSubState("count_visible_targets").AddTransition("wait_for_next_target", "set_alignment_color", [] {
         auto itTrackedTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         auto itCandidateNextTarget = std::end(Data.Sensors->ImageSensor.Detections.Targets);
         std::cerr << "sTrackedBlock(" << itTrackedTarget->Id << ").x = " << itTrackedTarget->Observations.front().Translation.GetX() << std::endl;
         for(auto it_other_target = std::begin(Data.Sensors->ImageSensor.Detections.Targets);
             it_other_target != std::end(Data.Sensors->ImageSensor.Detections.Targets);
             it_other_target++) {
            if(std::find(std::begin(Data.InspectedTargets), std::end(Data.InspectedTargets), it_other_target->Id) != std::end(Data.InspectedTargets)) {
               continue;
            }
            else {
               const SBlock& sTrackedBlock = itTrackedTarget->Observations.front();
               const SBlock& sOtherBlock = it_other_target->Observations.front();
               argos::CVector3 cInterBlockDist = sOtherBlock.Translation - sTrackedBlock.Translation;
               if(cInterBlockDist.GetX() > -0.025) {
                  std::cerr << "sOtherBlock(" << it_other_target->Id << ").x = " << sOtherBlock.Translation.GetX() << std::endl;
                  if(itCandidateNextTarget == std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
                     itCandidateNextTarget = it_other_target;
                  }
                  else {
                     const SBlock& sCandidateNextBlock = itCandidateNextTarget->Observations.front();
                     if((sCandidateNextBlock.Translation - sOtherBlock.Translation).GetX() > 0.000) {
                        itCandidateNextTarget = it_other_target;
                     }
                  }
               }
            }
         }
         if(itCandidateNextTarget == std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            std::cerr << "structure size = " << static_cast<int>(Data.FoundStructureSizes.front()) << std::endl;
            Data.TrackedTargetId = FindMostRecentTarget(Data.Sensors->ImageSensor.Detections.Targets)->Id;
            return false;
         }
         else {
            Data.TrackedTargetId = itCandidateNextTarget->Id;
            return true;
         }
      });
      /* next target not acquired => end of structure */
      GetSubState("count_visible_targets").AddExitTransition("wait_for_next_target");

      /// Top level transitions ///
      AddTransition("count_visible_targets", "search_for_structure", [] {
         return (Data.FoundStructureSizes.size() < 2);
      });
      AddTransition("count_visible_targets", "approach_structure_far", [] {
         auto itMaxStructureSize = std::max_element(std::begin(Data.FoundStructureSizes), std::end(Data.FoundStructureSizes));
         return (itMaxStructureSize == std::begin(Data.FoundStructureSizes));
      });
      AddTransition("count_visible_targets", "search_for_previous_structure");

      /**************** search_for_previous_structure transitions ****************/
      GetSubState("search_for_previous_structure").AddTransition("set_deck_color", "set_reverse_search_velocity");
      GetSubState("search_for_previous_structure").AddTransition("set_reverse_search_velocity", "wait_for_zero_targets");
      GetSubState("search_for_previous_structure").AddTransition("wait_for_zero_targets", "wait_for_next_target", [] {
         return (Data.Sensors->ImageSensor.Detections.Targets.size() == 0);
      });
      GetSubState("search_for_previous_structure").AddExitTransition("wait_for_next_target", IsNextTargetAcquired);

      /// Top level transitions ///
      AddTransition("search_for_previous_structure", "select_placement_target");
      AddTransition("select_placement_target", "approach_structure_far", [] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         auto itRightmostTarget = FindTargetFurthestToTheRight(Data.Sensors->ImageSensor.Detections.Targets);
         if((itRightmostTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) && 
            (itRightmostTarget != itTarget)) {
            Data.TrackedTargetId = itRightmostTarget->Id;
            return false;
         }
         else if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            return ((std::abs(s_block.Translation.GetX() - OBSERVE_BLOCK_X_TARGET) < OBSERVE_BLOCK_XZ_THRES) &&
                    (std::abs(s_block.Translation.GetZ() - OBSERVE_BLOCK_Z_TARGET) < OBSERVE_BLOCK_XZ_THRES));
         }
         else {  
            return false;
         }
      });

      /**************** approach_structure_far transitions ****************/
      GetSubState("approach_structure_far").AddTransition("set_deck_color", "align_with_target");
      GetSubState("approach_structure_far").AddExitTransition("align_with_target", IsTargetLost);

      GetSubState("approach_structure_far").AddTransition("align_with_target", "approach_target_from_left", [] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if((std::abs(s_block.Translation.GetX() - PREAPPROACH_BLOCK_X_TARGET) < PREAPPROACH_BLOCK_XZ_THRES) &&
               (std::abs(s_block.Translation.GetZ() - PREAPPROACH_BLOCK_Z_TARGET) < PREAPPROACH_BLOCK_XZ_THRES)) {
               argos::CRadians cEulerAngleZ, cEulerAngleY, cEulerAngleX;
               s_block.Rotation.ToEulerAngles(cEulerAngleZ, cEulerAngleY, cEulerAngleX);
               return (cEulerAngleZ.GetValue() >= (M_PI / 18.0));
            }
         }
         return false;
      });
      GetSubState("approach_structure_far").AddTransition("align_with_target", "approach_target_from_right", [] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if((std::abs(s_block.Translation.GetX() - PREAPPROACH_BLOCK_X_TARGET) < PREAPPROACH_BLOCK_XZ_THRES) &&
               (std::abs(s_block.Translation.GetZ() - PREAPPROACH_BLOCK_Z_TARGET) < PREAPPROACH_BLOCK_XZ_THRES)) {
               argos::CRadians cEulerAngleZ, cEulerAngleY, cEulerAngleX;
               s_block.Rotation.ToEulerAngles(cEulerAngleZ, cEulerAngleY, cEulerAngleX);
               return (cEulerAngleZ.GetValue() <= -(M_PI / 18.0));
            }
         }
         return false;
      });
      GetSubState("approach_structure_far").AddTransition("align_with_target", "approach_target_straight", [] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if((std::abs(s_block.Translation.GetX() - PREAPPROACH_BLOCK_X_TARGET) < PREAPPROACH_BLOCK_XZ_THRES) &&
               (std::abs(s_block.Translation.GetZ() - PREAPPROACH_BLOCK_Z_TARGET) < PREAPPROACH_BLOCK_XZ_THRES)) {
               argos::CRadians cEulerAngleZ, cEulerAngleY, cEulerAngleX;
               s_block.Rotation.ToEulerAngles(cEulerAngleZ, cEulerAngleY, cEulerAngleX);
               return ((cEulerAngleZ.GetValue() > -(M_PI / 18.0)) && (cEulerAngleZ.GetValue() <  (M_PI / 18.0)));
            }
         }
         return false;
      });
      GetSubState("approach_structure_far").AddExitTransition("approach_target_from_left");
      GetSubState("approach_structure_far").AddExitTransition("approach_target_from_right");
      GetSubState("approach_structure_far").AddExitTransition("approach_target_straight");

      /// Top level transitions ///
      AddExitTransition("approach_structure_far");
   }
};

#endif
