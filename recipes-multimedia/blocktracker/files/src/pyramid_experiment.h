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
#define LIFT_ACTUATOR_MIN_HEIGHT 3
#define LIFT_ACTUATOR_INC_HEIGHT 15
#define LIFT_ACTUATOR_BLOCK_HEIGHT 55
#define LIFT_ACTUATOR_BLOCK_PREATTACH_OFFSET 3

#define RF_UN_BLOCK_DETECT_THRES 2250
#define RF_LR_BLOCK_DETECT_THRES 3500
#define RF_FLR_BLOCK_DETECT_THRES 2500
#define RF_FLR_BLOCK_CONTACT_THRES 2750

#define OBSERVE_BLOCK_X_TARGET 0.000
#define OBSERVE_BLOCK_Z_TARGET 0.275
#define OBSERVE_BLOCK_XZ_THRES 0.050

#define PREAPPROACH_BLOCK_X_TARGET 0.000
#define PREAPPROACH_BLOCK_Z_TARGET 0.325
#define PREAPPROACH_BLOCK_XZ_THRES 0.025

#define PREPLACEMENT_BLOCK_X_TARGET 0.000
#define PREPLACEMENT_BLOCK_X_THRES 0.010

#define TAG_OFFSET_TARGET 0.700

#define APPROACH_BLOCK_X_FAIL_THRES 0.025

#define NEAR_APPROACH_TIMEOUT std::chrono::milliseconds(7500)
#define REVERSE_TIMEOUT_SHORT std::chrono::milliseconds(7500)
#define REVERSE_TIMEOUT_LONG std::chrono::milliseconds(10000)

#define BLOCK_TYPE_OFF "0"
#define BLOCK_TYPE_Q1  "1"
#define BLOCK_TYPE_Q2  "2"
#define BLOCK_TYPE_Q3  "3"
#define BLOCK_TYPE_Q4  "4"

#define BLOCK_SIDE_LENGTH 0.055
#define BLOCK_HALF_SIDE_LENGTH (BLOCK_SIDE_LENGTH / 2.0)

#define BASE_VELOCITY 30.0
#define BASE_XZ_GAIN 7.5

// when lift actuator is at zero
#define CAMERA_VERTICAL_OFFSET_MM 105

/************************************************************/
/*               Shared data for all states                 */
/************************************************************/

struct {
   /* pointers to sensors and actuators */
   CBlockDemo::SSensorData* Sensors = nullptr;
   CBlockDemo::SActuatorData* Actuators = nullptr;
   /* controllers */
   CPIDController TagApproachController = CPIDController(4.250,0.100,1.375,0.500);
   /* generic local data */
   unsigned int TrackedTargetId = 0;
   struct {
      argos::CVector3 Translation;
      argos::CQuaternion Rotation;
   } TrackedTargetLastObservation;
   unsigned int TrackedStructureId = 0;
   std::chrono::time_point<std::chrono::steady_clock> ElectromagnetSwitchOnTime;
   std::chrono::time_point<std::chrono::steady_clock> NearApproachStartTime;
   std::chrono::time_point<std::chrono::steady_clock> ReverseToFindTargetStartTime;
   bool TargetInRange = false;
   /* data specific to the pyramid experiment */
   ELedState NextLedStateToAssign = ELedState::OFF;
} Data;


/************************************************************/
/*            Common functions for all states               */
/************************************************************/

/**************** functions for transitions ****************/

bool SetTargetInRange() {
   Data.TargetInRange = true;
   return true;
}

bool ClearTargetInRange() {
   Data.TargetInRange = false;
   return true;
}

bool IsTargetInRange() {
   return Data.TargetInRange;
}

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

/**************** functions for statistics ****************/
double GetMedian(const std::list<uint16_t>& lst_data) { 
   std::vector<uint16_t> vecData(std::begin(lst_data), std::end(lst_data));
   size_t n = vecData.size() / 2;
   std::nth_element(std::begin(vecData), std::begin(vecData) + n, std::end(vecData));
   uint16_t vn = vecData[n];
   if(vecData.size() % 2 == 1) {
      return 1.0 * vn;
   }
   else {
      std::nth_element(std::begin(vecData), std::begin(vecData) + (n - 1), std::end(vecData));
      return 0.5 * (vn + vecData[n - 1]);
   }
}

ELedState GetBlockLedState(const SBlock& s_block) {
   std::map<ELedState, unsigned int> mapLedCounts = {
      std::make_pair(ELedState::OFF, GetLedCount(s_block, {ELedState::OFF})),
      std::make_pair(ELedState::Q1, GetLedCount(s_block, {ELedState::Q1})),
      std::make_pair(ELedState::Q2, GetLedCount(s_block, {ELedState::Q2})),
      std::make_pair(ELedState::Q3, GetLedCount(s_block, {ELedState::Q3})),
      std::make_pair(ELedState::Q4, GetLedCount(s_block, {ELedState::Q4})),
   };
   auto itMaxLedCount = std::max_element(std::begin(mapLedCounts), std::end(mapLedCounts), 
      [] (const std::pair<const ELedState, unsigned int>& c_pair_lhs,
          const std::pair<const ELedState, unsigned int>& c_pair_rhs) {
      return (c_pair_lhs.second < c_pair_rhs.second);
   });
   return ((itMaxLedCount != std::end(mapLedCounts)) ? itMaxLedCount->first : ELedState::OFF);
}

argos::CVector3 GetAdjBlockTranslation(const SBlock& s_block) {
   argos::CRadians cYZAngle = -argos::ATan2(s_block.Translation.GetY(), s_block.Translation.GetZ()) +
                               argos::CRadians::PI_OVER_FOUR;
   double fYZDistance = std::hypot(s_block.Translation.GetY(), s_block.Translation.GetZ());
   return argos::CVector3(fYZDistance * argos::Sin(cYZAngle),
                          s_block.Translation.GetX(),
                          fYZDistance * argos::Cos(cYZAngle));
}

/**************** functions for pyramid ****************/

unsigned int GetBlockLevel(const SBlock& s_block, unsigned int un_lift_actuator_position) {
   double fBlockPositionOffset = GetAdjBlockTranslation(s_block).GetZ();
   // TODO stop using millimeters, everything should be in SI units and stored with double-precision
   double fBlockPosition = ((CAMERA_VERTICAL_OFFSET_MM + un_lift_actuator_position) / 1000.0) - fBlockPositionOffset;
   double fBlockLevel = std::round(std::abs(fBlockPosition - BLOCK_HALF_SIDE_LENGTH) / BLOCK_SIDE_LENGTH);
   return static_cast<unsigned int>(fBlockLevel);
}

STarget::TConstListIterator FindPyramidTarget(const STarget::TList& t_list) {
   STarget::TConstListIterator itPyramidTarget = std::end(t_list);
   std::list<STarget::TConstListIterator> lstCandidateTargets;
   for(STarget::TConstListIterator it_target = std::begin(t_list); it_target != std::end(t_list); it_target++) {
      if(lstCandidateTargets.empty()) {
         lstCandidateTargets.push_front(it_target);
      }
      else {
         const SBlock& sBlock = it_target->Observations.front();
         STarget::TConstListIterator it_candidate_target = lstCandidateTargets.front();
         const SBlock& sCandidateBlock = it_candidate_target->Observations.front();
         double fDeltaX = GetAdjBlockTranslation(sCandidateBlock).GetX() - GetAdjBlockTranslation(sBlock).GetX();
         // block is closer to us than other block
         if(fDeltaX > BLOCK_HALF_SIDE_LENGTH) {
            lstCandidateTargets.clear();
            lstCandidateTargets.push_front(it_target);
         }
         // block is further away from us than other block
         else if(fDeltaX < -BLOCK_HALF_SIDE_LENGTH) {
         }
         // block is in the same row
         else {
            lstCandidateTargets.push_front(it_target);
         }
      }
   }
   for(STarget::TConstListIterator it_candidate_target : lstCandidateTargets) {
      if(itPyramidTarget == std::end(t_list)) {
         itPyramidTarget = it_candidate_target;
      }
      else {
         const SBlock& sBlockPyramidTarget = itPyramidTarget->Observations.front();
         const SBlock& sBlockOtherTarget = it_candidate_target->Observations.front();
         // since GetZ() is the distance from the camera position (always above visible blocks), the smaller value is higher up
         if(GetAdjBlockTranslation(sBlockOtherTarget).GetZ() < GetAdjBlockTranslation(sBlockPyramidTarget).GetZ()) {
            itPyramidTarget = it_candidate_target;
         }
      }
   }
   return itPyramidTarget;
}

/************************************************************/
/*               Common state definitions                   */
/************************************************************/

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
         auto tMinMaxPair = std::minmax_element(std::begin(Data.Sensors->ManipulatorModule.LiftActuator.Electromagnets.Charge),
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

class CStateAttachBlock : public CState {
public:
   CStateAttachBlock(const std::string& str_id) :
      CState(str_id, nullptr, nullptr, {
         CStateSetLiftActuatorPosition("init_lift_actuator_position", LIFT_ACTUATOR_MIN_HEIGHT + LIFT_ACTUATOR_BLOCK_PREATTACH_OFFSET),
         CStatePulseElectromagnets("generate_pre_alignment_pulse", std::chrono::milliseconds(500), CBlockDemo::EGripperFieldMode::CONSTRUCTIVE),
         CStateSetLiftActuatorPosition("lower_lift_actuator", LIFT_ACTUATOR_MIN_HEIGHT),
         CStatePulseElectromagnets("generate_attachment_pulse", std::chrono::milliseconds(1000), CBlockDemo::EGripperFieldMode::CONSTRUCTIVE),
      }) {
         AddTransition("init_lift_actuator_position","generate_pre_alignment_pulse");
         AddTransition("generate_pre_alignment_pulse", "lower_lift_actuator");
         AddTransition("lower_lift_actuator","generate_attachment_pulse");
         AddExitTransition("generate_attachment_pulse");
      }
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

class CStateSendNFCMessage : public CState {
public:
   CStateSendNFCMessage(const std::string& str_id, const std::string& str_data) :
      CState(str_id, [str_data] {
         Data.Actuators->ManipulatorModule.NFCInterface.OutboundMessage = str_data;
         Data.Actuators->ManipulatorModule.NFCInterface.UpdateReq = true;
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
                  std::abs(FindTagCornerFurthestToTheTop(s_block.Tags[0]).second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;
               double fTagOffsetBottom =
                  std::abs(FindTagCornerFurthestToTheBottom(s_block.Tags[0]).second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;
               fLeft *= std::max(1.000 - std::max(fTagOffsetTop, fTagOffsetBottom), 0.250);
               fRight *= std::max(1.000 - std::max(fTagOffsetTop, fTagOffsetBottom), 0.250);
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
                  std::abs(FindTagCornerFurthestToTheTop(s_block.Tags[0]).second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;
               double fTagOffsetBottom =
                  std::abs(FindTagCornerFurthestToTheBottom(s_block.Tags[0]).second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;
               fLeft *= std::max(1.000 - std::max(fTagOffsetTop, fTagOffsetBottom), 0.250);
               fRight *= std::max(1.000 - std::max(fTagOffsetTop, fTagOffsetBottom), 0.250);
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
                  std::abs(FindTagCornerFurthestToTheTop(s_block.Tags[0]).second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;
               double fTagOffsetBottom =
                  std::abs(FindTagCornerFurthestToTheBottom(s_block.Tags[0]).second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;
               fLeft *= std::max(1.000 - std::max(fTagOffsetTop, fTagOffsetBottom), 0.250);
               fRight *= std::max(1.000 - std::max(fTagOffsetTop, fTagOffsetBottom), 0.250);
            }
         }
         /* apply the approach velocity */
         SetVelocity(fLeft, fRight);
      }) {}
};

class CStateAlignWithTagOffset : public CState {
public:
   CStateAlignWithTagOffset(const std::string& str_id, double f_tag_offset_target,
                            std::function<const STag::TCoordinate&(const STag&)> fn_get_coordinate) :
      CState(str_id, [f_tag_offset_target, fn_get_coordinate] {
            /* default velocities, overwritten if target is detected */
            double fLeft = 0.000, fRight = 0.000;
            /* select tracked target */
            auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
            if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
               const SBlock& s_block = itTarget->Observations.front();
               double fTagOffset = (fn_get_coordinate(s_block.Tags[0]).first - IMAGE_SENSOR_HALF_WIDTH) / IMAGE_SENSOR_HALF_WIDTH;
               fLeft  = (fTagOffset - f_tag_offset_target) * BASE_VELOCITY;
               fRight = (f_tag_offset_target - fTagOffset) * BASE_VELOCITY;
            }
            SetVelocity(fLeft, fRight);
      }) {}
};

class CStateApproachTarget : public CState {
public:
   CStateApproachTarget(const std::string& str_id, double f_lift_actuator_min_height, double f_tag_offset_target, 
                        std::function<const STag::TCoordinate&(const STag&)> fn_get_coordinate) :
      CState(str_id, [f_lift_actuator_min_height, f_tag_offset_target, fn_get_coordinate] {
         // default velocities, overwritten if target is detected
         double fLeft = 0.000, fRight = 0.000;
         // select tracked target
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            /* update the last observation data */
            Data.TrackedTargetLastObservation.Rotation = s_block.Rotation;
            Data.TrackedTargetLastObservation.Translation = s_block.Translation;
            /* track the target by lowering the lift actuator position */
            TrackBlockViaLiftActuatorHeight(s_block,
                                            f_lift_actuator_min_height,
                                            Data.Actuators->ManipulatorModule.LiftActuator.Position.Value);
            /* calculate the steering variable */
            double fTagOffset = (fn_get_coordinate(s_block.Tags[0]).first - IMAGE_SENSOR_HALF_WIDTH) / IMAGE_SENSOR_HALF_WIDTH;
            double fOutput = Data.TagApproachController.Step(fTagOffset, f_tag_offset_target, Data.Sensors->Clock.Time);
            /* saturate the steering variable between 0 and 1 */
            fOutput = (fOutput > 0.750) ? 0.750 : ((fOutput < -0.750) ? -0.750 : fOutput);
            /* calculate the approach velocities */
            fRight = (1.000 + fOutput) * BASE_VELOCITY;
            fLeft  = (1.000 - fOutput) * BASE_VELOCITY;
            /* scale the speed by the lift actuator height error */
            double fTagOffsetTop =
               std::abs(FindTagCornerFurthestToTheTop(s_block.Tags[0]).second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;
            double fTagOffsetBottom =
               std::abs(FindTagCornerFurthestToTheBottom(s_block.Tags[0]).second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;
            fLeft *= std::max(1.000 - std::max(fTagOffsetTop, fTagOffsetBottom), 0.250);
            fRight *= std::max(1.000 - std::max(fTagOffsetTop, fTagOffsetBottom), 0.250);
         }
         /* apply the approach velocity */
         SetVelocity(fLeft, fRight);
      }) {}
};

class CStateApproachTargetFar : public CState {
public:
   CStateApproachTargetFar(const std::string& str_id, double f_tag_offset_target,
                           std::function<const STag::TCoordinate&(const STag&)> fn_get_coordinate) :
      CState(str_id, nullptr, nullptr, {
         /*** states (std::vector<CState> initializer list) ***/
         CStateAlignWithTagOffset("align_with_tag_offset", f_tag_offset_target, fn_get_coordinate),
         CStateApproachTarget("approach_target", LIFT_ACTUATOR_MIN_HEIGHT + LIFT_ACTUATOR_BLOCK_PREATTACH_OFFSET, f_tag_offset_target, fn_get_coordinate),
         // failure states
         CState("adjust_lift_actuator_height", [] {
            if(Data.Actuators->ManipulatorModule.LiftActuator.Position.Value < (LIFT_ACTUATOR_MAX_HEIGHT - LIFT_ACTUATOR_BLOCK_PREATTACH_OFFSET)) {
               Data.Actuators->ManipulatorModule.LiftActuator.Position.Value += LIFT_ACTUATOR_BLOCK_PREATTACH_OFFSET;
               Data.Actuators->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
            }
         }),
         CStateSetVelocity("set_reverse_velocity", -0.250 * BASE_VELOCITY, -0.250 * BASE_VELOCITY),
         CState("wait_for_target_or_timeout"),
      }) {
         /*** transitions (constructor body) ***/
         AddTransition("align_with_tag_offset", "approach_target", [f_tag_offset_target, fn_get_coordinate] {
            auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
            if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
               const SBlock& s_block = itTarget->Observations.front();
               double fTagOffset = (fn_get_coordinate(s_block.Tags[0]).first - IMAGE_SENSOR_HALF_WIDTH) / IMAGE_SENSOR_HALF_WIDTH;
               if(std::abs(fTagOffset - f_tag_offset_target) < 0.1) {         
                  Data.TagApproachController.Reset();
                  return true;
               }   
            }
            return false;
         });
         AddExitTransition("approach_target", [] {
            bool bTargetLost = IsTargetLost();
            bool bLiftActuatorAtBottom = 
               (Data.Actuators->ManipulatorModule.LiftActuator.Position.Value <= (LIFT_ACTUATOR_MIN_HEIGHT + LIFT_ACTUATOR_BLOCK_PREATTACH_OFFSET));
            bool bLastObservationInRange =
               (std::abs(Data.TrackedTargetLastObservation.Translation.GetX()) <= APPROACH_BLOCK_X_FAIL_THRES);
            if(bTargetLost && bLiftActuatorAtBottom && bLastObservationInRange) {
               SetTargetInRange();
               return true;
            }
            return false;
         });
         AddTransition("align_with_tag_offset", "set_reverse_velocity", IsTargetLost);
         AddTransition("approach_target", "set_reverse_velocity", IsTargetLost);
         AddTransition("set_reverse_velocity", "adjust_lift_actuator_height");
         // back off until target is re-acquired
         AddTransition("adjust_lift_actuator_height", "wait_for_target_or_timeout", [] {
            // reset timer for the reverse velocity search
            Data.ReverseToFindTargetStartTime = std::chrono::steady_clock::now();
            return true;
         });
         // try again
         AddTransition("wait_for_target_or_timeout", "align_with_tag_offset", IsNextTargetAcquired);
         // timer has expired
         AddExitTransition("wait_for_target_or_timeout", [] {
            if(Data.ReverseToFindTargetStartTime + REVERSE_TIMEOUT_SHORT < std::chrono::steady_clock::now()) {
               ClearTargetInRange();
               return true;
            }
            return false;
         });
      }
};

class CStateApproachTargetNear : public CState {
public:
   CStateApproachTargetNear(const std::string& str_id) :
      CState(str_id, nullptr, nullptr, {
         /*** states (std::vector<CState> initializer list) ***/
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
      }) {
         /*** transitions (constructor body) ***/
         AddTransition("set_deck_color", "lower_lift_actuator");
         AddTransition("lower_lift_actuator", "set_approach_velocity");
         AddTransition("set_approach_velocity", "wait_for_underneath_rf_or_timeout", [] {
            // reset timer for "wait_for_underneath_rf_or_timeout"
            Data.NearApproachStartTime = std::chrono::steady_clock::now();
            return true;
         });
         AddTransition("wait_for_underneath_rf_or_timeout", "wait_for_either_left_right_rf_or_timeout", [] {
            // block detected on the underneath rf
            if(Data.Sensors->ManipulatorModule.RangeFinders.Underneath > RF_UN_BLOCK_DETECT_THRES) {
               // reset timer for "wait_for_either_left_right_rf_or_timeout"
               Data.NearApproachStartTime = std::chrono::steady_clock::now();
               return true;
            }
            return false;
         });
         AddTransition("wait_for_either_left_right_rf_or_timeout", "set_zero_velocity", [] {
            // block detected on both the left & right rf
            return ((Data.Sensors->ManipulatorModule.RangeFinders.Left > RF_LR_BLOCK_DETECT_THRES) &&
                    (Data.Sensors->ManipulatorModule.RangeFinders.Right > RF_LR_BLOCK_DETECT_THRES));
         });
         AddTransition("wait_for_either_left_right_rf_or_timeout", "set_pivot_velocity", [] {
            // block detected on both the left & right rf
            return ((Data.Sensors->ManipulatorModule.RangeFinders.Left > RF_LR_BLOCK_DETECT_THRES) ||
                    (Data.Sensors->ManipulatorModule.RangeFinders.Right > RF_LR_BLOCK_DETECT_THRES));
         });
         AddTransition("set_pivot_velocity", "wait_for_both_left_right_rf_or_timeout", [] {
            // reset timer for "wait_for_both_left_right_rf_or_timeout"
            Data.NearApproachStartTime = std::chrono::steady_clock::now();
            return true;
         });
         AddTransition("wait_for_both_left_right_rf_or_timeout", "set_zero_velocity", [] {
            // block detected on both the left & right rf
            return ((Data.Sensors->ManipulatorModule.RangeFinders.Left > RF_LR_BLOCK_DETECT_THRES) &&
                    (Data.Sensors->ManipulatorModule.RangeFinders.Right > RF_LR_BLOCK_DETECT_THRES));
         });
         AddTransition("wait_for_underneath_rf_or_timeout", "set_zero_velocity", [] {
            if(Data.NearApproachStartTime + NEAR_APPROACH_TIMEOUT < std::chrono::steady_clock::now()) {
               ClearTargetInRange();
               return true;
            }
            return false;
         });
         AddTransition("wait_for_either_left_right_rf_or_timeout", "set_zero_velocity", [] {
            if(Data.NearApproachStartTime + NEAR_APPROACH_TIMEOUT < std::chrono::steady_clock::now()) {
               ClearTargetInRange();
               return true;
            }
            return false;
         });
         AddTransition("wait_for_both_left_right_rf_or_timeout", "set_zero_velocity", [] {
            if(Data.NearApproachStartTime + NEAR_APPROACH_TIMEOUT < std::chrono::steady_clock::now()) {
               ClearTargetInRange();
               return true;
            }
            return false;
         });
         AddExitTransition("set_zero_velocity");
      }
};

class CStatePickUpBlock : public CState {
public:
   CStatePickUpBlock(const std::string& str_id) :
      CState(str_id, nullptr, nullptr, {
         /*** states (std::vector<CState> initializer list) ***/
         CStateSetLedColors("set_deck_color_green", CBlockDemo::EColor::GREEN),
         CStateMoveToTargetXZ("align_with_block", PREAPPROACH_BLOCK_X_TARGET, PREAPPROACH_BLOCK_Z_TARGET, false),
         CStateApproachTargetFar("approach_block_from_left", TAG_OFFSET_TARGET, FindTagCornerFurthestToTheRight),
         CStateApproachTargetFar("approach_block_from_right", -TAG_OFFSET_TARGET, FindTagCornerFurthestToTheLeft),
         CStateApproachTargetFar("approach_block_straight", 0.000, GetTagCenter),
         CStateSetLedColors("set_deck_color_red", CBlockDemo::EColor::RED),
         CStateApproachTargetNear("approach_block_near"),
         CStateSetVelocity("set_zero_velocity", 0.000, 0.000),
         CStateAttachBlock("attach_block_to_end_effector"),
         CStateSetLiftActuatorPosition("set_lift_actuator_test_height", LIFT_ACTUATOR_MIN_HEIGHT + LIFT_ACTUATOR_BLOCK_HEIGHT),
         // failure states
         CStateSetVelocity("set_reverse_velocity", -0.250 * BASE_VELOCITY, -0.250 * BASE_VELOCITY),
         CStateSetLiftActuatorPosition("raise_lift_actuator", LIFT_ACTUATOR_MAX_HEIGHT),
      }) {
         /*** transitions (constructor body) ***/
         // select approach direction
         AddTransition("set_deck_color_green", "align_with_block");
         AddTransition("align_with_block", "approach_block_from_left", [] {
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
         AddTransition("align_with_block", "approach_block_from_right", [] {
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
         AddTransition("align_with_block", "approach_block_straight", [] {
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
         // check if target is in range and perform the near block approach
         AddTransition("approach_block_from_left", "set_deck_color_red", IsTargetInRange);
         AddTransition("approach_block_from_right", "set_deck_color_red", IsTargetInRange);
         AddTransition("approach_block_straight", "set_deck_color_red", IsTargetInRange);
         AddTransition("set_deck_color_red", "approach_block_near");
         AddTransition("approach_block_near", "set_zero_velocity", IsTargetInRange);
         AddTransition("set_zero_velocity", "attach_block_to_end_effector");
         // test if the block attached correctly to the end effector
         AddTransition("attach_block_to_end_effector", "set_lift_actuator_test_height");
         AddExitTransition("set_lift_actuator_test_height", [] {
            if(Data.Sensors->ManipulatorModule.RangeFinders.Underneath > RF_UN_BLOCK_DETECT_THRES) {
               SetTargetInRange();
               return true;
            }
            return false;
         });
         // failure transitions
         AddTransition("align_with_block", "set_reverse_velocity", IsTargetLost);
         AddTransition("approach_block_from_left", "set_reverse_velocity");
         AddTransition("approach_block_from_right", "set_reverse_velocity");
         AddTransition("approach_block_straight", "set_reverse_velocity");
         AddTransition("approach_block_near", "set_reverse_velocity");
         AddTransition("set_lift_actuator_test_height", "set_reverse_velocity");
         AddTransition("set_reverse_velocity", "raise_lift_actuator");
         AddExitTransition("raise_lift_actuator", ClearTargetInRange);
      }
};

class CStateApproachStructure : public CState {
public:
   CStateApproachStructure(const std::string& str_id, double f_tag_offset_target,
                           std::function<const STag::TCoordinate&(const STag&)> fn_get_coordinate) :
      CState(str_id, nullptr, nullptr, {
         /*** states (std::vector<CState> initializer list) ***/
         CStateAlignWithTagOffset("align_with_tag_offset", f_tag_offset_target, fn_get_coordinate),
         CStateApproachTarget("approach_target", LIFT_ACTUATOR_MIN_HEIGHT + (0.5 * LIFT_ACTUATOR_BLOCK_HEIGHT), f_tag_offset_target, fn_get_coordinate),

         // failure states
         CState("adjust_lift_actuator_height", [] {
            if(Data.Actuators->ManipulatorModule.LiftActuator.Position.Value < (LIFT_ACTUATOR_MAX_HEIGHT - LIFT_ACTUATOR_BLOCK_PREATTACH_OFFSET)) {
               Data.Actuators->ManipulatorModule.LiftActuator.Position.Value += LIFT_ACTUATOR_BLOCK_PREATTACH_OFFSET;
               Data.Actuators->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
            }
         }),
         CStateSetVelocity("set_reverse_velocity", -0.250 * BASE_VELOCITY, -0.250 * BASE_VELOCITY),
         CState("wait_for_target_or_timeout"),
      }) {
         /*** transitions (constructor body) ***/
         AddTransition("align_with_tag_offset", "approach_target", [f_tag_offset_target, fn_get_coordinate] {
            auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
            if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
               const SBlock& s_block = itTarget->Observations.front();
               double fTagOffset = (fn_get_coordinate(s_block.Tags[0]).first - IMAGE_SENSOR_HALF_WIDTH) / IMAGE_SENSOR_HALF_WIDTH;
               if(std::abs(fTagOffset - f_tag_offset_target) < 0.1) {         
                  Data.TagApproachController.Reset();
                  return true;
               }
            }
            return false;
         });

         AddExitTransition("approach_target", [] {
            auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
            bool bTargetInRange = (itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) && 
                                  (GetAdjBlockTranslation(itTarget->Observations.front()).GetX() < 0.085);
            bool bTargetLost = IsTargetLost();
            bool bLiftActuatorAtBottom =
               (Data.Actuators->ManipulatorModule.LiftActuator.Position.Value <= (LIFT_ACTUATOR_MIN_HEIGHT + (0.5 * LIFT_ACTUATOR_BLOCK_HEIGHT)));
            if(bTargetInRange || (bTargetLost && bLiftActuatorAtBottom)) {
               SetTargetInRange();
               return true;
            }
            return false;
         });

         AddTransition("align_with_tag_offset", "set_reverse_velocity", IsTargetLost);
         AddTransition("approach_target", "set_reverse_velocity", IsTargetLost);
         AddTransition("set_reverse_velocity", "adjust_lift_actuator_height");
         // back off until target is re-acquired
         AddTransition("adjust_lift_actuator_height", "wait_for_target_or_timeout", [] {
            // reset timer for the reverse velocity search
            Data.ReverseToFindTargetStartTime = std::chrono::steady_clock::now();
            return true;
         });
         // try again
         AddTransition("wait_for_target_or_timeout", "align_with_tag_offset", IsNextTargetAcquired);
         // timer has expired
         AddExitTransition("wait_for_target_or_timeout", [] {
            if(Data.ReverseToFindTargetStartTime + REVERSE_TIMEOUT_SHORT < std::chrono::steady_clock::now()) {
               ClearTargetInRange();
               return true;
            }
            return false;
         });
      }
};

class CStatePlaceBlock : public CState {
public:
   CStatePlaceBlock(const std::string& str_id) :
      CState(str_id, nullptr, nullptr, {
         /*** states (std::vector<CState> initializer list) ***/
         CStateSetLedColors("set_deck_color_green", CBlockDemo::EColor::GREEN),
         CStateMoveToTargetXZ("prealign_with_structure", PREAPPROACH_BLOCK_X_TARGET, PREAPPROACH_BLOCK_Z_TARGET, true),
         CStateApproachStructure("approach_structure_from_left", TAG_OFFSET_TARGET, FindTagCornerFurthestToTheRight),
         CStateApproachStructure("approach_structure_from_right", -TAG_OFFSET_TARGET, FindTagCornerFurthestToTheLeft),
         CStateApproachStructure("approach_structure_straight", 0.000, GetTagCenter),
         CStateSetVelocity("set_reverse_velocity", -0.250 * BASE_VELOCITY, -0.250 * BASE_VELOCITY),
         CState("wait_for_target"),
         CStateMoveToTargetX("align_with_structure", PREPLACEMENT_BLOCK_X_TARGET, false),
         CStateSetVelocity("set_zero_velocity", 0.000, 0.000),
         CStateSetLedColors("set_deck_color_red", CBlockDemo::EColor::RED),
         CStateSetLiftActuatorPosition("set_lift_actuator_base_height", LIFT_ACTUATOR_MIN_HEIGHT + (0.5 * LIFT_ACTUATOR_BLOCK_HEIGHT)),
         // if no targets place block, otherwise, if targets check led colors
         CState("increment_lift_actuator_height", [] {
            Data.Actuators->ManipulatorModule.LiftActuator.Position.Value += LIFT_ACTUATOR_BLOCK_HEIGHT;
            /* saturate a max height */
            if(Data.Actuators->ManipulatorModule.LiftActuator.Position.Value > LIFT_ACTUATOR_MAX_HEIGHT) {
               Data.Actuators->ManipulatorModule.LiftActuator.Position.Value = LIFT_ACTUATOR_MAX_HEIGHT;
            }
            Data.Actuators->ManipulatorModule.LiftActuator.Position.UpdateReq = true;

         }),
         CState("wait_for_lift_actuator"),
         CStateSetLiftActuatorPosition("lower_lift_actuator", LIFT_ACTUATOR_MIN_HEIGHT + (0.5 * LIFT_ACTUATOR_BLOCK_HEIGHT)),
         // correct height

         // "set_block_led_state"
         CState("set_approach_velocity", [] {
            switch(Data.NextLedStateToAssign) {
            case ELedState::OFF:
               Data.Actuators->ManipulatorModule.NFCInterface.OutboundMessage = BLOCK_TYPE_OFF;
               break;
            case ELedState::Q1:
               Data.Actuators->ManipulatorModule.NFCInterface.OutboundMessage = BLOCK_TYPE_Q1;
               break;
            case ELedState::Q2:
               Data.Actuators->ManipulatorModule.NFCInterface.OutboundMessage = BLOCK_TYPE_Q2;
               break;
            case ELedState::Q3:
               Data.Actuators->ManipulatorModule.NFCInterface.OutboundMessage = BLOCK_TYPE_Q3;
               break;
            case ELedState::Q4:
               Data.Actuators->ManipulatorModule.NFCInterface.OutboundMessage = BLOCK_TYPE_Q4;
               break;
            }
            Data.Actuators->ManipulatorModule.NFCInterface.UpdateReq = true;
         }),
         /*CState("set_approach_velocity", [] {
            double fLastObservationX = Data.TrackedTargetLastObservation.Translation.GetX();
            double fLeft = BASE_VELOCITY * (1.000 + (fLastObservationX * BASE_XZ_GAIN));
            double fRight = BASE_VELOCITY * (1.000 - (fLastObservationX * BASE_XZ_GAIN));
            SetVelocity(fLeft, fRight);
         }),*/




         CState("wait_for_either_front_rf_or_timeout"),
         CState("set_pivot_velocity", [] {
            bool bRfBlockDetectedLeft = (GetMedian(Data.Sensors->RangeFinders[5]) > RF_FLR_BLOCK_DETECT_THRES);
            bool bRfBlockDetectedRight = (GetMedian(Data.Sensors->RangeFinders[6]) > RF_FLR_BLOCK_DETECT_THRES);
            // pivot the robot towards the other sensor
            double fLeft = (bRfBlockDetectedLeft ? 0.250 : 0.500) * BASE_VELOCITY;
            double fRight = (bRfBlockDetectedRight ? 0.250 : 0.500) * BASE_VELOCITY;
            // apply the velocity
            SetVelocity(fLeft, fRight);
         }),
         CState("wait_for_both_front_rfs_or_timeout"),
         CStateSetVelocity("set_reverse_velocity_for_detachment", -0.500 * BASE_VELOCITY, -0.500 * BASE_VELOCITY),
         CStatePulseElectromagnets("deattach_block_from_end_effector", std::chrono::milliseconds(1000), CBlockDemo::EGripperFieldMode::DESTRUCTIVE),
         // Failure / completion states

         
      }) {
         /*** transitions (constructor body) ***/
         // select approach direction
         AddTransition("set_deck_color_green", "prealign_with_structure");
         AddTransition("prealign_with_structure", "approach_structure_from_left", [] {
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
         AddTransition("prealign_with_structure", "approach_structure_from_right", [] {
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
         AddTransition("prealign_with_structure", "approach_structure_straight", [] {
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
         /* tracking lost - exit */
         AddExitTransition("prealign_with_structure", IsTargetLost);

         AddExitTransition("approach_structure_from_left", IsTargetLost);
         AddExitTransition("approach_structure_from_right", IsTargetLost);
         AddExitTransition("approach_structure_straight", IsTargetLost);
         AddTransition("approach_structure_from_left", "align_with_structure");
         AddTransition("approach_structure_from_right", "align_with_structure");
         AddTransition("approach_structure_straight", "align_with_structure");
         AddTransition("set_reverse_velocity", "wait_for_target");
         AddTransition("wait_for_target", "align_with_structure", IsNextTargetAcquired);
   
         AddTransition("align_with_structure", "set_zero_velocity", [] {
            auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
            if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
               const SBlock& s_block = itTarget->Observations.front();
               if(std::abs(s_block.Translation.GetX() - PREPLACEMENT_BLOCK_X_TARGET) < PREPLACEMENT_BLOCK_X_THRES) {
                  argos::CRadians cEulerAngleZ, cEulerAngleY, cEulerAngleX;
                  s_block.Rotation.ToEulerAngles(cEulerAngleZ, cEulerAngleY, cEulerAngleX);
                  return ((cEulerAngleZ.GetValue() > -(M_PI / 18.0)) && (cEulerAngleZ.GetValue() < (M_PI / 18.0)));
               }
            }
            return false;
         });
         // loop back if alignment is to far out (more than +/- 10 deg)
         AddTransition("align_with_structure", "prealign_with_structure", [] {
            auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
            if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
               const SBlock& s_block = itTarget->Observations.front();
               if(std::abs(s_block.Translation.GetX() - PREPLACEMENT_BLOCK_X_TARGET) < PREPLACEMENT_BLOCK_X_THRES) {
                  return true;
               }
            }
            return false;
         });
         // retry if target is lost
         AddTransition("align_with_structure", "set_reverse_velocity", IsTargetLost);

         AddTransition("set_zero_velocity", "set_deck_color_red");
         AddTransition("set_deck_color_red", "set_lift_actuator_base_height");
         AddTransition("set_lift_actuator_base_height", "wait_for_lift_actuator");
         AddTransition("increment_lift_actuator_height", "wait_for_lift_actuator");

         // extend structure vertically
         AddTransition("wait_for_lift_actuator", "increment_lift_actuator_height", [] {
            if(Data.Sensors->ManipulatorModule.LiftActuator.State == CBlockDemo::ELiftActuatorSystemState::INACTIVE) {
               if((!IsTargetLost()) || IsNextTargetAcquired()) {
                  auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
                  const SBlock& s_block = itTarget->Observations.front();
                  ELedState eBlockLedState = GetBlockLedState(s_block);
                  unsigned int unBlockLevel = GetBlockLevel(s_block, Data.Sensors->ManipulatorModule.LiftActuator.EndEffector.Position);
                  if((eBlockLedState == ELedState::Q3) && (unBlockLevel < 2u)) {
                     Data.NextLedStateToAssign = ELedState::Q3;
                     return true;
                  }
                  if((eBlockLedState == ELedState::Q2) && (unBlockLevel < 1u)) {
                     Data.NextLedStateToAssign = ELedState::Q2;
                     return true;
                  }
               }              
            }
            return false;
         });
         // extend structure horizontally
         AddTransition("wait_for_lift_actuator", "lower_lift_actuator", [] {
            if(Data.Sensors->ManipulatorModule.LiftActuator.State == CBlockDemo::ELiftActuatorSystemState::INACTIVE) {
               auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
               if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
                  const SBlock& s_block = itTarget->Observations.front();
                  ELedState eBlockLedState = GetBlockLedState(s_block);
                  unsigned int unBlockLevel = GetBlockLevel(s_block, Data.Sensors->ManipulatorModule.LiftActuator.EndEffector.Position);
                  if((eBlockLedState == ELedState::Q3) && (unBlockLevel == 2u)) {
                     Data.NextLedStateToAssign = ELedState::Q2;
                     return true;
                  }
                  if((eBlockLedState == ELedState::Q2) && (unBlockLevel == 1u)) {
                     Data.NextLedStateToAssign = ELedState::Q1;
                     return true;
                  }
               }
            }
            return false;
         });
         AddTransition("lower_lift_actuator", "set_approach_velocity");
         AddTransition("wait_for_lift_actuator", "set_approach_velocity", [] {
            if(Data.Sensors->ManipulatorModule.LiftActuator.State == CBlockDemo::ELiftActuatorSystemState::INACTIVE) {
               return IsTargetLost();
            }
            return false;
         });

         AddTransition("set_approach_velocity", "wait_for_either_front_rf_or_timeout", [] {
            // reset timer for "wait_for_either_front_rf_or_timeout"
            Data.NearApproachStartTime = std::chrono::steady_clock::now();
            return true;
         });
         AddTransition("wait_for_either_front_rf_or_timeout", "set_reverse_velocity_for_detachment", [] {
            return ((GetMedian(Data.Sensors->RangeFinders[5]) > RF_FLR_BLOCK_CONTACT_THRES) ||
                    (GetMedian(Data.Sensors->RangeFinders[6]) > RF_FLR_BLOCK_CONTACT_THRES));

         });
         AddTransition("wait_for_either_front_rf_or_timeout", "set_pivot_velocity", [] {
            return ((GetMedian(Data.Sensors->RangeFinders[5]) > RF_FLR_BLOCK_DETECT_THRES) ||
                    (GetMedian(Data.Sensors->RangeFinders[6]) > RF_FLR_BLOCK_DETECT_THRES));
         });
         AddTransition("set_pivot_velocity", "wait_for_both_front_rfs_or_timeout", [] {
            // reset timer for "wait_for_both_front_rfs_or_timeout"
            Data.NearApproachStartTime = std::chrono::steady_clock::now();
            return true;
         });
         AddTransition("wait_for_both_front_rfs_or_timeout", "set_reverse_velocity_for_detachment", [] {
            return ((GetMedian(Data.Sensors->RangeFinders[5]) > RF_FLR_BLOCK_CONTACT_THRES) ||
                    (GetMedian(Data.Sensors->RangeFinders[6]) > RF_FLR_BLOCK_CONTACT_THRES));
         });
         AddTransition("set_reverse_velocity_for_detachment", "deattach_block_from_end_effector");
         AddExitTransition("deattach_block_from_end_effector");

         // Error transitions
         AddExitTransition("wait_for_either_front_rf_or_timeout", [] {
            if(Data.NearApproachStartTime + NEAR_APPROACH_TIMEOUT < std::chrono::steady_clock::now()) {
               return true;
            }
            return false;
         });
         AddExitTransition("wait_for_both_front_rfs_or_timeout", [] {
            if(Data.NearApproachStartTime + NEAR_APPROACH_TIMEOUT < std::chrono::steady_clock::now()) {
               return true;
            }
            return false;
         });

      }
};

/************************************************************/
/*             Main state machine definition                */
/************************************************************/
class CFiniteStateMachine : public CState {
public:
   CFiniteStateMachine() :
      CState("top_level_state", nullptr, nullptr, {
         // TODO: Remove - testing
         //CState("test"),
         CState("search_for_unused_block", nullptr, nullptr, {
            CStateSetLedColors("set_deck_color", CBlockDemo::EColor::BLUE),
            CStateSetLiftActuatorPosition("raise_lift_actuator", LIFT_ACTUATOR_MAX_HEIGHT),
            CStateSetVelocity("set_search_velocity", BASE_VELOCITY * 0.500, -BASE_VELOCITY * 0.500),
            CState("wait_for_next_target"),
            CStateMoveToTargetXZ("align_with_block", PREAPPROACH_BLOCK_X_TARGET, PREAPPROACH_BLOCK_Z_TARGET, false),
         }),
         CStatePickUpBlock("pick_up_unused_block"),
         // Assign the transport color to the block
         CStateSendNFCMessage("configure_block_for_transport", BLOCK_TYPE_Q4),
         CState("search_for_structure", nullptr, nullptr, {
            CStateSetLedColors("set_deck_color", CBlockDemo::EColor::BLUE),
            CStateSetLiftActuatorPosition("raise_lift_actuator", LIFT_ACTUATOR_MAX_HEIGHT),
            CStateSetVelocity("set_search_velocity", BASE_VELOCITY * 0.500, -BASE_VELOCITY * 0.500),
            CState("wait_for_next_target"),
            CStateMoveToTargetXZ("align_with_block", OBSERVE_BLOCK_X_TARGET, OBSERVE_BLOCK_Z_TARGET, false), // initial check if this is a seed block / structure
            // select closest ground level target to robot
         }),
         CStatePlaceBlock("place_block_into_structure"),
         CStateSetVelocity("set_reverse_velocity", BASE_VELOCITY * -0.250, -BASE_VELOCITY * 0.250),
         CStateSetLiftActuatorPosition("raise_lift_actuator", LIFT_ACTUATOR_MAX_HEIGHT),
         CState("wait_for_next_target"),
         CStateMoveToTargetXZ("align_with_target", PREAPPROACH_BLOCK_X_TARGET, PREAPPROACH_BLOCK_Z_TARGET, false), // initial check if this is a seed block / structure
      }) {

      // TODO: Remove - testing
      //AddTransition("test", "configure_block_for_transport");

      /**************** search_for_unused_block transitions ****************/
      GetSubState("search_for_unused_block").AddTransition("set_deck_color", "raise_lift_actuator");
      GetSubState("search_for_unused_block").AddTransition("raise_lift_actuator", "set_search_velocity");
      GetSubState("search_for_unused_block").AddTransition("set_search_velocity","wait_for_next_target");
      GetSubState("search_for_unused_block").AddTransition("wait_for_next_target", "align_with_block", IsNextTargetAcquired);
      /* keep searching if the target was lost */
      GetSubState("search_for_unused_block").AddTransition("align_with_block", "set_search_velocity", IsTargetLost);
      /* unused block found - exit */
      GetSubState("search_for_unused_block").AddExitTransition("align_with_block", [] {
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
      // keep searching if block belongs to a structure */
      GetSubState("search_for_unused_block").AddTransition("align_with_block", "set_search_velocity", [] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if((std::abs(s_block.Translation.GetX() - PREAPPROACH_BLOCK_X_TARGET) < PREAPPROACH_BLOCK_XZ_THRES) &&
               (std::abs(s_block.Translation.GetZ() - PREAPPROACH_BLOCK_Z_TARGET) < PREAPPROACH_BLOCK_XZ_THRES)) {
               // search all structures, does it belong to any */
               for(const SStructure& s_structure : Data.Sensors->ImageSensor.Detections.Structures) {
                  if(std::find(std::begin(s_structure.Members), std::end(s_structure.Members), itTarget) != std::end(s_structure.Members)) {
                     // is the size of the structure not 1?
                     return (s_structure.Members.size() != 1);
                  }
               }
            }
         }
         return false;
      });

      /// Top level transitions ///
      AddTransition("search_for_unused_block", "pick_up_unused_block");
      AddTransition("pick_up_unused_block", "configure_block_for_transport", [] {
return (Data.Sensors->ManipulatorModule.RangeFinders.Underneath > RF_UN_BLOCK_DETECT_THRES);
});
      AddTransition("configure_block_for_transport", "search_for_structure");

      /**************** search_for_structure transitions ****************/
      GetSubState("search_for_structure").AddTransition("set_deck_color", "raise_lift_actuator");
      GetSubState("search_for_structure").AddTransition("raise_lift_actuator", "set_search_velocity");
      GetSubState("search_for_structure").AddTransition("set_search_velocity", "wait_for_next_target");
      GetSubState("search_for_structure").AddTransition("wait_for_next_target", "align_with_block", IsNextTargetAcquired);
      GetSubState("search_for_structure").AddExitTransition("align_with_block", [] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if((std::abs(s_block.Translation.GetX() - OBSERVE_BLOCK_X_TARGET) < OBSERVE_BLOCK_XZ_THRES) &&
               (std::abs(s_block.Translation.GetZ() - OBSERVE_BLOCK_Z_TARGET) < OBSERVE_BLOCK_XZ_THRES)) {
               // search all structures, does it belong to any
               for(const SStructure& s_structure : Data.Sensors->ImageSensor.Detections.Structures) {
                  if(std::find(std::begin(s_structure.Members), std::end(s_structure.Members), itTarget) != std::end(s_structure.Members)) {
                     if(s_structure.Members.size() == 1) {
                        /* this target is a seed block */
                        return (GetLedCount(s_block, {ELedState::Q3}) > 2);
                     }
                     else {
                        /* this target is the correct target in a partially completed pyramid */
                        return (itTarget == FindPyramidTarget(Data.Sensors->ImageSensor.Detections.Targets));
                     }
                  }
               }
            }
         }
         return false;
      });
      // keep searching if block does not belong to a structure / is the seed block
      GetSubState("search_for_structure").AddTransition("align_with_block", "set_search_velocity", [] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if((std::abs(s_block.Translation.GetX() - OBSERVE_BLOCK_X_TARGET) < OBSERVE_BLOCK_XZ_THRES) &&
               (std::abs(s_block.Translation.GetZ() - OBSERVE_BLOCK_Z_TARGET) < OBSERVE_BLOCK_XZ_THRES)) {
               // if the above transition did not occur, then this is not our target
               // if we cannot switch targets, set_search_velocity
               return !IsNextTargetAcquired();
            }
         }
         return false;
      });
      // keep searching if target is lost
      GetSubState("search_for_structure").AddTransition("align_with_block", "set_search_velocity", IsTargetLost);

      /// Top level transitions ///
      AddTransition("search_for_structure", "place_block_into_structure");
      // Error handling
      AddTransition("pick_up_unused_block", "set_reverse_velocity");
      AddTransition("place_block_into_structure", "set_reverse_velocity");
      AddTransition("set_reverse_velocity", "raise_lift_actuator");
      AddTransition("raise_lift_actuator", "wait_for_next_target");
      AddTransition("wait_for_next_target", "align_with_target", IsNextTargetAcquired);
      AddTransition("align_with_target", "search_for_structure", [] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if(std::abs(s_block.Translation.GetZ() - PREAPPROACH_BLOCK_Z_TARGET) < PREAPPROACH_BLOCK_XZ_THRES) {
               // robot is laden - search for structure
               return (Data.Sensors->ManipulatorModule.RangeFinders.Underneath > RF_UN_BLOCK_DETECT_THRES);
            }
         }
         else {
            // target lost, try continue
            return (Data.Sensors->ManipulatorModule.RangeFinders.Underneath > RF_UN_BLOCK_DETECT_THRES);
         }
         return false;
      });
      AddTransition("align_with_target", "search_for_unused_block", [] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if(std::abs(s_block.Translation.GetZ() - PREAPPROACH_BLOCK_Z_TARGET) < PREAPPROACH_BLOCK_XZ_THRES) {
               // robot is not laden, search for unused block
               return !(Data.Sensors->ManipulatorModule.RangeFinders.Underneath > RF_UN_BLOCK_DETECT_THRES);
            }
         }
         else {
            // target lost, try continue
            return !(Data.Sensors->ManipulatorModule.RangeFinders.Underneath > RF_UN_BLOCK_DETECT_THRES);
         }
         return false;
      });
   }
};

#endif
