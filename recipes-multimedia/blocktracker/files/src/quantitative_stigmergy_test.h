#ifndef MANIPULATOR_TESTING_TASK_H
#define MANIPULATOR_TESTING_TASK_H

#include "state.h"
#include "block_demo.h"

#include <iostream>

#define IMAGE_SENSOR_WIDTH 640.0
#define IMAGE_SENSOR_HALF_WIDTH (IMAGE_SENSOR_WIDTH / 2.0)
#define IMAGE_SENSOR_HEIGHT 360.0
#define IMAGE_SENSOR_HALF_HEIGHT (IMAGE_SENSOR_HEIGHT / 2.0)

#define LIFT_ACTUATOR_MAX_HEIGHT 140
#define LIFT_ACTUATOR_MIN_HEIGHT 3
#define LIFT_ACTUATOR_INC_HEIGHT 15

#define BLOCK_PRE_ATTACH_OFFSET 5u

#define PREAPPROACH_BLOCK_X_TARGET 0.000
#define PREAPPROACH_BLOCK_Z_TARGET 0.325
#define PREAPPROACH_BLOCK_XZ_THRES 0.010

#define TAG_CENTERED_LOWER_THRES (0.40 * IMAGE_SENSOR_WIDTH)
#define TAG_CENTERED_UPPER_THRES (0.60 * IMAGE_SENSOR_WIDTH)

#define BASE_VELOCITY 30
#define BASE_XZ_GAIN 5.0

#define MTT_BLOCK_SEP_THRESHOLD 0.065

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

/**************** functions for locating targets ****************/
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

/**************** functions for transitions ****************/
bool IsTargetLost() {
   auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);
   if(itTarget == std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
      Data.TrackedTargetId = 0;
      return true;
   }
   return false;
}

/**************** functions for locating corners ****************/
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

TCornerConstIterator FindTagCornerFurthestToTheBottom(const SBlock& s_block) {
   return std::max_element(std::begin(s_block.Tags.front().Corners),
                           std::end(s_block.Tags.front().Corners),
                           [] (const std::pair<double, double>& c_pair_lhs, const std::pair<double, double>& c_pair_rhs) {
                              return c_pair_lhs.second < c_pair_rhs.second;
                           });
}

TCornerConstIterator FindTagCornerFurthestToTheTop(const SBlock& s_block) {
   return std::min_element(std::begin(s_block.Tags.front().Corners),
                           std::end(s_block.Tags.front().Corners),
                           [] (const std::pair<double, double>& c_pair_lhs, const std::pair<double, double>& c_pair_rhs) {
                              return c_pair_lhs.second < c_pair_rhs.second;
                           });
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
   CStateMoveToTargetXZ(const std::string& str_id, double f_x_target, double f_z_target, bool b_track_via_lift_actuator) :
      CState(str_id, [f_x_target, f_z_target, b_track_via_lift_actuator] {
         /* default velocities, overwritten if target is detected */
         double fLeft = 0.0, fRight = 0.0;
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
               TrackBlockViaLiftActuatorHeight(s_block, LIFT_ACTUATOR_MIN_HEIGHT + BLOCK_PRE_ATTACH_OFFSET);
               /* adjust speed if the tag is falling out of the frame */
               double fTagOffsetTop = 
                  std::abs(FindTagCornerFurthestToTheTop(s_block)->second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;  
               double fTagOffsetBottom = 
                  std::abs(FindTagCornerFurthestToTheBottom(s_block)->second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;
               fLeft *= (1.0 - std::max(fTagOffsetTop, fTagOffsetBottom));
               fRight *= (1.0 - std::max(fTagOffsetTop, fTagOffsetBottom));
            }
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
   CStateMoveToTargetX(const std::string& str_id, double f_x_target, bool b_track_via_lift_actuator) :
      CState(str_id, [f_x_target, b_track_via_lift_actuator] {
         /* default velocities, overwritten if target is detected */
         double fLeft = 0.0, fRight = 0.0;
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
               TrackBlockViaLiftActuatorHeight(s_block, LIFT_ACTUATOR_MIN_HEIGHT + BLOCK_PRE_ATTACH_OFFSET);
               /* adjust speed if the tag is falling out of the frame */
               double fTagOffsetTop = 
                  std::abs(FindTagCornerFurthestToTheTop(s_block)->second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;  
               double fTagOffsetBottom = 
                  std::abs(FindTagCornerFurthestToTheBottom(s_block)->second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;
               fLeft *= (1.0 - std::max(fTagOffsetTop, fTagOffsetBottom));
               fRight *= (1.0 - std::max(fTagOffsetTop, fTagOffsetBottom));
            }
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
   CStateMoveToTargetZ(const std::string& str_id, double f_z_target, bool b_track_via_lift_actuator) :
      CState(str_id, [f_z_target, b_track_via_lift_actuator] {
         /* default velocities, overwritten if target is detected */
         double fLeft = 0.0, fRight = 0.0;
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
               TrackBlockViaLiftActuatorHeight(s_block, LIFT_ACTUATOR_MIN_HEIGHT + BLOCK_PRE_ATTACH_OFFSET);
               /* adjust speed if the tag is falling out of the frame */
               double fTagOffsetTop = 
                  std::abs(FindTagCornerFurthestToTheTop(s_block)->second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;  
               double fTagOffsetBottom = 
                  std::abs(FindTagCornerFurthestToTheBottom(s_block)->second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;
               fLeft *= (1.0 - std::max(fTagOffsetTop, fTagOffsetBottom));
               fRight *= (1.0 - std::max(fTagOffsetTop, fTagOffsetBottom));
            }
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
         CStateSetLiftActuatorPosition("init_lift_actuator_position", LIFT_ACTUATOR_MIN_HEIGHT + BLOCK_PRE_ATTACH_OFFSET),
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

class CStateApproachTargetFromLeft : public CState {
public:
   CStateApproachTargetFromLeft(const std::string& str_id, double f_tag_offset_target = 0.9) :
      CState(str_id, nullptr, nullptr, {
         CState("pre_approach_target", [f_tag_offset_target] {
            /* default velocities, overwritten if target is detected */
            double fLeft = 0.0, fRight = 0.0, fLiftActuatorHeightError = 0.0;
            /* select tracked target */
            auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);                         
            if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
               const SBlock& s_block = itTarget->Observations.front();
               /* track the target using lift actuator position */
               TrackBlockViaLiftActuatorHeight(s_block, LIFT_ACTUATOR_MIN_HEIGHT + BLOCK_PRE_ATTACH_OFFSET);
               
               double fTagOffset = (FindTagCornerFurthestToTheRight(s_block)->first - IMAGE_SENSOR_HALF_WIDTH) / IMAGE_SENSOR_HALF_WIDTH;
               /* calculate the approach velocity */
               fRight = (f_tag_offset_target - fTagOffset) * BASE_VELOCITY;
               fLeft  = (fTagOffset - f_tag_offset_target) * BASE_VELOCITY;
               /* debug info */
               auto nFrameIdx = std::chrono::duration_cast<std::chrono::milliseconds>(Data.Sensors->Clock.Time - Data.Sensors->Clock.ExperimentStart).count();
               std::cerr << '[' << std::setfill('0') << std::setw(7) << nFrameIdx << ']' << " debug:" << std::endl;
               std::cerr << "f_tag_offset_target = " << f_tag_offset_target << std::endl;
               std::cerr << "fTagOffset = " << fTagOffset << std::endl;
               std::cerr << "Left' = " << (fTagOffset - f_tag_offset_target) << std::endl;
               std::cerr << "Right' = " << (f_tag_offset_target - fTagOffset) << std::endl;
               /* scale the speed by the lift actuator height error */
               double fTagOffsetTop = 
                  std::abs(FindTagCornerFurthestToTheTop(s_block)->second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;  
               double fTagOffsetBottom = 
                  std::abs(FindTagCornerFurthestToTheBottom(s_block)->second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;
               fLeft *= std::max(1.0 - std::max(fTagOffsetTop, fTagOffsetBottom), 0.1);
               fRight *= std::max(1.0 - std::max(fTagOffsetTop, fTagOffsetBottom), 0.1);
            }        
            /* apply the approach velocity */
            Data.Actuators->DifferentialDriveSystem.Left.Velocity = std::round(fLeft);
            Data.Actuators->DifferentialDriveSystem.Right.Velocity = std::round(fRight);
            Data.Actuators->DifferentialDriveSystem.Left.UpdateReq = true;
            Data.Actuators->DifferentialDriveSystem.Right.UpdateReq = true;
         }),
         CState("approach_target", [f_tag_offset_target] {
            /* default velocities, overwritten if target is detected */
            double fLeft = 0.0, fRight = 0.0, fLiftActuatorHeightError = 0.0;
            /* select tracked target */
            auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);                         
            if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
               const SBlock& s_block = itTarget->Observations.front();
               /* track the target using lift actuator position */
               TrackBlockViaLiftActuatorHeight(s_block, LIFT_ACTUATOR_MIN_HEIGHT + BLOCK_PRE_ATTACH_OFFSET);
               
               double fTagOffset = (FindTagCornerFurthestToTheRight(s_block)->first - IMAGE_SENSOR_HALF_WIDTH) / IMAGE_SENSOR_HALF_WIDTH;
               /* calculate the approach velocity */
               fRight = (1 + f_tag_offset_target - fTagOffset) * BASE_VELOCITY;
               fLeft  = (1 + fTagOffset - f_tag_offset_target) * BASE_VELOCITY;
               /* debug info */
               auto nFrameIdx = std::chrono::duration_cast<std::chrono::milliseconds>(Data.Sensors->Clock.Time - Data.Sensors->Clock.ExperimentStart).count();
               std::cerr << '[' << std::setfill('0') << std::setw(7) << nFrameIdx << ']' << " debug:" << std::endl;
               std::cerr << "f_tag_offset_target = " << f_tag_offset_target << std::endl;
               std::cerr << "fTagOffset = " << fTagOffset << std::endl;
               std::cerr << "Left' = " << (1 + fTagOffset - f_tag_offset_target) << std::endl;
               std::cerr << "Right' = " << (1 + f_tag_offset_target - fTagOffset) << std::endl;
               /* scale the speed by the lift actuator height error */       
               double fTagOffsetTop = 
                  std::abs(FindTagCornerFurthestToTheTop(s_block)->second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;  
               double fTagOffsetBottom = 
                  std::abs(FindTagCornerFurthestToTheBottom(s_block)->second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;
               fLeft *= std::max(1.0 - std::max(fTagOffsetTop, fTagOffsetBottom), 0.1);
               fRight *= std::max(1.0 - std::max(fTagOffsetTop, fTagOffsetBottom), 0.1);
            }        
            /* apply the approach velocity */
            Data.Actuators->DifferentialDriveSystem.Left.Velocity = std::round(fLeft);
            Data.Actuators->DifferentialDriveSystem.Right.Velocity = std::round(fRight);
            Data.Actuators->DifferentialDriveSystem.Left.UpdateReq = true;
            Data.Actuators->DifferentialDriveSystem.Right.UpdateReq = true;
         }),
      }) {
         AddTransition("pre_approach_target", "approach_target", [f_tag_offset_target] {
            auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);                         
            if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
               const SBlock& s_block = itTarget->Observations.front();
               double fTagOffset = (FindTagCornerFurthestToTheRight(s_block)->first - IMAGE_SENSOR_HALF_WIDTH) / IMAGE_SENSOR_HALF_WIDTH;
               return (std::abs(fTagOffset - f_tag_offset_target) < 0.1);
            }
            return false;
         });
         AddExitTransition("pre_approach_target", IsTargetLost);
         AddExitTransition("approach_target", IsTargetLost);
      }
};

class CStateApproachTargetFromRight : public CState {
public:
   CStateApproachTargetFromRight(const std::string& str_id, double f_tag_offset_target = -0.75) :
      CState(str_id, [f_tag_offset_target] {
         /* default velocities, overwritten if target is detected */
         double fLeft = 0.0, fRight = 0.0, fLiftActuatorHeightError = 0.0;
         /* select tracked target */
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);                         
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            /* track the target using lift actuator position */
            double fLiftActuatorHeightError = TrackBlockViaLiftActuatorHeight(s_block, LIFT_ACTUATOR_MIN_HEIGHT + BLOCK_PRE_ATTACH_OFFSET);
            
            double fTagOffset = (FindTagCornerFurthestToTheLeft(s_block)->first - IMAGE_SENSOR_HALF_WIDTH) / IMAGE_SENSOR_HALF_WIDTH;           
            /* calculate the approach velocity */
            fRight = (1 + f_tag_offset_target - fTagOffset) * BASE_VELOCITY;
            fLeft  = (1 + fTagOffset - f_tag_offset_target) * BASE_VELOCITY;
            /* debug info */
            auto nFrameIdx = std::chrono::duration_cast<std::chrono::milliseconds>(Data.Sensors->Clock.Time - Data.Sensors->Clock.ExperimentStart).count();
            std::cerr << '[' << std::setfill('0') << std::setw(7) << nFrameIdx << ']' << " debug:" << std::endl;
            std::cerr << "f_tag_offset_target = " << f_tag_offset_target << std::endl;
            std::cerr << "fTagOffset = " << fTagOffset << std::endl;
            std::cerr << "Left' = " << (1 + fTagOffset - f_tag_offset_target) << std::endl;
            std::cerr << "Right' = " << (1 + f_tag_offset_target - fTagOffset) << std::endl;
            /* scale the speed by the lift actuator height error */       
            double fTagOffsetTop = 
               std::abs(FindTagCornerFurthestToTheTop(s_block)->second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;  
            double fTagOffsetBottom = 
               std::abs(FindTagCornerFurthestToTheBottom(s_block)->second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;
            fLeft *= std::max(1.0 - std::max(fTagOffsetTop, fTagOffsetBottom), 0.1);
            fRight *= std::max(1.0 - std::max(fTagOffsetTop, fTagOffsetBottom), 0.1);
         }        
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
            CStateSetLiftActuatorPosition("set_lift_actuator_search_height", LIFT_ACTUATOR_MAX_HEIGHT),
            CStateSetVelocity("set_search_velocity", BASE_VELOCITY * 0.5, -BASE_VELOCITY * 0.5),
            CState("wait_for_next_target", [] {
               // add logic here to keep track of inspected targets, ignore structures with more than one block
               if(!Data.Sensors->ImageSensor.Detections.Structures.empty()) {
                  const SStructure& sStructure = Data.Sensors->ImageSensor.Detections.Structures.front();
                  Data.TrackedStructureId = sStructure.Id;
                  Data.TrackedTargetId = sStructure.Members.front()->Id;
               }
            }),
         }),
         CState("approach_block_far", nullptr, nullptr, {
            CStateSetLedColors("set_deck_color", CBlockDemo::EColor::GREEN),
            CStateMoveToTargetXZ("align_with_target", 0.0, PREAPPROACH_BLOCK_Z_TARGET, false),
            CStateApproachTargetFromLeft("approach_target_from_left"),
            CStateApproachTargetFromRight("approach_target_from_right"),
            CStateMoveToTargetX("turn_towards_target", 0.0, false),
            CStateMoveToTargetXZ("approach_target_straight", 0.0, 0.0, true),           
            CStateSetVelocity("set_reverse_velocity", -BASE_VELOCITY * 0.5, -BASE_VELOCITY * 0.5),
            CStateSetLiftActuatorPosition("lower_lift_actuator", LIFT_ACTUATOR_MIN_HEIGHT + BLOCK_PRE_ATTACH_OFFSET),
            CState("wait_for_target"),
            CStateSetVelocity("set_zero_velocity", 0.0, 0.0),
         }),
         CState("approach_block_near", nullptr, nullptr, {
            CStateSetLedColors("set_deck_color", CBlockDemo::EColor::RED),
            CState("init_near_block_approach", [this] {
               double fLeft = 0.0, fRight = 0.0;
               switch(m_eApproachDirection) {
                  case EApproachDirection::LEFT:
                     fLeft =  1.5 * BASE_VELOCITY;
                     fRight = 0.5 * BASE_VELOCITY;
                     break;
                  case EApproachDirection::RIGHT:
                     fLeft =  0.5 * BASE_VELOCITY;
                     fRight = 1.5 * BASE_VELOCITY;
                     break;
                  case EApproachDirection::STRAIGHT:
                     fLeft =  1.0 * BASE_VELOCITY;
                     fRight = 1.0 * BASE_VELOCITY;                    
                     break;
               }
               /* configure drive system */
               Data.Actuators->DifferentialDriveSystem.Left.Velocity = std::round(fLeft);
               Data.Actuators->DifferentialDriveSystem.Right.Velocity = std::round(fRight);
               Data.Actuators->DifferentialDriveSystem.Right.UpdateReq = true;
               Data.Actuators->DifferentialDriveSystem.Left.UpdateReq = true;
               /* lower the manipulator */
               Data.Actuators->ManipulatorModule.LiftActuator.Position.Value = (LIFT_ACTUATOR_MIN_HEIGHT + BLOCK_PRE_ATTACH_OFFSET);
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
         CState("pick_up_block", nullptr, nullptr, {
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
               double fLeft = 0.0, fRight = 0.0;
               switch(m_eApproachDirection) {
                  case EApproachDirection::LEFT:
                     fLeft =  1.25 * BASE_VELOCITY;
                     fRight = 0.75 * BASE_VELOCITY;
                     break;
                  case EApproachDirection::RIGHT:
                     fLeft =  0.75 * BASE_VELOCITY;
                     fRight = 1.25 * BASE_VELOCITY;
                     break;
                  case EApproachDirection::STRAIGHT:
                     fLeft =  1.0 * BASE_VELOCITY;
                     fRight = 1.0 * BASE_VELOCITY;                    
                     break;
               }
               Data.Actuators->DifferentialDriveSystem.Left.Velocity = std::round(fLeft);
               Data.Actuators->DifferentialDriveSystem.Right.Velocity = std::round(fRight);
               Data.Actuators->DifferentialDriveSystem.Left.UpdateReq = true;
               Data.Actuators->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
            CState("wait_until_approach_timer_expired"),
            CState("set_reverse_velocity", [this] {
               Data.Actuators->DifferentialDriveSystem.Left.Velocity = -BASE_VELOCITY;
               Data.Actuators->DifferentialDriveSystem.Right.Velocity = -BASE_VELOCITY;
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
               Data.Actuators->DifferentialDriveSystem.Left.UpdateReq = true;
               Data.Actuators->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
            CState("wait_until_reverse_timer_expired"),
         }),        
      }) {
      
      /**************** search_for_unused_block transitions ****************/
      (*this)["search_for_unused_block"].AddTransition("set_deck_color", "set_lift_actuator_search_height");
      (*this)["search_for_unused_block"].AddTransition("set_lift_actuator_search_height", "set_search_velocity");
      (*this)["search_for_unused_block"].AddTransition("set_search_velocity","wait_for_next_target");
      (*this)["search_for_unused_block"].AddExitTransition("wait_for_next_target", [] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);                         
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {        
            Data.TrackedTargetId = itTarget->Id;
            return true;
         }
         return false;
      });
      
      // Top level transition
      this->AddTransition("search_for_unused_block","approach_block_far");
      
      /**************** approach_block_far transitions ****************/
      (*this)["approach_block_far"].AddTransition("set_deck_color", "align_with_target");
      /* lost tracking transitions */
      (*this)["approach_block_far"].AddExitTransition("align_with_target", IsTargetLost);
      (*this)["approach_block_far"].AddExitTransition("approach_target_from_left", IsTargetLost);
      (*this)["approach_block_far"].AddExitTransition("approach_target_from_right", IsTargetLost);
      (*this)["approach_block_far"].AddExitTransition("turn_towards_target", IsTargetLost);
      /* select approach direction transitions */
      (*this)["approach_block_far"].AddTransition("align_with_target", "approach_target_from_left", [] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);                                         
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if((std::abs(s_block.Translation.GetX() - PREAPPROACH_BLOCK_X_TARGET) < PREAPPROACH_BLOCK_XZ_THRES) &&
               (std::abs(s_block.Translation.GetZ() - PREAPPROACH_BLOCK_Z_TARGET) < PREAPPROACH_BLOCK_XZ_THRES)) {
               argos::CRadians cEulerAngleZ, cEulerAngleY, cEulerAngleX;
               s_block.Rotation.ToEulerAngles(cEulerAngleZ, cEulerAngleY, cEulerAngleX);              
               if(cEulerAngleZ.GetValue() >= (M_PI / 18.0)) {
                  std::cerr << "Target angle = " << argos::ToDegrees(cEulerAngleZ).GetValue() << ": Using left approach" << std::endl;
                  return true;
               }
            }
         }
         return false;
      });
      (*this)["approach_block_far"].AddTransition("align_with_target", "approach_target_from_right", [] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);                                         
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if((std::abs(s_block.Translation.GetX() - PREAPPROACH_BLOCK_X_TARGET) < PREAPPROACH_BLOCK_XZ_THRES) &&
               (std::abs(s_block.Translation.GetZ() - PREAPPROACH_BLOCK_Z_TARGET) < PREAPPROACH_BLOCK_XZ_THRES)) {
               argos::CRadians cEulerAngleZ, cEulerAngleY, cEulerAngleX;
               s_block.Rotation.ToEulerAngles(cEulerAngleZ, cEulerAngleY, cEulerAngleX);    
               if(cEulerAngleZ.GetValue() <= -(M_PI / 18.0)) {
                  std::cerr << "Target angle = " << argos::ToDegrees(cEulerAngleZ).GetValue() << ": Using right approach" << std::endl;
                  return true;
               }
            }
         }
         return false;
      });
      (*this)["approach_block_far"].AddTransition("align_with_target", "approach_target_straight", [] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);                                         
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if((std::abs(s_block.Translation.GetX() - PREAPPROACH_BLOCK_X_TARGET) < PREAPPROACH_BLOCK_XZ_THRES) &&
               (std::abs(s_block.Translation.GetZ() - PREAPPROACH_BLOCK_Z_TARGET) < PREAPPROACH_BLOCK_XZ_THRES)) {
               argos::CRadians cEulerAngleZ, cEulerAngleY, cEulerAngleX;
               s_block.Rotation.ToEulerAngles(cEulerAngleZ, cEulerAngleY, cEulerAngleX);    
               if(cEulerAngleZ.GetValue() > -(M_PI / 18.0) &&
                  cEulerAngleZ.GetValue() <  (M_PI / 18.0)) {
                  std::cerr << "Target angle = " << argos::ToDegrees(cEulerAngleZ).GetValue() << ": Using straight approach" << std::endl;
                  return true;
               }
            }
         }
         return false;
      });
      (*this)["approach_block_far"].AddTransition("approach_target_from_right", "turn_towards_target", [] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);                                         
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            argos::CRadians cEulerAngleZ, cEulerAngleY, cEulerAngleX;
            s_block.Rotation.ToEulerAngles(cEulerAngleZ, cEulerAngleY, cEulerAngleX);

            double intermediate = (s_block.Translation.GetZ() + 0.15) / s_block.Translation.GetX();
            argos::CRadians cRobotToTargetAngle = argos::ATan2(s_block.Translation.GetZ() + 0.15, s_block.Translation.GetX());

            std::cerr << "intermediate = " << intermediate << std::endl;
            std::cerr << "cBlockAngle = " << argos::ToDegrees(cEulerAngleZ).GetValue() << std::endl;
            std::cerr << "cRobotToTargetAngle = " << argos::ToDegrees(cRobotToTargetAngle).GetValue() << std::endl;

            if(std::abs((cEulerAngleZ - cRobotToTargetAngle).GetValue()) < M_PI / 18.0) {
               return true;
            }
         }
         return false;
      });
      (*this)["approach_block_far"].AddTransition("approach_target_from_left", "turn_towards_target", [] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);                                         
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            argos::CRadians cEulerAngleZ, cEulerAngleY, cEulerAngleX;
            s_block.Rotation.ToEulerAngles(cEulerAngleZ, cEulerAngleY, cEulerAngleX);

            double intermediate = (s_block.Translation.GetZ() + 0.15) / s_block.Translation.GetX();
            argos::CRadians cRobotToTargetAngle = argos::ATan2(s_block.Translation.GetZ() + 0.15, s_block.Translation.GetX());

            std::cerr << "intermediate = " << intermediate << std::endl;
            std::cerr << "cBlockAngle = " << argos::ToDegrees(cEulerAngleZ).GetValue() << std::endl;
            std::cerr << "cRobotToTargetAngle = " << argos::ToDegrees(cRobotToTargetAngle).GetValue() << std::endl;

            if(std::abs((cEulerAngleZ - cRobotToTargetAngle).GetValue()) < M_PI / 18.0) {
               return true;
            }
         }
         return false;
      });
      (*this)["approach_block_far"].AddTransition("turn_towards_target", "approach_target_straight", [] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);                                         
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if(std::abs(s_block.Translation.GetX() - PREAPPROACH_BLOCK_X_TARGET) < PREAPPROACH_BLOCK_XZ_THRES) {
               return true;
            }
         }
         return false;
      });
      (*this)["approach_block_far"].AddTransition("approach_target_straight", "set_reverse_velocity", IsTargetLost);
      (*this)["approach_block_far"].AddTransition("set_reverse_velocity", "lower_lift_actuator");
      (*this)["approach_block_far"].AddTransition("lower_lift_actuator", "wait_for_target");
      (*this)["approach_block_far"].AddTransition("wait_for_target", "set_zero_velocity", [] {
         auto itTarget = FindTrackedTarget(Data.TrackedTargetId, Data.Sensors->ImageSensor.Detections.Targets);                         
         if(itTarget != std::end(Data.Sensors->ImageSensor.Detections.Targets)) {        
            Data.TrackedTargetId = itTarget->Id;
            return true;
         }
         return false;
      });
      (*this)["approach_block_far"].AddExitTransition("set_zero_velocity");

      // Top level transition
      this->AddTransition("approach_block_far","approach_block_near");

      /**************** approach_block_near transitions ****************/
      /*
      (*this)["approach_block_near"].AddTransition("init_near_block_approach","wait_for_underneath_rf");
      (*this)["approach_block_near"].AddTransition("wait_for_underneath_rf","set_approach_velocity", [this] {
         return (Data.Sensors->ManipulatorModule.RangeFinders.Underneath > 2300);
      });
      (*this)["approach_block_near"].AddTransition("set_approach_velocity", "set_zero_velocity", [this] {
         return (Data.Sensors->ManipulatorModule.RangeFinders.Left > 3000) &&
                (Data.Sensors->ManipulatorModule.RangeFinders.Right > 3000);
      });
      (*this)["approach_block_near"].AddExitTransition("set_zero_velocity");
           
      (*this).AddTransition("approach_block_near", "pick_up_block");
      */
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
   
   unsigned int m_unNumTargetsInStructA = 0;
   unsigned int m_unNumTargetsInStructB = 0;
};

#endif
