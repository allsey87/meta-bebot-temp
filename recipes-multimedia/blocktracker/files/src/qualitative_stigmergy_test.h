#ifndef MANIPULATOR_TESTING_TASK_H
#define MANIPULATOR_TESTING_TASK_H

#include "state.h"
#include "block_demo.h"

#include <iostream>

#define MTT_LIFT_ACTUATOR_MAX_HEIGHT 140
#define MTT_LIFT_ACTUATOR_OFFSET_HEIGHT 1

#define MTT_LIFT_ACTUATOR_INCREMENT 10

#define TARGET_Z_DIST_REVERSE_TO 0.25

#define MTT_BLOCK_SEP_THRESHOLD 0.065

#define BASE_VELOCITY 30

class CManipulatorTestingTask : public CState {
   
public:
   CManipulatorTestingTask(CBlockDemo::SSensorData* ps_sensor_data,
                           CBlockDemo::SActuatorData* ps_actuator_data) :
      m_psSensorData(ps_sensor_data),
      m_psActuatorData(ps_actuator_data),
      /* initialize the state machine */
      CState("top_level_state", nullptr, nullptr, {
         CState("init_end_effector_position", nullptr, nullptr, {
            CState("raise_lift_actuator", [this] {
               m_psActuatorData->ManipulatorModule.LiftActuator.Position.Value = (MTT_LIFT_ACTUATOR_MAX_HEIGHT - 5u);
               m_psActuatorData->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
            }),
            CState("wait_for_lift_actuator"),
         }),
         CState("search_for_unused_block", nullptr, nullptr, {
            CState("set_search_velocity", [this] {
               m_psActuatorData->DifferentialDriveSystem.Power.Enable = true;
               m_psActuatorData->DifferentialDriveSystem.Power.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Left.Velocity = BASE_VELOCITY;
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.Velocity = -BASE_VELOCITY;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
               for(CBlockDemo::EColor& e_color : m_psActuatorData->LEDDeck.Color) 
                  e_color = CBlockDemo::EColor::RED;
               for(bool& b_update : m_psActuatorData->LEDDeck.UpdateReq)
                  b_update = true;
            }),
            CState("wait_for_next_target"),
            CState("turn_towards_target", [this] {
               auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                            std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                            [this] (const STarget& s_target) {
                                               return (s_target.Id == m_unTrackedTargetId);
                                            });
               if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
                  const SBlock& s_block = itTarget->Observations.front();            
                  TrackBlockViaManipulatorHeight(s_block, MTT_LIFT_ACTUATOR_OFFSET_HEIGHT + 2u);
                  float fTagXOffset = (s_block.Tags.front().Center.first - 320.0f) / 320.0f;
                  if(fTagXOffset < 0) {
                     m_psActuatorData->DifferentialDriveSystem.Left.Velocity = -BASE_VELOCITY * std::abs(fTagXOffset);
                     m_psActuatorData->DifferentialDriveSystem.Right.Velocity = BASE_VELOCITY * std::abs(fTagXOffset);
                  }
                  else {
                     m_psActuatorData->DifferentialDriveSystem.Left.Velocity = BASE_VELOCITY * std::abs(fTagXOffset);
                     m_psActuatorData->DifferentialDriveSystem.Right.Velocity = -BASE_VELOCITY * std::abs(fTagXOffset);
                  }
               }
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
            CState("reverse_until_distance", [this] {
               auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                            std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                            [this] (const STarget& s_target) {
                                               return (s_target.Id == m_unTrackedTargetId);
                                            });
               if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
                  const SBlock& s_block = itTarget->Observations.front();            
                  TrackBlockViaManipulatorHeight(s_block, MTT_LIFT_ACTUATOR_OFFSET_HEIGHT + 2u);
                  float fTagXOffset = (s_block.Tags.front().Center.first - 320.0f) / 320.0f;
                  if(fTagXOffset < 0) {
                     m_psActuatorData->DifferentialDriveSystem.Left.Velocity = -BASE_VELOCITY * (1.0f + std::abs(fTagXOffset));
                     m_psActuatorData->DifferentialDriveSystem.Right.Velocity = -BASE_VELOCITY;
                  }
                  else {
                     m_psActuatorData->DifferentialDriveSystem.Left.Velocity = -BASE_VELOCITY;
                     m_psActuatorData->DifferentialDriveSystem.Right.Velocity = -BASE_VELOCITY * (1.0f + std::abs(fTagXOffset));
                  }
               }
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
         }),
         CState("far_block_approach", nullptr, nullptr, {
            CState("turn_towards_target", [this] {
               auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                            std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                            [this] (const STarget& s_target) {
                                               return (s_target.Id == m_unTrackedTargetId);
                                            });
               if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
                  const SBlock& s_block = itTarget->Observations.front();            
                  TrackBlockViaManipulatorHeight(s_block, MTT_LIFT_ACTUATOR_OFFSET_HEIGHT + 2u);
                  float fTagXOffset = (s_block.Tags.front().Center.first - 320.0f) / 320.0f;
                  if(fTagXOffset < 0) {
                     m_psActuatorData->DifferentialDriveSystem.Left.Velocity = -BASE_VELOCITY * (std::abs(fTagXOffset) + 0.25);
                     m_psActuatorData->DifferentialDriveSystem.Right.Velocity = BASE_VELOCITY * (std::abs(fTagXOffset) + 0.25);
                  }
                  else {
                     m_psActuatorData->DifferentialDriveSystem.Left.Velocity = BASE_VELOCITY * (std::abs(fTagXOffset) + 0.25);
                     m_psActuatorData->DifferentialDriveSystem.Right.Velocity = -BASE_VELOCITY * (std::abs(fTagXOffset) + 0.25);
                  }
               }
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;  
            }),
            CState("approach_block_straight", [this] {
               auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                            std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                            [this] (const STarget& s_target) {
                                               return (s_target.Id == m_unTrackedTargetId);
                                            });
               if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
                  const SBlock& s_block = itTarget->Observations.front();            
                  TrackBlockViaManipulatorHeight(s_block, MTT_LIFT_ACTUATOR_OFFSET_HEIGHT + 2u);
                  float fTagXOffset = (s_block.Tags.front().Center.first - 320.0f) / 320.0f;
                  if(fTagXOffset < 0) {
                     m_psActuatorData->DifferentialDriveSystem.Left.Velocity = BASE_VELOCITY;
                     m_psActuatorData->DifferentialDriveSystem.Right.Velocity = BASE_VELOCITY * (1.0f + std::abs(fTagXOffset));
                  }
                  else {
                     m_psActuatorData->DifferentialDriveSystem.Left.Velocity = BASE_VELOCITY * (1.0f + std::abs(fTagXOffset));
                     m_psActuatorData->DifferentialDriveSystem.Right.Velocity = BASE_VELOCITY;
                  }
               }
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
            CState("approach_block_left", [this] {
               float fLeft = 0.0f, fRight = 0.0f;
               /* search for target zero */
               auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                            std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                            [this] (const STarget& s_target) {
                                               return (s_target.Id == m_unTrackedTargetId);
                                            });
                                         
               if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
                  const SBlock& s_block = itTarget->Observations.front();
                 
                  float fTagXOffset = (s_block.Tags.front().Center.first - 320.0f) / 320.0f;
                  float fTagXOffsetTarget = 0.975f * std::abs((s_block.Translation.GetZ() - 0.1f) / 0.25f);

                  fRight = (1 + fTagXOffsetTarget - fTagXOffset) * BASE_VELOCITY;
                  fLeft  = (1 + fTagXOffset - fTagXOffsetTarget) * BASE_VELOCITY;
                                                    
                  TrackBlockViaManipulatorHeight(s_block, MTT_LIFT_ACTUATOR_OFFSET_HEIGHT + 2u);
               }
               m_psActuatorData->DifferentialDriveSystem.Left.Velocity = std::round(fLeft);
               m_psActuatorData->DifferentialDriveSystem.Right.Velocity = std::round(fRight);
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
            CState("approach_block_right", [this] {
               float fLeft = 0.0f, fRight = 0.0f;
               /* search for target zero */
               auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                            std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                            [this] (const STarget& s_target) {
                                               return (s_target.Id == m_unTrackedTargetId);
                                            });
                                         
               if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
                  const SBlock& s_block = itTarget->Observations.front();
                 
                  float fTagXOffset = (s_block.Tags.front().Center.first - 320.0f) / 320.0f;
                  float fTagXOffsetTarget = -0.975f * std::abs((s_block.Translation.GetZ() - 0.1f) / 0.25f);

                  fRight = (1 + fTagXOffsetTarget - fTagXOffset) * BASE_VELOCITY;
                  fLeft  = (1 + fTagXOffset - fTagXOffsetTarget) * BASE_VELOCITY;
                                                    
                  TrackBlockViaManipulatorHeight(s_block, MTT_LIFT_ACTUATOR_OFFSET_HEIGHT + 2u);
               }
               m_psActuatorData->DifferentialDriveSystem.Left.Velocity = std::round(fLeft);
               m_psActuatorData->DifferentialDriveSystem.Right.Velocity = std::round(fRight);
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
            CState("align_end_effector", nullptr, nullptr, {
               CState("lower_manipulator", [this] {              
                  m_psActuatorData->ManipulatorModule.LiftActuator.Position.Value = (MTT_LIFT_ACTUATOR_OFFSET_HEIGHT + 2u);
                  m_psActuatorData->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
               }),
               CState("wait_for_manipulator"),
               CState("set_reverse_velocity", [this] {              
                  float fRight = -BASE_VELOCITY * 0.5;
                  float fLeft  = -BASE_VELOCITY * 0.5;
                  m_psActuatorData->DifferentialDriveSystem.Left.Velocity = std::round(fLeft);
                  m_psActuatorData->DifferentialDriveSystem.Right.Velocity = std::round(fRight);
                  m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
                  m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
               }),
               CState("wait_for_target"),
               CState("turn_towards_target", [this] {
                  auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                               std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                               [this] (const STarget& s_target) {
                                                  return (s_target.Id == m_unTrackedTargetId);
                                               });
                  if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
                     const SBlock& s_block = itTarget->Observations.front();            
                     float fTagXOffset = (s_block.Tags.front().Center.first - 320.0f) / 320.0f;
                     if(fTagXOffset < 0) {
                        m_psActuatorData->DifferentialDriveSystem.Left.Velocity = -BASE_VELOCITY * std::abs(fTagXOffset);
                        m_psActuatorData->DifferentialDriveSystem.Right.Velocity = BASE_VELOCITY * std::abs(fTagXOffset);
                     }
                     else {
                        m_psActuatorData->DifferentialDriveSystem.Left.Velocity = BASE_VELOCITY * std::abs(fTagXOffset);
                        m_psActuatorData->DifferentialDriveSystem.Right.Velocity = -BASE_VELOCITY * std::abs(fTagXOffset);
                     }
                  }
                  m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
                  m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
               }),
            }),
         }),
         CState("near_block_approach", nullptr, nullptr, {
            CState("init_near_block_approach", [this] {
               m_psActuatorData->DifferentialDriveSystem.Power.Enable = true;
               m_psActuatorData->DifferentialDriveSystem.Power.UpdateReq = true;

               float fLeft = 0.0f, fRight = 0.0f;
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

               m_psActuatorData->DifferentialDriveSystem.Left.Velocity = std::round(fLeft);
               m_psActuatorData->DifferentialDriveSystem.Right.Velocity = std::round(fRight);

               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               /* lower the manipulator */
               m_psActuatorData->ManipulatorModule.LiftActuator.Position.Value = (MTT_LIFT_ACTUATOR_OFFSET_HEIGHT + 3u);
               m_psActuatorData->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
            }),
            CState("wait_for_underneath_rf"),
            CState("set_approach_velocity", [this] {
               float fLeft = m_psSensorData->ManipulatorModule.RangeFinders.Left / 
                  static_cast<float>(m_psSensorData->ManipulatorModule.RangeFinders.Right + 
                                     m_psSensorData->ManipulatorModule.RangeFinders.Left) * 120;
               float fRight = m_psSensorData->ManipulatorModule.RangeFinders.Right / 
                  static_cast<float>(m_psSensorData->ManipulatorModule.RangeFinders.Right + 
                                     m_psSensorData->ManipulatorModule.RangeFinders.Left) * 120;
               
               m_psActuatorData->DifferentialDriveSystem.Left.Velocity = std::round(fLeft);
               m_psActuatorData->DifferentialDriveSystem.Right.Velocity = std::round(fRight);
               
               std::cerr << "Output Velocity" << " Left = " << fLeft << ", Right = " << fRight << std::endl;
               
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
            CState("set_zero_velocity", [this] {
               m_psActuatorData->DifferentialDriveSystem.Left.Velocity = 0;
               m_psActuatorData->DifferentialDriveSystem.Right.Velocity = 0;
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
         }),
         CState("pick_up_block", nullptr, nullptr, {
            CState("lower_lift_actuator", [this] {
               m_psActuatorData->ManipulatorModule.LiftActuator.Position.Value = MTT_LIFT_ACTUATOR_OFFSET_HEIGHT;
               m_psActuatorData->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
            }),
            CState("wait_for_lift_actuator"),
            CState("align_block_magnets", nullptr, nullptr, {
               CState("enable_attachment_field", [this] {
                  for(CBlockDemo::EColor& e_color : m_psActuatorData->LEDDeck.Color)
                     e_color = CBlockDemo::EColor::BLUE;
                  for(bool& b_update : m_psActuatorData->LEDDeck.UpdateReq)
                     b_update = true;
                  m_psActuatorData->ManipulatorModule.EndEffector.FieldMode = 
                     CBlockDemo::EGripperFieldMode::CONSTRUCTIVE;
                  m_psActuatorData->ManipulatorModule.EndEffector.UpdateReq = true;
               }),
               CState("disable_attachment_field", [this] {
                  m_psActuatorData->ManipulatorModule.EndEffector.FieldMode = 
                     CBlockDemo::EGripperFieldMode::DISABLED;
                  m_psActuatorData->ManipulatorModule.EndEffector.UpdateReq = true;
               }),
               CState("wait_for_recharge"),
            }),
            CState("attach_block", nullptr, nullptr, {
               CState("enable_attachment_field", [this] {
                  m_psActuatorData->ManipulatorModule.EndEffector.FieldMode = 
                     CBlockDemo::EGripperFieldMode::CONSTRUCTIVE;
                  m_psActuatorData->ManipulatorModule.EndEffector.UpdateReq = true;
               }),
               CState("wait_for_discharge"),
               CState("disable_attachment_field", [this] {
                  m_psActuatorData->ManipulatorModule.EndEffector.FieldMode = 
                     CBlockDemo::EGripperFieldMode::DISABLED;
                  m_psActuatorData->ManipulatorModule.EndEffector.UpdateReq = true;
               }),
            }),
            CState("raise_lift_actuator", nullptr, nullptr, {
               CState("set_position", [this] {
                  for(CBlockDemo::EColor& e_color : m_psActuatorData->LEDDeck.Color)
                     e_color = CBlockDemo::EColor::BLUE;
                  for(bool& b_update : m_psActuatorData->LEDDeck.UpdateReq)
                     b_update = true;
                  m_psActuatorData->ManipulatorModule.LiftActuator.Position.Value = (MTT_LIFT_ACTUATOR_MAX_HEIGHT - 5u);
                  m_psActuatorData->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
                  m_psActuatorData->ManipulatorModule.EndEffector.UpdateReq = true;
                  m_psActuatorData->ManipulatorModule.NFCInterface.OutboundMessage = "3";
                  m_psActuatorData->ManipulatorModule.NFCInterface.UpdateReq = true;
               }),
               CState("wait_for_lift_actuator"),
            }), // raise_lift_actuator
         }), // pick up block
         CState("reattempt_block_pick_up", nullptr, nullptr, {
            CState("set_reverse_velocity", [this] {              
               float fRight = -BASE_VELOCITY * 0.5;
               float fLeft  = -BASE_VELOCITY * 0.5;
               m_psActuatorData->DifferentialDriveSystem.Left.Velocity = std::round(fLeft);
               m_psActuatorData->DifferentialDriveSystem.Right.Velocity = std::round(fRight);
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
               m_psActuatorData->ManipulatorModule.LiftActuator.Position.Value = (MTT_LIFT_ACTUATOR_OFFSET_HEIGHT + 10u);
               m_psActuatorData->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
            }),
            CState("wait_for_target"),
         }),
         CState("search_for_structure", nullptr, nullptr, {
            CState("set_search_velocity", [this] {
               m_psActuatorData->DifferentialDriveSystem.Power.Enable = true;
               m_psActuatorData->DifferentialDriveSystem.Power.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Left.Velocity = BASE_VELOCITY;
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.Velocity = -BASE_VELOCITY;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
               for(CBlockDemo::EColor& e_color : m_psActuatorData->LEDDeck.Color) 
                  e_color = CBlockDemo::EColor::RED;
               for(bool& b_update : m_psActuatorData->LEDDeck.UpdateReq)
                  b_update = true;
            }),
            CState("wait_for_next_target"),
            CState("turn_towards_target", [this] {
               auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                            std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                            [this] (const STarget& s_target) {
                                               return (s_target.Id == m_unTrackedTargetId);
                                            });
               if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
                  const SBlock& s_block = itTarget->Observations.front();            
                  TrackBlockViaManipulatorHeight(s_block, MTT_LIFT_ACTUATOR_OFFSET_HEIGHT + 2u);
                  float fTagXOffset = (s_block.Tags.front().Center.first - 320.0f) / 320.0f;
                  if(fTagXOffset < 0) {
                     m_psActuatorData->DifferentialDriveSystem.Left.Velocity = -BASE_VELOCITY * std::abs(fTagXOffset);
                     m_psActuatorData->DifferentialDriveSystem.Right.Velocity = BASE_VELOCITY * std::abs(fTagXOffset);
                  }
                  else {
                     m_psActuatorData->DifferentialDriveSystem.Left.Velocity = BASE_VELOCITY * std::abs(fTagXOffset);
                     m_psActuatorData->DifferentialDriveSystem.Right.Velocity = -BASE_VELOCITY * std::abs(fTagXOffset);
                  }
               }
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
            CState("reverse_until_distance", [this] {
               auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                            std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                            [this] (const STarget& s_target) {
                                               return (s_target.Id == m_unTrackedTargetId);
                                            });
               if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
                  const SBlock& s_block = itTarget->Observations.front();            
                  TrackBlockViaManipulatorHeight(s_block, MTT_LIFT_ACTUATOR_OFFSET_HEIGHT + 2u);
                  float fTagXOffset = (s_block.Tags.front().Center.first - 320.0f) / 320.0f;
                  if(fTagXOffset < 0) {
                     m_psActuatorData->DifferentialDriveSystem.Left.Velocity = -BASE_VELOCITY * (1.0f + std::abs(fTagXOffset));
                     m_psActuatorData->DifferentialDriveSystem.Right.Velocity = -BASE_VELOCITY;
                  }
                  else {
                     m_psActuatorData->DifferentialDriveSystem.Left.Velocity = -BASE_VELOCITY;
                     m_psActuatorData->DifferentialDriveSystem.Right.Velocity = -BASE_VELOCITY * (1.0f + std::abs(fTagXOffset));
                  }
               }
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
         }),
         CState("far_structure_approach", nullptr, nullptr, {
            CState("turn_towards_target", [this] {
               auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                            std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                            [this] (const STarget& s_target) {
                                               return (s_target.Id == m_unTrackedTargetId);
                                            });
               if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
                  const SBlock& s_block = itTarget->Observations.front();            
                  TrackBlockViaManipulatorHeight(s_block, MTT_LIFT_ACTUATOR_OFFSET_HEIGHT + 2u);
                  float fTagXOffset = (s_block.Tags.front().Center.first - 320.0f) / 320.0f;
                  if(fTagXOffset < 0) {
                     m_psActuatorData->DifferentialDriveSystem.Left.Velocity = -BASE_VELOCITY * (std::abs(fTagXOffset) + 0.25);
                     m_psActuatorData->DifferentialDriveSystem.Right.Velocity = BASE_VELOCITY * (std::abs(fTagXOffset) + 0.25);
                  }
                  else {
                     m_psActuatorData->DifferentialDriveSystem.Left.Velocity = BASE_VELOCITY * (std::abs(fTagXOffset) + 0.25);
                     m_psActuatorData->DifferentialDriveSystem.Right.Velocity = -BASE_VELOCITY * (std::abs(fTagXOffset) + 0.25);
                  }
               }
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;  
            }),
            CState("approach_block_straight", [this] {
               auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                            std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                            [this] (const STarget& s_target) {
                                               return (s_target.Id == m_unTrackedTargetId);
                                            });
               if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
                  const SBlock& s_block = itTarget->Observations.front();            
                  TrackBlockViaManipulatorHeight(s_block, MTT_LIFT_ACTUATOR_OFFSET_HEIGHT + 2u);
                  float fTagXOffset = (s_block.Tags.front().Center.first - 320.0f) / 320.0f;
                  if(fTagXOffset < 0) {
                     m_psActuatorData->DifferentialDriveSystem.Left.Velocity = BASE_VELOCITY;
                     m_psActuatorData->DifferentialDriveSystem.Right.Velocity = BASE_VELOCITY * (1.0f + std::abs(fTagXOffset));
                  }
                  else {
                     m_psActuatorData->DifferentialDriveSystem.Left.Velocity = BASE_VELOCITY * (1.0f + std::abs(fTagXOffset));
                     m_psActuatorData->DifferentialDriveSystem.Right.Velocity = BASE_VELOCITY;
                  }
               }
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
            CState("approach_block_left", [this] {
               float fLeft = 0.0f, fRight = 0.0f;
               /* search for target zero */
               auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                            std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                            [this] (const STarget& s_target) {
                                               return (s_target.Id == m_unTrackedTargetId);
                                            });
                                         
               if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
                  const SBlock& s_block = itTarget->Observations.front();
                  float fTagXOffset = (s_block.Tags.front().Center.first - 320.0f) / 320.0f;
                  float fTagXOffsetTarget = 0.975f * std::abs((s_block.Translation.GetZ() - 0.1f) / 0.25f);
                  fRight = (1 + fTagXOffsetTarget - fTagXOffset) * BASE_VELOCITY;
                  fLeft  = (1 + fTagXOffset - fTagXOffsetTarget) * BASE_VELOCITY;               
                  TrackBlockViaManipulatorHeight(s_block, MTT_LIFT_ACTUATOR_OFFSET_HEIGHT + 2u);
               }
               m_psActuatorData->DifferentialDriveSystem.Left.Velocity = std::round(fLeft);
               m_psActuatorData->DifferentialDriveSystem.Right.Velocity = std::round(fRight);
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
            CState("approach_block_right", [this] {
               float fLeft = 0.0f, fRight = 0.0f;
               /* search for target zero */
               auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                            std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                            [this] (const STarget& s_target) {
                                               return (s_target.Id == m_unTrackedTargetId);
                                            });
                                         
               if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
                  const SBlock& s_block = itTarget->Observations.front();
                 
                  float fTagXOffset = (s_block.Tags.front().Center.first - 320.0f) / 320.0f;
                  float fTagXOffsetTarget = -0.975f * std::abs((s_block.Translation.GetZ() - 0.1f) / 0.25f);
                  fRight = (1 + fTagXOffsetTarget - fTagXOffset) * BASE_VELOCITY;
                  fLeft  = (1 + fTagXOffset - fTagXOffsetTarget) * BASE_VELOCITY;
                                                                     
                  TrackBlockViaManipulatorHeight(s_block, MTT_LIFT_ACTUATOR_OFFSET_HEIGHT + 2u);
               }
               m_psActuatorData->DifferentialDriveSystem.Left.Velocity = std::round(fLeft);
               m_psActuatorData->DifferentialDriveSystem.Right.Velocity = std::round(fRight);
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
         }),
         CState("near_structure_approach", nullptr, nullptr, {
            CState("set_manipulator_height", [this] {
               m_psActuatorData->ManipulatorModule.LiftActuator.Position.Value = 83;
               m_psActuatorData->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
            }),
            CState("wait_for_manipulator"),
            CState("set_approach_velocity", [this] {
               float fLeft = 0.0f, fRight = 0.0f;
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
               m_psActuatorData->DifferentialDriveSystem.Left.Velocity = std::round(fLeft);
               m_psActuatorData->DifferentialDriveSystem.Right.Velocity = std::round(fRight);
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
               m_fNearApproachStartTime = m_psSensorData->Clock.Time;
            }),
            CState("wait_until_approach_timer_expired"),
            CState("set_reverse_velocity", [this] {
               m_psActuatorData->DifferentialDriveSystem.Left.Velocity = -BASE_VELOCITY;
               m_psActuatorData->DifferentialDriveSystem.Right.Velocity = -BASE_VELOCITY;
               m_fReverseStartTime = m_psSensorData->Clock.Time;
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
            CState("wait_until_reverse_timer_expired"),
            CState("set_zero_velocity", [this] {
               m_psActuatorData->DifferentialDriveSystem.Left.Velocity = 0;
               m_psActuatorData->DifferentialDriveSystem.Right.Velocity = 0;
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
         }),
         CState("place_block", nullptr, nullptr, {
            CState("wait_until_charged"),
            CState("enable_detachment_field", [this] {
               m_psActuatorData->ManipulatorModule.EndEffector.FieldMode = 
                  CBlockDemo::EGripperFieldMode::DESTRUCTIVE;
               m_psActuatorData->ManipulatorModule.EndEffector.UpdateReq = true;   
            }),
            CState("disable_detachment_field", [this] {
               m_psActuatorData->ManipulatorModule.EndEffector.FieldMode = 
                  CBlockDemo::EGripperFieldMode::DISABLED;
               m_psActuatorData->ManipulatorModule.EndEffector.UpdateReq = true;   
            }),
         }),
         CState("reverse", nullptr, nullptr, {
            CState("set_reverse_velocity", [this] {
               m_psActuatorData->DifferentialDriveSystem.Left.Velocity = -BASE_VELOCITY;
               m_psActuatorData->DifferentialDriveSystem.Right.Velocity = -BASE_VELOCITY;
               m_fReverseStartTime = m_psSensorData->Clock.Time;
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
            CState("wait_until_reverse_timer_expired"),
         }),        
      }) {
        
      
      (*this)["init_end_effector_position"].AddTransition("raise_lift_actuator","wait_for_lift_actuator");
      (*this)["init_end_effector_position"].AddExitTransition("wait_for_lift_actuator", [this] {
         return (m_psSensorData->ManipulatorModule.LiftActuator.State == 
                 CBlockDemo::ELiftActuatorSystemState::INACTIVE);
      });

      (*this).AddTransition("init_end_effector_position", "search_for_unused_block");
      //(*this).AddTransition("init_end_effector_position", "search_for_structure");
      
      /* search_for_target transitions */
      (*this)["search_for_unused_block"].AddTransition("set_search_velocity","wait_for_next_target");
      (*this)["search_for_unused_block"].AddTransition("wait_for_next_target", "turn_towards_target", [this] {
          STarget::TListIterator itTrackedTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                                                      std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                                                      [this] (const STarget& s_target) {
                                                                         return (s_target.Id != m_unTrackedTargetId);
                                                                      });
         
         if(itTrackedTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
            m_unTrackedTargetId = itTrackedTarget->Id;
            std::cerr << "Tracking target = " << m_unTrackedTargetId << std::endl;
            return true;
         }
         return false;
      });
      
      (*this)["search_for_unused_block"].AddTransition("turn_towards_target", "set_search_velocity", [this] {
         auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                      std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                      [this] (const STarget& s_target) {
                                         return (s_target.Id == m_unTrackedTargetId);
                                      });
         if(itTarget == std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
            std::cerr << "turn_towards_target: lost target " << m_unTrackedTargetId << std::endl;
            return true;
         }             
         for(const STarget& s_target : m_psSensorData->ImageSensor.Detections.Targets) {
            if(s_target.Id == itTarget->Id) {
               /* don't compare target with itself */
               continue;
            }
            else {
               /* if the two blocks are with MTT_BLOCK_SEP_THRESHOLD of each other, this is not an unused block */
               const SBlock& s_block_a = itTarget->Observations.front();
               const SBlock& s_block_b = s_target.Observations.front();
               float fNeighborBlockDist = argos::Distance(s_block_a.Translation, s_block_b.Translation);
               if(fNeighborBlockDist < MTT_BLOCK_SEP_THRESHOLD) {
                  std::cerr << "turn_towards_target: " << m_unTrackedTargetId << " and " << s_target.Id << " form a structure (" 
                            << fNeighborBlockDist << " < " << MTT_BLOCK_SEP_THRESHOLD << ")" << std::endl;
                  return true;
               }
            }
         }
         return false;
      });


      (*this)["search_for_unused_block"].AddTransition("turn_towards_target", "reverse_until_distance", [this] {
         auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                      std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                      [this] (const STarget& s_target) {
                                         return (s_target.Id == m_unTrackedTargetId);
                                      });
                                         
         if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            
            if(s_block.Tags.front().Center.first > (0.475f * 640.0f) && 
               s_block.Tags.front().Center.first < (0.525f * 640.0f)) {
               return true;
            }                
         }
         else {
            std::cerr << "turn_towards_target: lost target " << m_unTrackedTargetId  <<  std::endl;
         }
         return false;
      });

      (*this)["search_for_unused_block"].AddTransition("reverse_until_distance", "set_search_velocity", [this] {
         auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                      std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                      [this] (const STarget& s_target) {
                                         return (s_target.Id == m_unTrackedTargetId);
                                      });
         if(itTarget == std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
            std::cerr << "reverse_until_distance: lost target " << m_unTrackedTargetId << std::endl;
            return true;
         }             
         for(const STarget& s_target : m_psSensorData->ImageSensor.Detections.Targets) {
            if(s_target.Id == itTarget->Id) {
               /* don't compare target with itself */
               continue;
            }
            else {
               /* if the two blocks are with MTT_BLOCK_SEP_THRESHOLD of each other, this is not an unused block */
               const SBlock& s_block_a = itTarget->Observations.front();
               const SBlock& s_block_b = s_target.Observations.front();
               float fNeighborBlockDist = argos::Distance(s_block_a.Translation, s_block_b.Translation);
               if(fNeighborBlockDist < MTT_BLOCK_SEP_THRESHOLD) {
                  std::cerr << "reverse_until_distance: " << m_unTrackedTargetId << " belonged to a structure" << std::endl;
                  return true;
               }
            }
         }
         return false;
      });

      (*this)["search_for_unused_block"].AddExitTransition("reverse_until_distance", [this] {
         auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                      std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                      [this] (const STarget& s_target) {
                                         return (s_target.Id == m_unTrackedTargetId);
                                      });
      
         if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            std::cerr << "reverse_until_distance: target " << m_unTrackedTargetId << " is at Z = " << s_block.Translation.GetZ() << "m" << std::endl;
            return (s_block.Translation.GetZ() > TARGET_Z_DIST_REVERSE_TO);
         }
         return false; 
      });
      

      (*this).AddTransition("search_for_unused_block","far_block_approach");
      /* far_block_approach transitions */
      
      /*************************************************************************************************************************
      *************************************************************************************************************************/


      (*this)["far_block_approach"].AddTransition("turn_towards_target", "approach_block_straight", [this] {
         auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                      std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                      [this] (const STarget& s_target) {
                                         return (s_target.Id == m_unTrackedTargetId);
                                      });
         if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
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

      (*this)["far_block_approach"].AddTransition("turn_towards_target", "approach_block_right", [this] {
         auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                      std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                      [this] (const STarget& s_target) {
                                         return (s_target.Id == m_unTrackedTargetId);
                                      });
         if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if(s_block.Tags.front().Center.first > (0.475f * 640.0f) && 
               s_block.Tags.front().Center.first < (0.525f * 640.0f)) {
               argos::CRadians cEulerAngleZ, cEulerAngleY, cEulerAngleX;
               s_block.Rotation.ToEulerAngles(cEulerAngleZ, cEulerAngleY, cEulerAngleX);
               if(cEulerAngleZ.GetValue() <= -(M_PI / 18.0f)) {
                  m_eApproachDirection = EApproachDirection::RIGHT;
                  return true;
               }
            }                
         }
         return false;
      });
      
      (*this)["far_block_approach"].AddTransition("turn_towards_target", "approach_block_left", [this] {
         auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                      std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                      [this] (const STarget& s_target) {
                                         return (s_target.Id == m_unTrackedTargetId);
                                      });                                         
         if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if(s_block.Tags.front().Center.first > (0.475f * 640.0f) && 
               s_block.Tags.front().Center.first < (0.525f * 640.0f)) {
               argos::CRadians cEulerAngleZ, cEulerAngleY, cEulerAngleX;
               s_block.Rotation.ToEulerAngles(cEulerAngleZ, cEulerAngleY, cEulerAngleX);
               if(cEulerAngleZ.GetValue() >= (M_PI / 18.0f)) {
                  m_eApproachDirection = EApproachDirection::LEFT;
                  return true;
               }
            }
         }
         return false;
      });
      
      (*this)["far_block_approach"].AddExitTransition("approach_block_left", [this] {
         auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                      std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                      [this] (const STarget& s_target) {
                                         return (s_target.Id == m_unTrackedTargetId);
                                      });
      
         for(const STarget& s_target : m_psSensorData->ImageSensor.Detections.Targets) {
            if(s_target.Id == itTarget->Id) {
               /* don't compare target with itself */
               continue;
            }
            else {
               /* if the two blocks are with MTT_BLOCK_SEP_THRESHOLD of each other, this is not an unused block */
               const SBlock& s_block_a = itTarget->Observations.front();
               const SBlock& s_block_b = s_target.Observations.front();
               float fNeighborBlockDist = argos::Distance(s_block_a.Translation, s_block_b.Translation);
               if(fNeighborBlockDist < MTT_BLOCK_SEP_THRESHOLD) {
                  std::cerr << "approach_block_left: " << m_unTrackedTargetId << " belonged to a structure" << std::endl;
                  m_unTrackedTargetId = -1;
                  return true;
               }
            }
         }
         return false;
      });
      
      (*this)["far_block_approach"].AddExitTransition("approach_block_right", [this] {
         auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                      std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                      [this] (const STarget& s_target) {
                                         return (s_target.Id == m_unTrackedTargetId);
                                      });
      
         for(const STarget& s_target : m_psSensorData->ImageSensor.Detections.Targets) {
            if(s_target.Id == itTarget->Id) {
               /* don't compare target with itself */
               continue;
            }
            else {
               /* if the two blocks are with MTT_BLOCK_SEP_THRESHOLD of each other, this is not an unused block */
               const SBlock& s_block_a = itTarget->Observations.front();
               const SBlock& s_block_b = s_target.Observations.front();
               float fNeighborBlockDist = argos::Distance(s_block_a.Translation, s_block_b.Translation);
               if(fNeighborBlockDist < MTT_BLOCK_SEP_THRESHOLD) {
                  std::cerr << "approach_block_right: " << m_unTrackedTargetId << " belonged to a structure" << std::endl;
                  m_unTrackedTargetId = -1;
                  return true;
               }
            }
         }
         return false;
      });
      
      (*this)["far_block_approach"].AddExitTransition("approach_block_straight", [this] {
         auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                      std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                      [this] (const STarget& s_target) {
                                         return (s_target.Id == m_unTrackedTargetId);
                                      });
      
         for(const STarget& s_target : m_psSensorData->ImageSensor.Detections.Targets) {
            if(s_target.Id == itTarget->Id) {
               /* don't compare target with itself */
               continue;
            }
            else {
               /* if the two blocks are with MTT_BLOCK_SEP_THRESHOLD of each other, this is not an unused block */
               const SBlock& s_block_a = itTarget->Observations.front();
               const SBlock& s_block_b = s_target.Observations.front();
               float fNeighborBlockDist = argos::Distance(s_block_a.Translation, s_block_b.Translation);
               if(fNeighborBlockDist < MTT_BLOCK_SEP_THRESHOLD) {
                  std::cerr << "approach_block_straight: " << m_unTrackedTargetId << " belonged to a structure" << std::endl;
                  m_unTrackedTargetId = -1;
                  return true;
               }
            }
         }
         return false;
      });
     
      (*this)["far_block_approach"].AddTransition("approach_block_right", "align_end_effector", [this] {
         return (m_psSensorData->ImageSensor.Detections.Targets.size() == 0);
      });
      
      (*this)["far_block_approach"].AddTransition("approach_block_left", "align_end_effector", [this] {
         return (m_psSensorData->ImageSensor.Detections.Targets.size() == 0);
      });

      (*this)["far_block_approach"].AddTransition("approach_block_straight", "align_end_effector", [this] {
         return (m_psSensorData->ImageSensor.Detections.Targets.size() == 0);
      });

      (*this)["far_block_approach"]["align_end_effector"].AddTransition("lower_manipulator", "wait_for_manipulator");
      
      (*this)["far_block_approach"]["align_end_effector"].AddTransition("wait_for_manipulator", "set_reverse_velocity", [this] {
         return (m_psSensorData->ManipulatorModule.LiftActuator.State == CBlockDemo::ELiftActuatorSystemState::INACTIVE);
      });
      
      //(*this)["far_block_approach"]["align_end_effector"].AddTransition("set_reverse_velocity", "wait_for_target");
      
      (*this)["far_block_approach"]["align_end_effector"].AddExitTransition("set_reverse_velocity");
            
      (*this)["far_block_approach"]["align_end_effector"].AddTransition("wait_for_target", "turn_towards_target", [this] {
         if(m_psSensorData->ImageSensor.Detections.Targets.size() > 0) {
            m_unTrackedTargetId = m_psSensorData->ImageSensor.Detections.Targets.front().Id;
            return true;
         }
         return false;
      });
      
      (*this)["far_block_approach"]["align_end_effector"].AddExitTransition("turn_towards_target", [this] {
         auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                      std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                      [this] (const STarget& s_target) {
                                         return (s_target.Id == m_unTrackedTargetId);
                                      });
                                         
         if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
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
         return (m_unTrackedTargetId != -1);
      });
      
      (*this).AddTransition("far_block_approach", "init_end_effector_position", [this] {
         return (m_unTrackedTargetId == -1);
      });

      /*************************************************************************************************************************
      *************************************************************************************************************************/

      
      /* near_block_approach transitions */
      (*this)["near_block_approach"].AddTransition("init_near_block_approach","wait_for_underneath_rf");
      (*this)["near_block_approach"].AddTransition("wait_for_underneath_rf","set_approach_velocity", [this] {
         return (m_psSensorData->ManipulatorModule.RangeFinders.Underneath > 2300);
      });
      (*this)["near_block_approach"].AddTransition("set_approach_velocity", "set_zero_velocity", [this] {
         return (m_psSensorData->ManipulatorModule.RangeFinders.Left > 3000) &&
                (m_psSensorData->ManipulatorModule.RangeFinders.Right > 3000);
      });
      (*this)["near_block_approach"].AddExitTransition("set_zero_velocity");
           
      (*this).AddTransition("near_block_approach", "pick_up_block");

      /*************************************************************************************************************************
      *************************************************************************************************************************/

      /* pickup_block transistions */
      (*this)["pick_up_block"].AddTransition("lower_lift_actuator","wait_for_lift_actuator");
      (*this)["pick_up_block"].AddTransition("wait_for_lift_actuator", "align_block_magnets", [this] {
         return (m_psSensorData->ManipulatorModule.LiftActuator.State == 
                 CBlockDemo::ELiftActuatorSystemState::INACTIVE);
      });
      /* pickup_block.align_block_magnets */
      (*this)["pick_up_block"]["align_block_magnets"].AddTransition("enable_attachment_field","disable_attachment_field");
      (*this)["pick_up_block"]["align_block_magnets"].AddTransition("disable_attachment_field","wait_for_recharge");
      (*this)["pick_up_block"]["align_block_magnets"].AddExitTransition("wait_for_recharge", [this] {
         bool bChargeIsStable = true;
         for(unsigned int idx = 1; idx < m_psSensorData->ManipulatorModule.LiftActuator.Electromagnets.Charge.size(); idx++) {
            bChargeIsStable = bChargeIsStable &&
               (m_psSensorData->ManipulatorModule.LiftActuator.Electromagnets.Charge[idx] ==
                m_psSensorData->ManipulatorModule.LiftActuator.Electromagnets.Charge[idx - 1]);
         }
         return bChargeIsStable;
      });

      (*this)["pick_up_block"].AddTransition("align_block_magnets","attach_block");
      /* pickup_block.attach_block */
      (*this)["pick_up_block"]["attach_block"].AddTransition("enable_attachment_field","wait_for_discharge");
      (*this)["pick_up_block"]["attach_block"].AddTransition("wait_for_discharge","disable_attachment_field", [this] {
         return ((m_psSensorData->ManipulatorModule.LiftActuator.Electromagnets.Charge.size() > 0) &&
                 (m_psSensorData->ManipulatorModule.LiftActuator.Electromagnets.Charge.front() < 0x80));
      });
      (*this)["pick_up_block"]["attach_block"].AddExitTransition("disable_attachment_field");

      (*this)["pick_up_block"].AddTransition("attach_block", "raise_lift_actuator");
      /* pickup_block.raise_lift_actuator */
         
      (*this)["pick_up_block"]["raise_lift_actuator"].AddTransition("set_position","wait_for_lift_actuator");
      (*this)["pick_up_block"]["raise_lift_actuator"].AddExitTransition("wait_for_lift_actuator", [this] {
         if(m_psSensorData->ManipulatorModule.LiftActuator.State == CBlockDemo::ELiftActuatorSystemState::INACTIVE) {
            if(m_psSensorData->ManipulatorModule.RangeFinders.Underneath < 2300) {
               m_unTrackedTargetId = -1;
            }
            return true;
         }       
         return false;
      });

      (*this)["pick_up_block"].AddExitTransition("raise_lift_actuator");
      
      (*this).AddTransition("pick_up_block", "reattempt_block_pick_up", [this] {
         return (m_unTrackedTargetId == -1);
      });
      
      (*this).AddTransition("pick_up_block", "search_for_structure", [this] {
         return (m_unTrackedTargetId != -1);
      });
      
      /*************************************************************************************************************************
      *************************************************************************************************************************/
     
      
      (*this)["reattempt_block_pick_up"].AddTransition("set_reverse_velocity", "wait_for_target");
      
      (*this)["reattempt_block_pick_up"].AddExitTransition("wait_for_target", [this] {     
         if(m_psSensorData->ImageSensor.Detections.Targets.size() > 0) {
            m_unTrackedTargetId = m_psSensorData->ImageSensor.Detections.Targets.front().Id;
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
      
      (*this)["search_for_structure"].AddTransition("set_search_velocity","wait_for_next_target");
      (*this)["search_for_structure"].AddTransition("wait_for_next_target", "turn_towards_target", [this] {
         /* hack, swap to target with the most Q4 LED if available */
         auto itTargetWithMostQ4Leds = FindTargetWithMostQ4Leds(m_psSensorData->ImageSensor.Detections.Targets);
         if(itTargetWithMostQ4Leds != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
            if(m_unTrackedTargetId != itTargetWithMostQ4Leds->Id) {
               std::cerr << "swapping target from " << m_unTrackedTargetId << " to " << itTargetWithMostQ4Leds->Id << " because OpenCV is rubbish" << std::endl;
               m_unTrackedTargetId = itTargetWithMostQ4Leds->Id;
            }
         }
         STarget::TListIterator itTrackedTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                                                     std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                                                     [this] (const STarget& s_target) {
                                                                        return (s_target.Id != m_unTrackedTargetId);
                                                                     });
         if(itTrackedTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
            m_unTrackedTargetId = itTrackedTarget->Id;
            std::cerr << "Tracking target = " << m_unTrackedTargetId << std::endl;
            return true;
         }
         return false;
      });
      

      (*this)["search_for_structure"].AddTransition("turn_towards_target", "reverse_until_distance", [this] {
         /* hack, swap to target with the most Q4 LED if available */
         auto itTargetWithMostQ4Leds = FindTargetWithMostQ4Leds(m_psSensorData->ImageSensor.Detections.Targets);
         if(itTargetWithMostQ4Leds != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
            if(m_unTrackedTargetId != itTargetWithMostQ4Leds->Id) {
               std::cerr << "swapping target from " << m_unTrackedTargetId << " to " << itTargetWithMostQ4Leds->Id << " because OpenCV is rubbish" << std::endl;
               m_unTrackedTargetId = itTargetWithMostQ4Leds->Id;
            }
         }
         auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                      std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                      [this] (const STarget& s_target) {
                                         return (s_target.Id == m_unTrackedTargetId);
                                      });
                                         
         if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            
            if(s_block.Tags.front().Center.first > (0.475f * 640.0f) && 
               s_block.Tags.front().Center.first < (0.525f * 640.0f)) {
               return true;
            }                
         }
         else {
            std::cerr << "turn_towards_target: lost target " << m_unTrackedTargetId << std::endl;
         }
         return false;
      });
      
      (*this)["search_for_structure"].AddTransition("turn_towards_target", "set_search_velocity", [this] {
         auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                      std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                      [this] (const STarget& s_target) {
                                         return (s_target.Id == m_unTrackedTargetId);
                                      });
         if(itTarget == std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
            std::cerr << "turn_towards_target: lost target " << m_unTrackedTargetId << std::endl;
            m_unTrackedTargetId = -1;
            return true;
         }
         return false;
      });
         
      /**********************************************************************************************************/
      
      (*this)["search_for_structure"].AddTransition("reverse_until_distance", "set_search_velocity", [this] {
         /* hack, swap to target with the most Q4 LED if available */
         auto itTargetWithMostQ4Leds = FindTargetWithMostQ4Leds(m_psSensorData->ImageSensor.Detections.Targets);
         if(itTargetWithMostQ4Leds != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
            if(m_unTrackedTargetId != itTargetWithMostQ4Leds->Id) {
               std::cerr << "swapping target from " << m_unTrackedTargetId << " to " << itTargetWithMostQ4Leds->Id << " because OpenCV is rubbish" << std::endl;
               m_unTrackedTargetId = itTargetWithMostQ4Leds->Id;
            }
         }
         auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                      std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                      [this] (const STarget& s_target) {
                                         return (s_target.Id == m_unTrackedTargetId);
                                      });
         if(itTarget == std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
            std::cerr << "reverse_until_distance: lost target " << m_unTrackedTargetId  <<  std::endl;
            return true;
         }
         else {
            const SBlock& s_block = itTarget->Observations.front();
            auto itMatchingLed = std::find(std::begin(s_block.Tags.front().DetectedLeds),
                                           std::end(s_block.Tags.front().DetectedLeds),
                                           ELedState::Q4);            
            if(itMatchingLed == std::end(s_block.Tags.front().DetectedLeds)) {
               std::cerr << "reverse_until_distance: dropping target " << m_unTrackedTargetId  << ", couldn't find Q4 LED" <<  std::endl;
               /* could not find a Q4 LED */
               //return true;
            }
         }
         return false;
      });
     
      (*this)["search_for_structure"].AddExitTransition("reverse_until_distance", [this] {
         auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                      std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                      [this] (const STarget& s_target) {
                                         return (s_target.Id == m_unTrackedTargetId);
                                      });
      
         if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {           
            const SBlock& s_block = itTarget->Observations.front();
            if(s_block.Translation.GetZ() > TARGET_Z_DIST_REVERSE_TO) {
               return true;
            }
         }
         return false;
      });
      
      /**********************************************************************************************************/
     
      (*this).AddTransition("search_for_structure", "far_structure_approach");
      
      (*this)["far_structure_approach"].AddTransition("turn_towards_target", "approach_block_straight", [this] {
         /* hack, swap to target with the most Q4 LED if available */
         auto itTargetWithMostQ4Leds = FindTargetWithMostQ4Leds(m_psSensorData->ImageSensor.Detections.Targets);
         if(itTargetWithMostQ4Leds != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
            if(m_unTrackedTargetId != itTargetWithMostQ4Leds->Id) {
               std::cerr << "swapping target from " << m_unTrackedTargetId << " to " << itTargetWithMostQ4Leds->Id << " because OpenCV is rubbish" << std::endl;
               m_unTrackedTargetId = itTargetWithMostQ4Leds->Id;
            }
         }
         auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                      std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                      [this] (const STarget& s_target) {
                                         return (s_target.Id == m_unTrackedTargetId);
                                      });
         if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
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
         auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                      std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                      [this] (const STarget& s_target) {
                                         return (s_target.Id == m_unTrackedTargetId);
                                      });
         if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
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
         auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                      std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                      [this] (const STarget& s_target) {
                                         return (s_target.Id == m_unTrackedTargetId);
                                      });                                         
         if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
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
      
      (*this)["far_structure_approach"].AddExitTransition("turn_towards_target", [this] {
         auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                      std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                      [this] (const STarget& s_target) {
                                         return (s_target.Id == m_unTrackedTargetId);
                                      });                                         
         if(itTarget == std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
            std::cerr << "turn_towards_target: lost target " << m_unTrackedTargetId << std::endl;
            m_unTrackedTargetId = -1;
            return true;
         }
         return false;
      });
      
      (*this)["far_structure_approach"].AddExitTransition("approach_block_right", [this] {
         /* hack, swap to target with the most Q4 LED if available */
         auto itTargetWithMostQ4Leds = FindTargetWithMostQ4Leds(m_psSensorData->ImageSensor.Detections.Targets);
         if(itTargetWithMostQ4Leds != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
            if(m_unTrackedTargetId != itTargetWithMostQ4Leds->Id) {
               std::cerr << "swapping target from " << m_unTrackedTargetId << " to " << itTargetWithMostQ4Leds->Id << " because OpenCV is rubbish" << std::endl;
               m_unTrackedTargetId = itTargetWithMostQ4Leds->Id;
            }
         }
         if(itTargetWithMostQ4Leds == std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
            return true;
         }
         else {
            const SBlock& s_block = itTargetWithMostQ4Leds->Observations.front();
            if(s_block.Translation.GetZ() < 0.11) {
               return true;
            }
         }
         return false;
      });
      
      (*this)["far_structure_approach"].AddExitTransition("approach_block_left", [this] {
         /* hack, swap to target with the most Q4 LED if available */
         auto itTargetWithMostQ4Leds = FindTargetWithMostQ4Leds(m_psSensorData->ImageSensor.Detections.Targets);
         if(itTargetWithMostQ4Leds != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
            if(m_unTrackedTargetId != itTargetWithMostQ4Leds->Id) {
               std::cerr << "swapping target from " << m_unTrackedTargetId << " to " << itTargetWithMostQ4Leds->Id << " because OpenCV is rubbish" << std::endl;
               m_unTrackedTargetId = itTargetWithMostQ4Leds->Id;
            }
         }
         if(itTargetWithMostQ4Leds == std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
            return true;
         }
         else {
            const SBlock& s_block = itTargetWithMostQ4Leds->Observations.front();
            if(s_block.Translation.GetZ() < 0.11) {
               return true;
            }
         }
         return false;
      });

      (*this)["far_structure_approach"].AddExitTransition("approach_block_straight", [this] {
         /* hack, swap to target with the most Q4 LED if available */
         auto itTargetWithMostQ4Leds = FindTargetWithMostQ4Leds(m_psSensorData->ImageSensor.Detections.Targets);
         if(itTargetWithMostQ4Leds != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
            if(m_unTrackedTargetId != itTargetWithMostQ4Leds->Id) {
               std::cerr << "swapping target from " << m_unTrackedTargetId << " to " << itTargetWithMostQ4Leds->Id << " because OpenCV is rubbish" << std::endl;
               m_unTrackedTargetId = itTargetWithMostQ4Leds->Id;
            }
         }
         if(itTargetWithMostQ4Leds == std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
            return true;
         }
         else {
            const SBlock& s_block = itTargetWithMostQ4Leds->Observations.front();
            if(s_block.Translation.GetZ() < 0.11) {
               return true;
            }
         }
         return false;
      });
      
      (*this).AddTransition("far_structure_approach", "near_structure_approach", [this] {
         return (m_unTrackedTargetId != -1);
      });
      
      (*this).AddTransition("far_structure_approach", "search_for_structure", [this] {
         return (m_unTrackedTargetId == -1);
      });

      
      (*this)["near_structure_approach"].AddTransition("set_manipulator_height", "wait_for_manipulator");
      
      (*this)["near_structure_approach"].AddTransition("wait_for_manipulator", "set_approach_velocity", [this] {
         return (m_psSensorData->ManipulatorModule.LiftActuator.State == CBlockDemo::ELiftActuatorSystemState::INACTIVE);
      });
      
      (*this)["near_structure_approach"].AddTransition("set_approach_velocity", "wait_until_approach_timer_expired");
      
      (*this)["near_structure_approach"].AddTransition("wait_until_approach_timer_expired", "set_reverse_velocity", [this] {
         return (m_psSensorData->Clock.Time - m_fNearApproachStartTime > 10.0f);
      });
      
      (*this)["near_structure_approach"].AddTransition("set_reverse_velocity", "wait_until_reverse_timer_expired");
      
      (*this)["near_structure_approach"].AddTransition("wait_until_reverse_timer_expired", "set_zero_velocity", [this] {
         return (m_psSensorData->Clock.Time - m_fNearApproachStartTime > 2.0f);
      });

      (*this)["near_structure_approach"].AddExitTransition("set_zero_velocity");
      
      (*this).AddTransition("near_structure_approach", "place_block");
      
      (*this)["place_block"].AddTransition("wait_until_charged", "enable_detachment_field", [this] {
         bool bChargeIsStable = true;
         for(unsigned int idx = 1; idx < m_psSensorData->ManipulatorModule.LiftActuator.Electromagnets.Charge.size(); idx++) {
            bChargeIsStable = bChargeIsStable &&
               (m_psSensorData->ManipulatorModule.LiftActuator.Electromagnets.Charge[idx] ==
                m_psSensorData->ManipulatorModule.LiftActuator.Electromagnets.Charge[idx - 1]);
         }
         return bChargeIsStable;
      });
      
      (*this)["place_block"].AddTransition("enable_detachment_field", "disable_detachment_field");
      (*this)["place_block"].AddExitTransition("disable_detachment_field");
      
      
      (*this).AddTransition("place_block", "reverse");
      
      (*this)["reverse"].AddTransition("set_reverse_velocity", "wait_until_reverse_timer_expired");
      
      (*this)["reverse"].AddExitTransition("wait_until_reverse_timer_expired", [this] {
         return (m_psSensorData->Clock.Time - m_fReverseStartTime > 15.0f);
      });

      
      (*this).AddExitTransition("reverse");

   }

private:
   CBlockDemo::SSensorData* m_psSensorData;
   CBlockDemo::SActuatorData* m_psActuatorData;
   
   void TrackBlockViaManipulatorHeight(const SBlock& s_block, unsigned int un_min_position = MTT_LIFT_ACTUATOR_OFFSET_HEIGHT, unsigned int un_max_position = 140u) {
      float fEndEffectorPos = m_psSensorData->ManipulatorModule.LiftActuator.EndEffector.Position;                
      fEndEffectorPos += MTT_LIFT_ACTUATOR_INCREMENT * (180.0f - s_block.Tags.front().Center.second) / 180.0f;
      uint8_t unEndEffectorPos = 
         (fEndEffectorPos > un_max_position) ? un_max_position : (fEndEffectorPos < un_min_position) ? un_min_position : static_cast<uint8_t>(fEndEffectorPos);

      m_psActuatorData->ManipulatorModule.LiftActuator.Position.Value = unEndEffectorPos;
      m_psActuatorData->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
   }
   
   
   STarget::TList::const_iterator FindTargetWithMostQ4Leds(const STarget::TList& s_target_list) {
      STarget::TList::const_iterator itTargetWithMostQ4Leds = std::end(s_target_list);
      unsigned int unTargetWithMostQ4LedsCount = 0;      
      for(STarget::TList::const_iterator it_target = std::begin(s_target_list);
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
   
   unsigned int m_unTrackedTargetId = -1;
   
   enum class EApproachDirection {
      LEFT, RIGHT, STRAIGHT
   } m_eApproachDirection;
   
   double m_fNearApproachStartTime, m_fReverseStartTime;
};

#endif
