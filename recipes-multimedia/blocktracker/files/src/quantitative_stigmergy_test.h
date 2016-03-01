#ifndef MANIPULATOR_TESTING_TASK_H
#define MANIPULATOR_TESTING_TASK_H

#include "state.h"
#include "block_demo.h"

#include <iostream>

#define MTT_LIFT_ACTUATOR_MAX_HEIGHT 140
#define MTT_LIFT_ACTUATOR_OFFSET_HEIGHT 0

#define MTT_LIFT_ACTUATOR_INCREMENT 15

#define MTT_BLOCK_SEP_THRESHOLD 0.065

#define BASE_VELOCITY 60

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
               m_psActuatorData->ManipulatorModule.LiftActuator.Position.Value = MTT_LIFT_ACTUATOR_MAX_HEIGHT;
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
                  TrackBlockViaManipulatorHeight(s_block);
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
                  TrackBlockViaManipulatorHeight(s_block);
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
                  TrackBlockViaManipulatorHeight(s_block);
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
                  TrackBlockViaManipulatorHeight(s_block);
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
                  float fTagXOffsetTarget = 0.5f;

                  fRight = (1 + fTagXOffsetTarget - fTagXOffset) * BASE_VELOCITY;
                  fLeft  = (1 + fTagXOffset - fTagXOffsetTarget) * BASE_VELOCITY;
                                                    
                  TrackBlockViaManipulatorHeight(s_block);
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
                  float fTagXOffsetTarget = -0.5f;

                  fRight = (1 + fTagXOffsetTarget - fTagXOffset) * BASE_VELOCITY;
                  fLeft  = (1 + fTagXOffset - fTagXOffsetTarget) * BASE_VELOCITY;
                                                    
                  TrackBlockViaManipulatorHeight(s_block);
               }
               m_psActuatorData->DifferentialDriveSystem.Left.Velocity = std::round(fLeft);
               m_psActuatorData->DifferentialDriveSystem.Right.Velocity = std::round(fRight);
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
            CState("align_end_effector", nullptr, nullptr, {
               CState("lower_manipulator", [this] {              
                  m_psActuatorData->ManipulatorModule.LiftActuator.Position.Value = 5;
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
               m_psActuatorData->DifferentialDriveSystem.Left.Velocity = BASE_VELOCITY;
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.Velocity = BASE_VELOCITY;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
               /* computer vision isn't used during the near block approach */
               m_psSensorData->ImageSensor.Enable = false;
               /* lower the manipulator */
               m_psActuatorData->ManipulatorModule.LiftActuator.Position.Value = (MTT_LIFT_ACTUATOR_OFFSET_HEIGHT + 5);
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
                  m_psActuatorData->ManipulatorModule.LiftActuator.Position.Value = MTT_LIFT_ACTUATOR_MAX_HEIGHT;
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
            }),
            CState("wait_for_target"),
         }),
         CState("search_for_structure", nullptr, nullptr, {
            CState("set_search_velocity", [this] {
               m_psSensorData->ImageSensor.Enable = true;
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
                  TrackBlockViaManipulatorHeight(s_block);
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
                  TrackBlockViaManipulatorHeight(s_block);
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
                  TrackBlockViaManipulatorHeight(s_block);
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
                  TrackBlockViaManipulatorHeight(s_block);
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
                  float fTagXOffsetTarget = 0.5f;

                  fRight = (1 + fTagXOffsetTarget - fTagXOffset) * BASE_VELOCITY;
                  fLeft  = (1 + fTagXOffset - fTagXOffsetTarget) * BASE_VELOCITY;
                                                    
                  TrackBlockViaManipulatorHeight(s_block);
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
                  float fTagXOffsetTarget = -0.5f;

                  fRight = (1 + fTagXOffsetTarget - fTagXOffset) * BASE_VELOCITY;
                  fLeft  = (1 + fTagXOffset - fTagXOffsetTarget) * BASE_VELOCITY;
                                                    
                  TrackBlockViaManipulatorHeight(s_block);
               }
               m_psActuatorData->DifferentialDriveSystem.Left.Velocity = std::round(fLeft);
               m_psActuatorData->DifferentialDriveSystem.Right.Velocity = std::round(fRight);
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
         }),
         CState("place_block", nullptr, nullptr, {
            CState("set_block_placement_parameters", [this] {
               m_psActuatorData->DifferentialDriveSystem.Left.Velocity = 0;
               m_psActuatorData->DifferentialDriveSystem.Right.Velocity = 0;
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
               m_psActuatorData->ManipulatorModule.LiftActuator.Position.Value = 28;
               m_psActuatorData->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
            }),
            CState("wait_until_ready"),
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
          std::list<STarget>::iterator itTrackedTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
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
               float fNeighborBlockDist = sqrt(pow(s_block_a.Translation.X - s_block_b.Translation.X, 2) +
                                               pow(s_block_a.Translation.Y - s_block_b.Translation.Y, 2) +
                                               pow(s_block_a.Translation.Z - s_block_b.Translation.Z, 2));
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
               float fNeighborBlockDist = sqrt(pow(s_block_a.Translation.X - s_block_b.Translation.X, 2) +
                                               pow(s_block_a.Translation.Y - s_block_b.Translation.Y, 2) +
                                               pow(s_block_a.Translation.Z - s_block_b.Translation.Z, 2));
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
            std::cerr << "reverse_until_distance: target " << m_unTrackedTargetId << " is at Z = " << s_block.Translation.Z << "m" << std::endl;
            return (s_block.Translation.Z > 0.30);
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
               if(s_block.Rotation.Z > -(M_PI / 18.0f) &&
                  s_block.Rotation.Z <  (M_PI / 18.0f)) {
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
               if(s_block.Rotation.Z <= -(M_PI / 18.0f)) {
                  std::cerr << "turn_towards_target: selected right approach, Rot(Z) is " << s_block.Rotation.Z << std::endl;
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
               if(s_block.Rotation.Z >= (M_PI / 18.0f)) {
                  std::cerr << "turn_towards_target: selected left approach, Rot(Z) is " << s_block.Rotation.Z << std::endl;
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
               float fNeighborBlockDist = sqrt(pow(s_block_a.Translation.X - s_block_b.Translation.X, 2) +
                                               pow(s_block_a.Translation.Y - s_block_b.Translation.Y, 2) +
                                               pow(s_block_a.Translation.Z - s_block_b.Translation.Z, 2));
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
               float fNeighborBlockDist = sqrt(pow(s_block_a.Translation.X - s_block_b.Translation.X, 2) +
                                               pow(s_block_a.Translation.Y - s_block_b.Translation.Y, 2) +
                                               pow(s_block_a.Translation.Z - s_block_b.Translation.Z, 2));
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
               float fNeighborBlockDist = sqrt(pow(s_block_a.Translation.X - s_block_b.Translation.X, 2) +
                                               pow(s_block_a.Translation.Y - s_block_b.Translation.Y, 2) +
                                               pow(s_block_a.Translation.Z - s_block_b.Translation.Z, 2));
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
      
      (*this)["far_block_approach"]["align_end_effector"].AddTransition("set_reverse_velocity", "wait_for_target");
            
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
          std::list<STarget>::iterator itTrackedTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
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
      
      (*this)["search_for_structure"].AddTransition("reverse_until_distance", "set_search_velocity", [this] {
         auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                      std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                      [this] (const STarget& s_target) {
                                         return (s_target.Id == m_unTrackedTargetId);
                                      });
         if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if(s_block.Translation.Z > 0.30) {
               bool bBlockBelongsToStructure = false;
               for(const STarget& s_target_other : m_psSensorData->ImageSensor.Detections.Targets) {
                  if(s_target_other.Id == itTarget->Id) {
                     /* don't compare target with itself */
                     continue;
                  }
                  else {
                     const SBlock& s_block_other = s_target_other.Observations.front();
                     float fNeighborBlockDist = sqrt(pow(s_block.Translation.X - s_block_other.Translation.X, 2) +
                                                     pow(s_block.Translation.Y - s_block_other.Translation.Y, 2) +
                                                     pow(s_block.Translation.Z - s_block_other.Translation.Z, 2));
                     
                     if(fNeighborBlockDist < MTT_BLOCK_SEP_THRESHOLD) {
                        bBlockBelongsToStructure = true;
                        float fTrackedTargetDist = sqrt(pow(s_block.Translation.X, 2) +
                                                        pow(s_block.Translation.Y, 2) +
                                                        pow(s_block.Translation.Z, 2));
                        float fOtherTargetDist = sqrt(pow(s_block_other.Translation.X, 2) +
                                                      pow(s_block_other.Translation.Y, 2) +
                                                      pow(s_block_other.Translation.Z, 2));
                        if(fTrackedTargetDist > fOtherTargetDist) {
                           std::cerr << "reverse_until_distance: swapping target from " << m_unTrackedTargetId  << " to " << s_target_other.Id << std::endl;
                           m_unTrackedTargetId = s_target_other.Id;
                        }
                     }
                  }
               }
               /* if this block appears to belong to a structure, don't transition */
               return !bBlockBelongsToStructure;
            }
            else {
               /* s_block.Translation.Z <= 0.30 */
               return false;
            }
         }
         else {
            std::cerr << "reverse_until_distance: lost target " << m_unTrackedTargetId  <<  std::endl;
            return true;
         }
         /* s_block.Translation.Z > 0.30 and we couldn't find any other blocks with in MTT_BLOCK_SEP_THRESHOLD of the target */
         /* assume it isn't part of a structure */
         return true;
      });
     
      (*this)["search_for_structure"].AddExitTransition("reverse_until_distance", [this] {
         auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                      std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                      [this] (const STarget& s_target) {
                                         return (s_target.Id == m_unTrackedTargetId);
                                      });
      
         if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {           
            const SBlock& s_block = itTarget->Observations.front();
            if(s_block.Translation.Z > 0.30) {
               bool bBlockBelongsToStructure = false;
               for(const STarget& s_target_other : m_psSensorData->ImageSensor.Detections.Targets) {
                  if(s_target_other.Id == itTarget->Id) {
                     /* don't compare target with itself */
                     continue;
                  }
                  else {
                     const SBlock& s_block_other = s_target_other.Observations.front();
                     float fNeighborBlockDist = sqrt(pow(s_block.Translation.X - s_block_other.Translation.X, 2) +
                                                     pow(s_block.Translation.Y - s_block_other.Translation.Y, 2) +
                                                     pow(s_block.Translation.Z - s_block_other.Translation.Z, 2));
                     
                     if(fNeighborBlockDist < MTT_BLOCK_SEP_THRESHOLD) {
                        bBlockBelongsToStructure = true;
                        float fTrackedTargetDist = sqrt(pow(s_block.Translation.X, 2) +
                                                        pow(s_block.Translation.Y, 2) +
                                                        pow(s_block.Translation.Z, 2));
                        float fOtherTargetDist = sqrt(pow(s_block_other.Translation.X, 2) +
                                                      pow(s_block_other.Translation.Y, 2) +
                                                      pow(s_block_other.Translation.Z, 2));
                        if(fTrackedTargetDist > fOtherTargetDist) {
                           std::cerr << "reverse_until_distance: swapping target from " << m_unTrackedTargetId  << " to " << s_target_other.Id << std::endl;
                           m_unTrackedTargetId = s_target_other.Id;
                        }
                     }
                  }
               }
               /* return if we have found a structure */
               return bBlockBelongsToStructure;
            }
         }
         /* we couldn't find any other blocks with in MTT_BLOCK_SEP_THRESHOLD of the target */
         /* assume it isn't part of a structure */
         return false;
      });
     
      (*this).AddTransition("search_for_structure", "far_structure_approach");
      
      (*this)["far_structure_approach"].AddTransition("turn_towards_target", "approach_block_straight", [this] {
         auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                      std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                      [this] (const STarget& s_target) {
                                         return (s_target.Id == m_unTrackedTargetId);
                                      });
         if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if(s_block.Tags.front().Center.first > (0.475f * 640.0f) && 
               s_block.Tags.front().Center.first < (0.525f * 640.0f)) {
               if(s_block.Rotation.Z > -(M_PI / 18.0f) &&
                  s_block.Rotation.Z <  (M_PI / 18.0f)) {
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
               if(s_block.Rotation.Z <= -(M_PI / 18.0f)) {
                  std::cerr << "turn_towards_target: selected right approach, Rot(Z) is " << s_block.Rotation.Z << std::endl;
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
               if(s_block.Rotation.Z >= (M_PI / 18.0f)) {
                  std::cerr << "turn_towards_target: selected left approach, Rot(Z) is " << s_block.Rotation.Z << std::endl;
                  return true;
               }
            }
         }
         return false;
      });
      
      (*this)["far_structure_approach"].AddExitTransition("approach_block_right", [this] {
         return (m_psSensorData->ImageSensor.Detections.Targets.size() == 0);
      });
      
      (*this)["far_structure_approach"].AddExitTransition("approach_block_left", [this] {
         return (m_psSensorData->ImageSensor.Detections.Targets.size() == 0);
      });

      (*this)["far_structure_approach"].AddExitTransition("approach_block_straight", [this] {
         return (m_psSensorData->ImageSensor.Detections.Targets.size() == 0);
      });

      
      (*this).AddTransition("far_structure_approach", "place_block");
      
      (*this)["place_block"].AddTransition("set_block_placement_parameters", "wait_until_ready");
      
      (*this)["place_block"].AddTransition("wait_until_ready", "enable_detachment_field", [this] {
         bool bChargeIsStable = true;
         for(unsigned int idx = 1; idx < m_psSensorData->ManipulatorModule.LiftActuator.Electromagnets.Charge.size(); idx++) {
            bChargeIsStable = bChargeIsStable &&
               (m_psSensorData->ManipulatorModule.LiftActuator.Electromagnets.Charge[idx] ==
                m_psSensorData->ManipulatorModule.LiftActuator.Electromagnets.Charge[idx - 1]);
         }
         return bChargeIsStable && (m_psSensorData->ManipulatorModule.LiftActuator.State == CBlockDemo::ELiftActuatorSystemState::INACTIVE);
      });
      
      (*this)["place_block"].AddTransition("enable_detachment_field", "disable_detachment_field");
      (*this)["place_block"].AddExitTransition("disable_detachment_field");
      
      
      (*this).AddExitTransition("place_block");

   }

private:
   CBlockDemo::SSensorData* m_psSensorData;
   CBlockDemo::SActuatorData* m_psActuatorData;
   
   void TrackBlockViaManipulatorHeight(const SBlock& s_block) {
      float fEndEffectorPos = m_psSensorData->ManipulatorModule.LiftActuator.EndEffector.Position;                
      fEndEffectorPos += MTT_LIFT_ACTUATOR_INCREMENT * (180.0f - s_block.Tags.front().Center.second) / 180.0f;
      uint8_t unEndEffectorPos = 
         (fEndEffectorPos > 140.0f) ? 140u : (fEndEffectorPos < (MTT_LIFT_ACTUATOR_OFFSET_HEIGHT + 5.0f)) ? (MTT_LIFT_ACTUATOR_OFFSET_HEIGHT + 5u) : static_cast<uint8_t>(fEndEffectorPos);

      m_psActuatorData->ManipulatorModule.LiftActuator.Position.Value = unEndEffectorPos;
      m_psActuatorData->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
   }
   
   unsigned int m_unTrackedTargetId = -1;
};

#endif
