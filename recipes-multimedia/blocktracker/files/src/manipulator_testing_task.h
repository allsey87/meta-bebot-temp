#ifndef MANIPULATOR_TESTING_TASK_H
#define MANIPULATOR_TESTING_TASK_H

#include "state.h"
#include "block_demo.h"

#include <iostream>

#define MTT_LIFT_ACTUATOR_MAX_HEIGHT 140
#define MTT_LIFT_ACTUATOR_OFFSET_HEIGHT 5

#define MTT_LIFT_ACTUATOR_INCREMENT 10

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
               m_psActuatorData->ManipulatorModule.LiftActuator.Position.Value = MTT_LIFT_ACTUATOR_MAX_HEIGHT / 2;
               m_psActuatorData->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
            }),
            CState("wait_for_lift_actuator"),
         }),
         CState("search_for_target", nullptr, nullptr, {
            CState("set_spot_turn_velocity", [this] {
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
            CState("wait_for_target"),
         }),
         CState("far_block_approach", nullptr, nullptr, {
            CState("set_alignment_velocity", [this] {
               float fLeft = 0.0f, fRight = 0.0f;
               /* search for target zero */
               auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                            std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                            [this] (const STarget& s_target) {
                                               return (s_target.Id == m_unTrackedTargetId);
                                            });
                                         
               if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
                  const SBlock& s_block = itTarget->Observations.front();

                  /* During approach s_block.Rotation.Z => 0, s_block.Translation.X => 0, s_block.Translation.Z => 0.18 */
                  std::cerr << "Target: X = " << s_block.Translation.X << ", Z = " << s_block.Translation.Z << std::endl;
                  std::cerr << "Rot(Z, Y, X) = " << s_block.Rotation.Z << ", " << s_block.Rotation.Y << ", " << s_block.Rotation.X << std::endl;
                  
                  std::cerr << "TTC(X, Y) = " << s_block.Tags.front().Center.first << ", " << s_block.Tags.front().Center.second << std::endl;                  
                  
                  float fTagXOffset = (s_block.Tags.front().Center.first - 320.0f) / 320.0f;

                  float fTagXOffsetTarget = 0.75f * (std::signbit(fTagXOffset) ? -1.0f : 1.0f);

                  fRight = (1 + fTagXOffsetTarget - fTagXOffset) * BASE_VELOCITY;
                  fLeft  = (1 + fTagXOffset - fTagXOffsetTarget) * BASE_VELOCITY;
                  
                                   
                  float fEndEffectorPos = m_psSensorData->ManipulatorModule.LiftActuator.EndEffector.Position;                
                  fEndEffectorPos += MTT_LIFT_ACTUATOR_INCREMENT * (180.0f - s_block.Tags.front().Center.second) / 180.0f;
                  uint8_t unEndEffectorPos = 
                     (fEndEffectorPos > 140.0f) ? 140u : (fEndEffectorPos < MTT_LIFT_ACTUATOR_OFFSET_HEIGHT) ? MTT_LIFT_ACTUATOR_OFFSET_HEIGHT : static_cast<uint8_t>(fEndEffectorPos);
                  
                  m_psActuatorData->ManipulatorModule.LiftActuator.Position.Value = unEndEffectorPos;
                  m_psActuatorData->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
                  
                  std::cerr << "Vout(L,R) = " << fLeft << ", " << fRight << std::endl;

               }
               m_psActuatorData->DifferentialDriveSystem.Left.Velocity = std::round(fLeft);
               m_psActuatorData->DifferentialDriveSystem.Right.Velocity = std::round(fRight);
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
            CState("clockwise_spot_turn", [this] {
               m_psActuatorData->DifferentialDriveSystem.Left.Velocity = BASE_VELOCITY * 0.5;
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.Velocity = -BASE_VELOCITY * 0.5;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
            CState("anticlockwise_spot_turn", [this] {
               m_psActuatorData->DifferentialDriveSystem.Left.Velocity = -BASE_VELOCITY * 0.5;
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.Velocity = BASE_VELOCITY * 0.5;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
            CState("turn_and_face_target", [this] {
               auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                            std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                            [this] (const STarget& s_target) {
                                               return (s_target.Id == m_unTrackedTargetId);
                                            });

               if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
                  const SBlock& s_block = itTarget->Observations.front();            
               
                  if(s_block.Translation.X < 0) {
                     m_psActuatorData->DifferentialDriveSystem.Left.Velocity = -BASE_VELOCITY;
                     m_psActuatorData->DifferentialDriveSystem.Right.Velocity = BASE_VELOCITY;
                  }
                  else {
                     m_psActuatorData->DifferentialDriveSystem.Left.Velocity = BASE_VELOCITY;
                     m_psActuatorData->DifferentialDriveSystem.Right.Velocity = -BASE_VELOCITY;
                  }
               }
               else {
                  m_psActuatorData->DifferentialDriveSystem.Left.Velocity = 0;
                  m_psActuatorData->DifferentialDriveSystem.Right.Velocity = 0;
               }
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
            CState("set_approach_velocity", [this] {
               m_psActuatorData->DifferentialDriveSystem.Left.Velocity = BASE_VELOCITY;
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.Velocity = BASE_VELOCITY;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
            })
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
                  m_psActuatorData->ManipulatorModule.LiftActuator.Position.Value = 60;
                  m_psActuatorData->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
                  m_psActuatorData->ManipulatorModule.EndEffector.UpdateReq = true;
                  m_psActuatorData->ManipulatorModule.NFCInterface.OutboundMessage = "3";
                  m_psActuatorData->ManipulatorModule.NFCInterface.UpdateReq = true;
               }),
               CState("wait_for_lift_actuator"),
            }), // raise_lift_actuator
         }), // pick up block
      }) {
        
      
      (*this)["init_end_effector_position"].AddTransition("raise_lift_actuator","wait_for_lift_actuator");
      (*this)["init_end_effector_position"].AddExitTransition("wait_for_lift_actuator", [this] {
         return (m_psSensorData->ManipulatorModule.LiftActuator.State == 
                 CBlockDemo::ELiftActuatorSystemState::INACTIVE);
      });

      (*this).AddTransition("init_end_effector_position", "search_for_target");
      /* search_for_target transitions */
      (*this)["search_for_target"].AddTransition("set_spot_turn_velocity","wait_for_target");
      (*this)["search_for_target"].AddExitTransition("wait_for_target", [this] {
         return (m_psSensorData->ImageSensor.Detections.Targets.size() > 0);
      });

      (*this).AddTransition("search_for_target","far_block_approach");
      /* far_block_approach transitions */
      /*
      (*this)["far_block_approach"].AddTransition("set_alignment_velocity", "anticlockwise_spot_turn", [this] {
         auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                      std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                      [this] (const STarget& s_target) {
                                         return (s_target.Id == m_unTrackedTargetId);
                                      });
                                         
         if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if(s_block.Tags.front().Center.first < (0.25f * 640.0f)) {
               return true;
            }             
         }
         return false;
      });
      */
      
      (*this)["far_block_approach"].AddTransition("anticlockwise_spot_turn", "set_alignment_velocity", [this] {
         auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                      std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                      [this] (const STarget& s_target) {
                                         return (s_target.Id == m_unTrackedTargetId);
                                      });
                                         
         if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if(s_block.Tags.front().Center.first > (0.40f * 640.0f)) {
               return true;      
            }             
         }
         return false;
      });
      
      /*
      (*this)["far_block_approach"].AddTransition("set_alignment_velocity", "clockwise_spot_turn", [this] {
         auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                      std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                      [this] (const STarget& s_target) {
                                         return (s_target.Id == m_unTrackedTargetId);
                                      });
                                         
         if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if(s_block.Tags.front().Center.first > (0.75f * 640.0f)) {
               return true;
            }             
         }
         return false;
      });
      */
      
      (*this)["far_block_approach"].AddTransition("clockwise_spot_turn", "set_alignment_velocity", [this] {
         auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                      std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                      [this] (const STarget& s_target) {
                                         return (s_target.Id == m_unTrackedTargetId);
                                      });
                                         
         if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if(s_block.Tags.front().Center.first < (0.60f * 640.0f)) {
               return true;
            }             
         }
         return false; /* condition */
      });
          
      (*this)["far_block_approach"].AddTransition("set_alignment_velocity", "turn_and_face_target", [this] {
         auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                      std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                      [this] (const STarget& s_target) {
                                         return (s_target.Id == m_unTrackedTargetId);
                                      });
                                         
         if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            
            float fZOffsetA = s_block.Translation.Z - s_block.Translation.X * tan(s_block.Rotation.Z);
            float fZOffsetB = s_block.Translation.Z - s_block.Translation.X * tan(s_block.Rotation.Z + M_PI_2);
            
            if((std::abs(fZOffsetA) < 0.05) || (std::abs(fZOffsetB) < 0.05)) {
               false;            
            }
         }
         return false;
      });
      
      (*this)["far_block_approach"].AddTransition("turn_and_face_target", "set_approach_velocity", [this] {
         auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                      std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                      [this] (const STarget& s_target) {
                                         return (s_target.Id == m_unTrackedTargetId);
                                      });
                                         
         if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            /* +/- 10 degrees is acceptable */
            if(std::abs(s_block.Rotation.Z) < M_PI / 18) {
               true;
            }
         }
         return false;
      });
      
      (*this)["far_block_approach"].AddExitTransition("set_approach_velocity", [this] {
         auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                      std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                      [this] (const STarget& s_target) {
                                         return (s_target.Id == m_unTrackedTargetId);
                                      });                               
         if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if(s_block.Translation.Z < 0.18) {
               return true;
            }
         }
      });


      (*this).AddTransition("far_block_approach", "near_block_approach");
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
         return (m_psSensorData->ManipulatorModule.LiftActuator.State == 
                 CBlockDemo::ELiftActuatorSystemState::INACTIVE);
      });

      (*this)["pick_up_block"].AddExitTransition("raise_lift_actuator");
      (*this).AddExitTransition("pick_up_block");
   }

private:
   CBlockDemo::SSensorData* m_psSensorData;
   CBlockDemo::SActuatorData* m_psActuatorData;
   
   unsigned int m_unTrackedTargetId = 0;
};

#endif
