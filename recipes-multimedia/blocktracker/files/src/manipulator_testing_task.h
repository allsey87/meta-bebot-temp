#ifndef MANIPULATOR_TESTING_TASK_H
#define MANIPULATOR_TESTING_TASK_H

#include "state.h"
#include "block_demo.h"

#include <iostream>

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
               for(CBlockDemo::EColor& e_color : m_psActuatorData->LEDDeck.Color) 
                  e_color = CBlockDemo::EColor::RED;
               for(bool& b_update : m_psActuatorData->LEDDeck.UpdateReq)
                  b_update = true;
               m_psActuatorData->ManipulatorModule.LiftActuator.Position.Value = 140;
               m_psActuatorData->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
            }),
            CState("wait_for_lift_actuator"),
         }),
         CState("search_for_target", nullptr, nullptr, {
            CState("set_spot_turn_velocity", [this] {
               m_psActuatorData->DifferentialDriveSystem.Power.Enable = true;
               m_psActuatorData->DifferentialDriveSystem.Power.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Left.Velocity = -8;
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.Velocity = 8;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
            CState("wait_for_target"),
         }),
         CState("far_block_approach", nullptr, nullptr, {
            CState("set_alignment_velocity", [this] {
               float fLeft = 0.0f, fRight = 0.0f;
               /* search for target zero */
               auto itTarget = std::find_if(std::begin(m_psSensorData->ImageSensor.Detections.Targets),
                                            std::end(m_psSensorData->ImageSensor.Detections.Targets),
                                            [] (const STarget& s_target) {
                                               return (s_target.Id == 0);
                                            });
                                         
               if(itTarget != std::end(m_psSensorData->ImageSensor.Detections.Targets)) {
                  const SBlock& s_block = itTarget->Observations.front();

                  /* During approach s_block.Rotation.Z => 0, s_block.Translation.X => 0, s_block.Translation.Z => 0.18 */
                  std::cerr << "Target: X = " << s_block.Translation.X << ", Z = " << s_block.Translation.Z << std::endl;
                  
                  /* copy and saturate s_block.Translation.X into fOffsetFromCenter */                  
                  float fOffsetFromCenter = (std::abs(s_block.Translation.X) > 0.1f ? 0.1f : std::abs(s_block.Translation.X)) *
                                            (std::signbit(s_block.Translation.X) ? -1.0f : 1.0f);
                                    
                  if(fOffsetFromCenter < 0) {
                     fLeft = -50.0f * fOffsetFromCenter + 10.0f;
                     fRight = -150.0f * fOffsetFromCenter + 10.0f;
                  }
                  else {
                     fLeft = 150.0f * fOffsetFromCenter + 10.0f;
                     fRight = 50.0f * fOffsetFromCenter + 10.0f;
                  }
               }
               else {
                  /* transition back to spot turn or signal exit? */
                  std::cerr << "Target Lost" << std::endl;
                  fLeft = 0.0f;
                  fRight = 0.0f;
               }
               
               /* reduce speed as approaching block */
               /* low manipulator as approaching block */
           
               m_psActuatorData->DifferentialDriveSystem.Left.Velocity = std::round(fLeft);
               m_psActuatorData->DifferentialDriveSystem.Right.Velocity = std::round(fRight);
               
               std::cerr << "Output Velocity" << " Left = " << fLeft << ", Right = " << fRight << std::endl;
               
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
         }),
         CState("near_block_approach", nullptr, nullptr, {
            CState("set_approach_velocity", [this] {
               m_psActuatorData->DifferentialDriveSystem.Power.Enable = true;
               m_psActuatorData->DifferentialDriveSystem.Power.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Left.Velocity = 5;
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.Velocity = 5;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
            CState("wait_for_underneath_rf"),
            CState("set_alignment_velocity", [this] {
               float fLeft = m_psSensorData->ManipulatorModule.RangeFinders.Left / 
                  static_cast<float>(m_psSensorData->ManipulatorModule.RangeFinders.Right + 
                                     m_psSensorData->ManipulatorModule.RangeFinders.Left) * 10;
               float fRight = m_psSensorData->ManipulatorModule.RangeFinders.Right / 
                  static_cast<float>(m_psSensorData->ManipulatorModule.RangeFinders.Right + 
                                     m_psSensorData->ManipulatorModule.RangeFinders.Left) * 10;
               
               m_psActuatorData->DifferentialDriveSystem.Left.Velocity = std::round(fLeft);
               m_psActuatorData->DifferentialDriveSystem.Right.Velocity = std::round(fRight);
               
               std::cerr << "Output Velocity" << " Left = " << fLeft << ", Right = " << fRight << std::endl;
               
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
            CState("set_zero_velocity", [this] {
               m_psActuatorData->DifferentialDriveSystem.Power.Enable = false;
               m_psActuatorData->DifferentialDriveSystem.Power.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Left.Velocity = 0;
               m_psActuatorData->DifferentialDriveSystem.Right.Velocity = 0;
               m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = true;
               m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = true;
            }),
         }),
         CState("pick_up_block", nullptr, nullptr, {
            CState("lower_lift_actuator", [this] {
               m_psActuatorData->ManipulatorModule.LiftActuator.Position.Value = 5;
               m_psActuatorData->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
            }),
            CState("wait_for_lift_actuator"),
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
               m_psActuatorData->ManipulatorModule.NFCInterface.OutboundMessage = "3";
               m_psActuatorData->ManipulatorModule.NFCInterface.UpdateReq = true;
            }),
            CState("raise_lift_actuator", [this] {
               m_psActuatorData->ManipulatorModule.LiftActuator.Position.Value = 60;
               m_psActuatorData->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
            }),
            CState("wait_for_lift_actuator_2"),
         }),
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
      (*this)["far_block_approach"].AddExitTransition("set_alignment_velocity", [this] {
         return (m_psSensorData->ImageSensor.Detections.Targets.size() == 0);
      });

      (*this).AddTransition("far_block_approach", "near_block_approach");
      /* near_block_approach transitions */
      (*this)["near_block_approach"].AddTransition("set_approach_velocity","wait_for_underneath_rf");
      (*this)["near_block_approach"].AddTransition("wait_for_underneath_rf","set_alignment_velocity", [this] {
         return (m_psSensorData->ManipulatorModule.RangeFinders.Underneath > 2300);
      });
      (*this)["near_block_approach"].AddTransition("set_alignment_velocity", "set_zero_velocity", [this] {
         return (m_psSensorData->ManipulatorModule.RangeFinders.Left > 4000) &&
                (m_psSensorData->ManipulatorModule.RangeFinders.Right > 4000);
      });
      (*this)["near_block_approach"].AddExitTransition("set_zero_velocity");
           
           
      (*this).AddTransition("near_block_approach", "pick_up_block");
      /* pickup_block transistions */
      (*this)["pick_up_block"].AddTransition("lower_lift_actuator","wait_for_lift_actuator");
      (*this)["pick_up_block"].AddTransition("wait_for_lift_actuator", "enable_attachment_field", [this] {
         return (m_psSensorData->ManipulatorModule.LiftActuator.State == 
                 CBlockDemo::ELiftActuatorSystemState::INACTIVE);
      });
      (*this)["pick_up_block"].AddTransition("enable_attachment_field","disable_attachment_field");
      (*this)["pick_up_block"].AddTransition("disable_attachment_field","raise_lift_actuator");
      (*this)["pick_up_block"].AddTransition("raise_lift_actuator","wait_for_lift_actuator_2");
      (*this)["pick_up_block"].AddExitTransition("wait_for_lift_actuator_2", [this] {
         return (m_psSensorData->ManipulatorModule.LiftActuator.State == 
                 CBlockDemo::ELiftActuatorSystemState::INACTIVE);
      });
      (*this).AddExitTransition("pick_up_block");
      
      
   }

private:
   CBlockDemo::SSensorData* m_psSensorData;
   CBlockDemo::SActuatorData* m_psActuatorData;
};

#endif
