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
/*      
         CState("move_manipulator_to_top", nullptr, nullptr, {
            CState("set_manipulator_velocity", [this] {
               m_psActuatorData->ManipulatorModule.LiftActuator.Velocity.Value = 15;
               m_psActuatorData->ManipulatorModule.LiftActuator.Velocity.UpdateReq = true;
            }),
            CState("wait_for_switch_contact"),
         }),
         CState("move_manipulator_to_bottom", nullptr, nullptr, {
            CState("set_manipulator_velocity", [this] {
               m_psActuatorData->ManipulatorModule.LiftActuator.Velocity.Value = 15;
               m_psActuatorData->ManipulatorModule.LiftActuator.Velocity.UpdateReq = true;
            }),
            CState("wait_for_switch_contact"),
         }),     
*/       CState("init_end_effector_position", nullptr, nullptr, {
            CState("lower_lift_actuator", [this] {
               for(CBlockDemo::EColor& e_color : m_psActuatorData->LEDDeck.Color) 
                  e_color = CBlockDemo::EColor::RED;
               for(bool& b_update : m_psActuatorData->LEDDeck.UpdateReq)
                  b_update = true;
               m_psActuatorData->ManipulatorModule.LiftActuator.Position.Value = 20;
               m_psActuatorData->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
            }),
            CState("wait_for_lift_actuator"),
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
        
      
      (*this)["init_end_effector_position"].AddTransition("lower_lift_actuator","wait_for_lift_actuator");
      (*this)["init_end_effector_position"].AddExitTransition("wait_for_lift_actuator", [this] {
         return (m_psSensorData->ManipulatorModule.LiftActuator.State == 
                 CBlockDemo::ELiftActuatorSystemState::INACTIVE);
      });
        
      (*this).AddTransition("init_end_effector_position", "near_block_approach");
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
