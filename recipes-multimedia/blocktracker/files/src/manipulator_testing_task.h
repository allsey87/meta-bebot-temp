#ifndef MANIPULATOR_TESTING_TASK_H
#define MANIPULATOR_TESTING_TASK_H

#include "state.h"
#include "block_demo.h"

#include <iostream>

class CManipulatorTestingTask : public CState {

private:
   /*
   void SetLiftActuatorVelocity(int8_t n_velocity) {
      std::cerr << "SetActuatorVelocity(" << (int)n_velocity << ")" << std::endl;
      m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::SET_LIFT_ACTUATOR_SPEED,
                                           reinterpret_cast<const uint8_t*>(&n_velocity),
                                           1);
   }

   void IsLiftActuatorAtTop() {
      m_pcManipulatorInterface->SendPacket(CPacketControlInterface::CPacket::EType::GET_LIMIT_SWITCH_STATE);
      

   }

   void IsLiftActuatorAtBottom() {

   }
   */
      
   
public:
   CManipulatorTestingTask(CBlockDemo::SSensorData* ps_sensor_data,
                           CBlockDemo::SActuatorData* ps_actuator_data) :
      m_psSensorData(ps_sensor_data),
      m_psActuatorData(ps_actuator_data),
      /* initialize the state machine */
      CState("top_level_state", nullptr, nullptr, {
         CState("move_lift_actuator_to_top", nullptr, nullptr, {
            CState("set_lift_actuator_velocity", [this] {
               m_psActuatorData->ManipulatorModule.LiftActuator.Velocity = -80;
               m_psActuatorData->ManipulatorModule.LiftActuator.UpdateReq = true;
            }),
            CState("wait_for_switch_contact"),
         }),
         CState("move_lift_actuator_to_bottom", nullptr, nullptr, {
            CState("set_lift_actuator_velocity", [this] {
               m_psActuatorData->ManipulatorModule.LiftActuator.Velocity = 80;
               m_psActuatorData->ManipulatorModule.LiftActuator.UpdateReq = true;
            }),
            CState("wait_for_switch_contact"),
         }),
      }) {
        
      /* Define state transitions */
      (*this)["move_lift_actuator_to_top"].AddTransition("set_lift_actuator_velocity","wait_for_switch_contact");
      (*this)["move_lift_actuator_to_top"].AddExitTransition("wait_for_switch_contact", [this] {
            return (m_psSensorData->ManipulatorModule.LimitSwitches.Top == true);
      });

      (*this)["move_lift_actuator_to_bottom"].AddTransition("set_lift_actuator_velocity","wait_for_switch_contact");
      (*this)["move_lift_actuator_to_bottom"].AddExitTransition("wait_for_switch_contact", [this] {
            return (m_psSensorData->ManipulatorModule.LimitSwitches.Bottom == true);
      });

      (*this).AddTransition("move_lift_actuator_to_top","move_lift_actuator_to_bottom");
      (*this).AddExitTransition("move_lift_actuator_to_bottom");
   }

private:
   CBlockDemo::SSensorData* m_psSensorData;
   CBlockDemo::SActuatorData* m_psActuatorData;
};

#endif
