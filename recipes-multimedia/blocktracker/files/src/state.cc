#include "state.h"

#include <iostream>

CState::CState(const std::string& str_id,
               std::function<void()> fn_entry_method,
               std::function<void()> fn_exit_method,
               const std::vector<CState>& vec_sub_states) :
   m_strId(str_id),
   m_fnEntryMethod(fn_entry_method),
   m_fnExitMethod(fn_exit_method),
   m_vecSubStates(vec_sub_states) {
   m_itCurrentSubState = std::end(m_vecSubStates);  
}

/***********************************************************/
/***********************************************************/

CState::CState(const CState& c_state_other) :
   m_strId(c_state_other.m_strId),
   m_fnEntryMethod(c_state_other.m_fnEntryMethod),
   m_fnExitMethod(c_state_other.m_fnExitMethod),
   m_vecSubStates(c_state_other.m_vecSubStates) {
   
   /* Set the current substate iterator */
   if(c_state_other.m_itCurrentSubState != std::end(c_state_other.m_vecSubStates)) {
      std::string strCurrentSubStateId = c_state_other.m_itCurrentSubState->m_strId;  
      m_itCurrentSubState = std::find_if(std::begin(m_vecSubStates), 
                                         std::end(m_vecSubStates), 
                                         [&strCurrentSubStateId] (const CState& c_state) {
         return (c_state.m_strId == strCurrentSubStateId);
      });
   }
   else {
      m_itCurrentSubState = std::end(m_vecSubStates);
   }
   
   /* Copy and validate the transitions */
   for(const STransition& s_transition : c_state_other.m_vecTransitions) {
      std::string strFromStateId, strToStateId;
      if(s_transition.FromState != std::end(c_state_other.m_vecSubStates)) {
         strFromStateId = s_transition.FromState->m_strId;
      }
      if(s_transition.ToState != std::end(c_state_other.m_vecSubStates)) {
         strToStateId = s_transition.ToState->m_strId;
      }
      AddTransition(strFromStateId, strToStateId, s_transition.Guard);
   }
}

/***********************************************************/
/***********************************************************/

CState& CState::operator[](const std::string& str_id) {
   std::vector<CState>::iterator itSubState = 
      std::find_if(std::begin(m_vecSubStates), std::end(m_vecSubStates), [&str_id] (const CState& c_state) {
         return (c_state.m_strId == str_id);
      });

   if(itSubState == std::end(m_vecSubStates)) {
      throw std::invalid_argument(str_id + " doesn't exist in " + m_strId);
   }
   return *itSubState;
}

/***********************************************************/
/***********************************************************/
     
void CState::SetEntryFunction(std::function<void()> fn_entry_method) {
   m_fnEntryMethod = fn_entry_method;
}

/***********************************************************/
/***********************************************************/

void CState::SetExitFunction(std::function<void()> fn_exit_method) {
   m_fnExitMethod = fn_exit_method;
}

/***********************************************************/
/***********************************************************/

void CState::AddTransition(std::string str_from_state,
                           std::string str_to_state,
                           std::function<bool()> fn_guard) {
                   
   std::vector<CState>::iterator itFromState = 
      std::find_if(std::begin(m_vecSubStates), std::end(m_vecSubStates), [&str_from_state] (const CState& c_state) {
         return (c_state.m_strId == str_from_state);
      });

   std::vector<CState>::iterator itToState = 
      std::find_if(std::begin(m_vecSubStates), std::end(m_vecSubStates), [&str_to_state] (const CState& c_state) {
         return (c_state.m_strId == str_to_state);
      });

   if(itFromState != std::end(m_vecSubStates)) {
      if(itToState != std::end(m_vecSubStates)) {
         m_vecTransitions.push_back( {itFromState, itToState, fn_guard} );
      }
      else {
         throw std::invalid_argument(str_to_state + " doesn't exist in " + m_strId);
      }
   }
   else {
      throw std::invalid_argument(str_from_state + " doesn't exist in " + m_strId);
   }

}

/***********************************************************/
/***********************************************************/

void CState::AddExitTransition(std::string str_from_state,
                               std::function<bool()> fn_guard) {
                   
   std::vector<CState>::iterator itFromState = 
      std::find_if(std::begin(m_vecSubStates), std::end(m_vecSubStates), [&str_from_state] (const CState& c_state) {
         return (c_state.m_strId == str_from_state);
      });

   if(itFromState != std::end(m_vecSubStates)) {
      m_vecTransitions.push_back( {itFromState, std::end(m_vecSubStates), fn_guard} );
   }
   else {
      throw std::invalid_argument(str_from_state + " doesn't exist in " + m_strId);
   }
}

/***********************************************************/
/***********************************************************/

bool CState::Step() { 
   /* Run entry procedure if defined and set initial sub-state */
   if(m_itCurrentSubState == std::end(m_vecSubStates)) {
      if(m_fnEntryMethod != nullptr) {
         m_fnEntryMethod();
      }
      /* set the iterator to the first substate */
      m_itCurrentSubState = std::begin(m_vecSubStates);
   }
   else {
      bool bTransitionReq = m_itCurrentSubState->Step();
      /* If the step routine of the substate returns true, it is done and a 
         transition to neighboring state is required */
      if(bTransitionReq) {
         for(const STransition& s_transition : m_vecTransitions) {
            if(s_transition.FromState == m_itCurrentSubState && s_transition.Guard()) {
               m_itCurrentSubState = s_transition.ToState;
               break;
            }
         }
      }
   }
   /* if the current substate points to the end of the substates, we have either
      no substates, or we are doing an exit transition */
   if(m_itCurrentSubState == std::end(m_vecSubStates)) {
      if(m_fnExitMethod != nullptr) {
         m_fnExitMethod();
      }
      return true;
   }
   else {
      return false;
   }
}

/***********************************************************/
/***********************************************************/

std::ostream& operator<<(std::ostream& c_stream, const CState& c_state) {
   c_stream << c_state.m_strId;
   if(c_state.m_itCurrentSubState != std::end(c_state.m_vecSubStates)) {
      c_stream << "." << *c_state.m_itCurrentSubState;
   }
   return c_stream;
}

/***********************************************************/
/***********************************************************/

/*
std::ostream& operator<<(std::ostream& c_stream, const CState& c_state) {
   c_stream << c_state.m_strId 
          << (c_state.m_itCurrentSubState != std::end(c_state.m_vecSubStates) ? "*" : "")
          << "{";
   for(std::vector<CState>::const_iterator it_state = std::begin(c_state.m_vecSubStates);
       it_state != std::end(c_state.m_vecSubStates);
       it_state++) {
      c_stream << *it_state;
      if(std::next(it_state) != std::end(c_state.m_vecSubStates)) {
         c_stream << ", ";
      }
   }
   c_stream << "}";
   return c_stream;
}
*/
   
