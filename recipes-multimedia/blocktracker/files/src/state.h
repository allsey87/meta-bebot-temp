#ifndef STATE_H
#define STATE_H

#include <string>
#include <vector>
#include <functional>
#include <algorithm>
#include <ostream>

class CState {

public:
   CState(const std::string& str_id,
          std::function<void()> fn_entry_method = nullptr,
          std::function<void()> fn_exit_method = nullptr,
          const std::vector<CState>& vec_sub_states = {});
   
   CState(const CState& s_state_other);

   CState& operator[](const std::string& str_id);
        
   void SetEntryFunction(std::function<void()> fn_entry_method);
   
   void SetExitFunction(std::function<void()> fn_exit_method);
   
   void AddTransition(std::string str_from_state,
                      std::string str_to_state,
                      std::function<bool()> fn_guard = [] { return true; });

   void AddExitTransition(std::string str_from_state,
                          std::function<bool()> fn_guard = [] { return true; });
      
   bool Step();   
      
   struct STransition {
      std::vector<CState>::iterator FromState;
      std::vector<CState>::iterator ToState;
      std::function<bool()> Guard;
   };
   
   friend std::ostream& operator<<(std::ostream& c_stream, const CState& c_state);

private:     
   /* state name */
   std::string m_strId;
   /* entry and exit methods */
   std::function<void()> m_fnEntryMethod;
   std::function<void()> m_fnExitMethod;
   /* substates */
   std::vector<CState> m_vecSubStates;
   /* iterator to the current substate */
   std::vector<CState>::iterator m_itCurrentSubState;
   /* collection of transitions */
   std::vector<STransition> m_vecTransitions;
};

std::ostream& operator<<(std::ostream& c_stream, const CState& c_state);

#endif

