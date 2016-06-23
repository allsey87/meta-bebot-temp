#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <chrono>

class CPIDController {
public:
   /* constructor */
   CPIDController(double f_kp, double f_ki, double f_kd, double f_sat) :
      m_fKp(f_kp),
      m_fKi(f_ki),
      m_fKd(f_kd),
      m_fSat(f_sat),
      m_bResetReq(true) {}
   /* step the controller */
   double Step(double f_input, double f_target, std::chrono::steady_clock::time_point tp_tick) {
      if(m_bResetReq) {
         m_fLastError = f_target - f_input;
         m_fErrorAccumulated = 0.000;
         m_tpLastTick = tp_tick;
         m_bResetReq = false;
         //std::cout << "input,target,error,error_derivative,error_accumulated,output" << std::endl;
         return 0.000;
      }
      else {
         std::chrono::duration<double> tDelta = tp_tick - m_tpLastTick;
         double fError = f_target - f_input;
         double fErrorDerivative = (fError - m_fLastError) / tDelta.count();
         m_fErrorAccumulated += fError * tDelta.count();
         /* saturate the accumulated error at m_fSat */
         m_fErrorAccumulated = (m_fErrorAccumulated > m_fSat) ? m_fSat : ((m_fErrorAccumulated < -m_fSat) ? -m_fSat : m_fErrorAccumulated);
         /* calculate the output value */
         double fOutput = (m_fKp * fError) +
                          (m_fKi * m_fErrorAccumulated) +
                          (m_fKd * fErrorDerivative);
         //std::cout << f_input << "," << f_target << "," << fError << "," << fErrorDerivative << "," << m_fErrorAccumulated << "," << fOutput << std::endl;
         m_fLastError = fError;
         m_tpLastTick = tp_tick;
         return fOutput;
      }
   }
   /* request controller reset */
   void Reset() {
      m_bResetReq = true;
   }
private:
   bool m_bResetReq;
   const double m_fKp, m_fKi, m_fKd, m_fSat;
   double m_fLastError, m_fErrorAccumulated;
   std::chrono::steady_clock::time_point m_tpLastTick;
};

#endif

