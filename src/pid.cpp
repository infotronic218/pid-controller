#include "pid.h"
PIDConfig pidConfig ;
void PID_Init(double Kp, double Ki, double Kd, double Te , double out_max, double out_min){
  pidConfig.Kp = Kp;
  pidConfig.Ki = Ki ;
  pidConfig.Kd = Kd ;
  pidConfig.Te = Te;
  pidConfig.out_max = out_max ;
  pidConfig.out_min = out_min ;
}
double PID_Compute(double desired, double output ){
   pidConfig.error  = desired - output ; // Calculate the errror
   
   // Calculate the proportionnal term 
   pidConfig.pn  = pidConfig.Kp*pidConfig.error ;
   // Calculate the integral term
   pidConfig.in = (pidConfig.Te*pidConfig.Ki/2.0)*(pidConfig.error + pidConfig.error_prev) +pidConfig.in ;
   // Calculate the derivative form
   pidConfig.dn = (2.0*pidConfig.Kd/pidConfig.Te)*(pidConfig.error-pidConfig.error_prev)-pidConfig.dn ;
    // Sum the three terms 
    pidConfig.out = pidConfig.pn + pidConfig.in + pidConfig.dn ;
    // Save the previous error in the PID
    pidConfig.error_prev = pidConfig.error;

    if(pidConfig.out>pidConfig.out_max){
        pidConfig.out = pidConfig.out_max ;
    }else if(pidConfig.out<pidConfig.out_min){
        pidConfig.out = pidConfig.out_min;
    }
return pidConfig.out;

}