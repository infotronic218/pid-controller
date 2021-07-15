#ifndef __PID__
#define __PID__
typedef struct
{
   double Kp ; // proportionnal coefficient
   double Ki ; // integral coefficient
   double Kd; // 
   double Te ; // Sampling period 
   double out ; // output of the controller
   double error = 0.0; // error of the controller 
   double out_prev; 
   double error_prev; /* data */
   double in = 0.0 ;
   double dn = 0.0 ;
   double pn = 0.0 ;
   double out_max =100.0;
   double out_min =0.0 ;
}PIDConfig;
 
 void PID_Init(double Kp, double Ki, double Kd, double Te , double out_max, double out_min);
 double PID_Compute(double desired, double output );


#endif