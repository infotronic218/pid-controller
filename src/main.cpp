#include <Arduino.h>
#include "pid.h"
double setPoint = 3 ;


//PID Variables definition
double Kp = 1 ;
double Kd = 0 ;
double Ki = 1 ;
double Te = 0.1 ; // Sampling period
double T = 0 ;  // filter time constant
double pn = 0 ; // Actul proportional term
double in = 0 ; // actual integral term
double dn = 0 ; // actual derivative term
double output =0 ; // ouput 
double error = 0 ; // current error 

double computePID(double desired,double actual,double pre_actual){
  double error_prev = error ; // Copy the previous error
  error = desired - actual ; // calculate the actual error
  // Calculate the proportional term
  pn = Kp* error ;
  // Calculate the integral term
  in = (Ki*Te/2.0)*(error+error_prev)+in ;
  // Calculate the derivative plus filter term
  dn  = ((2*Kd)/(2.0*T+Te))*(error -error_prev) - ((Te-2.0*T)/(2*T+Te))*dn ;

  output =  pn + in + dn ;
  // Limit the PID output between  0 and 100
  if(output>100){
    output =100;
  }else if(output<0){
    output = 0;
  }
  return output ;
}

// END of the PID definition
void setup() {
  pinMode(13, OUTPUT);
  pinMode(6,OUTPUT);
  PID_Init(5,1/0.5, 0, 0.1, 100, 0);
  Serial.begin(9600);
  // put your setup code here, to run once:
}

void loop() {
  long start = millis() ;
  float reading = analogRead(A0)*5.0/1023.0;
  //double out = PID_Compute(setPoint, reading);
  double out = computePID(setPoint, reading, reading);
  analogWrite(6, (out*1023.0/100.0));
  long end =  millis();
  Serial.println("Set = "+String(setPoint)+", act = "+String(reading)+", e = "+String((setPoint-reading)) + ",Te= "+String((start-end)));
 
  // put your main code here, to run repeatedly:
}

