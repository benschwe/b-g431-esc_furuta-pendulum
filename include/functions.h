#include <Arduino.h>

int sign(int val){
  if (val < 0) return -1;
  if (val==0) return 0;
  return 1;

}


// Function constraining the angle in between -pi and pi, in degrees -180 and 180
float constrainAngle(float x){
    x = fmod(x + M_PI, 2.0*M_PI);
    if (x < 0)
        x += 2.0*M_PI;
    return x - M_PI;
}


// LQR stabilization controller functions
// calculates the voltage that needs to be set to the motor in order to stabilize the pendulum
float controllerLQR(float p_angle, float p_vel, float m_vel, float m_pos, float c1, float c2, float c3, float c4){
  // calculate the control law 
  // LQR controller u = k*x
  //  - k = [c1, c2, c3, c4]
  //  - x = [pendulum angle, pendulum velocity, motor velocity, motor position]' 
  float u =  c1*p_angle + c2*p_vel + c3*m_vel + c4*m_pos;
  
  return u;
}


//   shaping controller
// calculates the voltage that needs to be set to the motor in order to add/ remove energy to the pendulum
float energyShaping(float p_angle, float p_vel, float c6){
    
  float cosFourth = cos( p_angle )*cos( p_angle )*cos( p_angle )*cos( p_angle );
  float u = c6*cosFourth*(p_vel)*(9.81*(1-cos(p_angle)-(0.0015*(p_vel)*(p_vel))));

  return u;
}

float Ntc2TempV(float ADCVoltage) 
{
	// Formula: https://www.giangrandi.org/electronics/ntc/ntc.shtml
	const float ResistorBalance = 4700.0;
	const float Beta  = 3425.0F;
	const float RoomTempI = 1.0F / 298.15F; //[K]
	const float Rt = ResistorBalance * ((3.3F / ADCVoltage)-1);
	const float R25 = 10000.0F;
	
	float T = 1.0F / ((log(Rt/R25) / Beta) + RoomTempI);
	T = T - 273.15;

	return T;
}

 

