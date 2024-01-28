#include <SimpleFOC.h>
#include "functions.h"
#include <SimpleFOCDrivers.h>
#include "encoders/as5600/MagneticSensorAS5600.h"
#include <Arduino.h>

bool serial_print = false;
bool motor_off = false;

bool upright = false;
bool got_knockeddown = false;
bool ready_for_upright = false;
bool motor_disable = false;

float swing_up_max_current_limit = 0.8;
float upright_current_max_current_limit = 2.0;
float voltage_limit = 13.0;
float velocity_limit = 35.0;
float target_current = 0.0;

float pend_angle_offset_rads = 0.0;
float pend_angle_uncorrected_rads = 0.0;
float pend_angle_corrected_rads = 0.0;
float pend_angle_constrained_rads = 0;
float pend_velocity_radspersec = 0;

float motor_angle = 0.0;
float motor_angle_offset_rads = 0.0;
float motor_velocity = 0.0;

float motor_electrical_angle = 0.0;
float temp_val_raw = 0;
float temp_degC = 0.0;

// Upright controller
float c1 = 1.8; //1.0
float c2 = 0.2; //0.08
float c3 = 0.025; //0.015
float c4 = -0.0025; // -0.002 for angle limits

// Swing-up controller
float c6 = 0.0015; // 0.002 is fast

// Control loop execution time
long loop_time_ms = 0;

// Serial printing time interval
long serial_print_interval_ms = 50;
long serial_print_time_ms = 0;

// Motor instance
BLDCMotor motor = BLDCMotor(11);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
LowsideCurrentSense current_sense = LowsideCurrentSense(0.003, -64.0/7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// pend encoder instance
MagneticSensorI2C pend = MagneticSensorI2C(AS5600_I2C);

// Motor encoder
Encoder encoder = Encoder(PB6, PA15, 1024);
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

// instantiate the commander
Commander command = Commander(Serial);
void onMotor(char* cmd){ command.motor(&motor, cmd); }

// Low pass filters
LowPassFilter LPF_pend_velocity{0.002};
LowPassFilter LPF_pend_angle{0.002};
LowPassFilter LPF_motor_velocity{0.01};
LowPassFilter LPF_motor_angle{0.005};

void setup() {
  
  // initialise encoder hardware
  encoder.pullup = Pullup::USE_INTERN;
  encoder.init();
  
  // hardware interrupt enable
  encoder.enableInterrupts(doA, doB);
  
  // configure i2C
  //Wire.setClock(900000);
  pend.init();
  
  // link the motor to the encoder
  motor.linkSensor(&encoder);
  
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 20.0;
  driver.init();
  
  // link the motor and the driver
  motor.linkDriver(&driver);
  
  // velocity loop PID
  motor.PID_velocity.P = 0.1;
  motor.PID_velocity.I = 3.0;
  // Low pass filtering time constant 
  motor.LPF_velocity.Tf = 0.005;
  
  // angle loop PID
  motor.P_angle.P = 16.0;

  // Low pass filtering time constant 
  motor.LPF_angle.Tf = 0.0;
  
  // current q loop PID 
  motor.PID_current_q.P = 1.0;
  motor.PID_current_q.I = 200.0;
  // Low pass filtering time constant 
  motor.LPF_current_q.Tf = 0.001;
  
  // current d loop PID
  motor.PID_current_d.P = 1.0;
  motor.PID_current_d.I = 200.0;
  // Low pass filtering time constant 
  motor.LPF_current_d.Tf = 0.001;

  // Limits 
  motor.velocity_limit = 100.0;
  motor.voltage_limit = voltage_limit;
  motor.current_limit = 4.0;
  
  // sensor zero offset - home position 
  //motor.sensor_offset = 0.0;

   // pwm modulation settings 
  motor.foc_modulation = FOCModulationType::SinePWM;
  motor.modulation_centered = 1.0;

  // use monitoring with serial 
  Serial.begin(115200);

  // comment out if not needed
  motor.useMonitoring(Serial);
  
  // downsampling
  motor.monitor_downsample = 10; // default 10
  motor.motion_downsample = 1;
  
   // initialize motor
  current_sense.linkDriver(&driver);
  motor.init();
  current_sense.init();
  motor.linkCurrentSense(&current_sense);
  
  current_sense.skip_align = true;
  
  // aligning voltage [V]
  motor.voltage_sensor_align = 2;

  // control loop type and torque mode 
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::torque;
  motor.motion_downsample = 0.0;

  // align encoder and start FOC
  motor.initFOC();

  // set the inital target value
  motor.target = 0.0;
  
  // Delay to let pendulum settle out
  _delay(3000);

  pend.update();
  pend_angle_offset_rads = pend.getAngle();

  encoder.update();
  motor_angle_offset_rads = encoder.getAngle();
  
  loop_time_ms = millis();
  serial_print_time_ms = millis();

}

void loop() {
  
  if (motor_off == true){
    
    motor.disable();

  }

  // Main Control Loop - Running at 200 hz
  if (millis() - loop_time_ms > 5){

    // Get B-G431-ESC Temperature
    temp_val_raw = _readADCVoltageInline(A_TEMPERATURE, current_sense.params);  
    temp_degC = Ntc2TempV(temp_val_raw);
    
    // Get raw pendulum sensor values
    pend.update();
    pend_angle_uncorrected_rads = LPF_pend_angle(pend.getAngle());
    pend_velocity_radspersec = LPF_pend_velocity(pend.getVelocity());

    // Normalize angle to be pi at the bottom, 0 while upright
    pend_angle_corrected_rads = (pend_angle_uncorrected_rads - pend_angle_offset_rads);
    pend_angle_constrained_rads = constrainAngle(pend_angle_corrected_rads + M_PI);

    // Get motor sensor values
    encoder.update();
    motor_angle = LPF_motor_angle(encoder.getAngle());
    motor_velocity = LPF_motor_velocity(encoder.getVelocity());

    // Get the d and q axis currents
    motor_electrical_angle = motor.electricalAngle();
    DQCurrent_s current = current_sense.getFOCCurrents(motor_electrical_angle);

    // Check to see if the pendulum is ready to go upright - should be resting at the bottom
    if (upright == false && (abs(pend_velocity_radspersec) < 0.0001) && (abs(pend_angle_constrained_rads) < 3.15)){

      ready_for_upright = true;
      got_knockeddown = false;

    }

    // Swing Up
    if (ready_for_upright == true){

      if (abs(pend_angle_constrained_rads) > 0.5){

        target_current = energyShaping(pend_angle_constrained_rads, 
                                       pend_velocity_radspersec, c6);
        
        if (abs(target_current) > swing_up_max_current_limit){
          
          target_current = sign(target_current) * swing_up_max_current_limit;

        }
      }
    }

    // Upright Controller
    if (abs(pend_angle_constrained_rads) <= 0.5){
      
      target_current = 1.0 * controllerLQR(pend_angle_constrained_rads, 
                                           pend_velocity_radspersec, 
                                           -1.0 * motor_velocity, 
                                           motor_angle, c1, c2, c3, c4);
                                           //abs(motor_angle)

      if (abs(target_current) > upright_current_max_current_limit ){
        
        target_current = sign(target_current) * upright_current_max_current_limit ;

      }
    }
    
    // Check to see if we're upright
    if (abs(pend_angle_constrained_rads) < 0.1){

      upright = true;
      ready_for_upright = false;

    }

    // Check to see if we should turn off the current if someone disturbs pendulum 
    // hard enough to knock it down
    if (upright == true){
      
      if (abs(pend_angle_constrained_rads) > 0.4){

        got_knockeddown = true;
      
      }
    }
    

    if (got_knockeddown == true){
    
      target_current = -1.0 * energyShaping(pend_angle_constrained_rads, 
                                            pend_velocity_radspersec, c6);
        
        if (abs(target_current) > swing_up_max_current_limit){
          
          target_current = sign(target_current)*swing_up_max_current_limit;

        }

      upright = false;

    }

    motor.move(target_current);
    serialEvent();

    // Serial printing
    if (millis() - serial_print_time_ms > serial_print_interval_ms){
      if (serial_print == true){
        
        Serial.print(millis() / 1000.0, 3);
        Serial.print("\t");
        Serial.print(motor_angle);
        Serial.print("\t");
        Serial.print(motor_velocity);
        Serial.print("\t");
        Serial.print(pend_angle_constrained_rads);
        Serial.print("\t");
        Serial.print(pend_angle_corrected_rads);
        Serial.print("\t");
        Serial.print(pend_angle_uncorrected_rads);
        Serial.print("\t");
        Serial.print(pend_velocity_radspersec);
        Serial.print("\t");
        Serial.print(target_current);
        Serial.print("\t");
        Serial.print(current.q);
        Serial.print("\t");
        Serial.print(ready_for_upright);
        Serial.print("\t");
        Serial.print(upright);
        Serial.print("\t");
        Serial.print(got_knockeddown);
        Serial.print("\t");
        Serial.print(temp_degC, 1);
        Serial.print("\t");
        Serial.print(pend_angle_offset_rads, 2);
        Serial.print("\n"); 
      }
      serial_print_time_ms = millis();
    }

    loop_time_ms = millis();
  }

  motor.loopFOC();
  
}

// Print the LQR controller gains
void spaceGains(){
  Serial.print("C1 pAngle: ");
  Serial.print(c1);
  Serial.print("\t C2 pVel: ");
  Serial.print(c2,4);
  Serial.print("\t C3 mVel: ");
  Serial.print(c3,4);
  Serial.print("\t C4 mPos: ");
  Serial.print(c4,6);
  Serial.print("\t Swing up gain: ");
  Serial.print(c6,4);
  Serial.print("\t Serial print on/off: ");
  Serial.print(serial_print);
  Serial.print("\t Serial print interval (ms): ");
  Serial.print(serial_print_interval_ms);
  Serial.println("");
}

// Tune the LQR controller, the threshold transition angle and energy shaping controller from the Serial monitor.
void serialEvent() {
  // a string to hold incoming data
  static String inputString;
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline
    // end of input
    if (inChar == '\n') {
      if(inputString.charAt(0) == '1'){
        c1 = inputString.substring(1).toFloat();
        spaceGains();
      }else if(inputString.charAt(0) == '2'){
        c2 = inputString.substring(1).toFloat();
        spaceGains();
      }else if(inputString.charAt(0) == '3'){
        c3 = inputString.substring(1).toFloat();
        spaceGains();
      }else if(inputString.charAt(0) == '4'){
        c4 = inputString.substring(1).toFloat();
        spaceGains();
      }else if(inputString.charAt(0) == '6'){
        c6 = inputString.substring(1).toFloat();
        spaceGains();
      }else if(inputString.charAt(0) == 'g'){
        spaceGains();
      }else if(inputString.charAt(0) == 'p'){
        serial_print = !serial_print;
        spaceGains();
      }else if(inputString.charAt(0) == 'f'){
        serial_print_interval_ms = inputString.substring(1).toInt();
        spaceGains();
      }
      inputString = "";
    } 
  }
}