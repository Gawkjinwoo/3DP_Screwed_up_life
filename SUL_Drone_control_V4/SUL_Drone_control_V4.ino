#include <Servo.h>
#include <Wire.h>
#include <math.h>

#define MAX_SIGNAL                 2000   // 1025 ~ 1999 
#define MIN_SIGNAL                 1000
#define MOTOR_PIN_1                   4
#define MOTOR_PIN_2                   5
#define MOTOR_PIN_3                   6
#define MOTOR_PIN_4                   7
#define MPU6050_ACCEL_XOUT_H       0x3B   // R 
#define MPU6050_I2C_ADDRESS        0x68   // R/W
#define MPU6050_PWR_MGMT_1         0x6B   // R/W
#define LED_PIN                       3
#define PAUSE_SIGNAL_PIN             13
#define START_SIGNAL_PIN             12

//--------------------------------------------------//
//                 DEFINE VARIABLES                 //
//--------------------------------------------------//
float ROLL = 0, ROLL_ACC, ROLL_GYRO = 0, PITCH = 0, PITCH_ACC, PITCH_GYRO = 0, YAW = 0, YAW_ACC, YAW_GYRO = 0;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
float X_ACC_BASE = 0, Y_ACC_BASE = 0, Z_ACC_BASE = 0, X_GYRO_BASE = 0, Y_GYRO_BASE = 0, Z_GYRO_BASE = 0;
unsigned long TIME_LAST_ANGLE = 0, TIME_NOW_ANGLE, TIME_LAST_PID = 0, TIME_NOW_PID;
float CONTROLLED_VALUE_ROLL, CONTROLLED_VALUE_PITCH, CONTROLLED_VALUE_YAW;
float ROLL_RATE_MAX = 400.0, PITCH_RATE_MAX = 400.0, YAW_RATE_MAX = 400.0;
float MOTOR_OUTPUT_1, MOTOR_OUTPUT_2, MOTOR_OUTPUT_3, MOTOR_OUTPUT_4;
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
int16_t X_ACC, Y_ACC, Z_ACC, TEMP, X_GYRO, Y_GYRO, Z_GYRO;
float ROLL_INPUT = 0, PITCH_INPUT = 0, YAW_INPUT = 0;
Servo MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4;


float ERROR_PITCH, ERROR_PITCH_PREVIOUS = 0, ERROR_ROLL, ERROR_ROLL_PREVIOUS = 0, ERROR_YAW, ERROR_YAW_PREVIOUS = 0;
float I_CONTROL_PITCH = 0, I_CONTROL_ROLL = 0, I_CONTROL_YAW = 0;
float P_GAIN_PITCH = 0.8, I_GAIN_PITCH = 0.2, D_GAIN_PITCH = 55.0;
float P_GAIN_ROLL = 0.8, I_GAIN_ROLL = 0.2, D_GAIN_ROLL = 55.0;
float P_GAIN_YAW = 0.8, I_GAIN_YAW = 0.05, D_GAIN_YAW = 55.0;
float P_CONTROL_ROLL = 0, D_CONTROL_ROLL = 0;
float P_CONTROL_PITCH = 0, D_CONTROL_PITCH = 0;
float P_CONTROL_YAW = 0, D_CONTROL_YAW = 0;
float dT, DT;
//--------------------------------------------------//
//                    VOID SETUP                    //
//--------------------------------------------------//
void setup(){
  pinMode(LED_PIN, OUTPUT);
  pinMode(PAUSE_SIGNAL_PIN, INPUT);
  pinMode(START_SIGNAL_PIN, INPUT);

  while(digitalRead(START_SIGNAL_PIN) == 0);
  SET_MPU6050();
  CALIBRATE_MPU6050();
  SET_MOTOR();
  PCICR |= (1 << PCIE0);                                       //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                     //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                     //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                     //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                                     //Set PCINT3 (digital input 11)to trigger an interrupt on state change.
  while(digitalRead(START_SIGNAL_PIN) == 0);
  TIME_LAST_ANGLE = millis();
  TIME_LAST_PID = millis();
}
//--------------------------------------------------//
//                    VOID LOOP                     //
//--------------------------------------------------//
void loop(){
  GET_RAW_DATA();
  TIME_NOW_ANGLE = millis();
  GET_ANGLE();
  TIME_LAST_ANGLE = TIME_NOW_ANGLE;
  TIME_NOW_PID = millis();
  PID_MOTOR(ROLL_INPUT, PITCH_INPUT, YAW_INPUT, 0.0, 0.0, 0.0);
  TIME_LAST_PID = TIME_NOW_PID;
  UPDATE_MOTOR();

  PAUSE_DATA_ESP32();
}
//--------------------------------------------------//
//              USER DEFINED FUNCTION               //
//--------------------------------------------------//
void GET_ANGLE(){
  DT = (TIME_NOW_ANGLE - TIME_LAST_ANGLE)/1000.0;

  float X_ANGVEL = (double(X_GYRO) - X_GYRO_BASE)/131.0;
  float Y_ANGVEL = (double(Y_GYRO) - Y_GYRO_BASE)/131.0;
  float Z_ANGVEL = (double(Z_GYRO) - Z_GYRO_BASE)/131.0;
  float X_FORCE = double(X_ACC) - X_ACC_BASE;  
  float Y_FORCE = double(Y_ACC) - Y_ACC_BASE;
  float Z_FORCE = double(Z_ACC) - (16384 - Z_ACC_BASE);
  
  ROLL_ACC = atan2(double(Y_FORCE), sqrt(sq(double(X_FORCE)) + sq(double(Z_FORCE))));   
  ROLL_ACC = ROLL_ACC * 180 / M_PI;
  PITCH_ACC = - atan2(double(X_FORCE), sqrt(sq(double(Y_FORCE)) + sq(double(Z_FORCE))));
  PITCH_ACC = PITCH_ACC * 180 / M_PI;
  YAW_ACC = 0;
  YAW_ACC = YAW_ACC * 180 / M_PI;

  ROLL_GYRO = ROLL_GYRO + X_ANGVEL * DT;
  PITCH_GYRO = PITCH_GYRO + Y_ANGVEL * DT;
  YAW_GYRO = YAW_GYRO + Z_ANGVEL * DT;

  ROLL = 0.9996 * (X_ANGVEL * DT + ROLL) + 0.0004 * ROLL_ACC;
  PITCH = 0.9996 * (Y_ANGVEL * DT + PITCH) + 0.0004 * PITCH_ACC; 
  YAW = 0.9996 * (Z_ANGVEL * DT + YAW) + 0.0004 * YAW_ACC; 

  ROLL_INPUT = ROLL_INPUT * 0.8 + ROLL * 0.2;
  PITCH_INPUT = PITCH_INPUT * 0.8 + PITCH * 0.2;
  YAW_INPUT = YAW_INPUT * 0.8 + YAW * 0.2;
}

//RC transmitter has minimum : 1112(1116) ~ 1912(1916)
void PID_MOTOR(float ROLL, float PITCH, float YAW, float DESIRED_ANGLE_PITCH, float DESIRED_ANGLE_ROLL, float DESIRED_ANGLE_YAW){
    dT = (TIME_NOW_PID - TIME_LAST_PID)/1000.0;

    //Pitch Calculation
    ERROR_PITCH = DESIRED_ANGLE_PITCH - PITCH;
    P_CONTROL_PITCH = P_GAIN_PITCH * ERROR_PITCH;
    I_CONTROL_PITCH = I_GAIN_PITCH * ERROR_PITCH * dT + I_CONTROL_PITCH;
    D_CONTROL_PITCH = D_GAIN_PITCH * (ERROR_PITCH - ERROR_PITCH_PREVIOUS);
    if(I_CONTROL_PITCH >= PITCH_RATE_MAX) I_CONTROL_PITCH = PITCH_RATE_MAX;
    else if(I_CONTROL_PITCH <= -1 * PITCH_RATE_MAX) I_CONTROL_PITCH = -1 * PITCH_RATE_MAX;
    
    CONTROLLED_VALUE_PITCH = P_CONTROL_PITCH + I_CONTROL_PITCH + D_CONTROL_PITCH;
    if(CONTROLLED_VALUE_PITCH >= PITCH_RATE_MAX) CONTROLLED_VALUE_PITCH = PITCH_RATE_MAX;
    else if(CONTROLLED_VALUE_PITCH <= -1 * PITCH_RATE_MAX) CONTROLLED_VALUE_PITCH = -1 * PITCH_RATE_MAX;
    ERROR_PITCH_PREVIOUS = ERROR_PITCH;

    //Roll Calculation
    ERROR_ROLL = DESIRED_ANGLE_ROLL - ROLL;
    P_CONTROL_ROLL = P_GAIN_ROLL * ERROR_ROLL;
    I_CONTROL_ROLL = I_GAIN_ROLL * ERROR_ROLL * dT + I_CONTROL_ROLL;
    D_CONTROL_ROLL = D_GAIN_ROLL * (ERROR_ROLL - ERROR_ROLL_PREVIOUS);
    if(I_CONTROL_ROLL >= ROLL_RATE_MAX) I_CONTROL_ROLL = ROLL_RATE_MAX;
    else if(I_CONTROL_ROLL <= -1 * ROLL_RATE_MAX) I_CONTROL_ROLL = -1 * ROLL_RATE_MAX; 

    CONTROLLED_VALUE_ROLL = P_CONTROL_ROLL + I_CONTROL_ROLL + D_CONTROL_ROLL;
    if(CONTROLLED_VALUE_ROLL >= ROLL_RATE_MAX) CONTROLLED_VALUE_ROLL = ROLL_RATE_MAX;
    else if(CONTROLLED_VALUE_ROLL <= -1 * ROLL_RATE_MAX) CONTROLLED_VALUE_ROLL = -1 * ROLL_RATE_MAX;
    ERROR_ROLL_PREVIOUS = ERROR_ROLL;

    //Yaw Calculation
    ERROR_YAW = DESIRED_ANGLE_YAW - YAW;
    P_CONTROL_YAW = P_GAIN_YAW * ERROR_YAW;
    I_CONTROL_YAW = I_GAIN_YAW * ERROR_YAW * dT + I_CONTROL_YAW;
    D_CONTROL_YAW = D_GAIN_YAW * (ERROR_YAW - ERROR_YAW_PREVIOUS);
    if(I_CONTROL_YAW >= YAW_RATE_MAX) I_CONTROL_YAW = YAW_RATE_MAX;
    else if(I_CONTROL_YAW <= -1 * YAW_RATE_MAX) I_CONTROL_YAW = -1 * YAW_RATE_MAX;

    CONTROLLED_VALUE_YAW = P_CONTROL_YAW + I_CONTROL_YAW + D_CONTROL_YAW;
    if(CONTROLLED_VALUE_YAW >= YAW_RATE_MAX) CONTROLLED_VALUE_YAW = YAW_RATE_MAX;
    else if(CONTROLLED_VALUE_YAW <= -1 * YAW_RATE_MAX) CONTROLLED_VALUE_YAW = -1 * YAW_RATE_MAX;
    ERROR_YAW_PREVIOUS = ERROR_YAW;
}

void UPDATE_MOTOR(){
  MOTOR_OUTPUT_1 = constrain((receiver_input_channel_3 - 50) + CONTROLLED_VALUE_PITCH - CONTROLLED_VALUE_ROLL + CONTROLLED_VALUE_YAW, MIN_SIGNAL, MAX_SIGNAL);
  MOTOR_OUTPUT_2 = constrain((receiver_input_channel_3 - 50) - CONTROLLED_VALUE_PITCH + CONTROLLED_VALUE_ROLL + CONTROLLED_VALUE_YAW, MIN_SIGNAL, MAX_SIGNAL);
  MOTOR_OUTPUT_3 = constrain((receiver_input_channel_3 - 50) - CONTROLLED_VALUE_PITCH - CONTROLLED_VALUE_ROLL - CONTROLLED_VALUE_YAW, MIN_SIGNAL, MAX_SIGNAL);
  MOTOR_OUTPUT_4 = constrain((receiver_input_channel_3 - 50) + CONTROLLED_VALUE_PITCH + CONTROLLED_VALUE_ROLL - CONTROLLED_VALUE_YAW, MIN_SIGNAL, MAX_SIGNAL);
  MOTOR_1.writeMicroseconds(MOTOR_OUTPUT_1);
  MOTOR_2.writeMicroseconds(MOTOR_OUTPUT_2);
  MOTOR_3.writeMicroseconds(MOTOR_OUTPUT_3);
  MOTOR_4.writeMicroseconds(MOTOR_OUTPUT_4);
}

inline void PAUSE_DATA_ESP32(){
    if(digitalRead(PAUSE_SIGNAL_PIN) == HIGH){
      MOTOR_1.writeMicroseconds(MIN_SIGNAL);
      MOTOR_2.writeMicroseconds(MIN_SIGNAL);
      MOTOR_3.writeMicroseconds(MIN_SIGNAL);
      MOTOR_4.writeMicroseconds(MIN_SIGNAL);
      while(true) delay(10000);
    }
}

void SET_MPU6050(){
  Wire.begin();
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(0);
  Wire.endTransmission(true);
}

void CALIBRATE_MPU6050(){
  int NUM_READING = 2000;
  int i;

  for(i = 0; i < 2000; i++){
    GET_RAW_DATA();
  }
  for(i = 0; i < NUM_READING; i++){
    GET_RAW_DATA();
    X_ACC_BASE += X_ACC;
    Y_ACC_BASE += Y_ACC;
    Z_ACC_BASE += Z_ACC;
    X_GYRO_BASE += X_GYRO;
    Y_GYRO_BASE += Y_GYRO;
    Z_GYRO_BASE += Z_GYRO;
  }

  X_ACC_BASE /= NUM_READING;
  Y_ACC_BASE /= NUM_READING;
  Z_ACC_BASE /= NUM_READING;
  X_GYRO_BASE /= NUM_READING;
  Y_GYRO_BASE /= NUM_READING;
  Z_GYRO_BASE /= NUM_READING;

  digitalWrite(LED_PIN, HIGH);
}

void SET_MOTOR(){
  MOTOR_1.attach(MOTOR_PIN_1, MIN_SIGNAL, MAX_SIGNAL); 
  MOTOR_2.attach(MOTOR_PIN_2, MIN_SIGNAL, MAX_SIGNAL);
  MOTOR_3.attach(MOTOR_PIN_3, MIN_SIGNAL, MAX_SIGNAL);
  MOTOR_4.attach(MOTOR_PIN_4, MIN_SIGNAL, MAX_SIGNAL);
  MOTOR_1.writeMicroseconds(MIN_SIGNAL);
  MOTOR_2.writeMicroseconds(MIN_SIGNAL);
  MOTOR_3.writeMicroseconds(MIN_SIGNAL);
  MOTOR_4.writeMicroseconds(MIN_SIGNAL);
  delay(2000);
}

void GET_RAW_DATA(){
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  Wire.write(MPU6050_ACCEL_XOUT_H);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU6050_I2C_ADDRESS, 14, true);
  X_ACC = Wire.read() << 8 | Wire.read();
  Y_ACC = Wire.read() << 8 | Wire.read();
  Z_ACC = Wire.read() << 8 | Wire.read();
  
  TEMP = Wire.read() << 8 | Wire.read();
  
  X_GYRO = Wire.read() << 8 | Wire.read();
  Y_GYRO = Wire.read() << 8 | Wire.read();
  Z_GYRO = Wire.read() << 8 | Wire.read();
  Wire.endTransmission(true);
}

ISR(PCINT0_vect){
  current_time = micros();
  //Channel 1=========================================
  if(PINB & B00000001){                                        //Is input 8 high?
    if(last_channel_1 == 0){                                   //Input 8 changed from 0 to 1
      last_channel_1 = 1;                                      //Remember current input state
      timer_1 = current_time;                                  //Set timer_1 to current_time
    }
  }
  else if(last_channel_1 == 1){                                //Input 8 is not high and changed from 1 to 0
    last_channel_1 = 0;                                        //Remember current input state
    receiver_input_channel_1 = current_time - timer_1;         //Channel 1 is current_time - timer_1
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){                                       //Is input 9 high?
    if(last_channel_2 == 0){                                   //Input 9 changed from 0 to 1
      last_channel_2 = 1;                                      //Remember current input state
      timer_2 = current_time;                                  //Set timer_2 to current_time
    }
  }
  else if(last_channel_2 == 1){                                //Input 9 is not high and changed from 1 to 0
    last_channel_2 = 0;                                        //Remember current input state
    receiver_input_channel_2 = current_time - timer_2;         //Channel 2 is current_time - timer_2
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){                                       //Is input 10 high?
    if(last_channel_3 == 0){                                   //Input 10 changed from 0 to 1
      last_channel_3 = 1;                                      //Remember current input state
      timer_3 = current_time;                                  //Set timer_3 to current_time
    }
  }
  else if(last_channel_3 == 1){                                //Input 10 is not high and changed from 1 to 0
    last_channel_3 = 0;                                        //Remember current input state
    receiver_input_channel_3 = current_time - timer_3;         //Channel 3 is current_time - timer_3

  }
  //Channel 4=========================================
  if(PINB & B00001000 ){                                       //Is input 11 high?
    if(last_channel_4 == 0){                                   //Input 11 changed from 0 to 1
      last_channel_4 = 1;                                      //Remember current input state
      timer_4 = current_time;                                  //Set timer_4 to current_time
    }
  }
  else if(last_channel_4 == 1){                                //Input 11 is not high and changed from 1 to 0
    last_channel_4 = 0;                                        //Remember current input state
    receiver_input_channel_4 = current_time - timer_4;         //Channel 4 is current_time - timer_4
  }
}
