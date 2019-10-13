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

//--------------------------------------------------//
//                 DEFINE VARIABLES                 //
//--------------------------------------------------//
Servo MOTOR_1;
Servo MOTOR_2;
Servo MOTOR_3;
Servo MOTOR_4;

byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;

unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long TIME_LAST_M = 0;
unsigned long TIME_NOW_M;
unsigned long TIME_LAST_PID = 0;
unsigned long TIME_NOW_PID;

float ROLL = 0;
float ROLL_ACC;
float ROLL_GYRO = 0;

float PITCH = 0;
float PITCH_ACC;
float PITCH_GYRO = 0;

float YAW = 0;
float YAW_ACC;
float YAW_GYRO = 0;

int16_t X_ACC, Y_ACC, Z_ACC, TEMP;
int16_t X_GYRO, Y_GYRO, Z_GYRO;

float X_ACC_BASE = 0;
float Y_ACC_BASE = 0;
float Z_ACC_BASE = 0;
float X_GYRO_BASE = 0;
float Y_GYRO_BASE = 0;
float Z_GYRO_BASE = 0;

float MOTOR_OUTPUT_1;
float MOTOR_OUTPUT_2;
float MOTOR_OUTPUT_3;
float MOTOR_OUTPUT_4;

float CONTROLLED_VALUE_ROLL;
float CONTROLLED_VALUE_PITCH;
float CONTROLLED_VALUE_YAW;

float ROLL_RATE_MAX = 400.0;
float PITCH_RATE_MAX = 400.0;
float YAW_RATE_MAX = 400.0;
//--------------------------------------------------//
//                    VOID SETUP                    //
//--------------------------------------------------//
void setup(){
  Serial.begin(38400);
  Serial.println(F("Serial communication start..."));
  Serial.println(F("Check the battery connection and horizontal attitude."));
  Serial.println(F("If all of the condition is right, Please input any character."));
  PAUSE();
    
  SET_MPU6050();
  CALIBRATE_MPU6050();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  SET_MOTOR();

  PCICR |= (1 << PCIE0);                                       //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                     //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                     //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                     //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                                     //Set PCINT3 (digital input 11)to trigger an interrupt on state change.
  
  TIME_LAST_M = millis();
  TIME_LAST_PID = millis();

  //Serial.println(F("For start, Please input any character."));
  //PAUSE();
}
//--------------------------------------------------//
//                    VOID LOOP                     //
//--------------------------------------------------//
void loop(){
  TIME_NOW_M = millis();
  GET_RAW_DATA();
  GET_ANGLE();
  
  Serial.print("RPY : ");
  Serial.print(ROLL); Serial.print(" \t");
  //Serial.print(PITCH); Serial.print(" \t");
  //Serial.print(YAW); Serial.print(" \t");
  TIME_LAST_M = millis();
  //--------------------------------------------------//
  //                 PID CONTROL PART                 //
  //--------------------------------------------------//
  TIME_NOW_PID = millis();
  
  PID_MOTOR(ROLL, PITCH, YAW, 0.0, 0.0);
  UPDATE_MOTOR();
  Serial.println();

  TIME_LAST_PID = TIME_NOW_PID;
  //--------------------------------------------------//
  PAUSE_DATA();
}

//--------------------------------------------------//
//              USER DEFINED FUNCTION               //
//--------------------------------------------------//
inline void PAUSE(){
    while(Serial.available() && Serial.read());
    while(!Serial.available());
    while(Serial.available() && Serial.read());
}

inline void PAUSE_DATA(){
    if(Serial.available() && Serial.read() == '1'){
      Serial.println(F("Stop collecting"));
      MOTOR_1.writeMicroseconds(MIN_SIGNAL);
      MOTOR_2.writeMicroseconds(MIN_SIGNAL);
      MOTOR_3.writeMicroseconds(MIN_SIGNAL);
      MOTOR_4.writeMicroseconds(MIN_SIGNAL);
      while(true){
        if(Serial.available() && Serial.read() == '0'){
          Serial.println(F("Continue collecting"));
          break;
        }
      }
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

  Serial.println(F("MPU6050 Calibration is started..."));
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

  Serial.println(F("MPU6050 Calibration is done!"));
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
  Serial.println(F("Motor Setup is done."));
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

double DT;
float X_ANGVEL;
float Y_ANGVEL;
float Z_ANGVEL;
float X_FORCE;  
float Y_FORCE;
float Z_FORCE;

void GET_ANGLE(){
  double DT = (TIME_NOW_M - TIME_LAST_M)/1000.0;

  X_ANGVEL = (double(X_GYRO) - X_GYRO_BASE)/131.0;
  Y_ANGVEL = (double(Y_GYRO) - Y_GYRO_BASE)/131.0;
  Z_ANGVEL = (double(Z_GYRO) - Z_GYRO_BASE)/131.0;
  X_FORCE = double(X_ACC) - X_ACC_BASE;  
  Y_FORCE = double(Y_ACC) - Y_ACC_BASE;
  Z_FORCE = double(Z_ACC) - (16384 - Z_ACC_BASE);
  
  ROLL_ACC = atan2(double(Y_FORCE), sqrt(sq(double(X_FORCE)) + sq(double(Z_FORCE))));   
  ROLL_ACC = ROLL_ACC * 180 / M_PI;
  PITCH_ACC = - atan2(double(X_FORCE), sqrt(sq(double(Y_FORCE)) + sq(double(Z_FORCE))));
  PITCH_ACC = PITCH_ACC * 180 / M_PI;
  YAW_ACC = 0;
  YAW_ACC = YAW_ACC * 180 / M_PI;

  ROLL_GYRO = ROLL_GYRO + X_ANGVEL * DT;
  PITCH_GYRO = PITCH_GYRO + Y_ANGVEL * DT;
  YAW_GYRO = YAW_GYRO + Z_ANGVEL * DT;

  ROLL = 0.98 * (X_ANGVEL * DT + ROLL) + 0.02 * ROLL_ACC;
  PITCH = 0.98 * (Y_ANGVEL * DT + PITCH) + 0.02 * PITCH_ACC; 
  YAW = 0.98 * (Z_ANGVEL * DT + YAW) + 0.02 * YAW_ACC; 
}

float ERROR_PITCH;
float ERROR_PITCH_PREVIOUS = 0;
float ERROR_ROLL;
float ERROR_ROLL_PREVIOUS = 0;
float ERROR_YAW;
float ERROR_YAW_PREVIOUS = 0;

float I_CONTROL_PITCH = 0;
float I_CONTROL_ROLL = 0;
float I_CONTROL_YAW = 0;

float P_GAIN_PITCH = 0.8; 
float I_GAIN_PITCH = 0.05;
float D_GAIN_PITCH = 55;

float P_GAIN_ROLL = 0.8;
float I_GAIN_ROLL = 0.05;
float D_GAIN_ROLL = 55;

float P_GAIN_YAW;
float I_GAIN_YAW;
float D_GAIN_YAW;

float P_CONTROL_ROLL = 0;
float D_CONTROL_ROLL = 0;
float P_CONTROL_PITCH = 0;
float D_CONTROL_PITCH = 0;
float dT;

//RC transmitter has minimum : 1112(1116) ~ 1912(1916)
void PID_MOTOR(float ROLL, float PITCH, float YAW, float DESIRED_ANGLE_PITCH, float DESIRED_ANGLE_ROLL){
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

    //Serial.print(dT*1000.0); Serial.print(" \t");
    //Serial.print(P_CONTROL_ROLL); Serial.print(" \t");
    //Serial.print(I_CONTROL_ROLL); Serial.print(" \t");
    //Serial.print(D_CONTROL_ROLL); Serial.print(" \t");
    //Serial.print(CONTROLLED_VALUE_PITCH); Serial.print("\t");
    Serial.print(CONTROLLED_VALUE_ROLL); Serial.print("\t");
}

void UPDATE_MOTOR(){
  MOTOR_OUTPUT_1 = constrain(1350 + CONTROLLED_VALUE_PITCH - CONTROLLED_VALUE_ROLL, MIN_SIGNAL, MAX_SIGNAL);
  MOTOR_OUTPUT_2 = constrain(1350 - CONTROLLED_VALUE_PITCH + CONTROLLED_VALUE_ROLL, MIN_SIGNAL, MAX_SIGNAL);
  MOTOR_OUTPUT_3 = constrain(1350 - CONTROLLED_VALUE_PITCH - CONTROLLED_VALUE_ROLL, MIN_SIGNAL, MAX_SIGNAL);
  MOTOR_OUTPUT_4 = constrain(1350 + CONTROLLED_VALUE_PITCH + CONTROLLED_VALUE_ROLL, MIN_SIGNAL, MAX_SIGNAL);
  MOTOR_1.writeMicroseconds(MOTOR_OUTPUT_1);
  MOTOR_2.writeMicroseconds(MOTOR_OUTPUT_2);
  MOTOR_3.writeMicroseconds(MOTOR_OUTPUT_3);
  MOTOR_4.writeMicroseconds(MOTOR_OUTPUT_4);
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
