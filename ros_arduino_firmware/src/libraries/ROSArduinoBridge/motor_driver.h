/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);


/**********
 * Pin assignments for the Arduino Motor Shield R3
 */

#ifdef ARDUINO_MOTOR_SHIELD_R3

  #define LEFT_MOTOR_PIN_DIR    4 // original 12
  #define LEFT_MOTOR_PIN_BRAKE  9 // not used for Bradley
  #define LEFT_MOTOR_PIN_SPEED  5 // original 3
  
  #define RIGHT_MOTOR_PIN_DIR   7 // original 13
  #define RIGHT_MOTOR_PIN_BRAKE 8 // not used for Bradley
  #define RIGHT_MOTOR_PIN_SPEED 6 // original 11

  #define MOTOR_BOARD_POWER     12 //NEW for Bradley Motor Driver Board

#endif

#ifdef ADAFRUIT_MOTOR_SHIELD_V2

  #define LEFT_MOTOR_HEADER   1
  #define RIGHT_MOTOR_HEADER  2
  
  #include <Wire.h>
  #include <Adafruit_MotorShield.h>

  // Create the motor shield object with the default I2C address
  Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
  // Or, create it with a different I2C address (say for stacking)
  // Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 
  
  // Select which 'port' M1, M2, M3 or M4.  NOTE: M1 and M2 conflict with the Robogaia 3-axis encoder shield.
  Adafruit_DCMotor *myLeftMotor = AFMS.getMotor(LEFT_MOTOR_HEADER);
  Adafruit_DCMotor *myRightMotor = AFMS.getMotor(RIGHT_MOTOR_HEADER);
#endif
