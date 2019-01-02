/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
   
#ifdef ARDUINO_ENC_COUNTER
  //below can be changed, but should be PORTD pins; 
  //otherwise additional changes in the code are required
  #define LEFT_ENC_PIN_A PD2  //pin 2
  #define LEFT_ENC_PIN_B PD3  //pin 3
  
  //below can be changed, but should be PORTC pins
  #define RIGHT_ENC_PIN_A PC4  //pin A4
  #define RIGHT_ENC_PIN_B PC5  //pin A5
#endif

#ifdef LEONARDO_XOR_ENC_COUNTER
  #define LEFT_ENC_PIN_A 10 //PB6
  #define LEFT_ENC_PIN_B 11 //PB7
  #define LEFT_ENC_XOR   3  //INT0
  
  #define RIGHT_ENC_PIN_A 8 //PB4
  #define RIGHT_ENC_PIN_B 9 //PB5  
  #define RIGHT_ENC_XOR   2 //INT1
#endif


#ifdef ROBOGAIA_3_AXIS
  #define chipSelectPin1 10
  #define chipSelectPin2 9
  #define chipSelectPin3 8

  #define CNTR B00100000
  #define CLR B00000000
#endif

void initEncoders();
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
