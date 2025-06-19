#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

// #define USE_SPI       // Uncomment this to use SPI

#define Serial Serial

#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 10     // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif






#define p 1 //proportinal const  (possible values 0 -1)
#define LED_PIN 13  // LED PIN
#define ADC_PIN A0  // Define the ADC pin
// Define motor control pins
#define FR_B 4  // Front right backword
#define FR_F 5  // Front right Forward
#define FL_B 10  // Front right backword
#define FL_F 11  // Front right backword
#define BL_B 8  // Front right backword
#define BL_F 9  // Front right backword
#define BR_B 6  // Front right backword
#define BR_F 7  // Front right backword
 
// Hall sensor encoder pins
#define HALL_SENSOR_BL_A_PIN 18  // Channel A of the encoder
#define HALL_SENSOR_BL_B_PIN 23  // Channel B of the encoder
#define HALL_SENSOR_BR_A_PIN 19  // Channel A of the encoder
#define HALL_SENSOR_BR_B_PIN 25  // Channel B of the encoder
#define HALL_SENSOR_FR_A_PIN 2  // Channel A of the encoder
#define HALL_SENSOR_FR_B_PIN 27  // Channel B of the encoder
#define HALL_SENSOR_FL_A_PIN 3  // Channel A of the encoder
#define HALL_SENSOR_FL_B_PIN 29  // Channel B of the encoder
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define max_pwm_value 255  // Front right backword
#define targetSpeedMax  100       //cap
// Gear ratio (example: 4 means for every 4 motor revolutions, the output shaft rotates once)
#define GEAR_RATIO 1
#define thro 1  // error thro
String inputString = "";  
bool stringComplete = false;  
// Variables to store encoder information
volatile int pulseCount_BL = 0;  // Number of pulses from Channel A
volatile int pulseCount_BR = 0;  // Number of pulses from Channel A
volatile int pulseCount_FL = 0;  // Number of pulses from Channel A
volatile int pulseCount_FR = 0;  // Number of pulses from Channel A
volatile int direction_BL = 0;   // direction_BL of motor (1: clockwise, -1: counterclockwise)
volatile int direction_BR = 0;   // direction_BR of motor (1: clockwise, -1: counterclockwise)
volatile int direction_FL = 0;   // direction_FL of motor (1: clockwise, -1: counterclockwise)
volatile int direction_FR = 0;   // direction_FR of motor (1: clockwise, -1: counterclockwise)
unsigned long previousMillis = 0;
unsigned long Tx_previousMillis = 0;
float measuredSpeed_BL = 0;       // Measured speed of the motor in RPM
float measuredSpeed_BR = 0;       // Measured speed of the motor in RPM
float measuredSpeed_FL = 0;       // Measured speed of the motor in RPM
float measuredSpeed_FR = 0;       // Measured speed of the motor in RPM
float BAT_VOLT;
int targetSpeed_FR = 00 ;
int targetSpeed_FL = 00 ; 
int targetSpeed_BR = 00 ;
int targetSpeed_BL = 00 ;  // Variables to store parsed values
int pwmOutput_BL_F = 0;             // PWM signal for motor speed control
int pwmOutput_BL_B = 0;             // PWM signal for motor speed control
int pwmOutput_BR_F = 0;             // PWM signal for motor speed control
int pwmOutput_BR_B = 0;             // PWM signal for motor speed control
int pwmOutput_FL_F = 0;             // PWM signal for motor speed control
int pwmOutput_FL_B = 0;             // PWM signal for motor speed control
int pwmOutput_FR_F = 0;             // PWM signal for motor speed control
int pwmOutput_FR_B = 0;             // PWM signal for motor speed control
// long dummyOdom=0;
// Control parameters (proportional control)
float Kp = 1;  // Proportional gain (adjust as needed)
long odometry_BL=0;
long odometry_BR=0;
long odometry_FL=0;
long odometry_FR=0;
unsigned long Tx_interval = 100;  // Time interval for speed calculation (in milliseconds)       ///  CONST publisging rate
unsigned long interval = 100;  // Time interval for speed calculation (in milliseconds)       ///  CONST publisging rate


void printPaddedInt16b(int16_t val)
{
  if (val > 0)
  {
    Serial3.print(" ");
    if (val < 10000)
    {
      Serial3.print("0");
    }
    if (val < 1000)
    {
      Serial3.print("0");
    }
    if (val < 100)
    {
      Serial3.print("0");
    }
    if (val < 10)
    {
      Serial3.print("0");
    }
  }
  else
  {
    Serial3.print("-");
    if (abs(val) < 10000)
    {
      Serial3.print("0");
    }
    if (abs(val) < 1000)
    {
      Serial3.print("0");
    }
    if (abs(val) < 100)
    {
      Serial3.print("0");
    }
    if (abs(val) < 10)
    {
      Serial3.print("0");
    }
  }
  Serial3.print(abs(val));
}

void printRawAGMT(ICM_20948_AGMT_t agmt)
{
  Serial3.print("RAW. Acc [ ");
  printPaddedInt16b(agmt.acc.axes.x);
  Serial3.print(", ");
  printPaddedInt16b(agmt.acc.axes.y);
  Serial3.print(", ");
  printPaddedInt16b(agmt.acc.axes.z);
  Serial3.print(" ], Gyr [ ");
  printPaddedInt16b(agmt.gyr.axes.x);
  Serial3.print(", ");
  printPaddedInt16b(agmt.gyr.axes.y);
  Serial3.print(", ");
  printPaddedInt16b(agmt.gyr.axes.z);
  Serial3.print(" ], Mag [ ");
  printPaddedInt16b(agmt.mag.axes.x);
  Serial3.print(", ");
  printPaddedInt16b(agmt.mag.axes.y);
  Serial3.print(", ");
  printPaddedInt16b(agmt.mag.axes.z);
  Serial3.print(" ], Tmp [ ");
  printPaddedInt16b(agmt.tmp.val);
  Serial3.print(" ]");
  Serial3.println();
}

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
  float aval = abs(val);
  if (val < 0)
  {
    Serial3.print("-");
  }
  else
  {
    Serial3.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++)
  {
    uint32_t tenpow = 0;
    if (indi < (leading - 1))
    {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++)
    {
      tenpow *= 10;
    }
    if (aval < tenpow)
    {
      Serial3.print("0");
    }
    else
    {
      break;
    }
  }
  if (val < 0)
  {
    Serial3.print(-val, decimals);
  }
  else
  {
    Serial3.print(val, decimals);
  }
}

#ifdef USE_SPI
void printScaledAGMT(ICM_20948_SPI *sensor)
{
#else
void printScaledAGMT(ICM_20948_I2C *sensor)
{
#endif
  Serial3.print("Scaled. Acc (mg) [ ");
  printFormattedFloat(sensor->accX(), 5, 2);
  Serial3.print(", ");
  printFormattedFloat(sensor->accY(), 5, 2);
  Serial3.print(", ");
  printFormattedFloat(sensor->accZ(), 5, 2);
  Serial3.print(" ], Gyr (DPS) [ ");
  printFormattedFloat(sensor->gyrX(), 5, 2);
  Serial3.print(", ");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  Serial3.print(", ");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  Serial3.print(" ], Mag (uT) [ ");
  printFormattedFloat(sensor->magX(), 5, 2);
  Serial3.print(", ");
  printFormattedFloat(sensor->magY(), 5, 2);
  Serial3.print(", ");
  printFormattedFloat(sensor->magZ(), 5, 2);
  Serial3.print(" ], Tmp (C) [ ");
  printFormattedFloat(sensor->temp(), 5, 2);
  Serial3.print(" ]");
  // Serial3.println();
}


void serial_print( String mtr__id, float f_error ,    int f_direction , int f_measured_speed ,int f_targetSpeed_BL  , int f_pwmOutput_F , int f_pwmOutput_B , long odometry )
{
int table =0;
    Serial.println("");
   if(table==1) Serial.print("Mtr ID:");
    Serial.print(mtr__id);
    Serial.print("\t"); 
        if (f_direction == -1) { Serial.print("Back"); }
        if (f_direction ==  1) { Serial.print("Forw"); }
 
    Serial.print("\t");
    if(table==1)  Serial.print("target_Speed:");
    Serial.print(f_targetSpeed_BL); 
    Serial.print("\t");
    if(table==1)  Serial.print("measured_Speed: ");
    Serial.print(f_measured_speed); 
    
    Serial.print("\t");
    if(table==1)  Serial.print("Error: ");
    Serial.print(f_error); 
    
 
    if(table==1)  Serial.print("\tPWM_F:");
    Serial.print("\t");
    Serial.print(f_pwmOutput_F);
 
    if(table==1)  Serial.print("\tPWM_B:");
    Serial.print("\t");
    Serial.print(f_pwmOutput_B);
    if(table==1)  Serial.print("\tOdom:");
    Serial.print("\t");
    Serial.print(odometry); 
    if(table==1)  Serial.print("\tBAT_V:");
    Serial.print("\t");
    Serial.print(BAT_VOLT,1); 
    Serial.print("V");
}
void forward(int speed)
{
   
/**********************************/                  /***********************************/
/**********************************/                  /***********************************/
 /**/ analogWrite(FL_F, speed);    /********************/  analogWrite(FR_F, speed); /**/    
 /**/ analogWrite(FL_B,  0   );    /********************/  analogWrite(FR_B,  0   ); /**/    
/**********************************/       /****/     /***********************************/
/**********************************/       /****/     /***********************************/
                                           /****/
                                           /****/
                                           /****/
                                           /****/
                                           /****/
/**********************************/                  /***********************************/
/**********************************/                  /***********************************/
 /**/ analogWrite(BL_F, speed);    /********************/  analogWrite(BR_F, speed); /**/    
 /**/ analogWrite(BL_B,  0   );    /********************/  analogWrite(BR_B,  0   ); /**/    
/**********************************/                  /***********************************/
/**********************************/                  /***********************************/
                                              
}
void motor_speed_FL(int speed_FL )
{
/***********************************************************************/                   
/***********************************************************************/                 
//// ░█▀▀░█▀▄░█▀█░█▀█░▀█▀░░░█░░░█▀▀░█▀▀░▀█▀                          /**/
//// ░█▀▀░█▀▄░█░█░█░█░░█░░░░█░░░█▀▀░█▀▀░░█░                          /**/
//// ░▀░░░▀░▀░▀▀▀░▀░▀░░▀░░░░▀▀▀░▀▀▀░▀░░░░▀░                          /**/
/**/                                                                 /**/
/**/ if     (speed_FL>0)  { analogWrite(FL_F, speed_FL);             /**/                    
/**/                        analogWrite(FL_B, 0000000);         }    /**/                    
/**/else if (speed_FL<0)  { analogWrite(FL_F, 0000000);              /**/                      
/**/                        analogWrite(FL_B, abs(speed_FL) );  }    /**/                    
/**/   else               { analogWrite(FL_F, 0000000);              /**/                      
/**/                        analogWrite(FL_B, 0000000);         }    /**/    
/***********************************************************************/                  
/***********************************************************************/                   
 
}
void motor_speed_FR(int speed_FR )
{
/***********************************************************************/                   
/***********************************************************************/             
//// ░█▀▀░█▀▄░█▀█░█▀█░▀█▀░░░█▀▄░▀█▀░█▀▀░█░█░▀█▀                      /**/
//// ░█▀▀░█▀▄░█░█░█░█░░█░░░░█▀▄░░█░░█░█░█▀█░░█░                      /**/
//// ░▀░░░▀░▀░▀▀▀░▀░▀░░▀░░░░▀░▀░▀▀▀░▀▀▀░▀░▀░░▀░                      /**/         
/**/                                                                 /**/
/**/ if     (speed_FR>0)  { analogWrite(FR_F, speed_FR);             /**/                    
/**/                        analogWrite(FR_B, 0000000);         }    /**/                    
/**/else if (speed_FR<0)  { analogWrite(FR_F, 0000000);              /**/                      
/**/                        analogWrite(FR_B, abs(speed_FR) );  }    /**/                    
/**/   else               { analogWrite(FR_F, 0000000);              /**/                      
/**/                        analogWrite(FR_B, 0000000);         }    /**/    
/***********************************************************************/                  
/***********************************************************************/                   
 
}
 
void motor_speed_BR(int speed_BR )
{
/***********************************************************************/                   
/***********************************************************************/    
////   ░█▀▄░█▀█░█▀▀░█░█░░░█▀▄░▀█▀░█▀▀░█░█░▀█▀                        /**/ 
////   ░█▀▄░█▀█░█░░░█▀▄░░░█▀▄░░█░░█░█░█▀█░░█░                        /**/
////   ░▀▀░░▀░▀░▀▀▀░▀░▀░░░▀░▀░▀▀▀░▀▀▀░▀░▀░░▀░                        /**/        
/**/                                                                 /**/
/**/ if     (speed_BR>0)  { analogWrite(BR_F, speed_BR);             /**/                    
/**/                        analogWrite(BR_B, 0000000);         }    /**/                    
/**/else if (speed_BR<0)  { analogWrite(BR_F, 0000000);              /**/                      
/**/                        analogWrite(BR_B, abs(speed_BR) );  }    /**/                    
/**/   else               { analogWrite(BR_F, 0000000);              /**/                      
/**/                        analogWrite(BR_B, 0000000);         }    /**/    
/***********************************************************************/                  
/***********************************************************************/                   
 
}
void motor_speed_BL(int speed_BL )
{
/***********************************************************************/                   
/***********************************************************************/     
////░█▀▄░█▀█░█▀▀░█░█░░░█░░░█▀▀░█▀▀░▀█▀                               /**/
////░█▀▄░█▀█░█░░░█▀▄░░░█░░░█▀▀░█▀▀░░█░                               /**/
////░▀▀░░▀░▀░▀▀▀░▀░▀░░░▀▀▀░▀▀▀░▀░░░░▀░                               /**/           
/**/                                                                 /**/
/**/ if     (speed_BL>0)  { analogWrite(BL_F, speed_BL);             /**/                    
/**/                        analogWrite(BL_B, 0000000);         }    /**/                    
/**/else if (speed_BL<0)  { analogWrite(BL_F, 0000000);              /**/                      
/**/                        analogWrite(BL_B, abs(speed_BL) );  }    /**/                    
/**/   else               { analogWrite(BL_F, 0000000);              /**/                      
/**/                        analogWrite(BL_B, 0000000);         }    /**/    
/***********************************************************************/                  
/***********************************************************************/                   
 
}
void ros_input(int speed_fl,int speed_fr,int speed_bl,int speed_br)
{
/*****************************************************************/                   
/*******************************************************************/                 
// ░█▀▀░█▀▄░█▀█░█▀█░▀█▀░░░█▀▄░▀█▀░█▀▀░█░█░▀█▀                   /**/
// ░█▀▀░█▀▄░█░█░█░█░░█░░░░█▀▄░░█░░█░█░█▀█░░█░                   /**/
// ░▀░░░▀░▀░▀▀▀░▀░▀░░▀░░░░▀░▀░▀▀▀░▀▀▀░▀░▀░░▀░                   /**/                                                             
 /**/if(speed_fr>0)   { analogWrite(FR_F, speed_fr);            /**/   
/**/                   analogWrite(FR_B,      0        );    }  /**/   
/**/ else            { analogWrite(FR_F, 0       );             /**/   
/**/                    analogWrite(FR_B, abs(speed_fr) );    } /**/   
 /**/                                                            /**/     
/******************************************************************/                  
/*****************************************************************/                   
/*****************************************************************/                   
/*******************************************************************/        
//░█▀▄░█▀█░█▀▀░█░█░░░█░░░█▀▀░█▀▀░▀█▀                              /**/
//░█▀▄░█▀█░█░░░█▀▄░░░█░░░█▀▀░█▀▀░░█░                              /**/
//░▀▀░░▀░▀░▀▀▀░▀░▀░░░▀▀▀░▀▀▀░▀░░░░▀░                              /**/      
/**/                                                             /**/
/**/ if(speed_bl>0)   { analogWrite(BL_F, speed_bl);             /**/                    
/**/                    analogWrite(BL_B,      0        );  }    /**/                    
/**/  else            { analogWrite(BL_F, 0       );             /**/                      
/**/                    analogWrite(BL_B, abs(speed_bl) );  }    /**/                    
/**/
/*******************************************************************/                  
/*****************************************************************/                   
 
/*****************************************************************/                   
/*******************************************************************/    
//   ░█▀▄░█▀█░█▀▀░█░█░░░█▀▄░▀█▀░█▀▀░█░█░▀█▀                     /**/ 
//   ░█▀▄░█▀█░█░░░█▀▄░░░█▀▄░░█░░█░█░█▀█░░█░                     /**/
//   ░▀▀░░▀░▀░▀▀▀░▀░▀░░░▀░▀░▀▀▀░▀▀▀░▀░▀░░▀░                     /**/                                                       
 /**/if(speed_br>0)   { analogWrite(BR_F, speed_br);            /**/   
/**/                   analogWrite(BR_B,      0        );    }  /**/   
/**/ else            { analogWrite(BR_F, 0       );             /**/   
/**/                    analogWrite(BR_B, abs(speed_br) );    } /**/   
 /**/                                                            /**/     
/******************************************************************/                  
/*****************************************************************/                   
 
                                              
}
void backward(int speed)
{
   
/**********************************/                  /***********************************/
/**********************************/                  /***********************************/
  /**/ analogWrite(FL_F,  0   );    /********************/  analogWrite(FR_F,  0   ); /**/   
 /**/ analogWrite(FL_B, speed);    /********************/  analogWrite(FR_B, speed); /**/  
/**********************************/       /****/     /***********************************/
/**********************************/       /****/     /***********************************/
                                           /****/
                                           /****/
                                           /****/
                                           /****/
                                           /****/
/**********************************/                  /***********************************/
/**********************************/                  /***********************************/
 /**/ analogWrite(BL_F,  0   );    /********************/  analogWrite(BR_F,  0   ); /**/    
 /**/ analogWrite(BL_B, speed);    /********************/  analogWrite(BR_B, speed); /**/    
/**********************************/                  /***********************************/
/**********************************/                  /***********************************/
                                              
}
void rotate_cw(int speed)
{
   
/**********************************/                  /***********************************/
/**********************************/                  /***********************************/
  /**/ analogWrite(FL_F,speed );    /********************/  analogWrite(FR_F,  0   ); /**/   
 /**/ analogWrite(FL_B,    0  );    /********************/  analogWrite(FR_B, speed); /**/  
/**********************************/       /****/     /***********************************/
/**********************************/       /****/     /***********************************/
                                           /****/
                                           /****/
                                           /****/
                                           /****/
                                           /****/
/**********************************/                  /***********************************/
/**********************************/                  /***********************************/
 /**/ analogWrite(BL_F,  speed);    /********************/  analogWrite(BR_F,  0   ); /**/    
 /**/ analogWrite(BL_B, 0     );    /********************/  analogWrite(BR_B, speed); /**/    
/**********************************/                  /***********************************/
/**********************************/                  /***********************************/
                                              
}
void rotate_ccw(int speed)
{
   
/**********************************/                  /***********************************/
/**********************************/                  /***********************************/
/**/ analogWrite(FL_F,  0   );    /********************/  analogWrite(FR_F,speed ); /**/   
/**/ analogWrite(FL_B,speed );    /********************/  analogWrite(FR_B,   0  ); /**/  
/**********************************/       /****/     /***********************************/
/**********************************/       /****/     /***********************************/
                                           /****/
                                           /****/
                                           /****/
                                           /****/
                                           /****/
/**********************************/                  /***********************************/
/**********************************/                  /***********************************/
 /**/ analogWrite(BL_F, 0     );    /********************/  analogWrite(BR_F,speed ); /**/    
 /**/ analogWrite(BL_B, speed );    /********************/  analogWrite(BR_B, 0    ); /**/    
/**********************************/                  /***********************************/
/**********************************/                  /***********************************/
                                              
}
void turn_left(int speed)
{
   
/**********************************/                  /***********************************/
/**********************************/                  /***********************************/
/**/ analogWrite(FL_F,  0   );    /********************/  analogWrite(FR_F,speed ); /**/   
/**/ analogWrite(FL_B,  0   );    /********************/  analogWrite(FR_B,   0  ); /**/  
/**********************************/       /****/     /***********************************/
/**********************************/       /****/     /***********************************/
                                           /****/
                                           /****/
                                           /****/
                                           /****/
                                           /****/
/**********************************/                  /***********************************/
/**********************************/                  /***********************************/
 /**/ analogWrite(BL_F, 0     );    /********************/  analogWrite(BR_F,speed ); /**/    
 /**/ analogWrite(BL_B,   0   );    /********************/  analogWrite(BR_B, 0    ); /**/    
/**********************************/                  /***********************************/
/**********************************/                  /***********************************/
                                              
}
void turn_right(int speed)
{
   
/**********************************/                  /***********************************/
/**********************************/                  /***********************************/
/**/ analogWrite(FL_F,speed );    /********************/  analogWrite(FR_F,   0  ); /**/   
/**/ analogWrite(FL_B,  0   );    /********************/  analogWrite(FR_B,   0  ); /**/  
/**********************************/       /****/     /***********************************/
/**********************************/       /****/     /***********************************/
                                           /****/
                                           /****/
                                           /****/
                                           /****/
                                           /****/
/**********************************/                  /***********************************/
/**********************************/                  /***********************************/
 /**/ analogWrite(BL_F,speed  );    /********************/  analogWrite(BR_F, 0    ); /**/    
 /**/ analogWrite(BL_B,   0   );    /********************/  analogWrite(BR_B, 0    ); /**/    
/**********************************/                  /***********************************/
/**********************************/                  /***********************************/
                                              
}
void stop(int speed=0)
{
   
/**********************************/                  /***********************************/
/**********************************/                  /***********************************/
  /**/ analogWrite(FL_F,  0   );    /********************/  analogWrite(FR_F,  0   ); /**/   
 /**/ analogWrite(FL_B,   0   );    /********************/  analogWrite(FR_B,   0  ); /**/  
/**********************************/       /****/     /***********************************/
/**********************************/       /****/     /***********************************/
                                           /****/
                                           /****/
                                           /****/
                                           /****/
                                           /****/
/**********************************/                  /***********************************/
/**********************************/                  /***********************************/
 /**/ analogWrite(BL_F,  0   );    /********************/  analogWrite(BR_F,  0   ); /**/    
 /**/ analogWrite(BL_B,   0  );    /********************/  analogWrite(BR_B,   0  ); /**/    
/**********************************/                  /***********************************/
/**********************************/                  /***********************************/
              // delay(1000);
}
void setup() {
 
          Serial.begin(115200);  // Start serial communication for debugging
          Serial3.begin(115200);  // Start serial communication for debugging
 
 
          // Initialize motor driver pins
          pinMode(BL_B, OUTPUT);
          pinMode(BL_F, OUTPUT);
          pinMode(BR_B, OUTPUT);
          pinMode(BR_F, OUTPUT);
          pinMode(FL_B, OUTPUT);
          pinMode(FL_F, OUTPUT);
          pinMode(FR_B, OUTPUT);
          pinMode(FR_F, OUTPUT);
          pinMode(LED_PIN, OUTPUT);
          // Initialize Hall sensor pins
          pinMode(HALL_SENSOR_BL_A_PIN, INPUT_PULLUP);  // Hall sensor channel A pin
          pinMode(HALL_SENSOR_BL_B_PIN, INPUT_PULLUP);  // Hall sensor channel B pin
          pinMode(HALL_SENSOR_BR_A_PIN, INPUT_PULLUP);  // Hall sensor channel A pin
          pinMode(HALL_SENSOR_BR_B_PIN, INPUT_PULLUP);  // Hall sensor channel B pin
          pinMode(HALL_SENSOR_FR_A_PIN, INPUT_PULLUP);  // Hall sensor channel A pin
          pinMode(HALL_SENSOR_FR_B_PIN, INPUT_PULLUP);  // Hall sensor channel B pin
          pinMode(HALL_SENSOR_FL_A_PIN, INPUT_PULLUP);  // Hall sensor channel A pin
          pinMode(HALL_SENSOR_FL_B_PIN, INPUT_PULLUP);  // Hall sensor channel B pin
          attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_BL_A_PIN), onHallSensor_BL_A, RISING);
          attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_BR_A_PIN), onHallSensor_BR_A, RISING); 
          attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_FL_A_PIN), onHallSensor_FL_A, RISING);
          attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_FR_A_PIN), onHallSensor_FR_A, RISING); 
          delay(500);




        #ifdef USE_SPI
        SPI_PORT.begin();
        #else
        WIRE_PORT.begin();
        WIRE_PORT.setClock(400000);
        #endif

          bool initialized = false;
  while (!initialized)
  {

#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif

    // Serial3.print(F("Initialization of the sensor returned: "));
    Serial3.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      Serial3.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
  
}
void loop() { 
// ros_input(60,60,60,60);  // ros_input(int speed_fl,int speed_fr,int speed_bl,int speed_br)
 
if (stringComplete) {
        parseData(inputString);  // Parse the received string
        inputString = "";  // Clear the input buffer
        stringComplete = false;  // Reset flag for next data
    }
 
  // Calculate the measured speed every second
  unsigned long tx_currentMillis = millis();
  if (tx_currentMillis - Tx_previousMillis >= Tx_interval) {

    // odometry_BR=odometry_FL=odometry_BL=odometry_FR=dummyOdom;
    

    
    Tx_previousMillis = tx_currentMillis;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
     Serial3.print("$BR:");
     Serial3.print(odometry_BR); 
     Serial3.print(",BL:");
     Serial3.print(odometry_BL); 
     Serial3.print(",FL:");
     Serial3.print(odometry_FL); 
     Serial3.print(",FR:");
     Serial3.print(odometry_FR); 
     Serial3.print(",BT:");
     Serial3.print(BAT_VOLT); 
     Serial3.print(","); 
     if (myICM.dataReady())
  {
    myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
                             //    printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
    printScaledAGMT(&myICM); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
    delay(30);
  }





     
     Serial3.println("#");
    //  dummyOdom=dummyOdom+1;
  }
  
  // Calculate the measured speed every second
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    // Calculate the speed (RPM)
    
    measuredSpeed_BL = ((pulseCount_BL / 2.0) * (60.0 / interval)) * GEAR_RATIO * direction_BL ;  // RPM calculation with gear ratio
    measuredSpeed_BR = ((pulseCount_BR / 2.0) * (60.0 / interval)) * GEAR_RATIO * direction_BR ;  // RPM calculation with gear ratio    
    measuredSpeed_FL = ((pulseCount_FL / 2.0) * (60.0 / interval)) * GEAR_RATIO * direction_FL ;  // RPM calculation with gear ratio
    measuredSpeed_FR = ((pulseCount_FR / 2.0) * (60.0 / interval)) * GEAR_RATIO * direction_FR ;  // RPM calculation with gear ratio
    // Calculate the error between target speed and measured speed
    float error_BL = targetSpeed_BL - measuredSpeed_BL;
    float error_BR = targetSpeed_BR - measuredSpeed_BR;  
    float error_FL = targetSpeed_FL - measuredSpeed_FL;
    float error_FR = targetSpeed_FR - measuredSpeed_FR; 
 
 //___________________________________________________________________proportional control________________________________________________________________________________________________________
 //___________________________________________________________________BAck left________________________________________________________________________________________________________
    if (abs(error_BL) > thro)
     {
        if      (targetSpeed_BL > 0) { pwmOutput_BL_F = pwmOutput_BL_F + 0 + (error_BL / 1);     pwmOutput_BL_B =0;  }    // PWM output based on error 
        else if (targetSpeed_BL < 0) {   pwmOutput_BL_B = pwmOutput_BL_B + 0 -error_BL / 1;      pwmOutput_BL_F =0;  }    // PWM output based on error 
        else                         { pwmOutput_BL_F = 0;                                       pwmOutput_BL_B =0;  }    // PWM output based on error 
        
      }  
      pwmOutput_BL_F = constrain(pwmOutput_BL_F, 0, max_pwm_value);  
      pwmOutput_BL_B = constrain(pwmOutput_BL_B, 0, max_pwm_value);  
      analogWrite(BL_F,  pwmOutput_BL_F   );
      analogWrite(BL_B,  pwmOutput_BL_B   );
 
 //___________________________________________________________________BAck right________________________________________________________________________________________________________
    if (abs(error_BR) > thro)
     {
        if      (targetSpeed_BR > 0) { pwmOutput_BR_F = pwmOutput_BR_F + 0 + (error_BR * p);     pwmOutput_BR_B =0;  }    // Proportional control
        else if (targetSpeed_BR < 0) { pwmOutput_BR_B = pwmOutput_BR_B + 0 -error_BR * p;        pwmOutput_BR_F =0;  }    // PWM output based on error 
        else                         { pwmOutput_BR_F = 0;                                       pwmOutput_BR_B =0;  }    // PWM output based on error 
      }  
      pwmOutput_BR_F = constrain(pwmOutput_BR_F, 0, max_pwm_value);  
      pwmOutput_BR_B = constrain(pwmOutput_BR_B, 0, max_pwm_value);  
      analogWrite(BR_F,  pwmOutput_BR_F   );
      analogWrite(BR_B,  pwmOutput_BR_B   );
  
 //___________________________________________________________________Front right________________________________________________________________________________________________________
    if (abs(error_FR) > thro)
     {
        if      (targetSpeed_FR > 0) { pwmOutput_FR_F = pwmOutput_FR_F + 0 + (error_FR * p);    pwmOutput_FR_B =0;   }    // PWM output based on error 
        else if (targetSpeed_FR < 0) { pwmOutput_FR_B = pwmOutput_FR_B + 0 -error_FR * p;       pwmOutput_FR_F =0;   }    // PWM output based on error 
        else                         { pwmOutput_FR_F = 0;                                      pwmOutput_FR_B =0;   }    // PWM output based on error 
        
      }  
      pwmOutput_FR_F = constrain(pwmOutput_FR_F, 0, max_pwm_value);  
      pwmOutput_FR_B = constrain(pwmOutput_FR_B, 0, max_pwm_value);  
      analogWrite(FR_F,  pwmOutput_FR_F   );
      analogWrite(FR_B,  pwmOutput_FR_B   );
 
 //___________________________________________________________________Front left________________________________________________________________________________________________________
    if (abs(error_FL) > thro)
     {
        if      (targetSpeed_FL > 0) { pwmOutput_FL_F = pwmOutput_FL_F + 0 + (error_FL * p);     pwmOutput_FL_B =0;   }    // PWM output based on error 
        else if (targetSpeed_FL < 0) { pwmOutput_FL_B = pwmOutput_FL_B + 0 -error_FL * p;        pwmOutput_FL_F =0;      }    // PWM output based on error 
        else                         { pwmOutput_FL_F = 0;                                       pwmOutput_FL_B =0;  }    // PWM output based on error 
        
      }  
      pwmOutput_FL_F = constrain(pwmOutput_FL_F, 0, max_pwm_value);  
      pwmOutput_FL_B = constrain(pwmOutput_FL_B, 0, max_pwm_value);  
      analogWrite(FL_F,  pwmOutput_FL_F   );
      analogWrite(FL_B,  pwmOutput_FL_B   );
 
      Serial.println("");
      Serial.print(" ID");  
      Serial.print("\tDir");  
      Serial.print("\tT_Speed");  
      Serial.print("\tM_Speed");     
      Serial.print("\tError");   
      Serial.print("\tPWM_F");
      Serial.print("\tPWM_B");
      Serial.print("\tODOM");
      Serial.print("\tBAT_V");
   BAT_VOLT = analogRead(ADC_PIN)* 0.014648438 ;  // Read ADC value 
    serial_print( "BL" , error_BL , direction_BL , measuredSpeed_BL ,targetSpeed_BL  , pwmOutput_BL_F , pwmOutput_BL_B , odometry_BL);  
    serial_print( "BR" , error_BR , direction_BR , measuredSpeed_BR ,targetSpeed_BR  , pwmOutput_BR_F , pwmOutput_BR_B , odometry_BR); 
    serial_print( "FR" , error_FR , direction_FR , measuredSpeed_FR ,targetSpeed_FR  , pwmOutput_FR_F , pwmOutput_FR_B , odometry_FR); 
    serial_print( "FL" , error_FL , direction_FL , measuredSpeed_FL ,targetSpeed_FL  , pwmOutput_FL_F , pwmOutput_FL_B , odometry_FL); 
 
    // Reset pulse count for next interval
    pulseCount_BL = 0;
    pulseCount_BR = 0;
    pulseCount_FL = 0;
    pulseCount_FR = 0;
  }
}
// Interrupt function for Channel A (pulse detection)
void onHallSensor_BL_A() {
  // Increment pulse count whenever Channel A transitions
  pulseCount_BL++;
  // Determine the direction_BL based on Channel B's state when Channel A goes HIGH
  if (digitalRead(HALL_SENSOR_BL_B_PIN) == HIGH) {
    direction_BL = -1;  // Clockwise
    odometry_BL--;
  } else {
    direction_BL = 1;  // Counterclockwise
    odometry_BL++;
  }
} 
// Interrupt function for Channel A (pulse detection)
void onHallSensor_BR_A() {
  // Increment pulse count whenever Channel A transitions
  pulseCount_BR++;
  // Determine the direction_BL based on Channel B's state when Channel A goes HIGH
  if (digitalRead(HALL_SENSOR_BR_B_PIN) == HIGH) {
    direction_BR = 1;  // Clockwise
    odometry_BR++;
  } else {
    direction_BR =  -1;  // Counterclockwise
    odometry_BR--;
  }
} 
 
// Interrupt function for Channel A (pulse detection)
void onHallSensor_FL_A() {
  // Increment pulse count whenever Channel A transitions
  pulseCount_FL++;
  // Determine the direction_BL based on Channel B's state when Channel A goes HIGH
  if (digitalRead(HALL_SENSOR_FL_B_PIN) == HIGH) {
    direction_FL = -1;  // Clockwise
    odometry_FL--;
  } else {
    direction_FL = 1;  // Counterclockwise
    odometry_FL++;
  }
} 
// Interrupt function for Channel A (pulse detection)
void onHallSensor_FR_A() {
  // Increment pulse count whenever Channel A transitions
  pulseCount_FR++;
  // Determine the direction_BL based on Channel B's state when Channel A goes HIGH
  if (digitalRead(HALL_SENSOR_FR_B_PIN) == HIGH) {
    direction_FR =  1;  // Clockwise
    odometry_FR++;
  } else {
    direction_FR = -1;  // Counterclockwise
    odometry_FR--;
  }
} 
///////////////////////////////////////////////////   serial receptions  /////////////////////////////////////////////////////////////////////////////////////
void serialEvent3() {
    while (Serial3.available()) {
        char inChar = (char)Serial3.read();
        if (inChar == '$') {  
            inputString = "";  // Start a new string when '$' is detected
        }
        inputString += inChar;
        if (inChar == '#') {  
            stringComplete = true;  // Mark string as complete
        }
    }
}
// ///////////////////////////////////////////////////   serial receptions  /////////////////////////////////////////////////////////////////////////////////////
void serialEvent() {
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        if (inChar == '$') {  
            inputString = "";  // Start a new string when '$' is detected
        }
        inputString += inChar;
        if (inChar == '#') {  
            stringComplete = true;  // Mark string as complete
        }
    }
}
 
void parseData(String data) {
    if (data.startsWith("$") && data.endsWith("#")) {
        data = data.substring(1, data.length() - 1);  // Remove '$' and '#'
        if (sscanf(data.c_str(), "FR:%d,FL:%d,BR:%d,BL:%d",&targetSpeed_FR, &targetSpeed_FL, &targetSpeed_BR, &targetSpeed_BL) == 4)
         {
            
            targetSpeed_FL = constrain(targetSpeed_FL, - targetSpeedMax, targetSpeedMax);  
            targetSpeed_FR = constrain(targetSpeed_FR, - targetSpeedMax, targetSpeedMax);  
            targetSpeed_BL = constrain(targetSpeed_BL, - targetSpeedMax, targetSpeedMax);  
            targetSpeed_BR = constrain(targetSpeed_BR, - targetSpeedMax, targetSpeedMax);  
            Serial.print("targetSpeed_FR: "); Serial.println(targetSpeed_FR);
            Serial.print("targetSpeed_FL: "); Serial.println(targetSpeed_FL);
            Serial.print("targetSpeed_BR: "); Serial.println(targetSpeed_BR);
            Serial.print("targetSpeed_BL: "); Serial.println(targetSpeed_BL);
        }
        else
         {
            Serial.println("Parsing Error!");
        }
    }
}
///////////////////////////////////////////////////   serial receptions  /////////////////////////////////////////////////////////////////////////////////////
/*
 $FR:0,FL:0,BR:0,BL:0#
 $FR:30,FL:-3,BR:12,BL:30#
 $FR:200,FL:200,BR:200,BL:200#
 $FR:20,FL:20,BR:20,BL:20#
 $FR:2,FL:2,BR:2,BL:2#
 $FR:1,FL:1,BR:1,BL:1#
*/
