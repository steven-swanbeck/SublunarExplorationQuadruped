#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <Adafruit_PWMServoDriver.h>
#include <MPU6050_light.h>
#include "TFMini.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
int Freq = 60;
#define DEFAULT_PULSE_WIDTH 1500;
int polarangle = 0;
int azimuthalanglesupplement = 0;
int azimuth = 0;
int lastpolarangle;
double xcoord, ycoord, zcoord;

unsigned long previousMillisServo = 0;
const long interval = 10;
unsigned long previousMillisLIDAR = previousMillisServo + interval/5;

//Define LIDAR Device
TFMini tfmini;

//Define Serial Data Communication Ports
char serial_buffer[15];
float theta,x,y;

//Servo Angle Calculation
int pulseWidth(int angle){
  int pulse_wide, analog_value;
  pulse_wide = map(angle, 0, 180, 650, 2350);
  analog_value = int(float(pulse_wide) / 1000000 * Freq *4096);
  return analog_value;
}

//LIDAR Data Reading Protocol
void getTFminiData(int* distance) {
  static char i = 0;
  char j = 0;
  int checksum = 0; 
  static int rx[9];
  if(Serial3.available())
  {  
    // Serial.println( "tfmini serial available" );
    rx[i] = Serial3.read();
    if(rx[0] != 0x59) {
      i = 0;
    } else if(i == 1 && rx[1] != 0x59) {
      i = 0;
    } else if(i == 8) {
      for(j = 0; j < 8; j++) {
        checksum += rx[j];
      }
      if(rx[8] == (checksum % 256)) {
        *distance = rx[2] + rx[3] * 256;
      }
      i = 0;
    } else 
    {
      i++;
    } 
  }  
}

//Setup
void setup() { 
  //PWM Specifications
  pwm.begin();
  pwm.setPWMFreq(Freq);
  
  //Initialize hardware serial port (serial debug port)
  Serial.begin(115200);
  
  //Wait for serial port to connect
  while (!Serial3);
  Serial.println ("Initializing...");
  
  //Initialize the data rate for the SoftwareSerial port
  Serial3.begin(TFMINI_BAUDRATE);
  
  //Initialize the TF Mini
  tfmini.begin(&Serial3);  
  Serial.println("setup complete");

  //Set Initial Servo Angle 
  pwm.setPWM(12,0,pulseWidth(polarangle));
  pwm.setPWM(13,0,pulseWidth(azimuthalanglesupplement));
  lastpolarangle = polarangle;
}

//Reading and Motion Loops
void loop() 
{
  int distance = 0;
  
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillisLIDAR >= interval){
    previousMillisLIDAR = currentMillis;
    getTFminiData(&distance);
    while(!distance){
      getTFminiData(&distance);
      if(distance){
      }
    }
    if (currentMillis - previousMillisServo >= interval){
      previousMillisServo = currentMillis;

      //Bidriectional Sweep
      if ((lastpolarangle <= polarangle) && (polarangle < 180)){
        lastpolarangle = polarangle;
        polarangle++;
      }
      else if (polarangle == 180){
        lastpolarangle = polarangle;
        polarangle--;
        azimuthalanglesupplement++;
        if (azimuthalanglesupplement == 90){ //180
            azimuthalanglesupplement = 0;
        }
      }
      else if ((lastpolarangle > polarangle) && (polarangle > 0)){
        lastpolarangle = polarangle;
        polarangle--;
      }
      else if (polarangle == 0){
        lastpolarangle = polarangle;
        polarangle = polarangle++;
      }
      else{
        Serial.print("sweep error");
      }

        //Unidirectional Sweep
//      polarangle = polarangle + 1;
//        if (polarangle == 180){
//            polarangle = 0;
//            azimuthalanglesupplement = azimuthalanglesupplement + 1;
//            if (azimuthalanglesupplement == 175){
//              azimuthalanglesupplement = 80;
//            }
//        }

      pwm.setPWM(12,0,pulseWidth(polarangle));
      pwm.setPWM(13,0,pulseWidth(azimuthalanglesupplement));
      
      theta = polarangle;
      azimuth = 90 - azimuthalanglesupplement;
      double standardxcoord = -1 * distance * sin(azimuth * PI / 180.0f) * cos(theta * PI / 180.0f);
      double standardycoord = distance * sin(azimuth * PI / 180.0f) * sin(theta * PI / 180.0f);
      double standardzcoord = distance * cos(azimuth * PI / 180.0f);
      
      if (azimuth >= 0)
      {
        xcoord = standardxcoord;
        ycoord = standardycoord;
        zcoord = standardzcoord;
      }

      else if (azimuth < 0)
      {
        xcoord = standardxcoord;
        ycoord = standardycoord;
        zcoord = standardzcoord;
      }

       if ((lastpolarangle <= polarangle) && (polarangle < 180)){
           sprintf(serial_buffer,"%d\t%d\t%d\n\0",(int)(xcoord),(int)(ycoord),(int)(zcoord));
           Serial.print(serial_buffer);
          }
       else if (polarangle == 180){
           sprintf(serial_buffer,"%d\t%d\t%d\n\0",(int)(xcoord),(int)(ycoord),(int)(zcoord));
           Serial.print(serial_buffer);
         }
    }
  }
}