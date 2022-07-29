#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <Adafruit_PWMServoDriver.h>
#include <MPU6050_light.h>
#include "TFMini.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
MPU6050 mpu(Wire);
int Freq = 60;
#define DEFAULT_PULSE_WIDTH 1500;

//Name Functions and Set Vectors
int angle [12];
void setposition();
void IK();
float x[4], y[4], z[4];
float w[4];
int v;
int stopcondition = false;

//Shoulder Parameters
float shoulderangle; 
const int shoulderoffset = 75;  //mm

//Leg Parameters
const int a = 105;       //upper leg length, mm
const int b = 80;      //lower leg length, mm

//Body Parameters
float bodyheight;
const int bodywidth = 102;
float stepwidth;
float pitchangle;
float rollangle;
float yawangle;
float setpitchangle;
const int bodyradius = 70;     //mm

//Gait Parameters
float xmax;
float xmin;
float zmax;
float stepinterval;
float ymax;
float ymin;
float stepintervaly;

//Miscellaneous
float constc;
float constcy;

//Calibrated Rest Configuration
int servopositionint[12] = {175, 48, 90, 0, 134, 92, 172, 45, 91, 1, 133, 90};

//Baseline Starting Point
int intservopos[12] = {0, 135, 80, 180, 45, 101, 0, 135, 103, 180, 45, 84}; // good with 20, -50, 7 gait pitchangle 4
int servopos[12];

//Xprev Vector
float xprev[4];
float yprev[4];

//Servo Calibration Angles
const int p4setpoint = 175;
const int p5setpoint = 44;
const int p6setpoint = 90;
const int p7setpoint = 0;
const int p8setpoint = 134;
const int p9setpoint = 96;
const int p10setpoint = 175;
const int p11setpoint = 43;
const int p12setpoint = 93;
const int p13setpoint = 0;
const int p14setpoint = 133;
const int p15setpoint = 89;

//Pitch Control Variables
float kp = 0.05; //0.05;
float kd = 1.5; //1.5
float ki = 0.00001;

int milliOld;
int milliNew;
int dt;
float pitchTarget = 0;
float pitchActual;
float pitchError = 0;
float pitchErrorOld;
float pitchErrorChange;
float pitchErrorSlope = 0;
float pitchErrorArea = 0;
float pitchCorrection = 0;

float rollTarget = 0;
float rollActual;
float rollError = 0;
float rollErrorOld;
float rollErrorChange;
float rollErrorSlope = 0;
float rollErrorArea = 0;
float rollCorrection = 0;

int legParameter = 0;

// Function Prototypes
void walkForwardStabilize();
void walkforward();
void walkbackward();
void turnleft();
void turnright();
int pulseWidth(int angle);
void setpos();
void IK();
void MPUsetup();
void selfStabilize();
void remoteControl();
int selectLeg(int command);
void resetstance();
void checkstopcondition();
void pitchforward();
void pitchbackward();
void rollleft();
void rollright();
void yawleft();
void yawright();
void increaseX();
void decreaseX();
void increaseY();
void decreaseY();
void increaseZ();
void decreaseZ();

void setup() {
    pwm.begin();
    pwm.setPWMFreq(Freq);
  
    Serial.begin(9600);

    MPUsetup();

    //Adjustable Parameters
    bodyheight = 140;                    //mm
    pitchTarget = 0;                      //degrees
    pitchangle = pitchTarget + pitchCorrection;
    rollTarget = 0;
    rollangle = rollTarget + rollCorrection;
    yawangle = 0;
    stepwidth = 102;
    x[0] = -35;
    x[1] = -35;
    x[2] = -35;
    x[3] = -35;    //mm
    y[0] = 15;
    y[1] = -15;
    y[2] = 15;
    y[3] = -15;   //mm
    z[0] = 0;
    z[1] = 0;
    z[2] = 0;
    z[3] = 0;       //mm
    zmax = 20;                           //mm
    xmax = 0;                           //mm
    xmin = -45;                          //mm
    stepinterval = (xmax - xmin) / 5;    //mm
    xprev[0] = xprev[1] = xprev[2] = xprev[3] = 0;
    ymax = 0;
    ymin = -30;
    stepintervaly = (ymax - ymin) / 5;
    yprev[0] = yprev[1] = yprev[2] = yprev[3] = 0;

    IK();
    setpos(); 

    milliNew = millis();
}

void loop() {
    remoteControl();
    selfStabilize();
}

void walkForwardStabilize() // xmax 0 xmin -45
{
  constc = zmax / (-0.25 * pow(xmin + xmax, 2) + xmin * xmax);
  
  x[0] = xmax;
  x[1] = xmin; //+ (xmax - xmin) / 3;
  x[2] = xmin; //+ (xmax - xmin) / 3;
  x[3] = xmax;

  while (stopcondition == false)
  {
   if (millis() - milliNew > 5)
   {
   for (int j = 0; j <= 3; j++)
  {
    if ((xprev[j] <= x[j]) && (x[j] < xmax))
    {
      xprev[j] = x[j];
      x[j] = x[j] + stepinterval;
      z[j] = constc * (x[j] - xmin) * (x[j] - xmax);
    }
    else if (x[j] >= xmax)
    {
      xprev[j] = x[j];
      x[j] = x[j] - stepinterval;
      z[j] = 0;
    }
    else if ((xprev[j] > x[j]) && (x[j] > xmin))
    {
      xprev[j] = x[j];
      x[j] = x[j] - stepinterval;
      z[j] = 0;
    }
    else if (x[j] <= xmin)
    {
      xprev[j] = x[j];
      x[j] = x[j] + stepinterval;
      z[j] = constc * (x[j] - xmin) * (x[j] - xmax);
    }
    else
    {
      x[j] = xprev[j];
      z[j] = 0;
    }

    mpu.update();
    pitchActual = -1 * mpu.getAngleX();
    Serial.println(pitchActual);
    rollActual = -1 * mpu.getAngleY();
  
    milliOld = milliNew;
    milliNew = millis();
    dt = milliNew - milliOld;

    pitchErrorOld = pitchError;
    pitchError = pitchTarget - pitchActual;
    Serial.println(pitchError);
    pitchErrorChange = pitchError - pitchErrorChange;
    pitchErrorSlope = pitchErrorChange / dt;
    pitchErrorArea = pitchErrorArea + pitchErrorArea * dt;

    rollErrorOld = rollError;
    rollError = rollTarget - rollActual;
    rollErrorChange = rollError - rollErrorChange;
    rollErrorSlope = rollErrorChange / dt;
    rollErrorArea = rollErrorArea + rollErrorArea * dt;

    pitchCorrection = pitchCorrection + kp * pitchError;// + kd * pitchErrorSlope + ki * pitchErrorArea;
    if (pitchCorrection >= 15)
    {
      pitchCorrection = 15;
    }
    else if (pitchCorrection <= -15)
    {
      pitchCorrection = -15;
    }

    rollCorrection = rollCorrection + kp * rollError;// + kd * rollErrorSlope + ki * rollErrorArea;
    if (rollCorrection >= 15)
    {
      rollCorrection = 15;
    }
    else if (rollCorrection <= -15)
    {
      rollCorrection = -15;
    }
    pitchangle = pitchTarget + pitchCorrection;
    rollangle = rollTarget + rollCorrection;

    
    IK();
    setpos();

    checkstopcondition();

    if (stopcondition == true)
    {
      break;
    }
  } 
  }
  }
}

void walkforward() // xmax 0 xmin -45
{
  constc = zmax / (-0.25 * pow(xmin + xmax, 2) + xmin * xmax);
  
  x[0] = xmax;
  x[1] = xmin; //+ (xmax - xmin) / 3;
  x[2] = xmin; //+ (xmax - xmin) / 3;
  x[3] = xmax;

//  checkstopcondition();
  
  while (stopcondition == false)
  {
   if (millis() - milliNew > 5)
   {
   for (int j = 0; j <= 3; j++)
  {
    if ((xprev[j] <= x[j]) && (x[j] < xmax))
    {
      xprev[j] = x[j];
      x[j] = x[j] + stepinterval;
      z[j] = constc * (x[j] - xmin) * (x[j] - xmax);
    }
    else if (x[j] >= xmax)
    {
      xprev[j] = x[j];
      x[j] = x[j] - stepinterval;
      z[j] = 0;
    }
    else if ((xprev[j] > x[j]) && (x[j] > xmin))
    {
      xprev[j] = x[j];
      x[j] = x[j] - stepinterval;
      z[j] = 0;
    }
    else if (x[j] <= xmin)
    {
      xprev[j] = x[j];
      x[j] = x[j] + stepinterval;
      z[j] = constc * (x[j] - xmin) * (x[j] - xmax);
    }
    else
    {
      x[j] = xprev[j];
      z[j] = 0;
    }

    IK();
    setpos();

    checkstopcondition();
  } 
  }
  }
}

void walkbackward()
{
  constc = zmax / (-0.25 * pow(xmin + xmax, 2) + xmin * xmax);
  
  x[0] = xmax;
  x[1] = xmin; //+ (xmax - xmin) / 3;
  x[2] = xmin; //+ (xmax - xmin) / 3;
  x[3] = xmax;

  while (stopcondition == false)
  {
   for (int j = 0; j <= 3; j++)
  {
    if ((xprev[j] <= x[j]) && (x[j] < xmax))
    {
      xprev[j] = x[j];
      x[j] = x[j] + stepinterval;
      z[j] = 0;
    }
    else if (x[j] >= xmax)
    {
      xprev[j] = x[j];
      x[j] = x[j] - stepinterval;
      z[j] = constc * (x[j] - xmin) * (x[j] - xmax);
    }
    else if ((xprev[j] > x[j]) && (x[j] > xmin))
    {
      xprev[j] = x[j];
      x[j] = x[j] - stepinterval;
      z[j] = constc * (x[j] - xmin) * (x[j] - xmax);
    }
    else if (x[j] <= xmin)
    {
      xprev[j] = x[j];
      x[j] = x[j] + stepinterval;
      z[j] = 0;
    }
    else
    {
      x[j] = xprev[j];
      z[j] = 0;
    }
    IK();
    setpos();

    checkstopcondition();
  } 
  }
}

void turnleft()
{
  y[0] = ymin;
  y[1] = ymin;
  y[2] = ymax;
  y[3] = ymax;

  constcy = zmax / (-0.25 * pow(ymin + ymax, 2) + ymin * ymax);

  while (stopcondition == false)
  {
    for (int j =0; j <= 3; j++)
    {
      if ((yprev[j] <= y[j]) && (y[j] < ymax))
      {
        yprev[j] = y[j];
        y[j] = y[j] + stepintervaly;
        if ((j == 1) || (j == 3))
        {
          z[j] = constcy * (y[j] - ymin) * (y[j] - ymax);
        }
        else if ((j == 0) || (j == 2))
        {
          z[j] = 0;
        }
      }
      else if (y[j] >= ymax)
      {
        yprev[j] = y[j];
        y[j] = y[j] - stepintervaly;
        if ((j == 1) || (j == 3))
        {
          z[j] = 0;
        }
        else if ((j == 0) || (j == 2))
        {
          z[j] = constcy * (y[j] - ymin) * (y[j] - ymax);
        }
      }
      else if ((yprev[j] > y[j]) && (y[j] > ymin))
      {
        yprev[j] = y[j];
        y[j] = y[j] - stepintervaly;
        if ((j == 1) || (j == 3))
        {
          z[j] = 0;
        }
        else if ((j == 0) || (j == 2))
        {
          z[j] = constcy * (y[j] - ymin) * (y[j] - ymax);
        }
      }
      else if (y[j] <= ymin)
      {
        yprev[j] = y[j];
        y[j] = y[j] + stepintervaly;
        if ((j == 1) || (j == 3))
        {
          z[j] = constcy * (y[j] - ymin) * (y[j] - ymax);
        }
        else if ((j == 0) || (j == 2))
        {
          z[j] = 0;
        }
      }
      else
      {
        y[j] = yprev[j];
        z[j] = 0;
      }
    IK();
    setpos();

    checkstopcondition();
    }
  }
}

void turnright()
{
  y[0] = ymax;
  y[1] = ymax;
  y[2] = ymin;
  y[3] = ymin;

  constcy = zmax / (-0.25 * pow(ymin + ymax, 2) + ymin * ymax);

  while (stopcondition == false)
  {
    for (int j =0; j <= 3; j++)
    {
      if ((yprev[j] <= y[j]) && (y[j] < ymax))
      {
        yprev[j] = y[j];
        y[j] = y[j] + stepintervaly;
        if ((j == 0) || (j == 2))
        {
          z[j] = constcy * (y[j] - ymin) * (y[j] - ymax);
        }
        else if ((j == 1) || (j == 3))
        {
          z[j] = 0;
        }
      }
      else if (y[j] >= ymax)
      {
        yprev[j] = y[j];
        y[j] = y[j] - stepintervaly;
        if ((j == 0) || (j == 2))
        {
          z[j] = 0;
        }
        else if ((j == 1) || (j == 3))
        {
          z[j] = constcy * (y[j] - ymin) * (y[j] - ymax);
        }
      }
      else if ((yprev[j] > y[j]) && (y[j] > ymin))
      {
        yprev[j] = y[j];
        y[j] = y[j] - stepintervaly;
        if ((j == 0) || (j == 2))
        {
          z[j] = 0;
        }
        else if ((j == 1) || (j == 3))
        {
          z[j] = constcy * (y[j] - ymin) * (y[j] - ymax);
        }
      }
      else if (y[j] <= ymin)
      {
        yprev[j] = y[j];
        y[j] = y[j] + stepintervaly;
        if ((j == 0) || (j == 2))
        {
          z[j] = constcy * (y[j] - ymin) * (y[j] - ymax);
        }
        else if ((j == 1) || (j == 3))
        {
          z[j] = 0;
        }
      }
      else
      {
        y[j] = yprev[j];
        z[j] = 0;
      }
    IK();
    setpos();

    checkstopcondition();
    }
  }
}

int pulseWidth(int angle){
  int pulse_wide, analog_value;
  pulse_wide = map(angle, 0, 180, 500, 2500);
  analog_value = int(float(pulse_wide) / 1000000 * Freq *4096);
  return analog_value;
}

void setpos()
{
  for (int i = 0; i < 12; i++)
  {
    pwm.setPWM(i, 0, pulseWidth(servopos[i]));
  }
}

void IK()
{
  float frontheight, rearheight, frontc, rearc, frontphi, rearphi;
  float leftgroundtoshoulderheight, rightgroundtoshoulderheight, leftstepwidthcorrectionangle, rightstepwidthcorrectionangle, correctedleftgroundtoshoulderheight, correctedrightgroundtoshoulderheight;
  float frontleftheight, frontrightheight, rearleftheight, rearrightheight, frontleftc, frontrightc, rearleftc, rearrightc, frontleftphi, frontrightphi, rearleftphi, rearrightphi;
  float yx[4], thetax[4], d[4], A[4], B[4], C[4];
  float frontrightsidestepangle, frontleftsidestepangle, rearrightsidestepangle, rearleftsidestepangle, frontleftpitchheight, frontrightpitchheight, rearleftpitchheight, rearrightpitchheight;
  float deltax, deltay;
  float zyaw[4], yyaw[4], xyaw[4];

  pitchangle = pitchTarget + pitchCorrection;
  rollangle = rollTarget + rollCorrection;

//***Yaw***
  deltay = (bodywidth / 2) * sin(yawangle * 3.142 / 180);
  deltax = (bodywidth / 2) * (1 - cos(yawangle * 3.142 / 180));
  yyaw[0] = y[0] - deltay;
  xyaw[0] = x[0] + deltax;
  yyaw[1] = y[1] - deltay;
  xyaw[1] = x[1] - deltax;
  yyaw[2] = y[2] + deltay;
  xyaw[2] = x[2] + deltax;
  yyaw[3] = y[3] + deltay;
  xyaw[3] = x[3] - deltax;
  
//  zyaw[0] = z[0] + yawangle / 10;
//  zyaw[1] = z[1] - yawangle / 10;
//  zyaw[2] = z[2] - yawangle / 10;
//  zyaw[3] = z[3] + yawangle / 10;

  zyaw[1] = z[1] - pow(y[1] / 100, 3); 

//***Roll***Roll Corrected Ground-to-shoulder Heights
  leftgroundtoshoulderheight = bodyheight + (bodywidth / 2) * sin(rollangle * 3.142 / 180);
  rightgroundtoshoulderheight = bodyheight - (bodywidth / 2) * sin(rollangle * 3.142 / 180);

//***Roll***Correction Angle to Ensure Foot Remains Planted in Same Location
  leftstepwidthcorrectionangle = atan((stepwidth / (2 * leftgroundtoshoulderheight)) * (1 - cos(rollangle * 3.142 / 180)));
  rightstepwidthcorrectionangle = atan((stepwidth / (2 * rightgroundtoshoulderheight)) * (1 - cos(rollangle * 3.142 / 180)));

//***Roll***Corrected Ground-to-shoulder Height Using Correction Angle
  correctedleftgroundtoshoulderheight = leftgroundtoshoulderheight / (cos(leftstepwidthcorrectionangle));
  correctedrightgroundtoshoulderheight = rightgroundtoshoulderheight / (cos(rightstepwidthcorrectionangle));

//***Pitch***Ground-to-shoulder Heights taking Pitch Angle into account
  frontleftpitchheight = correctedleftgroundtoshoulderheight + bodyradius * sin(pitchangle * 3.142 / 180);
  frontrightpitchheight = correctedrightgroundtoshoulderheight + bodyradius * sin(pitchangle * 3.142 / 180);
  rearleftpitchheight = correctedleftgroundtoshoulderheight - bodyradius * sin(pitchangle * 3.142 / 180);
  rearrightpitchheight = correctedrightgroundtoshoulderheight - bodyradius * sin(pitchangle * 3.142 / 180);

//***Pitch***Corrected x Location to Ensure Foot Remains Planted in Same Location
  yx[0] = xyaw[0] - bodyradius * (1 - cos(pitchangle * 3.142 / 180));
  yx[1] = xyaw[1] - bodyradius * (1 - cos(pitchangle * 3.142 / 180));
  yx[2] = xyaw[2] + bodyradius * (1 - cos(pitchangle * 3.142 / 180));
  yx[3] = xyaw[3] + bodyradius * (1 - cos(pitchangle * 3.142 / 180));

//***EXP***Adding Side-step
  frontrightsidestepangle = atan(yyaw[0] / frontrightpitchheight);
  frontleftsidestepangle = atan(yyaw[1] / frontleftpitchheight);
  rearrightsidestepangle = atan(yyaw[2] / rearrightpitchheight);
  rearleftsidestepangle = atan(yyaw[3] / rearleftpitchheight);

  frontrightheight = frontrightpitchheight / cos(frontrightsidestepangle);
  frontleftheight = frontleftpitchheight / cos(frontleftsidestepangle);
  rearrightheight = rearrightpitchheight / cos(rearrightsidestepangle);
  rearleftheight = rearleftpitchheight / cos(rearleftsidestepangle);
  
//Calculation of Leg Length  
  frontleftc = sqrt(pow(frontleftheight, 2) - pow(shoulderoffset, 2));
  frontrightc = sqrt(pow(frontrightheight, 2) - pow(shoulderoffset, 2));
  rearleftc = sqrt(pow(rearleftheight, 2) - pow(shoulderoffset, 2));
  rearrightc = sqrt(pow(rearrightheight, 2) - pow(shoulderoffset, 2));
  
//Correction Angle to Ensure Foot Remains Vertically Under Shoulder
  frontleftphi = asin(shoulderoffset / frontleftheight);
  frontrightphi = asin(shoulderoffset / frontrightheight);
  rearleftphi = asin(shoulderoffset / rearleftheight);
  rearrightphi = asin(shoulderoffset / rearrightheight);

  thetax[0] = atan(yx[0] / frontrightc);
  thetax[1] = atan(yx[1] / frontleftc);
  thetax[2] = atan(yx[2] / rearrightc);
  thetax[3] = atan(yx[3] / rearleftc);

  d[0] = (frontrightc - z[0]) / cos(thetax[0]);
  d[1] = (frontleftc - z[1]) / cos(thetax[1]);
  d[2] = (rearrightc - z[2]) / cos(thetax[2]);
  d[3] = (rearleftc - z[3]) / cos(thetax[3]);

  A[0] = acos((pow(b, 2) + pow(d[0], 2) - pow(a, 2)) / (2 * b * d[0]));
  B[0] = acos((pow(a, 2) + pow(d[0], 2) - pow(b, 2)) / (2 * a * d[0]));
  C[0] = acos((pow(a, 2) + pow(b, 2) - pow(d[0], 2)) / (2 * a * b));

  A[1] = acos((pow(b, 2) + pow(d[1], 2) - pow(a, 2)) / (2 * b * d[1]));
  B[1] = acos((pow(a, 2) + pow(d[1], 2) - pow(b, 2)) / (2 * a * d[1]));
  C[1] = acos((pow(a, 2) + pow(b, 2) - pow(d[1], 2)) / (2 * a * b));
  
  A[2] = acos((pow(b, 2) + pow(d[2], 2) - pow(a, 2)) / (2 * b * d[2]));
  B[2] = acos((pow(a, 2) + pow(d[2], 2) - pow(b, 2)) / (2 * a * d[2]));
  C[2] = acos((pow(a, 2) + pow(b, 2) - pow(d[2], 2)) / (2 * a * b));

  A[3] = acos((pow(b, 2) + pow(d[3], 2) - pow(a, 2)) / (2 * b * d[3]));
  B[3] = acos((pow(a, 2) + pow(d[3], 2) - pow(b, 2)) / (2 * a * d[3]));
  C[3] = acos((pow(a, 2) + pow(b, 2) - pow(d[3], 2)) / (2 * a * b));

  servopos[3] = (intservopos[0] + (180 - (C[0] * 180 / 3.142)));
  servopos[4] = (intservopos[1] - ((B[0] * 180 / 3.142) - (thetax[0] * 180 / 3.142)));
  servopos[5] = (intservopos[2] + (frontrightphi * 180 / 3.142) - rollangle - (rightstepwidthcorrectionangle * 180 / 3.142) - (frontrightsidestepangle * 180 / 3.142));

  servopos[0] = (intservopos[3] - (180 - (C[1] * 180 / 3.142)));
  servopos[1] = (intservopos[4] + (B[1] * 180 / 3.142) - (thetax[1] * 180 / 3.142));
  servopos[2] = (intservopos[5] - (frontleftphi * 180 / 3.142) - rollangle - (leftstepwidthcorrectionangle * 180 / 3.142) - (frontleftsidestepangle * 180 / 3.142));

  servopos[9] = (intservopos[6] + (180 - (C[2] * 180 / 3.142)));
  servopos[10] = (intservopos[7] - ((B[2] * 180 / 3.142) - (thetax[2] * 180 / 3.142)));
  servopos[11] = (intservopos[8] - (rearrightphi * 180 / 3.142) + rollangle + (rightstepwidthcorrectionangle * 180 / 3.142) + (rearrightsidestepangle * 180 / 3.142));

  servopos[6] = (intservopos[9] - (180 - (C[3] * 180 / 3.142)));
  servopos[7] = (intservopos[10] + (B[3] * 180 / 3.142) - (thetax[3] * 180 / 3.142));
  servopos[8] = (intservopos[11] + (rearleftphi * 180 / 3.142) + rollangle + (leftstepwidthcorrectionangle * 180 / 3.142) + (rearleftsidestepangle * 180 / 3.142));
}

void MPUsetup()
{
  bodyheight = 82;                    //mm
  pitchTarget = 0;                      //degrees
  pitchangle = pitchTarget + pitchCorrection;
  rollTarget = 0;
  rollangle = rollTarget + rollCorrection;
  stepwidth = 102;
  x[0] = -35;
  x[1] = -35;
  x[2] = -35;
  x[3] = -35;    //mm
  y[0] = 20;
  y[1] = -20;
  y[2] = 20;
  y[3] = -20;   //mm
  z[0] = 0;
  z[1] = 0;
  z[2] = 0;
  z[3] = 0;       //mm
  zmax = 25;                           //mm
  xmax = 15;                           //mm
  xmin = -25;                          //mm
  stepinterval = (xmax - xmin) / 10;    //mm
  xprev[0] = xprev[1] = xprev[2] = xprev[3] = 0;
  IK();
  setpos(); 
  
  delay(5000);
  
  Wire.begin();

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status != 0){}

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets();
  Serial.println("DONE\n");
  delay(2000);
}

void selfStabilize()
{
  if((millis() - milliNew) > 10)
  {
    mpu.update();
    pitchActual = -1 * mpu.getAngleX();
    Serial.println(pitchActual);
    rollActual = -1 * mpu.getAngleY();
  
    milliOld = milliNew;
    milliNew = millis();
    dt = milliNew - milliOld;

    pitchErrorOld = pitchError;
    pitchError = pitchTarget - pitchActual;
    Serial.println(pitchError);
    pitchErrorChange = pitchError - pitchErrorChange;
    pitchErrorSlope = pitchErrorChange / dt;
    pitchErrorArea = pitchErrorArea + pitchErrorArea * dt;

    rollErrorOld = rollError;
    rollError = rollTarget - rollActual;
    rollErrorChange = rollError - rollErrorChange;
    rollErrorSlope = rollErrorChange / dt;
    rollErrorArea = rollErrorArea + rollErrorArea * dt;

    pitchCorrection = pitchCorrection + kp * pitchError + kd * pitchErrorSlope + ki * pitchErrorArea;
    if (pitchCorrection >= 15)
    {
      pitchCorrection = 15;
    }
    else if (pitchCorrection <= -15)
    {
      pitchCorrection = -15;
    }

    rollCorrection = rollCorrection + kp * rollError + kd * rollErrorSlope + ki * rollErrorArea;
    if (rollCorrection >= 15)
    {
      rollCorrection = 15;
    }
    else if (rollCorrection <= -15)
    {
      rollCorrection = -15;
    }
    pitchangle = pitchTarget + pitchCorrection;
    rollangle = rollTarget + rollCorrection;

    IK();
    setpos();
  }
}

void remoteControl()
{
  char command = Serial.read();
  if (command == 'w')
  {
    pitchTarget = -8; 
    IK();
    setpos();
    walkforward();
  }

  else if (command == 'c')
  {
    pitchTarget = -8; 
    IK();
    setpos();
    walkForwardStabilize();
  }
  
  else if (command == 's')
  {
    pitchTarget = -8;
    IK();
    setpos();
    walkbackward();
  }

  else if (command == 'a')
  {
//    walkleft();
  }

  else if (command == 'd')
  {
//    walkright();
  }

  else if (command == 'q')
  {
    pitchTarget = -8;
    rollTarget = -5; 
    IK();
    setpos();
    turnleft();
  }

  else if (command == 'e')
  {
    pitchTarget = -8;
    rollTarget = 5;
    IK();
    setpos();
    turnright();
  }

  else if (command == 'z')
  {
    resetstance();
  }

  else if (command == 'i')
  {
    pitchforward();
  }

  else if (command == 'k')
  {
    pitchbackward();
  }

  else if (command == 'j')
  {
    rollleft();
  }

  else if (command == 'l')
  {
    rollright();
  }

  else if (command == 'u')
  {
    yawleft();
  }

  else if (command == 'o')
  {
    yawright();
  }

  else if (command == 'm')
  {
    MPUsetup();
  }
  else if (command == 'n')
  {
    selfStabilize();
  }
  else if ((command == '1')||(command == '2')||(command == '3')||(command == '4'))
  {
    selectLeg(command);
  }
  else if (command == 'r')
  {
    increaseX();
  }
  else if (command == 'f')
  {
    decreaseX();
  }
  else if (command == 't')
  {
    increaseY();
  }
  else if (command == 'g')
  {
    decreaseY();
  }
  else if (command == 'y')
  {
    increaseZ();
  }
  else if (command == 'h')
  {
    decreaseZ();
  }
  else if (command == 'x')
  {
    checkstopcondition();
  }
}

int selectLeg(int command)
{
  if (command == '1')
  {
    legParameter = 0;
  }
  else if (command == '2')
  {
    legParameter = 1;
  }
  else if (command == '3')
  {
    legParameter = 2;
  }
  else if (command == '4')
  {
    legParameter = 3;
  }
}

void resetstance()
{
  bodyheight = 140;                    //mm
  pitchTarget = 0;                      //degrees
  pitchangle = pitchTarget + pitchCorrection;
  rollTarget = 0;
  rollangle = rollTarget + rollCorrection;
  yawangle = 0;
  stepwidth = 102;
  x[0] = -35;
  x[1] = -35;
  x[2] = -35;
  x[3] = -35;    //mm
  y[0] = 15;
  y[1] = -15;
  y[2] = 15;
  y[3] = -15;   //mm
  z[0] = 0;
  z[1] = 0;
  z[2] = 0;
  z[3] = 0;       //mm
  zmax = 15;                           //mm
  xmax = 0;                           //mm
  xmin = -45;                          //mm

  IK();
  setpos();

  legParameter = 0;

  stopcondition = false;
}

void checkstopcondition()
{
  char check = Serial.read();
  if ((check == 'x') && (stopcondition == false))
  {
    stopcondition = true;
    Serial.println(stopcondition);
  }
  else if ((check == 'x') && (stopcondition == true)) 
  {
    stopcondition = false;
    Serial.println(stopcondition);
  }
}

void pitchforward()
{
  pitchTarget = pitchTarget - 5;
  Serial.println(pitchTarget);
  IK();
  setpos();
}

void pitchbackward()
{
  pitchTarget = pitchTarget + 5;
  Serial.println(pitchTarget);
  IK();
  setpos();
}

void rollleft()
{
  rollTarget = rollTarget - 5;
  Serial.println(rollTarget);
  IK();
  setpos();
}

void rollright()
{
  rollTarget = rollTarget + 5;
  Serial.println(rollTarget);
  IK();
  setpos();
}

void yawleft()
{
  yawangle = yawangle - 5;
  Serial.println(yawangle);
  IK();
  setpos();
}

void yawright()
{
  yawangle = yawangle + 5;
  Serial.println(yawangle);
  IK();
  setpos();
}

void increaseX()
{
  x[legParameter] = x[legParameter] + 10;
  Serial.println(legParameter);
  Serial.println(x[legParameter]);
  IK();
  setpos();
}

void decreaseX()
{
  x[legParameter] = x[legParameter] - 10;
  Serial.println(legParameter);
  Serial.println(x[legParameter]);
  IK();
  setpos();
}

void increaseY()
{
  y[legParameter] = y[legParameter] + 5;
  Serial.println(legParameter);
  Serial.println(y[legParameter]);
  IK();
  setpos();
}

void decreaseY()
{
  y[legParameter] = y[legParameter] - 5;
  Serial.println(legParameter);
  Serial.println(y[legParameter]);
  IK();
  setpos();
}

void increaseZ()
{
  z[legParameter] = z[legParameter] + 5;
  Serial.println(legParameter);
  Serial.println(z[legParameter]);
  IK();
  setpos();
}

void decreaseZ()
{
  z[legParameter] = z[legParameter] - 5;
  Serial.println(legParameter);
  Serial.println(z[legParameter]);
  IK();
  setpos();
}