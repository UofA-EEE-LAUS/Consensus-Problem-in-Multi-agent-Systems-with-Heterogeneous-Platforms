#include <AccelStepper.h>
#include "GY521.h"
//#define WIINDOW_SIZE 50
GY521 sensor(0x68);

uint32_t counter = 0;

// Define some steppers and the pins the will use
AccelStepper stepper1(AccelStepper::DRIVER, 5,4); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper stepper2(AccelStepper::DRIVER, 10,9);
AccelStepper stepper3(AccelStepper::DRIVER, 7,6);

//const int ms3Pin   = 11;   // Ms3
//const int ms2Pin   = 12;   // Ms2
//const int ms1Pin   = 13;   // Ms1

float offset = PI/18;//((5/6)*PI);//((5/6)*PI);//
float rover_radius = 13;
float wheel_radius = 5;
float wheel_perimeter = wheel_radius * 2 * PI;
float cur_x = 0;
float cur_y = 0;
float cur_angle = offset;
float w = 0;
float v_n = 0;
float v_x = 0;
float v_y = 0;
float vxconst = 0;
float vyconst = 0;
float wconst = 0;
float v0 = 0;
float v1 = 0;
float v2 = 0;
float pos_0 = 0;
float pos_1 = 0;
float pos_2 = 0;
float time_limit = 0;
float time_accel = 0;
double time_pre = micros();
double timer_start = millis();
float total_elapsed = 0;
double time_elapsed = 0;
double time_cur =0;
float e_x = 0;
float e_y = 0;
float e_angle = 0;
float des_x = 0;
float des_y = 0;
float des_angle = 0;
float pre_x = 0;
float pre_y = 0;
float pre_angle = 0;
float kp = 0.2;
float ki = 0;
float kd = 0;
float e_lim = 0;
float pre_v_x = 0;
float pre_v_y = 0;
float pre_w = 0;
float v_max = 4;
float v_xtemp, v_ytemp, w_temp;
float yaw = 0;
float yaw_pre = 0;
float gyz_HPF_cur = 0;
float gyz_HPF_pre = 0;
float alpha = 0.7;
float x = sensor.getGyroX();
float y = sensor.getGyroY();
float Pm1 = 0;
float Pm2 = 0;
float Pm3 = 0;
float Xm1 = 0;
float Xm2 = 0;
float Xm3 = 0;
float Ym1 = 0;
float Ym2 = 0;
float Ym3 = 0;
long duration, distance;

void setup()

{  
    stepper1.setMaxSpeed(5000);
    stepper2.setMaxSpeed(5000);
    stepper3.setMaxSpeed(5000);
  
    stepper1.setSpeed(0);
    stepper2.setSpeed(0);
    stepper3.setSpeed(0); 

    Serial.begin(9600);

    //pinMode(ms3Pin,OUTPUT);  
    //pinMode(ms2Pin,OUTPUT);  
//    pinMode(ms1Pin,OUTPUT); 
//
//    digitalWrite(ms1Pin, LOW);
   // digitalWrite(ms2Pin, HIGH); 
    //digitalWrite(ms3Pin, HIGH); 

 
  Serial.begin(115200);
  Serial.println(__FILE__);

  Wire.begin();

  delay(100);
  while (sensor.wakeup() == false)
  {
    Serial.print(millis());
    Serial.println("\tCould not connect to GY521");
    delay(1000);
  }
  sensor.setAccelSensitivity(2);  // 8g
  sensor.setGyroSensitivity(1);   // 500 degrees/s

  sensor.setThrottle();
  Serial.println("start...");

  //calibration
  sensor.read();
  for (int i = 0; i < 10; i++)
  {
    sensor.read();
  }
  sensor.gze = -sensor.getGyroZ();
  //sensor.getGyroZ();
  //yaw = 0
  // set callibration values from calibration sketch.
//  sensor.axe = -1.044;
//  sensor.aye = 0.650;
//  sensor.aze = -19.550;
//  sensor.gxe = 1.206;
//  sensor.gye = -1.292;
  sensor.gze = -2.4;



  //sensor
  delay(1000);
  time_pre = micros();

}


void loop()
{   

   delay(0.0010);
    des_x = 50;  
    des_y = 50;      
    des_angle = PI/2; // PI/2;

   

    sensor.read();

    yaw = sensor.getYaw()/180*PI;


    cur_x = cur_x + (pre_v_x+v_x)/2 * time_elapsed;
    cur_y = cur_y + (pre_v_y+v_y)/2 * time_elapsed;
    //cur_angle = cur_angle + (pre_w+w)/2 * time_elapsed + offset;
    cur_angle = yaw  + offset;



    if (cur_angle > PI) {
        cur_angle = cur_angle - 2 * PI;
    }

    time_cur = micros();
    
    time_elapsed = (time_cur - time_pre)/1000000;
    //Serial.println(time_elapsed);
    //Serial.println(time);
    time_pre = time_cur;
    
    

    Serial.print(cur_x);
    Serial.print(";");
    Serial.println(cur_angle);
    //Serial.println(";");
    //cur_angle = cur_angle + (pre_w+w)/2 * time_elapsed + offset;
    //cur_angle = 0;

    e_x = des_x - cur_x;
    e_y = des_y - cur_y;
    e_angle = des_angle - cur_angle;



    //v_x = (pre_x - cur_x) /time_elapsed;
    //v_y = (pre_y - cur_y) /time_elapsed;
    //w   = (pre_angle - cur_angle) /time_elapsed;
    pre_x     = cur_x;
    pre_y     = cur_y;
    pre_angle = cur_angle;
    

    v_x = (kp*e_x +  kd*v_x);
    v_y = kp*e_y +  kd*v_y;
    w   = -kp*e_angle +  kd*w;

    v0 = -sin(cur_angle)        * v_x + cos(cur_angle)        * v_y + rover_radius * w;
    v1 = -sin(PI/3 - cur_angle) * v_x - cos(PI/3 - cur_angle) * v_y + rover_radius * w;
    v2 =  sin(PI/3 + cur_angle) * v_x - cos(PI/3 + cur_angle) * v_y + rover_radius * w;

  
    v0 = v0;
    v1 = v1;
    v2 = v2;


    

   e_lim = 0.01;
    if (abs(e_x) <= e_lim && abs(e_y) <= e_lim && abs(e_angle) <= e_lim){
        stepper1.stop();
        stepper2.stop();
        stepper3.stop(); 
    

     } 
      else{
        stepper1.setSpeed(((v0)/(2*PI*5))*700*5.18);//*3200*64
        stepper2.setSpeed(((v1)/(2*PI*5))*700*5.18);
        stepper3.setSpeed(((v2)/(2*PI*5))*700*5.18); 
      
        stepper1.runSpeed();
        stepper2.runSpeed();
        stepper3.runSpeed();
      }  
    

}
