#include <AccelStepper.h>
#include "GY521.h"
GY521 sensor(0x68);
AccelStepper stepper1(AccelStepper::DRIVER, 5,4); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper stepper2(AccelStepper::DRIVER, 10,9);
AccelStepper stepper3(AccelStepper::DRIVER, 7,6);


//const int ms3Pin   = 11;   // Ms3
//const int ms2Pin   = 12;   // Ms2
//const int ms1Pin   = 13;   // Ms1

float offset = 0; //((5/6)*PI);//((5/6)*PI);//
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
//float cerrorx, cerrory, cerrora;
int i;
int dat[32]={0};

unsigned long a,q;
long distance();


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
//    digitalWrite(ms1Pin, HIGH);
    //digitalWrite(ms2Pin, HIGH); 
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
  Serial1.begin(115200);

  delay(1000);
  time_pre = micros();



}

void loop()
{   
    delay(0.0010);
   // Serial.print(distance());
   // Serial.println("  completed");
    des_x = 50;  
    des_y = 0;      
    des_angle = 0; // PI/2;  
//    if(Serial1.available()){
//       q=Serial1.read();
//       Serial.println(q);
//    }
//  

    
   
  // Serial.println(time_elapsed);
    time_cur = micros();
    
    time_elapsed = (time_cur - time_pre)/1000000;

    time_pre = time_cur;
    sensor.read();
    yaw = sensor.getYaw()/180*PI;

    if (distance()<=200 ){
      cur_x = cur_x;
      cur_y = cur_y;
      cur_angle = cur_angle;
    
      v0 = 0;
      v1 = 0;
      v2 = 0;
  

      
    }else{
    cur_x = cur_x + (pre_v_x+v_x)/2 * time_elapsed;
    cur_y = cur_y + (pre_v_y+v_y)/2 * time_elapsed;
    cur_angle = yaw  + offset;
    

    if (cur_angle > PI) {
        cur_angle = cur_angle - 2 * PI;
    }

    
    e_x = des_x - cur_x;
    e_y = des_y - cur_y;
    e_angle = des_angle - cur_angle;


    v_x = kp*e_x +  kd*v_x;
    v_y = kp*e_y +  kd*v_y;
    w =   -kp*e_angle +  kd*w;
//    if(distance <= 10){
//      v_x = 0;
//      v_y = 0;
//    }

    pre_v_x = v_x;
    pre_v_y = v_y;
    pre_w = w;

    
    v0 = -sin(cur_angle)        * v_x + cos(cur_angle)        * v_y + rover_radius * w;
    v1 = -sin(PI/3 - cur_angle) * v_x - cos(PI/3 - cur_angle) * v_y + rover_radius * w;
    v2 =  sin(PI/3 + cur_angle) * v_x - cos(PI/3 + cur_angle) * v_y + rover_radius * w;

    v0 = v0;
    v1 = v1;
    v2 = v2;
    }
    Serial.println(cur_x);
    
        stepper1.setSpeed(((v0)/(2*PI*5.2))*1600*5.18);//*700
        stepper2.setSpeed(((v1)/(2*PI*5.2))*1600*5.18);//*1400
        stepper3.setSpeed(((v2)/(2*PI*5.2))*1600*5.18); 
        
        stepper1.runSpeed();
        stepper2.runSpeed();
        stepper3.runSpeed();
    
    e_lim = 0.01;
    if (abs(e_x) <= e_lim && abs(e_y) <= e_lim && abs(e_angle) <= e_lim){
        stepper1.setSpeed(0);
        stepper2.setSpeed(0);
        stepper3.setSpeed(0); 
    
        stepper1.runSpeed();
        stepper2.runSpeed();
        stepper3.runSpeed();

    }

    

}
long distance(){
      if(Serial1.available())
    {
      if(millis()-a>500)
      {
        a=millis();
        for(i=0;i<32;i++)
        {
          dat[i]=Serial1.read();
        }
        for(i=0;i<16;i++)
        {
          if(dat[i]==0x57&&dat[i+1]==0&&dat[i+2]==0xff&&dat[i+3]==0)
          {
            if(dat[i+12]+dat[i+13]*255==0)
            {
              Serial.println("Out of range!");
            }
            else
            { 

              q=dat[i+8]+dat[i+9]*255;
      
              
            }
         
          }
             break; 
        }
      }
    }
    return q;
}
