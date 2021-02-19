#include <AccelStepper.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include "GY521.h"
// Define some steppers and the pins the will use
GY521 sensor(0x68);

uint32_t counter = 0;

AccelStepper stepper1(AccelStepper::DRIVER, 5,4); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper stepper2(AccelStepper::DRIVER, 10,9);
AccelStepper stepper3(AccelStepper::DRIVER, 7,6);

//const int ms3Pin   = 11;   // Ms3
//const int ms2Pin   = 12;   // Ms2
//const int ms1Pin   = 13;   // Ms1

float offset = 0;//((5/6)*PI);//((5/6)*PI);//
float rover_radius = 13;
float wheel_radius = 5;
float wheel_perimeter = wheel_radius * 2 * PI;
float cur_x = 0;
float cur_y = 0;
float cur_angle = 0;
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
float cerrorx, cerrory, cerrora;
int status = WL_IDLE_STATUS;
float yaw = 0;
float yaw_pre = 0;
float gyz_HPF_cur = 0;
float gyz_HPF_pre = 0;
float alpha = 0.7;
float x = sensor.getGyroX();
float y = sensor.getGyroY();

///////please enter your sensitive data in the Secret tab/arduino_secrets.h

char ssid[] = "EEELAB";        // your network SSID (name)

char pass[] = "@adelaide";    // your network password (use for WPA, or use as key for WEP)

int keyIndex = 0;            // your network key Index number (needed only for WEP)

 

unsigned int localPort = 2390;      // local port to listen on

 

char packetBuffer[255]; //buffer to hold incoming packet

char ReplyBuffer[] = "acknowledged";       // a string to send back


WiFiUDP Udp;

void printWiFiStatus() {

  // print the SSID of the network you're attached to:

  Serial.print("SSID: ");

  Serial.println(WiFi.SSID());

 

  // print your WiFi shield's IP address:

  IPAddress ip = WiFi.localIP();

  Serial.print("IP Address: ");

  Serial.println(ip);

 

  // print the received signal strength:

  long rssi = WiFi.RSSI();

  Serial.print("signal strength (RSSI):");

  Serial.print(rssi);

  Serial.println(" dBm");

}

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
    //pinMode(ms1Pin,OUTPUT); 

    //digitalWrite(ms1Pin, HIGH);
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

  if (WiFi.status() == WL_NO_SHIELD) {

    Serial.println("WiFi shield not present");

    // don't continue:

    while (true);

  }

 

  // attempt to connect to WiFi network:

  while ( status != WL_CONNECTED) {

    Serial.print("Attempting to connect to SSID: ");

    Serial.println(ssid);

    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:

    status = WiFi.begin(ssid, pass);

 

    // wait 10 seconds for connection:

    delay(10000);

  }

  Serial.println("Connected to wifi");

  printWiFiStatus();

 

  Serial.println("\nStarting connection to server...");

  // if you get a connection, report back via serial:

  Udp.begin(localPort); 

  delay(1000);
   time_pre = micros(); 
}

void loop()
{
// if there's data available, read a packet

  int packetSize = Udp.parsePacket();

  if (packetSize)

  {

    Serial.print("Received packet of size ");

    Serial.println(packetSize);

    Serial.print("From ");

    IPAddress remoteIp = Udp.remoteIP();

    Serial.print(remoteIp);

    Serial.print(", port ");

    Serial.println(Udp.remotePort());

 

    // read the packet into packetBufffer

    int len = Udp.read(packetBuffer, 255);

    if (len > 0) packetBuffer[len] = 0;

 

    String s = packetBuffer;

 

    int Package[3];

 

    for (int i = 0; i < 2; i++)

    {

      int brk = s.indexOf(",");

      String tString = s.substring(0, brk);

      Package[i] = tString.toInt();

      s.remove(0, brk + 1);

    }

    Package[2] = s.toInt();

 

    // UDP server

    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());

    Udp.write(ReplyBuffer);

    Udp.endPacket();

 

    // Assigning all Packages

    delay(0.0010);

    des_x = Package[0];  //in m

    des_y = Package[1];  //in m    

    des_angle = Package[2]*PI/180;   //in radian

  }       
    delay(0.0010);
    //des_x = 50;  
    //des_y = -50;      
    //des_angle = 0; // PI/2;   

   //Serial.println(time_elapsed);
    time_cur = micros();
    
    time_elapsed = (time_cur - time_pre)/1000000;

    time_pre = time_cur;
    sensor.read();
    yaw = sensor.getYaw()/180*PI;
    cur_x = cur_x + (pre_v_x+v_x)/2 * time_elapsed;
//    Serial.print(cur_y);
//    Serial.print("; ");
//    Serial.println(cur_x);
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

    
   

   e_lim = 0.01;
    if (abs(e_x) <= e_lim && abs(e_y) <= e_lim && abs(e_angle) <= e_lim){
        stepper1.stop();
        stepper2.stop();
        stepper3.stop(); 
    

     } 
      else{
        stepper1.setSpeed(((v0)/(2*PI*5))*1400*5.18);//*3200*64
        stepper2.setSpeed(((v1)/(2*PI*5))*1400*5.18);
        stepper3.setSpeed(((v2)/(2*PI*5))*1400*5.18); 
      
        stepper1.runSpeed();
        stepper2.runSpeed();
        stepper3.runSpeed();
      }  
    

    

    
}
