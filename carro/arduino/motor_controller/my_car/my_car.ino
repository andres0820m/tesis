#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <DynamixelWorkbench.h>

#if defined(__OPENCM904__)
  #define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif    

#define BAUDRATE  1000000
DynamixelWorkbench dxl_wb;
#define M1 3
#define M2 4
#define encoder_pulse   13
#define gear_ratio      120
#define wheel_diameter  0.069   //m
#define wheel_width     0.027   //m
#define track_width     0.276   //m
#define MAX_RPM         59
#define pi              3.1415926
#define two_pi          6.2831853
#define LOOPTIME        100   // 
#define SMOOTH      10

ros::NodeHandle nh;
double rpm_req1 = 0;
double rpm_req2 = 0;
double rpm_ac1 = 0;
double rpm_ac2 = 0;
unsigned long lastMilli = 0;
unsigned long lastMilliPub = 0;
// funcion para controlar lo que envia ROS 
void handle_cmd( const geometry_msgs::Twist& cmd_msg) {
  double x = cmd_msg.linear.x;
  double z = cmd_msg.angular.z;
  if (z == 0) {     // no hay giro va hacia adelante 
    // convert m/s to rpm
    rpm_req1 = x*60/(pi*wheel_diameter);
    rpm_req2 = rpm_req1;
  }
  else if (x == 0) {
    // convert rad/s to rpm
    rpm_req2 = z*track_width*60/(wheel_diameter*pi*2);
    rpm_req1 = -rpm_req2;
  }
  else {
    rpm_req1 = x*60/(pi*wheel_diameter)-z*track_width*60/(wheel_diameter*pi*2);
    rpm_req2 = x*60/(pi*wheel_diameter)+z*track_width*60/(wheel_diameter*pi*2);
  }
}

double value_to_rpm(int value)
{
  double Final=(value/59)*1023;
  return Final;
  
  }
//////////////// parte de ROS  se subcribe a cmd publica geometria rpm
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", handle_cmd);
geometry_msgs::Vector3Stamped rpm_msg;
ros::Publisher rpm_pub("rpm", &rpm_msg);
ros::Time current_time;
ros::Time last_time;
////////////////
void setup() {
nh.initNode();
nh.getHardware()->setBaud(57600);
nh.subscribe(sub);
nh.advertise(rpm_pub);
dxl_wb.begin(DEVICE_NAME, BAUDRATE);
dxl_wb.ping(M1);
dxl_wb.ping(M2);
dxl_wb.wheelMode(M1);
dxl_wb.wheelMode(M2);

}

void loop() {
  nh.spinOnce();
  unsigned long time = millis();
  if(time-lastMilli>= LOOPTIME)   {      // enter tmed loop
    rpm_ac1=value_to_rpm(rpm_req1);
    rpm_ac2=value_to_rpm(rpm_req2);
    dxl_wb.goalSpeed(M1, rpm_ac1);
    dxl_wb.goalSpeed(M2, rpm_ac2);
    
    
    publishRPM(time-lastMilli);
    lastMilli = time;
  }
  if(time-lastMilliPub >= LOOPTIME) {
  //  publishRPM(time-lastMilliPub);
    lastMilliPub = time;
  }

}

void publishRPM(unsigned long time) {
  rpm_msg.header.stamp = nh.now();
  rpm_msg.vector.x = rpm_req1;
  rpm_msg.vector.y = rpm_req2;
  rpm_msg.vector.z = double(time)/1000;
  rpm_pub.publish(&rpm_msg);
  nh.spinOnce();
}
