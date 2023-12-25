#include "Motor.h"
#include "ultrasonic.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <PID_v1.h>
#include <geometry_msgs/Vector3Stamped.h>

//rosrun teleop_twist_keyboard teleop_twist_keyboard.py _speed:=0.35 _turn:=2.65
//rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
//rostopic echo /right_pwm

ros::NodeHandle nh;

#define LOOPTIME 10

Motor right(9, 10, 8);
Motor left(5, 6, 7);
Ultrasonic leftSensor(A0, A1);
Ultrasonic rightSensor(A2, A3);
int ledr = 4;
int ledg = 13;


double left_kp = 3, left_ki = 0, left_kd = 0.0;
double right_kp = 3, right_ki = 0, right_kd = 0.0;

double right_input = 0, right_output = 0, right_setpoint = 0;
PID rightPID(&right_input, &right_output, &right_setpoint, right_kp, right_ki, right_kd, DIRECT);

double left_input = 0, left_output = 0, left_setpoint = 0;
PID leftPID(&left_input, &left_output, &left_setpoint, left_kp, left_ki, left_kd, DIRECT);

float demandx = 0;
float demandz = 0;

double demand_speed_left;
double demand_speed_right;

unsigned long currentMillis;
unsigned long prevMillis;

float speed_act_left = 0;
float speed_act_right = 0;

int left_pwm_value = 0; 
int right_pwm_value = 0; 

int a = 100;
unsigned long ledDelayStart = 0;
const unsigned long ledDelayDuration = 1000;

geometry_msgs::Vector3Stamped ticks_msg;
ros::Publisher left_ticks_pub("left_ticks", &ticks_msg);
ros::Publisher right_ticks_pub("right_ticks", &ticks_msg);

geometry_msgs::Vector3Stamped pwm_msg;
ros::Publisher left_pwm_pub("left_pwm", &pwm_msg);
ros::Publisher right_pwm_pub("right_pwm", &pwm_msg);

void cmd_vel_cb(const geometry_msgs::Twist &twist)
{
  demandx = twist.linear.x;
  demandz = twist.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb);
void publishTicksAndPWM(double time)
{
  pwm_msg.header.stamp = nh.now();
  pwm_msg.vector.x = left_pwm_value;
  pwm_msg.vector.y = right_pwm_value;
  pwm_msg.vector.z = time / 1000;
  left_pwm_pub.publish(&pwm_msg);
  right_pwm_pub.publish(&pwm_msg);
}



void setup()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(left_pwm_pub);
  nh.advertise(right_pwm_pub);

  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(1);
  rightPID.SetOutputLimits(-a, a);

  leftPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(1);
  leftPID.SetOutputLimits(-a, a);
  pinMode(ledr, OUTPUT);
  pinMode(ledg, OUTPUT);
  digitalWrite(ledg, LOW);
  digitalWrite(ledr, LOW);
}

void loop()
{
  currentMillis = millis();
  if (currentMillis - prevMillis >= LOOPTIME)
  {
    prevMillis = currentMillis;

    // Đọc giá trị từ cảm biến siêu âm
    int leftDistance = leftSensor.getDistance();
    int rightDistance = rightSensor.getDistance();
    // Kiểm tra và điều khiển bánh dựa trên khoảng cách
    if (leftDistance < 12)
    {
      // Cảm biến bên trái gần vật, điều khiển bánh phải lui
      left_pwm_value = 80;  // Adjust the direction and speed as needed
      right_pwm_value = -110;
    }
    else if (rightDistance < 12)
    {
      // Cảm biến bên phải gần vật, điều khiển bánh trái lui
      left_pwm_value = -110;
      right_pwm_value = 80;  // Adjust the direction and speed as needed
    }
    else
    {
      int k = 0.1075;
      int l = 39.65;
      demand_speed_left = demandx - (demandz * k);
      demand_speed_right = demandx + (demandz * k);

      left_setpoint = demand_speed_left * l;
      right_setpoint = demand_speed_right * l;
      
      // Đưa giá trị PWM vào motor
      leftPID.Compute();
      rightPID.Compute();
      left.rotate(left_output);
      right.rotate(right_output);
    }



    //digitalWrite(ledg, LOW);
    // LED Control
    if (left_pwm_value == 0 && right_pwm_value == 0)
    {
      digitalWrite(ledr, HIGH);
      digitalWrite(ledg, LOW);

      if (millis() - ledDelayStart >= ledDelayDuration)
      {
        ledDelayStart = millis(); // Reset the timer
      }
    }
    else
    {
      digitalWrite(ledg, HIGH);
      digitalWrite(ledr, LOW);

      if (millis() - ledDelayStart >= ledDelayDuration)
      {
        ledDelayStart = millis(); // Reset the timer
      }
    }

    // Gán giá trị PWM cho ROS
    left_pwm_value = map(left_output, -a, a, -255, 255);
    right_pwm_value = map(right_output, -a, a, -255, 255);
    /*int m = 100;
    int h = 80;
    if (left_pwm_value >= 140) {
      left_pwm_value = m;
      right_pwm_value = h;
    }
    if (right_pwm_value >= 140) {
      right_pwm_value = m;
      left_pwm_value = h;
    }
    if (left_pwm_value <= -140) {
      left_pwm_value = -m;
      right_pwm_value = -h;
    }
    if (right_pwm_value <= -140) {
      right_pwm_value = -m;
      left_pwm_value = -h;
    }*/
    // Xuất thông điệp
    publishTicksAndPWM(LOOPTIME);
    nh.spinOnce();
  }
}
