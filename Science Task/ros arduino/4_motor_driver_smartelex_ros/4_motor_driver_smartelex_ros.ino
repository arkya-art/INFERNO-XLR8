//2 subscriber and 1 publisher

#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;
unsigned int motor_freq = 0;
unsigned int wheel_radius = 0;
//setting up pins
const int MOTOR_DIRECTION_PIN = A0;
const int MOTOR_PWM_PIN = 9;

// a publisher
std_msgs::Float32 rect_speed;
ros::Publisher pub_rect_speed("rect_speed", &rect_speed);

// some function declaretions

void SetWheelRadius(const std_msgs::UInt8 &rad) 
{
  wheel_radius = rad.data;
}

void SetMotorFreq(const std_msgs::UInt8 &freq_msg) 
{
  // freq_msg. its unit is radians/sec
  motor_freq = freq_msg.data;
  motor_freq = motor_freq > 255 ? 255 : motor_freq;
  rect_speed.data = motor_freq * wheel_radius;
  pub_rect_speed.publish(&rect_speed);
}


// setting subscribers details
ros::Subscriber<std_msgs::UInt8> sub_radius("radius", &SetWheelRadius);
ros::Subscriber<std_msgs::UInt8> sub_freq("freq", &SetMotorFreq);

void setup()
{
  //for motor driver (motor)
  //to control direction(CW/ACW)
  pinMode(MOTOR_DIRECTION_PIN, OUTPUT);
  //to control speed
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  // initialize this node
  nh.initNode();
  // advertise 
  nh.advertise(pub_rect_speed);
  // subscribe
  nh.subscribe(sub_radius);
  nh.subscribe(sub_freq);
}

void loop()
{
  cloclwise_run_motor_variable_speed();
  nh.spinOnce();
  delay(1);
}

void cloclwise_run_motor_variable_speed()
{
  analogWrite(MOTOR_PWM_PIN, motor_freq); //PWM CONCEPT,  SPEED BETWEEN 0 TO 255
  digitalWrite(MOTOR_DIRECTION_PIN, HIGH);
}
