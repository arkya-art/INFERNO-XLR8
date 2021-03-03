//one subscriber only

#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;
unsigned int motor_freq = 0;
//setting up pins
const int MOTOR_DIRECTION_PIN = A0;
const int MOTOR_PWM_PIN = 9;



// some function declaretions
void SetMotorFreq(const std_msgs::UInt8 &freq_msg)
{
  // freq_msg. its unit is radians/sec
  motor_freq = freq_msg.data;
  motor_freq = motor_freq > 255 ? 255 : motor_freq;
}

// setting subscribers details
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
  
  // subscribe
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
