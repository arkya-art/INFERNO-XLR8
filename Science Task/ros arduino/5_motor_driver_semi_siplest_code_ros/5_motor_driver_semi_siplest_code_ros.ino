//1 subscriber and 1 publisher
//subscriber will take value of motor rotating frequency (in radians / sec)
//publisher will print value of motor frequency

#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;
unsigned int motor_freq = 0;
//setting up pins
const int MOTOR_DIRECTION_PIN = A0;
const int MOTOR_PWM_PIN = 9;

// a publisher
std_msgs::Float32 frequency_publisher_value;
ros::Publisher pub_frequency_publisher_value("frequency_publisher_value", &frequency_publisher_value);


void SetMotorFreq(const std_msgs::UInt8 &freq_msg) ////////////type b/////////////
{
  // freq_msg. its unit is radians/sec
  motor_freq = freq_msg.data;
  
  motor_freq = motor_freq > 255 ? 255 : motor_freq;
  frequency_publisher_value.data = motor_freq;
  
  pub_frequency_publisher_value.publish(&frequency_publisher_value);//just for echo
}

// two subscribers
ros::Subscriber<std_msgs::UInt8> sub_freq("freq_subscriber", &SetMotorFreq);////////////type b/////////////


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
  nh.advertise(pub_frequency_publisher_value);
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
