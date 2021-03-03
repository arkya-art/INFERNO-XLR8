
 // Prints "hello world!"

#include <ros.h>
#include <std_msgs/String.h>//arduino node publishes to this data type (arduino sends string type message to ros)


ros::NodeHandle  nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void setup()
{
  pinMode(13, OUTPUT);//led pin
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(500);
}
