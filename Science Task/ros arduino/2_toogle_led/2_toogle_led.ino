
 // rosserial PubSub Example
 // Prints "hello world!" and toggles led

#include <ros.h>
#include <std_msgs/Empty.h>//arduino node subscribes to this data type (arduino receives empty type message from ros)

ros::NodeHandle  nh;


void messageCb( const std_msgs::Empty& toggle_msg)//callback function for led
{
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", messageCb );

void setup()
{
  pinMode(13, OUTPUT);//led pin
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(500);
}
