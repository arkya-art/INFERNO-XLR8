
#include <ros.h>
#include <pcl_msgs/Vertices.h>
#include<Servo.h>
Servo panservo;
Servo tiltservo;

ros::NodeHandle  nh;
 


int c ;
int i=0;
int directionPin = 5, pwmPin=6;
int panServopin = 9, tiltServopin = 11;
void pan_msg( const pcl_msgs::Vertices& cmd_msg)
{

   //for typing
   int pan_angle = cmd_msg.vertices[0];
   int tilt_angle = cmd_msg.vertices[1];
   panservo.write(pan_angle);
   tiltservo.write(tilt_angle);
   
   
   //for actuator x
    if(cmd_msg.vertices[2]==0)
    {
      digitalWrite(directionPin, 0);
      analogWrite(pwmPin, 0);
    }
    if(cmd_msg.vertices[2]==1)
    {
      digitalWrite(directionPin, 0);
      analogWrite(pwmPin, 100);
    }
    if(cmd_msg.vertices[2]==2)
    {
      digitalWrite(directionPin, 1);
      analogWrite(pwmPin, 100);
    }

 
}


ros::Subscriber<pcl_msgs::Vertices> sub("servo_pantilt", pan_msg);

void setup(){
  
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  panservo.attach(panServopin);
  tiltservo.attach(tiltServopin);

  //motor1.init();
  
  
  
}

void loop(){
  
  delay(50);
  nh.spinOnce();
  delay(1);
}
