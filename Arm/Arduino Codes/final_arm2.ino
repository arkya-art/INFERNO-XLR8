


#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif


#include <ros.h>
#include <pcl_msgs/Vertices.h>
#include<geometry_msgs/Point.h>
ros::NodeHandle  nh;
geometry_msgs::Point ref_msg ; 
ros::Publisher chatter("angle",&ref_msg);

int c ;
int i=0;
int actpwm1=12, actpwm2=11, basepwm=9, grippwm1=4, grippwm2=2, wristpwm=7;
int actdir1=13, actdir2=32, basedir=8, gripdir1=26, gripdir2=22, wristdir=6;

void arm_msg( const pcl_msgs::Vertices& cmd_msg)
{

   //for typing
   int t=cmd_msg.vertices[6];
   int base_pwm = cmd_msg.vertices[7];
   int wrist_pwm = cmd_msg.vertices[8];
   if(t!=0){
   
   digitalWrite(actdir1, 1);
   analogWrite(actpwm1, 255);
   digitalWrite(actdir2, 0);
   analogWrite(actpwm2, 255);
   delay(t*1000);
   digitalWrite(actdir1, 0);
   analogWrite(actpwm1, 255);
   digitalWrite(actdir2, 0);
   analogWrite(actpwm2, 255);
   delay(t*1000);
   analogWrite(actpwm1, 0);
   analogWrite(actpwm2, 0);
   }
   
   //for actuator x
    if(cmd_msg.vertices[0]==0)
    {
      digitalWrite(actdir1, 0);
      analogWrite(actpwm1, 0);
    }
    if(cmd_msg.vertices[0]==1)
    {
      digitalWrite(actdir1, 0);
      analogWrite(actpwm1, 255);
    }
    if(cmd_msg.vertices[0]==2)
    {
      digitalWrite(actdir1, 1);
      analogWrite(actpwm1, 255);
    }

    //for actuator y
    if(cmd_msg.vertices[1]==0)
    {
      digitalWrite(actdir2, 0);
      analogWrite(actpwm2, 0);
    }
    if(cmd_msg.vertices[1]==1)
    {
      digitalWrite(actdir2, 0);
      analogWrite(actpwm2, 255);
    }
    if(cmd_msg.vertices[1]==2)
    {
      digitalWrite(actdir2, 1);
      analogWrite(actpwm2, 255);
    }

    //for base
    if(cmd_msg.vertices[2]==0)
    {
      digitalWrite(basedir, 0);
      analogWrite(basepwm, 0);
    }
    if(cmd_msg.vertices[2]==1)
    {
      digitalWrite(basedir, 0);
      analogWrite(basepwm, base_pwm);
    }
    if(cmd_msg.vertices[2]==2)
    {
      digitalWrite(basedir, 1);
      analogWrite(basepwm, base_pwm);
    }

    //for gripper angle 1(rotation)
    if(cmd_msg.vertices[4]==0)
    {
      digitalWrite(gripdir1, 0);
      analogWrite(grippwm1, 0);
    }
    if(cmd_msg.vertices[4]==1)
    {
      digitalWrite(gripdir1, 0);
      analogWrite(grippwm1, 200);
    }
    if(cmd_msg.vertices[4]==2)
    {
      digitalWrite(gripdir1, 1);
      analogWrite(grippwm1, 200);
    }

    //for gripper angle2(jaw)
    if(cmd_msg.vertices[5]==0)
    {
      digitalWrite(gripdir2, 0);
      analogWrite(grippwm2, 0);
    }
    if(cmd_msg.vertices[5]==1)
    {
      digitalWrite(gripdir2, 0);
      analogWrite(grippwm2, 255);
    }
    if(cmd_msg.vertices[5]==2)
    {
      digitalWrite(gripdir2, 1);
      analogWrite(grippwm2, 255);
    }

    //for wrist
    if(cmd_msg.vertices[3]==0)
    {
      digitalWrite(wristdir, 0);
      analogWrite(wristpwm, 0);
    }
    if(cmd_msg.vertices[3]==1)
    {
      digitalWrite(wristdir, 0);
      analogWrite(wristpwm, wrist_pwm);
    }
    if(cmd_msg.vertices[3]==2)
    {
      digitalWrite(wristdir, 1);
      analogWrite(wristpwm, wrist_pwm);
    }
 
}


ros::Subscriber<pcl_msgs::Vertices> sub("move_arm", arm_msg);

void setup(){
  
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
  //motor1.init();
  
  
  
}

void loop(){
   int x = analogRead(A0);
  int y = analogRead(A1);
  x = map(x, 25, 970, 20, 120);
  y = map(y, 25, 970, 20, 120);
  /*if(x>120)
  {
    x=120;
    analogWrite(actpwm1, 0);
  }
  if(x<20)
  {
    x=20;
    analogWrite(actpwm1, 0);
  }
  if(y>120)
  {
    y=120;
    analogWrite(actpwm2, 0);
  }
  if(y<20)
  {
    y=20;
    analogWrite(actpwm2, 0);
  }*/
  ref_msg.x=x;
  ref_msg.y=y;
  ref_msg.z=i;
  chatter.publish(&ref_msg);
  delay(50);
  nh.spinOnce();
  delay(1);
}
