
#include <ros.h>
#include <pcl_msgs/Vertices.h>
#include<geometry_msgs/Point.h>
ros::NodeHandle  nh;
geometry_msgs::Point ref_msg ; 
ros::Publisher drill_ref("drill_ref",&ref_msg);

int c ;
int i=0;
int assembly_pwm=12, drill_pwm=11, container_pwm=9;
int assembly_dir=13, drill_dir=10, container_dir=8;
int xpwm=0, ypwm=0, zpwm=0, ang=0, container=0;
int current_angle=0, new_angle=0, range_min=0, range_max=1023, angle_min=0, angle_max=360;

void drill_msg( const pcl_msgs::Vertices& cmd_msg)
{
   //for drill up-down movement
    xpwm=cmd_msg.vertices[1];
    if(cmd_msg.vertices[0]==0)
    {
      digitalWrite(assembly_dir, 0);
      analogWrite(assembly_pwm, 0);
    }
    if(cmd_msg.vertices[0]==1)
    {
      digitalWrite(assembly_dir, 0);
      analogWrite(assembly_pwm, xpwm);
    }
    if(cmd_msg.vertices[0]==2)
    {
      digitalWrite(assembly_dir, 1);
      analogWrite(assembly_pwm, xpwm);
    }

    //for drill on off
    ypwm=cmd_msg.vertices[3];
    if(cmd_msg.vertices[2]==0)
    {
      digitalWrite(drill_dir, 0);
      analogWrite(drill_pwm, 0);
    }
    if(cmd_msg.vertices[2]==1)
    {
      digitalWrite(drill_dir, 0);
      analogWrite(drill_pwm, ypwm);
    }
    if(cmd_msg.vertices[2]==2)
    {
      digitalWrite(drill_dir, 1);
      analogWrite(drill_pwm, ypwm);
    }

    //for container roataion
    ang = cmd_msg.vertices[6];
    zpwm=cmd_msg.vertices[5];
    container=cmd_msg.vertices[4];
    if(container==0)
    {
      digitalWrite(container_dir, 0);
      analogWrite(container_pwm, 0);
    }
    if(container==1)
    {
      digitalWrite(container_dir, 0);
      analogWrite(container_pwm, zpwm);
    }
    if(container==2)
    {
      digitalWrite(container_dir, 1);
      analogWrite(container_pwm, zpwm);
      ang*=-1;
    }
    new_angle=current_angle+ang;

    
 
}


ros::Subscriber<pcl_msgs::Vertices> sub("move_drill", drill_msg);

void setup(){
  
  
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
  
}

void loop(){
   current_angle=analogRead(A0);
   current_angle=map(current_angle, range_min, range_max, angle_min, angle_max);
   if(current_angle>(new_angle-3) && current_angle<(new_angle+3))
   {
    analogWrite(container_pwm, 0); 
   }
   ref_msg.x=current_angle;
   ref_msg.y=new_angle;
   ref_msg.z=ang;
   chatter.publish(&ref_msg);
   delay(50);
   nh.spinOnce();
   delay(1);
}
