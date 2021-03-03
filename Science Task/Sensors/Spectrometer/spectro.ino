#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>
#include "SparkFun_AS7265X.h"
AS7265X sensor;

#include <Wire.h>

//#include <WProgram.h>
//#include <Servo.h>

ros::NodeHandle nh;

std_msgs::Float32MultiArray spectro;
ros::Publisher spectro_pub("spectro", &spectro);

char dim0_label[] = "spectro";
int k=0;
void setup()
{
  Wire.begin();
  sensor.begin();
  sensor.disableIndicator(); //Turn off the blue status LED
  nh.initNode();
  spectro.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  spectro.layout.dim[0].label = dim0_label;
  spectro.layout.dim[0].size = 18;
  spectro.layout.dim[0].stride = 1*18;
  spectro.layout.data_offset = 0;
  //spectro.layout.dim_length = 1;
  spectro.data_length = 18;

  spectro.data = (float *)malloc(sizeof(float)*18);
  nh.advertise(spectro_pub);
}

void loop()
{
  
  sensor.takeMeasurementsWithBulb();
float A= sensor.getCalibratedA();
float B= sensor.getCalibratedB() ;
float C= sensor.getCalibratedC();
float D= sensor.getCalibratedD();
float E= sensor.getCalibratedE();
float F= sensor.getCalibratedF();
float G= sensor.getCalibratedG();
float H= sensor.getCalibratedH();
float I= sensor.getCalibratedI();
float J= sensor.getCalibratedJ();
float K= sensor.getCalibratedK();
float L= sensor.getCalibratedL();
float R= sensor.getCalibratedR();
float S= sensor.getCalibratedS();
float T= sensor.getCalibratedT();
float U= sensor.getCalibratedU();
float V= sensor.getCalibratedV();
float W= sensor.getCalibratedW();

spectro.data[0]=A;
spectro.data[1]=B;
spectro.data[2]=C;
spectro.data[3]=D;
spectro.data[4]=E;
spectro.data[5]=F;
spectro.data[6]=G;
spectro.data[7]=H;
spectro.data[8]=I;
spectro.data[9]=J;
spectro.data[10]=K;
spectro.data[11]=L;
spectro.data[12]=R;
spectro.data[13]=S;
spectro.data[14]=T;
spectro.data[15]=U;
spectro.data[16]=V;
spectro.data[17]=W;


  spectro_pub.publish( &spectro );   
  nh.spinOnce();
  delay(500);
}
