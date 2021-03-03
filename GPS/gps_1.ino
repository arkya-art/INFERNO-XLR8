#include <TinyGPS++.h>
#include <SoftwareSerial.h>
SoftwareSerial GPS_SoftSerial(11, 10);/* (Rx, Tx) */
#include <ros.h>
#include <geometry_msgs/Point.h>

ros::NodeHandle  nh;

geometry_msgs::Point ref_msg;
ros::Publisher chatter("chatter", &ref_msg);

     

volatile float minutes, seconds;
volatile int degree, secs, mins;

void setup() {
  //Serial.begin(9600); /* Define baud rate for //Serial communication */
  GPS_SoftSerial.begin(9600); /* Define baud rate for software //Serial communication */
   nh.initNode();
  nh.advertise(chatter);
}
TinyGPSPlus gps; 
void loop() {
        smartDelay(1000); /* Generate precise delay of 1ms */
        unsigned long start;
        double lat_val, lng_val, alt_m_val;
        uint8_t hr_val, min_val, sec_val;
        bool loc_valid, alt_valid, time_valid;
        lat_val = gps.location.lat(); /* Get latitude data */
        loc_valid = gps.location.isValid(); /* Check if valid location data is available */
        lng_val = gps.location.lng(); /* Get longtitude data */
        alt_m_val = gps.altitude.meters();  /* Get altitude data in meters */
        alt_valid = gps.altitude.isValid(); /* Check if valid altitude data is available */
        hr_val = gps.time.hour(); /* Get hour */
        min_val = gps.time.minute();  /* Get minutes */
        sec_val = gps.time.second();  /* Get seconds */
        time_valid = gps.time.isValid();  /* Check if valid time data is available */
        if (!loc_valid)
        {          
          //Serial.print("Latitude : ");
          //Serial.println("*****");
          //Serial.print("Longitude : ");
          //Serial.println("*****");
        }
        else
        {
          DegMinSec(lat_val);
          //Serial.print("Latitude in Decimal Degrees : ");
          //Serial.println(lat_val, 6);
           DegMinSec(lng_val); /* Convert the decimal degree value into degrees minutes seconds form */
          //Serial.print("Longitude in Decimal Degrees : ");
          //Serial.println(lng_val, 6);
          }
        if (!alt_valid)
        {
          //Serial.print("Altitude : ");
          //Serial.println("*****");
        }
        else
        {
          //Serial.print("Altitude : ");
          //Serial.println(alt_m_val, 6);    
        }
          ref_msg.x=lat_val;
  ref_msg.y=lng_val;
  ref_msg.z=alt_m_val;
  chatter.publish(&ref_msg);
  delay(50);
  nh.spinOnce();
  delay(1);
}
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (GPS_SoftSerial.available())  /* Encode data read from GPS while data is available on //Serial port */
   gps.encode(GPS_SoftSerial.read());
/* Encode basically is used to parse the Point received by the GPS and to store it in a buffer so that information can be extracted from it */
  } while (millis() - start < ms);
}

void DegMinSec( double tot_val)   /* Convert data in decimal degrees into degrees minutes seconds form */
{  
  degree = (int)tot_val;
  minutes = tot_val - degree;
  seconds = 60 * minutes;
  minutes = (int)seconds;
  mins = (int)minutes;
  seconds = seconds - minutes;
  seconds = 60 * seconds;
  secs = (int)seconds;
}
