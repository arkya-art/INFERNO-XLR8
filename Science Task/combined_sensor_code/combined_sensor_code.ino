#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Quaternion.h>

#include <Wire.h>


#include <SparkFunCCS811.h>
#include <SparkFunBME280.h>
#include <ClosedCube_HDC1080.h> //cjmcu
#include <SparkFunTSL2561.h> //luminisity sensor
#include "DFRobot_PH.h"
#include "DFRobot_EC.h"
#include <EEPROM.h>    //ph and conductivity sensor

geometry_msgs::Twist sensval1;  //for cjmcu sensor(6 values)
std_msgs::Float32 value;
geometry_msgs::Accel sensval2;   //for 3 gas sensors(6 values)
geometry_msgs::Quaternion sensval3; // for soil-mos,lum,ph-cond(4 values)

//cjmcu
#define CCS811_ADDR 0x5A    //Alternate I2C Address
CCS811 myCCS811(CCS811_ADDR);
ClosedCube_HDC1080 myHDC1080; 
BME280 myBME280;               

//mq4
int gas_sensor1 = A1; //Sensor pin
float m1 = -0.318; //Slope
float b1 = 1.133; //Y-Intercept
float R01 = 11.820; //Sensor Resistance in fresh air from previous code

//mq9
int gas_sensor2 = A2; //Sensor pin
float m2 = -1.749; //Slope
float b2 = 2.827; //Y-Intercept
float R02 = 0.91; //Sensor Resistance in fresh air from previous code

//mq8
#define         MQ_PIN                       (3)     //define which analog input channel you are going to use
#define         RL_VALUE                     (10)    //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR          (9.21)
#define         CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
                                                     //cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)
#define         GAS_H2                      (0)

float           H2Curve[3]  =  {2.3, 0.93,-1.44};                                                                                                                                                              
float           Ro           =  10;    

//soil moisture
const int sensor_pin = A4;

//luminosity
SFE_TSL2561 light;
boolean gain;     // Gain setting, 0 = X1, 1 = X16;
unsigned int ms;
float lux_val;  

//ph and conductivity
#define PH_PIN A5
#define EC_PIN A6
float  voltage,voltageEC,phValue,ecValue,d,temperature = 15;
DFRobot_PH ph;
DFRobot_EC ec;

ros::NodeHandle nh;
ros::Publisher chatter1("cjm", &sensval1);
ros::Publisher chatter2("gas", &sensval2);
ros::Publisher chatter3("oth", &sensval3);


void setup()
{
  nh.initNode();
  nh.advertise(chatter1);
  nh.advertise(chatter2);
  nh.advertise(chatter3);


  //cjmcu
  myBME280.settings.commInterface = I2C_MODE;
  myBME280.settings.I2CAddress = 0x76;
  myBME280.settings.runMode = 3; //Normal mode
  myBME280.settings.tStandby = 0;
  myBME280.settings.filter = 4;
  myBME280.settings.tempOverSample = 5;
  myBME280.settings.pressOverSample = 5;
  myBME280.settings.humidOverSample = 5;

  myBME280.begin();
  myHDC1080.begin(0x40);

  //It is recommended to check return status on .begin(), but it is not required.
  CCS811Core::status returnCode = myCCS811.begin();
  

  if (returnCode != CCS811Core::SENSOR_SUCCESS)
  {
    //Serial.println(".begin() returned with an error.");
    while (1);      //No reason to go further
  }

  
//mq4
 pinMode(gas_sensor1, INPUT); //Set gas sensor as input

//mq9
  pinMode(gas_sensor2, INPUT); //Set gas sensor as input

//mq8 
 Ro = MQCalibration(MQ_PIN); 

//soil moisture 

//luminosity
 light.begin();
  unsigned char ID;  
  if (light.getID(ID))
  {
    //Serial.print("Got factory ID: 0X");
   // Serial.print(ID,HEX);
    //Serial.println(", should be 0X5X");
    }
  else
  {
    byte error = light.getError();
//    printError(error);
  }
  gain = 0;
  unsigned char time = 2;
 // Serial.println("Set timing...");
  light.setTiming(gain,time,ms); 
 // Serial.println("Powerup...");
  light.setPowerUp();

//ph and conductivity
 ph.begin();
 ec.begin();  

}
//---------------------------------------------------loop()-------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------
void loop() 
{  

  //-----------------------------------cjmcu-----------------------------------
  if (myCCS811.dataAvailable())
  {
     //If so, have the sensor read and calculate the results.
    //Get them later
    myCCS811.readAlgorithmResults();

   // Serial.print("P[");
    float a = myBME280.readFloatPressure() * 0.00750062 ;
   // Serial.print("mmHg] ");

   // Serial.print("Alt[");
    float b = myBME280.readFloatAltitudeMeters();
   // Serial.print("m ");

   // Serial.print("] Temp[");
    float c = myHDC1080.readTemperature();
   // Serial.print("C] RH[");
    float d = myHDC1080.readHumidity();

  //  Serial.print("%] CO2[");
    float e = myCCS811.getCO2();  //Returns calculated CO2 reading
  //  Serial.print("] tVOC[");
    float f = myCCS811.getTVOC(); //Returns calculated TVOC reading
  //  Serial.print("] sec[");
    
  //  Serial.print(millis()/1000); //seconds since start
  //  Serial.print("]");
  //  Serial.println();
    
    //compensating the CCS811 with humidity and temperature readings from the HDC1080
    myCCS811.setEnvironmentalData(myHDC1080.readHumidity(), myHDC1080.readTemperature());
 
    sensval1.linear.x = a;
    sensval1.linear.y = b;
    sensval1.linear.z = c;

    sensval1.angular.x = d;
    sensval1.angular.y = e;
    sensval1.angular.z = f;

    delay(2000); //Don't spam the I2C bus

  float x[7]={a,b,c,d,e,f};
  for(int i=0 ; i<7 ; i++)
  { value.data = x[i];
  }
   
  }


//--------------------------------mq4--------------------------------------------
  float sensor_volt1; //Define variable for sensor voltage
  float RS_gas1; //Define variable for sensor resistance
  float ratio1; //Define variable for ratio
  float sensorValue1 = analogRead(gas_sensor1); //Read analog values of sensor
  sensor_volt1 = (sensorValue1 * (5.0 / 1023.0)); //Convert analog values to voltage
  RS_gas1 = ((5.0 * 10.0) / sensor_volt1) - 10.0; //Get value of RS in a gas
  ratio1 = RS_gas1 / R01;   // Get ratio RS_gas/RS_air

  double ppm_log1 = (log10(ratio1) - b1) / m1; //Get ppm value in linear scale according to the the ratio value
  double ppm1 = pow(10, ppm_log1); //Convert ppm value to log scale
  double percentage1 = ppm1 / 10000; //Convert to percentage                //
 
 // Serial.print("CO Concentration = ");
  float concentration1 = ppm1;
  float perc1 = percentage1;
   sensval2.linear.x = concentration1;
   sensval2.linear.y = perc1;

//----------------------------------mq9--------------------------------------------  
float sensor_volt2; //Define variable for sensor voltage
  float RS_gas2; //Define variable for sensor resistance
  float ratio2; //Define variable for ratio
  float sensorValue2 = analogRead(gas_sensor2); //Read analog values of sensor
  sensor_volt2 = (sensorValue2 * (5.0 / 1023.0)); //Convert analog values to voltage
  RS_gas2 = ((5.0 * 10.0) / sensor_volt2) - 10.0; //Get value of RS in a gas
  ratio2 = RS_gas2 / R02;   // Get ratio RS_gas/RS_air

  double ppm_log2 = (log10(ratio2) - b2) / m2; //Get ppm value in linear scale according to the the ratio value
  double ppm2 = pow(10, ppm_log2); //Convert ppm value to log scale
  double percentage2 = ppm2 / 10000; //Convert to percentage                //
 
 // Serial.print("CO Concentration = ");
  float concentration2 = ppm2;
  float perc2 = percentage2;
   sensval2.linear.z = concentration2;
   sensval2.angular.x = perc2;

//---------------------------------mq8-------------------------------------------
  float concentration3 = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_H2);
  float percentage3 = concentration3 / 10000;
  //Serial.print( "ppm" );
  float concentration33 = concentration3;
  float perc3 = percentage3;
 // Serial.print("\n");
  delay(200);  
   sensval2.angular.y = concentration33;
   sensval2.angular.z = perc3;

//----------------------------soil-moisture---------------------------------------
  float moisture_percentage;
  int sensor_analog;
  sensor_analog = analogRead(sensor_pin);
  moisture_percentage = ( 100 - ( (sensor_analog/1023.00) * 100 ) );
  sensval3.x = moisture_percentage;


//----------------------------luminosity------------------------------------------
// light.manualStart();
  delay(ms);
  unsigned int data0, data1;
  
  if (light.getData(data0,data1))
  { 
    
    // getData() returned true, communication was successful  
    //Serial.print("data0: ");
   // Serial.print(data0);
  //  Serial.print(" data1: ");
   // Serial.print(data1);
    // To calculate lux, pass all your settings and readings
    // to the getLux() function.  
    // The getLux() function will return 1 if the calculation
    // was successful, or 0 if one or both of the sensors was
    // saturated (too much light). If this happens, you can
    // reduce the integration time and/or gain.
    // For more information see the hookup guide at: https://learn.sparkfun.com/tutorials/getting-started-with-the-tsl2561-luminosity-sensor
    double lux;    // Resulting lux value
    boolean good;  // True if neither sensor is saturated  
    // Perform lux calculation:
    good = light.getLux(gain,ms,data0,data1,lux);   
    // Print out the results:  
   // Serial.print(" lux: ");
   // float lux_val ;
     lux_val = lux;
    if (good) Serial.println(" (good)"); 
    else Serial.println(" (BAD)");
  }
  else
  {
    // getData() returned false because of an I2C error, inform the user.
    byte error = light.getError();
   //printError(error);
  }  
  
  sensval3.y = lux_val;

//---------------------------ph and conductivity------------------------------------  

  float ph;
    static unsigned long timepoint = millis();
    if(millis()-timepoint>1000U)
    {                            //time interval: 1s
        timepoint = millis();
        //temperature = readTemperature();   // read your temperature sensor to execute temperature compensation
        int sensorvalue_ph;
        sensorvalue_ph = analogRead(PH_PIN);
        voltage = (sensorvalue_ph*(5.0/1023.0))*1000;         // read the ph voltage
      //  phValue = float(ph.readPH(voltage,temperature));       // convert voltage to pH with temperature compensation
       // Serial.println(voltagePH); 
        if (voltage > 8000)
       {   ph = 0;
          
        }
         if (voltage < 8000 && voltage > 6292 )
       {   ph = 1;
        } if (voltage < 6292 && voltage > 4796)
       {  ph = 2;
        } if (voltage < 4796 && voltage > 4600)
       {   ph = 3;
        } 
         
        if (voltage < 4600 && voltage > 4425)
       {   ph = 4.1;
       
       } 
        if (voltage < 4425 && voltage > 4250)
       {   ph = 4.2;
       
       } 
        if (voltage < 4250 && voltage > 4075)
       {    ph = 4.3;
       
       } 
       
       if (voltage < 4075 && voltage > 3900 )
       {   ph =4.4;
        }
        if (voltage < 3900 && voltage > 3735)
       {   ph =4.5 ;
       
       } 
         if (voltage < 3735 && voltage > 3570)
       {   ph = 4.6;
       
       } 
        if (voltage < 3570 && voltage > 3410)
       {  ph = 4.7;
       
       } 
        if (voltage < 3410 && voltage > 3250)
       {  ph =4.8;
       
       } 
        if (voltage < 3250 && voltage > 3105)
       {   ph =4.9;
       
       } 
         if (voltage < 3105 && voltage > 3045)
       {  ph = 5.0;
       
       } 
         if (voltage < 3045 && voltage > 2922.5)
       {  ph = 5.1;
       
       } 
         if (voltage < 2922.5 && voltage > 2800)
       {   ph = 5.2;
       
       } 
       if (voltage < 2800 && voltage > 2640)
       {  ph = 5.3;
       
       } 
        if (voltage < 2640 && voltage > 2480)
       {  ph = 5.4 ;
       
       } 

       if (voltage < 2480 && voltage > 2320)
       {  ph = 5.5 ;
       
       } 

        if (voltage < 2320 && voltage > 2160)
       {  ph = 5.6;
       
       } 

        if (voltage < 2160 && voltage > 2000)
       {  ph = 5.7;
       
       } 

       if (voltage < 2000 && voltage > 1853.4)
       {  ph = 5.8 ;
       
       } 

       if (voltage < 1853.4 && voltage > 1687)
       {  ph = 5.9;
       
       } 
       if (voltage < 1687 && voltage > 1560)
       {  ph=6.0 ;
       
       } 


       
        if (voltage < 1560 && voltage >1434)
       {  ph = 6.1;
        }
        if (voltage < 1434 && voltage > 1308)
       {   ph = 6.2;
        }
        if (voltage < 1308 && voltage > 1182)
       {  ph = 6.3;
        }
        if (voltage < 1182 && voltage > 1056)
       {   ph = 6.4;
        }
       if (voltage < 1056 && voltage > 930)
       {   ph = 6.5;
        }
        if (voltage < 930 && voltage > 804)
       {   ph = 6.6;
        }
        if (voltage < 804 && voltage > 678)
       {   ph = 6.7;
        }if (voltage < 678 && voltage > 552)
       {   ph = 6.8 ;
        }
        if (voltage < 552 && voltage > 426)
       {   ph = 6.9;
        }

        if (voltage < 426 && voltage > 300)
       {   ph = 7.0 ;
        }
        if (voltage < 300 && voltage > 219)
       {   ph = 7.1 ;
        }
        if (voltage < 219 && voltage > 140)
       {  ph = 7.2 ;
        }
        if (voltage < 140 && voltage > 137)
       {  ph = 7.3 ;
       
       } 
       if (voltage < 137 && voltage > 134)
       {  ph = 7.4 ;
       
       } 
       if (voltage < 134 && voltage > 131)
       {   ph = 7.5 ;
       
       } 
       if (voltage < 131 && voltage > 129)
       {   ph = 7.6 ;
       
       } 
       if (voltage < 129 && voltage > 122)
       {   ph = 7.7 ;
       
       } 
       if (voltage < 122 && voltage > 118)
       {   ph = 7.8 ;
       
       } 
       if (voltage < 118 && voltage > 115)
       {   ph = 7.9 ;
       
       } 
       if (voltage < 115 && voltage > 112)
       {   ph = 8.0 ;
       
       } 
       if (voltage < 112 && voltage > 109)
       {   ph = 8.2 ;
       
       } 
       if (voltage < 109 && voltage > 101)
       {   ph = 8.7 ;
       
       } 

       
        if (voltage < 101 && voltage > 50)
       {   ph = 9.0 ;
        }
       if (voltage < 50)
       {   ph = 10 ;
       }  
  

  
        
        voltageEC = analogRead(PH_PIN)/1024.0*5000;
        ecValue    = ec.readEC(voltageEC,temperature);       // convert voltage to EC with temperature compensation
       // Serial.print(", EC:");
        d = (ecValue);
       // Serial.println("ms/cm");
    

    }
   sensval3.z = ph;
   sensval3.w = d;

   chatter1.publish(&sensval1); 
   chatter2.publish(&sensval2); 
   chatter3.publish(&sensval3); 
   nh.spinOnce();

   delay(1000);
   
  }







  float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}

float MQCalibration(int mq_pin) 
{
  int i;
  float val=0;
 
  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value
 
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                        //according to the chart in the datasheet 
 
  return val; 
}

float MQRead(int mq_pin) 
{
  int i;
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
 
  rs = rs/READ_SAMPLE_TIMES;
 
  return rs;  
}

int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_H2) {
     return MQGetPercentage(rs_ro_ratio,H2Curve);
  }  
  return 0;
}

int  MQGetPercentage(float rs_ro_ratio, float *pcurve) 
{
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}
