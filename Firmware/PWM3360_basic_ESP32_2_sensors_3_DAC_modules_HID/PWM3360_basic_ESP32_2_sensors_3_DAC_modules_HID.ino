#include <PMW3360.h>
#include <elapsedMillis.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include "USB.h"
#include "USBHIDMouse.h"
USBHIDMouse Mouse;

/* 
# PIN CONNECTION
  * MI = MISO
  * MO = MOSI
  * SS = Slave Select / Chip Select
  * SC = SPI Clock
  * MT = Motion (active low interrupt line)
  * RS = Reset
  * GD = Ground
  * VI = Voltage in up to +5.5V

# PMW3360_DATA struct format and description
  - PMW3360_DATA.isMotion      : bool, True if a motion is detected. 
  - PMW3360_DATA.isOnSurface   : bool, True when a chip is on a surface 
  - PMW3360_DATA.dx, data.dy   : integer, displacement on x/y directions.
  - PMW3360_DATA.SQUAL         : byte, Surface Quality register, max 0x80
                               * Number of features on the surface = SQUAL * 8
  - PMW3360_DATA.rawDataSum    : byte, It reports the upper byte of an 18‐bit counter 
                               which sums all 1296 raw data in the current frame;
                               * Avg value = Raw_Data_Sum * 1024 / 1296
  - PMW3360_DATA.maxRawData    : byte, Max/Min raw data value in current frame, max=127
    PMW3360_DATA.minRawData
  - PMW3360_DATA.shutter       : unsigned int, shutter is adjusted to keep the average
                               raw data values within normal operating ranges.
 */


// This example is using the Adafruit Feather ESP32-S2 https://www.adafruit.com/product/4769

#define SSF  7    // Slave Select pin for the front sensor.
#define SSL  1   // Slave Select pin for lateral sensor.

Adafruit_MCP4725 dac;
Adafruit_MCP4725 dac2;
Adafruit_MCP4725 dac3;

elapsedMillis timer;

boolean surface_quality = 0;

#define Motion  0x02
#define Delta_X_L 0x03
#define Delta_X_H 0x04
#define Delta_Y_L 0x05
#define Delta_Y_H 0x06
#define SQUAL 0x07

PMW3360 front_sensor;
PMW3360 lateral_sensor;

int x_front = 0;
int y_front = 0;
int x_lateral = 0;

int sumx_front = 0;
int sumy_front = 0;
int squal_front;

int sumx_lateral = 0;
int sumy_lateral = 0;
int squal_lateral;

//DAC variables

int ppr = 20000; //Approximate Need to calibrate 

int DAC_x;
int DAC_y;
int DAC_theta;

int g = 1;

float out_max = 4095;//max number of values of the DAC

void setup() 
{
  Serial.begin(9600);  
  //while(!Serial);

  dac.begin(0x61);
  dac2.begin(0x63);
  dac3.begin(0x60);
  
  //sensor.begin(10, 1600); // to set CPI (Count per Inch), pass it as the second parameter
  
  //Initialize front sensor
  if(front_sensor.begin(SSF)) 
    Serial.println("Sensor initialization successed");
  else
    Serial.println("Sensor initialization failed");

  //Initialize lateral sensor
    if(lateral_sensor.begin(SSL)) 
    Serial.println("Sensor initialization successed");
  else
    Serial.println("Sensor initialization failed");
  
  //sensor.setCPI(1600);    // or, you can set CPI later by calling setCPI();

  Mouse.begin();
  USB.begin();
}

void loop() 
{

  //PMW3360_DATA data read
  PMW3360_DATA data_front = front_sensor.readBurst();
  PMW3360_DATA data_lateral = lateral_sensor.readBurst();
  
  read_front_sensor();
  read_lateral_sensor();

  // Read the squal register of both sensors
  read_squal();
  
  // DAC
  get_DAC_y();
  get_DAC_x();
  get_DAC_theta();

  Mouse.move(g*x_front, g*x_lateral);
  
  if (timer > 20)
  {
  
    if (surface_quality == 1)
    {
      Serial.print(squal_front);
      Serial.print("\t");
      Serial.println(squal_lateral);
      timer = 0;
    }
    else
    {
      Serial.print("Y:");      
      Serial.print(sumx_front);
      Serial.print("\t");
      Serial.print("T:");
      Serial.print(sumy_front);
      Serial.print("\t");
      Serial.print("X:");
      Serial.println(sumx_lateral);

      


      //Serial.println(DAC_y);
      timer = 0;
    }
  }
}

int convTwosComp(int b)
{
  //Convert from 2's complement
  if(b & 0x80)
  {
    b = -1 * ((b ^ 0xff) + 1);
  }
  return b;
}

void read_front_sensor()
{
  int xh = front_sensor.readReg(Delta_X_H);
  int xl = front_sensor.readReg(Delta_X_L);
  x_front = convTwosComp(xl);
  sumx_front += x_front;

  int yh = front_sensor.readReg(Delta_Y_H);
  int yl = front_sensor.readReg(Delta_Y_L);
  y_front = convTwosComp(yl);
  sumy_front += y_front;
}

void read_lateral_sensor()
{
  int xh = lateral_sensor.readReg(Delta_X_H);
  int xl = lateral_sensor.readReg(Delta_X_L);
  x_lateral = convTwosComp(xl);
  sumx_lateral += x_lateral;
}

void read_squal()
{
  squal_front = convTwosComp(front_sensor.readReg(SQUAL));
  squal_lateral = convTwosComp(lateral_sensor.readReg(SQUAL));
}

void get_DAC_y()
{
  DAC_y = int(0.1*sumx_front + 2047.5);
  dac.setVoltage(DAC_y, false);
}

void get_DAC_x()
{
  DAC_x = int(0.1*sumx_lateral + 2047.5);
  dac2.setVoltage(DAC_x, false);
}

void get_DAC_theta()
{
  DAC_theta = int(0.1*sumy_front + 2047.5);
  dac3.setVoltage(DAC_theta, false);
}
