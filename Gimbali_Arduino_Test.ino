// Simple scanning routine for a LIDAR-based System
// LIDAR-based Environmental Perception System for Experimental Unmanned Aerial Vehicle
// Evgenia Udod (Lauri Kurs) 2016

#include <inttypes.h>
#include <SBGC.h>
#include <SBGC_Arduino.h>
#include <Wire.h>

//Serial baud rate should match with the rate, configured for the SimpleBGC controller
#define SERIAL_SPEED 115200
#define M_PII 3.141592653589793238462643

//delay between commands, ms
#define SBGC_CMD_DELAY 20

// Addressing for LIDAR-Lite sensor
#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.
int reading = 0;

//Gimbal Controller is conntected to TX2, RX2, GND
HardwareSerial &serial = Serial2;

//Mandatory struct for gimbal system
SBGC_cmd_control_t c = { 0, 0, 0, 0, 0, 0, 0 };


//Initialization of the arduino. I2C is initialized and Gimbal isi initialized
void setup()
{
  //Serial initialization
  Serial.begin(115200);
  serial.begin(SERIAL_SPEED);
  SBGC_Demo_setup(&serial);

  //join i2c bus, SCL, SDA ports on the Arduino Controller
  Wire.begin(); 

  //Take a pause to let gimbal controller to initialize
  delay(3000);

  //Move camera to initial position (all angles are zero)
  //Set speed 30 degree/sec
  c.mode = /*SBGC_CONTROL_MODE_ANGLE*/ 2;
  c.speedROLL = c.speedPITCH = c.speedYAW = 30 * SBGC_SPEED_SCALE;
  SBGC_cmd_control_send(c, sbgc_parser);
  Serial.print("Init is done (all angles are zero)");
  delay(3000);
}

// Control of the gimbal is given from serial to the main gimbal initial 
// configuration for allowing to follow the frame in "Follow Mode"
void no_control()
{
  c.mode = 0;
  c = { 0, 0, 0, 0, 0, 0, 0 };
  SBGC_cmd_control_send(c, sbgc_parser);
  Serial.print("Control: No Control");
  delay(3000);
}

// Here is the main loop
void loop()
{
  doFullScan(-10/*minDegreePitch*/, 10/*maxDegreePitch*/,-10/*minDegreeYaw*/, 10/*maxDegreeYaw*/, 70/*speedscale*/, 1/*scanningGranularityPitch*/, 1/*scanningGranularityYaw*/);
  delay(3000);
  no_control();
}

// Measurement acquisition from the LIDAR-Lite sensor via I2C (example code)
void getDistance()
{
  Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
  Wire.write((int)RegisterMeasure); // sets register pointer to  (0x00)  
  Wire.write((int)MeasureValue); // sets register pointer to  (0x00)  
  Wire.endTransmission(); // stop transmitting

  delay(20); // Wait 20ms for transmit

  Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
  Wire.write((int)RegisterHighLowB); // sets register pointer to (0x8f)
  Wire.endTransmission(); // stop transmitting

  delay(20); // Wait 20ms for transmit
  
  Wire.requestFrom((int)LIDARLite_ADDRESS, 2); // request 2 bytes from LIDAR-Lite

  if(2 <= Wire.available()) // if two bytes were received
  {
    reading = Wire.read(); // receive high byte (overwrites previous reading)
    reading = reading << 8; // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
  }
}

void goToYaw(float degree, int speedscale)
{
  c.mode = SBGC_CONTROL_MODE_ANGLE;
  c.speedROLL = c.speedPITCH = c.speedYAW = speedscale * SBGC_SPEED_SCALE;
  c.angleYAW = SBGC_DEGREE_TO_ANGLE(degree);
  SBGC_cmd_control_send(c, sbgc_parser);
}

void goToPitch(float degree, int speedscale)
{
  c.mode = SBGC_CONTROL_MODE_ANGLE;
  c.speedROLL = c.speedPITCH = c.speedYAW = speedscale * SBGC_SPEED_SCALE;
  c.anglePITCH = SBGC_DEGREE_TO_ANGLE(degree);
  SBGC_cmd_control_send(c, sbgc_parser);
}

void goToRoll(float degree, int speedscale)
{
  c.mode = SBGC_CONTROL_MODE_ANGLE;
  c.speedROLL = c.speedPITCH = c.speedYAW = speedscale * SBGC_SPEED_SCALE;
  c.angleROLL = SBGC_DEGREE_TO_ANGLE(degree);
  SBGC_cmd_control_send(c, sbgc_parser);
}

void getLidarReadings(float degreePitch, float degreeYaw, int distance)
{
  float x, y, z;
  /* old not correct
  x = distance * cos(degreePitch) * sin(degreeYaw);
  y = distance * cos(degreePitch) * cos(degreeYaw);
  z = distance * sin(degreePitch);*/

  //a = pitch, b = yaw, exelist
  y = distance * cos(degreePitch*M_PII/180) * sin(degreeYaw*M_PII/180);
  x = distance * cos(degreePitch*M_PII/180) * cos(degreeYaw*M_PII/180);
  z = distance * sin(degreePitch*M_PII/180);
  
  /*Serial.print(degreePitch);
  Serial.print("\t");
  Serial.print(degreeYaw);
  Serial.print("\t");
  Serial.print(distance);
  Serial.print("\t"); */ 

  if (x != 0 && y != 0 && z != 0)
  {
  Serial.print(x, 4);
  Serial.print(" ");
  Serial.print(y, 4);
  Serial.print(" ");
  Serial.println(z, 4);
  }
}

// Function for the main routine (plain scan)
void doFullScan(int minDegreePitch, int maxDegreePitch, int minDegreeYaw, int maxDegreeYaw, int speedscale, float scanningGranularityPitch, float scanningGranularityYaw)
{
  goToYaw(minDegreeYaw/*degree*/, speedscale/*speedscale*/);
  goToPitch(minDegreePitch/*degree*/, speedscale/*speedscale*/);
  Serial.println("Yaw at min degree. In 5 seconds starting full scan");
  delay (5000);

  for (float targetpitch = minDegreePitch; targetpitch <= maxDegreePitch; targetpitch += scanningGranularityPitch)
  {
    goToPitch(targetpitch/*degree*/, speedscale/*speedscale*/);
    delay (800);

    for (float targetyaw = minDegreeYaw; targetyaw <= maxDegreeYaw; targetyaw += scanningGranularityYaw)
    {
      goToYaw(targetyaw/*degree*/, speedscale/*speedscale*/);
      getDistance();
      getLidarReadings(targetpitch, targetyaw, reading);
      delay (10);
    }

    targetpitch++;

    goToPitch(targetpitch/*degree*/, speedscale/*speedscale*/);
    delay (800);

    for (float targetyaw = maxDegreeYaw; targetyaw >= minDegreeYaw; targetyaw -= scanningGranularityYaw)
    {
      goToYaw(targetyaw/*degree*/, speedscale/*speedscale*/);
      getDistance();
      getLidarReadings(targetpitch, targetyaw, reading);
      delay (10);
    }
  }
}
