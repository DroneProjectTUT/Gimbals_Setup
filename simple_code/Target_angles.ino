/******************************************************************************
	This is example sketch for Arduino.
	Shows how to control SimpleBGC-driven gimbal via Serial API.
	API specs are available at http://www.basecamelectronics.com/
  Initial Example: Aleksey Moskalenko
  Demo: E. Udod / L. Kurs

  Demo: How to get Target Angles from Simple BGC board. 
  Look at Serial API documentation
*******************************************************************************/
#include <inttypes.h>
#include <SBGC.h>
#include <SBGC_Arduino.h>

// delay between commands, ms
#define SBGC_CMD_DELAY 20

/*****************************************************************************/
//GLOBAL
typedef struct {
  int16_t ANGLE_ROLL;       /*Actual angle from IMU*/
  int16_t RC_ANGLE_ROLL;    /*Target angle gimbal should keep*/
  int16_t RC_SPEED_ROLL;    /*Target speed gimbal should keep*/
  int16_t ANGLE_PITCH;      /*Actual angle from IMU*/
  int16_t RC_ANGLE_PITCH;   /*Target angle gimbal should keep*/
  int16_t RC_SPEED_PITCH;   /*Target speed gimbal should keep*/
  int16_t ANGLE_YAW;        /*Actual angle from IMU*/
  int16_t RC_ANGLE_YAW;     /*Target angle gimbal should keep*/
  int16_t RC_SPEED_YAW;     /*Target speed gimbal should keep*/
} SBGC_imu_data;

SBGC_cmd_control_data c = { 0, 0, 0, 0, 0, 0, 0 };
SBGC_imu_data imuData = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

uint8_t header_array[4], data_array[19];

void setup()
{
  Serial1.begin(115200);

  // Take a pause to let gimbal controller to initialize
  delay(5000);
  move_pitch();
}

void loop()
{
  blink_led(5);
  SBGC_sendCommand(SBGC_CMD_GET_ANGLES, &c, sizeof(c));
  // Move camera to initial position (all angles are zero)
  // Set speed 30 degree/sec
  delay(30);
  get_imu_data();
  decode_angle_header_msg();
  print_angles();


}

void move_pitch()
{
// Move camera to initial position (all angles are zero)
// Set speed 30 degree/sec
c.mode = SBGC_CONTROL_MODE_ANGLE;
c.speedROLL = c.speedPITCH = c.speedYAW = 30 * SBGC_SPEED_SCALE;
SBGC_sendCommand(SBGC_CMD_CONTROL, &c, sizeof(c));
delay(3000);

blink_led(1);
//// MOVE pitch 25 degrees
c.mode = SBGC_CONTROL_MODE_ANGLE;
c.angleROLL = SBGC_DEGREE_TO_ANGLE(25);
SBGC_sendCommand(SBGC_CMD_CONTROL, &c, sizeof(c));
delay(3000);
}


// 4baiti head
// body 18 + 1baiti
// 23baiti

void get_imu_data()
{

  while (Serial.available())
  {
    for (int i = 0; i < 4; i++)
    {
      /* header struct
      0 character
      1 cmd_id
      2 data_size
      3 check_csum
      */
      header_array[i] = Serial1.read();
    }

    for (int i = 0; i < 19; i++)
    {
      /* imu data struct
      0 roll angle lower byte
      1 roll angle upper byte
      2 roll target angle lower byte
      3 roll target angle upper byte
      4 roll speed lower byte
      5 roll speed upper byte

      6 pitch angle lower byte
      7 pitch angle upper byte
      8 pitch target angle lower byte
      9 pitch target angle upper byte
      10 pitch speed lower byte
      11 pitch speed upper byte

      12 yaw angle lower byte
      13 yaw angle upper byte
      14 yaw target angle lower byte
      15 yaw target angle upper byte
      16 yaw speed lower byte
      17 yaw speed upper byte

      18 checksum
      */
      data_array[i] = Serial1.read();
      /*Serial.print(data_array[i]);
      Serial.print("\t");
      Serial.println(i);*/
    }
  }
}

void decode_angle_header_msg()
{
  /*Serial.print(header_array[0]);
  Serial.print("\t");
  Serial.print(header_array[1]);
  Serial.print("\t");
  Serial.print(header_array[2]);
  Serial.print("\t");
  Serial.println(header_array[3]);
*/

  if (header_array[0] == '>' && header_array[1] == 'I' && header_array[2] == 18 && header_array[3] == 91)
  {
    //Let's decode IMU message
    decode_angle_msg();
  }
  else
  {
    Serial.println("Message header bad, return from decode_angle_header_msg");
    return;
  }
}

void shift_bits(int8_t a /*lower*/, int8_t b /*upper*/, int16_t *c /*out*/)
{
  int16_t z = 0;
  z = b << 8;
  z = z | a;
  *c = z;
}

void decode_angle_msg()
{
  //CheckSum calc
  uint8_t check_sum = 0;

  for (int i = 0; i < 18; i++)
  {
    check_sum += data_array[i];
  }

  if (check_sum % 256 == data_array[18])
  {
    shift_bits(data_array[0], data_array[1], &imuData.ANGLE_ROLL);
    shift_bits(data_array[2], data_array[3], &imuData.RC_ANGLE_ROLL);
    shift_bits(data_array[4], data_array[5], &imuData.RC_SPEED_ROLL);

    shift_bits(data_array[6], data_array[7], &imuData.ANGLE_PITCH);
    shift_bits(data_array[8], data_array[9], &imuData.RC_ANGLE_PITCH);
    shift_bits(data_array[10], data_array[11], &imuData.RC_SPEED_PITCH);

    shift_bits(data_array[12], data_array[13], &imuData.ANGLE_YAW);
    shift_bits(data_array[14], data_array[15], &imuData.RC_ANGLE_YAW);
    shift_bits(data_array[16], data_array[17], &imuData.RC_SPEED_YAW);
  }
  else
  {
    Serial.println("Bad message body from IMU, return from decode_angle_msg");
    return;
  }
}

void print_angles()
{
  Serial.println("");
  Serial.print("REAL: ");
  Serial.print(SBGC_ANGLE_TO_DEGREE(imuData.ANGLE_ROLL));
  Serial.print("\t");
  Serial.print(SBGC_ANGLE_TO_DEGREE(imuData.ANGLE_PITCH));
  Serial.print("\t");
  Serial.print(SBGC_ANGLE_TO_DEGREE(imuData.ANGLE_YAW));
  Serial.print("\t");

  Serial.print("TARGET: ");
  Serial.print(SBGC_ANGLE_TO_DEGREE(imuData.RC_ANGLE_ROLL));
  Serial.print("\t");
  Serial.print(SBGC_ANGLE_TO_DEGREE(imuData.RC_ANGLE_PITCH));
  Serial.print("\t");
  Serial.print(SBGC_ANGLE_TO_DEGREE(imuData.RC_ANGLE_YAW));
  Serial.print("\t");
  Serial.println("");

}
