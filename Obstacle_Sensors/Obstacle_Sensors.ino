/*
 * © Copyright 2020, TSUPORT (Tsunami Post Disaster Robot) Research Team.
 * By: Hardefa Rizky Putu Rogonondo (Mechatronics Engineering 2016, 3110161019).
 * For more information, please visit https://github.com/noxand/TSUPORT-rescue-drone.
 * 
 * Note:
 * The system ID of the message should match the system ID of the vehicle (default is "1" but can be changed using the SYSID_THISMAV parameter).
 * The component id can be anything but MAV_COMP_ID_PATHPLANNER (195) or MAV_COMP_ID_PERIPHERAL (158) are probably good choices.
*/

// Library
#include "mavlink.h"
#include "mavlink_msg_distance_sensor.h"
#include <VL53L0X.h>
#include <Wire.h>

// Initialization
const int idle = 200;
const int MAX = 1600;
const int MIN = 10;
// Valid values for variables below are (even numbers only).
const int PreRng = 18; // Pre: 12 to 18 (initialized to 14 by default).
const int PostRng = 14; //Post: 8 to 14 (initialized to 10 by default).
const int Scale = 10;
#define XSHUT_pin1 A3 // Front Sensor
#define XSHUT_pin2 A0 // Right Sensor
#define XSHUT_pin3 7 // Back Sensor
#define XSHUT_pin4 2 // Left Sensor
#define XSHUT_pin5 8 // Bottom Sensor
#define XSHUT_pin6 A6 // Top Sensor *not used now
// Default address is 0b0101001 or 41 for the first sensor (The first sensor doesn't require address change).
#define Sensor2_newAddress 42
#define Sensor3_newAddress 43
#define Sensor4_newAddress 44
#define Sensor5_newAddress 45
//#define Sensor6_newAddress 46
VL53L0X Sensor1; // Front Sensor
VL53L0X Sensor2; // Right Sensor
VL53L0X Sensor3; // Back Sensor
VL53L0X Sensor4; // Left Sensor
VL53L0X Sensor5; // Bottom Sensor
//VL53L0X Sensor6; // Top Sensor

void setup(){
  // DO NOT give XSHUT pin 5V as it is not 5V-tolerant. Doing so will fry the sensor. 
  pinMode(XSHUT_pin1, OUTPUT);
  digitalWrite(XSHUT_pin1, LOW); 
  pinMode(XSHUT_pin2, OUTPUT);
  digitalWrite(XSHUT_pin2, LOW);
  pinMode(XSHUT_pin3, OUTPUT);
  digitalWrite(XSHUT_pin3, LOW);
  pinMode(XSHUT_pin4, OUTPUT);
  digitalWrite(XSHUT_pin4, LOW);
  pinMode(XSHUT_pin5, OUTPUT);
  digitalWrite(XSHUT_pin5, LOW);
  //pinMode(XSHUT_pin6, OUTPUT);
  //digitalWrite(XSHUT_pin6, LOW);
  Serial.begin(115200);
  Wire.begin();
  // Change address of sensor and power up next one
  // For power-up procedure, please refer to the VL53L0X datasheet --> 2.9 Power Sequence.
  // Note: t-boot max = 1.2ms.
  //Sensor6.setAddress(Sensor6_newAddress);
  //pinMode(XSHUT_pin4, INPUT);
  //delay(10);
  pinMode(XSHUT_pin5, INPUT);
  delay(10);
  Sensor5.setAddress(Sensor5_newAddress);
  pinMode(XSHUT_pin4, INPUT);
  delay(10);
  Sensor4.setAddress(Sensor4_newAddress);
  pinMode(XSHUT_pin3, INPUT);
  delay(10);
  Sensor3.setAddress(Sensor3_newAddress);
  pinMode(XSHUT_pin2, INPUT);
  delay(10);
  Sensor2.setAddress(Sensor2_newAddress);
  pinMode(XSHUT_pin1, INPUT);
  delay(10);
  // Default address is 0b0101001 or 41 for the first sensor (The first sensor doesn't require address change).
  Sensor1.init();
  Sensor2.init();
  Sensor3.init();
  Sensor4.init();
  Sensor5.init();
  //Sensor6.init();
  Sensor1.setTimeout(idle);
  Sensor2.setTimeout(idle);
  Sensor3.setTimeout(idle);
  Sensor4.setTimeout(idle);
  Sensor5.setTimeout(idle);
  //Sensor6.setTimeout(idle);
  // Lower the return signal rate limit (default is 0.25 MCPS).
  Sensor1.setSignalRateLimit(0.25);
  Sensor2.setSignalRateLimit(0.25);
  Sensor3.setSignalRateLimit(0.25);
  Sensor4.setSignalRateLimit(0.25);
  Sensor5.setSignalRateLimit(0.25);
  //Sensor6.setSignalRateLimit(0.25);
  // Increase laser pulse periods (defaults are 14 and 10 PCLKs).
  Sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, PreRng);
  Sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, PostRng);
  Sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, PreRng);
  Sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, PostRng);
  Sensor3.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, PreRng);
  Sensor3.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, PostRng);
  Sensor4.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, PreRng);
  Sensor4.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, PostRng);
  Sensor5.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, PreRng);
  Sensor5.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, PostRng);
  //Sensor6.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, PreRng);
  //Sensor6.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, PostRng);
}

void loop(){
  //command_heartbeat();
  command_distance_1(); // Front Sensor
  command_distance_2(); // Right Sensor
  command_distance_3(); // Back Sensor
  command_distance_4(); // Left Sensor
  command_distance_5(); // Bottom Sensor
  //command_distance_6(); // Top Sensor
}

void command_heartbeat(){
  // System ID = 1.
  int sysid = 100;
  // The component sending the message.
  int compid = MAV_COMP_ID_PATHPLANNER;
  // Define the system type, in this case ground control station.
  uint8_t system_type = MAV_TYPE_GCS;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
  uint8_t system_mode = 0; 
  uint32_t custom_mode = 0;
  uint8_t system_state = 0;
  // Initialize the required buffers.
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  // Pack the message.
  mavlink_msg_heartbeat_pack(sysid, compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);
  // Copy the message to send the buffer.
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message.
  //delay(1);
  Serial.write(buf, len);
}

void command_distance_1(){
  float Sensor1Smooth = Sensor1.readRangeSingleMillimeters();
  Sensor1Smooth = constrain(Sensor1Smooth, MIN, MAX);
  float dist1 = Sensor1Smooth/Scale;
  int sysid = 1;
  // The component sending the message.
  int compid = 158;
  uint32_t time_boot_ms = 0; // Time since system boot.
  uint16_t min_distance = 1; // Minimum distance the sensor can measure in centimeters.
  uint16_t max_distance = 170; // Maximum distance the sensor can measure in centimeters.
  uint16_t current_distance = dist1; // Current distance reading.
  uint8_t type = 0; // Type from MAV_DISTANCE_SENSOR enum.
  uint8_t id = 1; // Onboard ID of the sensor.
  uint8_t orientation = 0; // (0=forward, each increment is 45degrees more in clockwise direction), 24 (upwards) or 25 (downwards).
  // Consumed within ArduPilot by the proximity class.
  uint8_t covariance = 0; // Measurement covariance in centimeters, 0 for unknown/invalid readings.
  // Initialize the required buffers.
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  // Pack the message.
  mavlink_msg_distance_sensor_pack(sysid, compid, &msg, time_boot_ms, min_distance, max_distance, current_distance, type, id, orientation, covariance);
  // Copy the message to send the buffer.
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes).
  //delay(1);
  Serial.write(buf, len);
  //Serial.print("Front:");
  //Serial.println(dist1);
}

void command_distance_2(){
  float Sensor2Smooth = Sensor2.readRangeSingleMillimeters();
  Sensor2Smooth = constrain(Sensor2Smooth, MIN, MAX);
  float dist2 = Sensor2Smooth/Scale;
  int sysid = 1;
  // The component sending the message.
  int compid = 158;
  uint32_t time_boot_ms = 0; // Time since system boot.
  uint16_t min_distance = 1; // Minimum distance the sensor can measure in centimeters.
  uint16_t max_distance = 170; // Maximum distance the sensor can measure in centimeters.
  uint16_t current_distance = dist2; // Current distance reading.
  uint8_t type = 0; // Type from MAV_DISTANCE_SENSOR enum.
  uint8_t id = 2; // Onboard ID of the sensor.
  uint8_t orientation = 2; // (0=forward, each increment is 45degrees more in clockwise direction), 24 (upwards) or 25 (downwards).
  // Consumed within ArduPilot by the proximity class.
  uint8_t covariance = 0; // Measurement covariance in centimeters, 0 for unknown/invalid readings.
  // Initialize the required buffers.
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  // Pack the message.
  mavlink_msg_distance_sensor_pack(sysid, compid, &msg, time_boot_ms, min_distance, max_distance, current_distance, type, id, orientation, covariance);
  // Copy the message to send the buffer.
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes).
  //delay(1);
  Serial.write(buf, len);
  //Serial.print("Right:");
  //Serial.println(dist2);
}

void command_distance_3(){
  float Sensor3Smooth = Sensor3.readRangeSingleMillimeters();
  Sensor3Smooth = constrain(Sensor3Smooth, MIN, MAX);
  float dist3 = Sensor3Smooth/Scale;
  int sysid = 1;
  // The component sending the message.
  int compid = 158;
  uint32_t time_boot_ms = 0; // Time since system boot.
  uint16_t min_distance = 1; // Minimum distance the sensor can measure in centimeters.
  uint16_t max_distance = 170; // Maximum distance the sensor can measure in centimeters.
  uint16_t current_distance = dist3; // Current distance reading.
  uint8_t type = 0; // Type from MAV_DISTANCE_SENSOR enum.
  uint8_t id = 3; // Onboard ID of the sensor.
  uint8_t orientation = 4; // (0=forward, each increment is 45degrees more in clockwise direction), 24 (upwards) or 25 (downwards).
  // Consumed within ArduPilot by the proximity class.
  uint8_t covariance = 0; // Measurement covariance in centimeters, 0 for unknown/invalid readings.
  // Initialize the required buffers.
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  // Pack the message.
  mavlink_msg_distance_sensor_pack(sysid, compid, &msg, time_boot_ms, min_distance, max_distance, current_distance, type, id, orientation, covariance);
  // Copy the message to the send buffer.
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes).
  //delay(1);
  Serial.write(buf, len);
  //Serial.print("Back:");
  //Serial.println(dist3);
}

void command_distance_4(){
  float Sensor4Smooth = Sensor4.readRangeSingleMillimeters();
  Sensor4Smooth = constrain(Sensor4Smooth, MIN, MAX);
  float dist4 = Sensor4Smooth/Scale;
  int sysid = 1;
  // The component sending the message.
  int compid = 158;
  uint32_t time_boot_ms = 0; // Time since system boot.
  uint16_t min_distance = 1; // Minimum distance the sensor can measure in centimeters.
  uint16_t max_distance = 170; // Maximum distance the sensor can measure in centimeters.
  uint16_t current_distance = dist4; // Current distance reading.
  uint8_t type = 0; // Type from MAV_DISTANCE_SENSOR enum.
  uint8_t id = 4; // Onboard ID of the sensor.
  uint8_t orientation = 6; // (0=forward, each increment is 45degrees more in clockwise direction), 24 (upwards) or 25 (downwards).
  // Consumed within ArduPilot by the proximity class.
  uint8_t covariance = 0; // Measurement covariance in centimeters, 0 for unknown/invalid readings.
  // Initialize the required buffers.
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  // Pack the message.
  mavlink_msg_distance_sensor_pack(sysid, compid, &msg, time_boot_ms, min_distance, max_distance, current_distance, type, id, orientation, covariance);
  // Copy the message to the send buffer.
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes).
  //delay(1);
  Serial.write(buf, len);
  //Serial.print("Left:");
  //Serial.println(dist4);
}

void command_distance_5(){
  float Sensor5Smooth = Sensor5.readRangeSingleMillimeters();
  Sensor5Smooth = constrain(Sensor5Smooth, MIN, MAX);
  float dist5 = Sensor5Smooth/Scale;
  int sysid = 1;
  //  The component sending the message.
  int compid = 158;
  uint32_t time_boot_ms = 0; // Time since system boot.
  uint16_t min_distance = 1; // Minimum distance the sensor can measure in centimeters.
  uint16_t max_distance = 170; // Maximum distance the sensor can measure in centimeters.
  uint16_t current_distance = dist5; // Current distance reading.
  uint8_t type = 0; // Type from MAV_DISTANCE_SENSOR enum.
  uint8_t id = 5; // Onboard ID of the sensor.
  uint8_t orientation = 25; // (0=forward, each increment is 45degrees more in clockwise direction), 24 (upwards) or 25 (downwards).
  // Consumed within ArduPilot by the proximity class.
  uint8_t covariance = 0; // Measurement covariance in centimeters, 0 for unknown/invalid readings.
  // Initialize the required buffers.
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  // Pack the message.
  mavlink_msg_distance_sensor_pack(sysid, compid, &msg, time_boot_ms, min_distance, max_distance, current_distance, type, id, orientation, covariance);
  // Copy the message to the send buffer.
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes).
  //delay(1);
  Serial.write(buf, len);
  //Serial.print("Bottom:");
  //Serial.println(dist5);
}

//void command_distance_6(){
//  float Sensor6Smooth = Sensor6.readRangeSingleMillimeters();
//  Sensor6Smooth = constrain(Sensor6Smooth, MIN, MAX);
//  float dist6 = Sensor6Smooth/Scale;
//  int sysid = 1;
//  //  The component sending the message.
//  int compid = 158;
//  uint32_t time_boot_ms = 0; // Time since system boot.
//  uint16_t min_distance = 1; // Minimum distance the sensor can measure in centimeters.
//  uint16_t max_distance = 170; // Maximum distance the sensor can measure in centimeters.
//  uint16_t current_distance = dist6; // Current distance reading.
//  uint8_t type = 0; // Type from MAV_DISTANCE_SENSOR enum.
//  uint8_t id = 6; // Onboard ID of the sensor.
//  uint8_t orientation = 24; // (0=forward, each increment is 45degrees more in clockwise direction), 24 (upwards) or 25 (downwards).
//  // Consumed within ArduPilot by the proximity class.
//  uint8_t covariance = 0; // Measurement covariance in centimeters, 0 for unknown/invalid readings.
//  // Initialize the required buffers.
//  mavlink_message_t msg;
//  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
//  // Pack the message.
//  mavlink_msg_distance_sensor_pack(sysid, compid, &msg, time_boot_ms, min_distance, max_distance, current_distance, type, id, orientation, covariance);
//  // Copy the message to the send buffer.
//  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
//  // Send the message (.write sends as bytes).
//  //delay(1);
//  Serial.write(buf, len);
//  //Serial.print("Top:");
//  //Serial.println(dist6);
//}
