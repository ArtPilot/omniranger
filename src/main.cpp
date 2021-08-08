/*
   This Projects Reads out I2C Rangefinders and sends out MAVLink
   Distance Message via Serial Port.
   That are connected via an TCA9548A I2C Multiplexer Board
   We used 8 GY-US42v2 Sonar Rangefinders
   And 8 TOF10120 Time of Flight Optical Rangefinders

   Tested on:
   - Arduino Pro Mini
*/

#include <Arduino.h>
#include <common/mavlink.h>
#include <common/mavlink_msg_distance_sensor.h>
#include <common/mavlink_msg_heartbeat.h>
#include <Wire.h>
#include <tca9548.h>
#include <i2c_range_sensor.h>

#define DEBUG_LOG false

// TODO: Change the Interface to be more expressive
// instead of a digit to configure the sensor type
// a word like I2C_RANGER_GYUS42 would be better,
OmniRanger::I2CRangeSensor srf_range_sensors[8] = {OmniRanger::I2CRangeSensor(1, 0), OmniRanger::I2CRangeSensor(1, 1), OmniRanger::I2CRangeSensor(1, 2), OmniRanger::I2CRangeSensor(1, 3), OmniRanger::I2CRangeSensor(1, 4), OmniRanger::I2CRangeSensor(1, 5), OmniRanger::I2CRangeSensor(1, 6), OmniRanger::I2CRangeSensor(1, 7)};
OmniRanger::I2CRangeSensor lrf_range_sensors[8] = {OmniRanger::I2CRangeSensor(0, 0), OmniRanger::I2CRangeSensor(0, 1), OmniRanger::I2CRangeSensor(0, 2), OmniRanger::I2CRangeSensor(0, 3), OmniRanger::I2CRangeSensor(0, 4), OmniRanger::I2CRangeSensor(0, 5), OmniRanger::I2CRangeSensor(0, 6), OmniRanger::I2CRangeSensor(0, 7)};


const char *sensor_positions[8] = {"FC", "FL", "SL",
                     "BL", "BC", "BR", "SR", "FR"
                    };

#define TCA_5948_ADDRESS byte(0x72)
#define SRF_TRIGGER_RESPONSE_WAIT_MS 100

// #define _TASK_TIMECRITICAL      // Enable monitoring scheduling overruns
#define _TASK_SLEEP_ON_IDLE_RUN // Enable 1 ms SLEEP_IDLE powerdowns between tasks if no callback methods were invoked during the pass
// #define _TASK_STATUS_REQUEST    // Compile with support for StatusRequest functionality - triggering tasks on status change events in addition to time only
// #define _TASK_WDT_IDS           // Compile with support for wdt control points and task ids
// #define _TASK_LTS_POINTER       // Compile with support for local task storage pointer
// #define _TASK_PRIORITY          // Support for layered scheduling priority
// #define _TASK_MICRO_RES         // Support for microsecond resolution
// #define _TASK_STD_FUNCTION      // Support for std::function (ESP8266 and ESP32 ONLY)
// #define _TASK_DEBUG             // Make all methods and variables public for debug purposes
// #define _TASK_INLINE            // Make all methods "inline" - needed to support some multi-tab, multi-file implementations
// #define _TASK_TIMEOUT           // Support for overall task timeout
// #define _TASK_OO_CALLBACKS      // Support for dynamic callback method binding
#include <TaskScheduler.h>

// Debug and Test options
#define _DEBUG_
//#define _TEST_

#ifdef _DEBUG_
#define _PP(a) Serial.print(a);
#define _PL(a) Serial.println(a);
#else
#define _PP(a)
#define _PL(a)
#endif

// LED_BUILTIN  13
#if defined(ARDUINO_ARCH_ESP32)
#define LED_BUILTIN 23 // esp32 dev2 kit does not have LED
#endif

// Scheduler
Scheduler ts;

/*
   Approach 1: LED is driven by the boolean variable; false = OFF, true = ON
*/
#define PERIOD1 16
void updateLasers();
Task tScanLasers(PERIOD1 * TASK_MILLISECOND, TASK_FOREVER, &updateLasers, &ts, true);

#define PERIOD2 50
void updateSonic();
Task tScanUltraSounds(PERIOD2 * TASK_MILLISECOND, TASK_FOREVER, &updateSonic, &ts, true);

#define PERIOD3 20
void sendDistanceSensorMsgs();
Task tPrintData(PERIOD3 * TASK_MILLISECOND, TASK_FOREVER, &sendDistanceSensorMsgs, &ts, true);

#define PERIOD4 250
void sendHearbeatMsg();
Task tSendHeartbeat(PERIOD4 * TASK_MILLISECOND, TASK_FOREVER, &sendHearbeatMsg, &ts, true);


void setup()
{
    // put your setup code here, to run once:
#if defined(_DEBUG_) || defined(_TEST_)
    Serial.begin(115200);
    delay(1*TASK_SECOND);
#endif
    pinMode(LED_BUILTIN, OUTPUT);
    Wire.setClock(400000);

    Wire.begin();
    while (!TCA9548.begin(TCA_5948_ADDRESS))
    {
        Serial.println("No Connection to TCA9548");
        delay(50);
    }
    // Serial.println("Got Connection to TCA9548");
}

void print_srf_ln()
{
  Serial.print("SRF: ");
  for (uint8_t t = 0; t < 8; t++)
  {
    Serial.print(srf_range_sensors[t].range);
    if (t < 7)
      Serial.print(", ");
    else
      Serial.println("");
  }
}

void print_lrf_ln()
{
    Serial.print("LRF: ");

    for (uint8_t t = 0; t < 8; t++)
    {
        Serial.print(lrf_range_sensors[t].range);
        if (t < 7)
            Serial.print(", ");
        else
            Serial.println("");
    }
}

unsigned int get_best_distance_for_channel(uint8_t channel)
{
  if (lrf_range_sensors[channel].range == 2000)
  {
    return srf_range_sensors[channel].range;
  }
  else
  {
    return min(srf_range_sensors[channel].range, lrf_range_sensors[channel].range);
  }
}

void print_best_distance_for_channel(uint8_t channel)
{
  static char range[7];
  if (lrf_range_sensors[channel].range == 2000 || srf_range_sensors[channel].range < lrf_range_sensors[channel].range)
  {
    sprintf(range, "%c-%4d", 'S', srf_range_sensors[channel].range);
  }
  else
  {
    sprintf(range, "%c-%4d", 'L', lrf_range_sensors[channel].range);
  }
  Serial.print(range);

}


void print_min_distance() {
    for (uint8_t t = 0; t < 8; t++)
    {
        Serial.print(sensor_positions[t]);
        Serial.print(": ");
        print_best_distance_for_channel(t);
        Serial.print("  ");

        if (t < 7)
            Serial.print(", ");
        else
            Serial.println("");
    }
}

void loop()
{
    ts.execute();
}

// === 1 =======================================
void updateLasers()
{
    uint8_t offset = (tScanUltraSounds.getRunCounter() % 2 == 0) ? 1 : 0;

    for (uint8_t t = offset; t < 8; t = t + 2)
    {
        if (lrf_range_sensors[t].triggered == true) {
            lrf_range_sensors[t].read();
        }
    }
    for (uint8_t t = offset; t < 8; t = t + 2)
    {
        lrf_range_sensors[t].trigger();
    }
}

void updateSonic()
{
    uint8_t offset = (tScanUltraSounds.getRunCounter() % 2 == 0) ? 1 : 0;
    for (uint8_t t = offset; t < 8; t = t + 2)
    {
        if (srf_range_sensors[t].triggered == true) {
            srf_range_sensors[t].read();
        }
    }
    for (uint8_t t = offset; t < 8; t = t + 2)
    {
        srf_range_sensors[t].trigger();
    }
}


void sendDistanceSensorMsg(uint8_t orient, uint16_t rngDist) {
  //MAVLINK DISTANCE MESSAGE
  int sysid = 1;                   
  //< The component sending the message.
  int compid = MAV_COMP_ID_OBSTACLE_AVOIDANCE;

  uint32_t time_boot_ms = millis(); /*< Time since system boot*/
  uint16_t min_distance = 1; /*< Minimum distance the sensor can measure in centimeters*/
  uint16_t max_distance = 720; /*< Maximum distance the sensor can measure in centimeters*/
  uint16_t current_distance = rngDist / 10; /*< Current distance reading in cm*/
  uint8_t type = MAV_DISTANCE_SENSOR_UNKNOWN; /*< Type from MAV_DISTANCE_SENSOR enum.*/
  uint8_t id = 1; /*< Onboard ID of the sensor*/
  uint8_t orientation = orient; /*(0=forward, each increment is 45degrees more in clockwise direction), 24 (upwards) or 25 (downwards)*/
  // Consumed within ArduPilot by the proximity class
  uint8_t covariance = 0; /*< Measurement covariance in centimeters, 0 for unknown / invalid readings*/

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_distance_sensor_pack(sysid,compid,&msg,time_boot_ms,min_distance,max_distance,current_distance,type,id,orientation,covariance,0.0,0.0,0);
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes) 
  Serial.write(buf, len);
  // Serial.print (orientation);
  //Serial.print (" - ");
  //Serial.println (current_distance);
}

void sendDistanceSensorMsgs() {
  for (uint8_t t = 0; t < 8; t = t + 1) {
    sendDistanceSensorMsg(t, get_best_distance_for_channel(t));
  }
}

void sendHearbeatMsg() {
  const int sysid = 1;                            //< ID 1 for this system               
  const int compid = MAV_COMP_ID_OBSTACLE_AVOIDANCE;       //< The component sending the message.
  const uint8_t system_type =MAV_TYPE_GCS;         // Define the system type, in this case ground control station
  const uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
  const uint8_t system_mode = 0; 
  const uint32_t custom_mode = 0;                
  const uint8_t system_state = 0;
  
    // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  // Pack the message
  mavlink_msg_heartbeat_pack(sysid,compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);
  
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  // Send the message 
  Serial.write(buf, len);
}

void printData() {
    print_srf_ln();
    print_lrf_ln();
    // print_min_distance();
}