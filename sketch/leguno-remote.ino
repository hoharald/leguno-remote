/*  This sketch was made to remote control a LEGO Technic vehicle
    Copyright (C) 2020  hoharald 
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>
#include <math.h>
#include "TechnicHub.h"

//#define DEBUG                                         //Uncomment this line to debug via a connected serial terminal

#ifdef DEBUG
  #define DPRINT(...)    Serial.print(__VA_ARGS__)
  #define DPRINTLN(...)  Serial.println(__VA_ARGS__)
#else
  #define DPRINT(...)     //now defines a blank line
  #define DPRINTLN(...)   //now defines a blank line
#endif


enum class ControllerState {
  idle = 0,
  setup = 1,
  initDirection = 2,
  running = 3
};

enum class InitDirectionState {
  idle = 0,
  left = 1,
  wait_for_left = 2,
  wait_for_left_read = 3,
  right = 4,
  wait_for_right = 5,
  wait_for_right_read = 6,
  center = 7,
  done = 8
};

const int ledPinR = 22;
const int ledPinG = 23;
const int ledPinB = 24;

#define GATT_SERVICE "00001623-1212-efde-1623-785feabcd123"
#define GATT_CHARACTERISTIC "00001624-1212-efde-1623-785feabcd123"

unsigned long previousMillisInitDirection, previousMillisGyro, previousMillisDebugAngles, previousMillisDebugSpeed, previousMillisDebugDirection = 0;

const long debugInterval = 1000;
const long directionLimitDelay = 2000;

float accel_angle_x, accel_angle_y, accel_angle_z = 0.0f;
float current_angle_x, current_angle_y, current_angle_z = 0.0f;

ControllerState controller_state = ControllerState::idle;
InitDirectionState init_direction_state = InitDirectionState::idle;

uint8_t last_speed = 0;
uint8_t last_direction = 0;

uint8_t received_length;
byte received_buffer[64];

uint8_t write_length;
int write_result;

int32_t direction_center, direction_range, direction_limit_left, direction_limit_right, direction_position, direction_target_position;

BLECharacteristic characteristic;

void setup() {
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
  pinMode(24, OUTPUT);

  digitalWrite(ledPinR, HIGH);
  digitalWrite(ledPinG, HIGH);
  digitalWrite(ledPinB, HIGH);

#ifdef DEBUG
  Serial.begin(115200);
  while (!Serial);
#endif

  // begin initialization
  if (!IMU.begin()) {
    DPRINT("Failed to initialize IMU!");
    while (1);
  }
  
  if (!BLE.begin()) {
    DPRINT("Starting BLE failed!");
    while (1);
  }

  DPRINTLN("Scanning for Technic Hub");

  // start scanning for peripheral
  BLE.scanForName("Technic Hub");

}

void loop() {
  
  BLEDevice technicHub = BLE.available();
  
  if (technicHub)
  {
    DPRINT("Device found: ");
    DPRINTLN(technicHub.address());

    DPRINTLN("Connecting ...");

    BLE.stopScan();

    controller(technicHub);
    
    BLE.scanForName("Technic Hub");  
  }  
}

void controller(BLEDevice technicHub) {
  if (technicHub.connect()) {
    digitalWrite(ledPinR, HIGH);
    digitalWrite(ledPinG, HIGH);
    digitalWrite(ledPinB, LOW);
    DPRINTLN("Connected");    
  
  } else {
    DPRINTLN("Failed to connect!");
    return;
  } 

  if(technicHub.discoverAttributes()) {   
    DPRINTLN("Attributes discovered");
  } else {
    DPRINTLN("ERROR: Attribute discovery failed!");
    technicHub.disconnect();
    return;
  }

  BLEService service = technicHub.service(GATT_SERVICE);

  if(!service) {
    DPRINTLN("ERROR: Service not found!"); 
    technicHub.disconnect();
    return;        
  }

  characteristic = technicHub.characteristic(GATT_CHARACTERISTIC);

  if(!characteristic) {
    DPRINTLN("ERROR: Characteristic not found!");  
    technicHub.disconnect(); 
    return;      
  } else {
    if (characteristic.subscribe()) {
      DPRINTLN("Successfully subscribed");  
    } else {
      DPRINTLN("ERROR: Was unable to subscribe!");  
    };
  } 

  while(technicHub.connected()) {

    if (characteristic.valueUpdated()) {
      received_length = characteristic.valueLength();
      characteristic.readValue(&received_buffer, received_length);
      logByteBuffer("Received data from hub", received_length);  

      //Switch message type
      switch(received_buffer[2]) {
        case (uint8_t)TechnicHubMessageType::portValueSingle:
          //Check Port (ID 2 => directional motor)
          if(received_buffer[3] == 2) {
            memcpy(&direction_position, &received_buffer[4], 4);
          }
          break;
      }
    }    

    switch(controller_state) {
      case ControllerState::idle:
        controller_state = ControllerState::setup;
        break;
      case ControllerState::setup:
        if(!setupHub()) {
          return;
        }
        controller_state = ControllerState::initDirection;
        break;
      case ControllerState::initDirection:
        if(!initDirection()) {
          return;
        };
        if(init_direction_state == InitDirectionState::done) {
          controller_state = ControllerState::running;
        }
        break;
      case ControllerState::running:
        updateAngles();  
        if(!controlSpeedAndDirection()) {
          return;
        }
        break;
      default: break;
    }
  }
  digitalWrite(ledPinR, HIGH);
  digitalWrite(ledPinG, HIGH);
  digitalWrite(ledPinB, HIGH);
}

void updateAngles() {
  float gyro_x, gyro_y, gyro_z = 0.0f;
  float ra, pa, yaw, accel_x, accel_y, accel_z, mag_x, mag_y, mag_z;  
  unsigned long currentMillis = millis();
  float deltaTime = (currentMillis - previousMillisGyro) / 1000.0f;
  
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyro_x, gyro_y, gyro_z);      
    previousMillisGyro = currentMillis;
  }

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accel_x, accel_y, accel_z);
    ra = atan2(accel_y, accel_z);
    pa = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z));
    accel_angle_x = ra * 180/PI;
    accel_angle_y = pa * 180/PI;
  }  

  // if (IMU.magneticFieldAvailable()) {
  //   IMU.readMagneticField(mag_x, mag_y, mag_z);
  //   float mag_norm = sqrt((mag_x*mag_x)+(mag_y*mag_y)+(mag_z*mag_z));
  //   mag_x=mag_x/mag_norm;
  //   mag_y=mag_y/mag_norm;
  //   mag_z=mag_z/mag_norm;    
  // }

  //float mx = mag_x * cos(pa) + mag_z * sin(pa);
  //float my = mag_x * sin(ra) * sin(pa) + mag_y * cos(ra) - mag_z * sin(ra) * cos(pa);
  //yaw = atan2(-my, mx) * 180/PI; 
  
  current_angle_x = 0.5 * (current_angle_x + gyro_x * deltaTime) + 0.5 * accel_angle_x;
  current_angle_y = 0.5 * (current_angle_y + gyro_y * deltaTime) + 0.5 * accel_angle_y;
  //current_angle_z = 0.98 * (current_angle_z + gyro_x * deltaTime) + 0.02 * yaw;
  
  if ((currentMillis - previousMillisDebugAngles) > debugInterval) {
    previousMillisDebugAngles = currentMillis;
    DPRINTLN("Current calculated angles");
    DPRINT(current_angle_x);
    DPRINT('\t');
    DPRINT(current_angle_y);
    DPRINT('\t');
    DPRINTLN(current_angle_z);
  }
}

bool setupHub()
{
  write_length = TechnicHubMessageBuilder::createVirtualPortConnect(0, 1);
  write_result = characteristic.writeValue(&TechnicHubMessageBuilder::buffer, write_length);    

  if(write_result == 0) {
    DPRINTLN("ERROR: Unable to map ports A and B to virtual port"); 
    return false;
  } else {
    logByteBuffer("Port setup for virtual mapping done", write_length);
  }

  write_length = TechnicHubMessageBuilder::createPortInputFormatSetupSingle(2, 2, 0x1, true);
  write_result = characteristic.writeValue(&TechnicHubMessageBuilder::buffer, write_length);

  if(write_result == 0) {
    DPRINTLN("ERROR: Unable to setup input format"); 
    return false;
  } else {
    logByteBuffer("Port setup for input format done", write_length);
  }
  return true;
}

bool initDirection() {
  unsigned long currentMillis = millis();
  
  switch(init_direction_state) {
    case InitDirectionState::idle:
      init_direction_state = InitDirectionState::left;
      break;
    case InitDirectionState::left:
      if(!initDirectionLimit(false)) {
        return false;
      };
      init_direction_state = InitDirectionState::wait_for_left;
      break;
    case InitDirectionState::wait_for_left:
      if((currentMillis - previousMillisInitDirection) > directionLimitDelay) {
        write_length = TechnicHubMessageBuilder::createPortInformationRequest(2, TechnicHubPortInformationType::portValue);
        write_result = characteristic.writeValue(&TechnicHubMessageBuilder::buffer, write_length);
        if (write_result == 0) {
          return false;
        }
        logByteBuffer("Read value of direction limit left requested", write_length); 
        previousMillisInitDirection = currentMillis;
        init_direction_state = InitDirectionState::wait_for_left_read;
      }
      break;
    case InitDirectionState::wait_for_left_read:
      if((currentMillis - previousMillisInitDirection) > 500) {
        direction_limit_left = direction_position;
        init_direction_state = InitDirectionState::right;
      }
      break;
    case InitDirectionState::right:
      if(!initDirectionLimit(true)) {
        return false;
      };
      init_direction_state = InitDirectionState::wait_for_right;
      break;
    case InitDirectionState::wait_for_right:
      if((currentMillis - previousMillisInitDirection) > directionLimitDelay) {
        write_length = TechnicHubMessageBuilder::createPortInformationRequest(2, TechnicHubPortInformationType::portValue);
        write_result = characteristic.writeValue(&TechnicHubMessageBuilder::buffer, write_length);
        if (write_result == 0) {
          return false;
        }
        logByteBuffer("Read value of direction limit right requested", write_length); 
        previousMillisInitDirection = currentMillis;
        init_direction_state = InitDirectionState::wait_for_right_read;
      }
      break;
    case InitDirectionState::wait_for_right_read:
      if((currentMillis - previousMillisInitDirection) > 500) {
        direction_limit_right = direction_position;
        direction_range = abs(direction_limit_left - direction_limit_right);
        direction_center = direction_limit_left + direction_range / 2;
        DPRINT("Initialized direction center at ");
        DPRINT(direction_center);
        DPRINT(" with an range of ");
        DPRINTLN(direction_range);
        DPRINT("Left limit was recognized at ");
        DPRINTLN(direction_limit_left);
        DPRINT("Right limit was recognized at ");
        DPRINTLN(direction_limit_right);
        characteristic.unsubscribe();
        init_direction_state = InitDirectionState::center;
      }
      break;
    case InitDirectionState::center:
      write_length = TechnicHubMessageBuilder::createPortOutputGoToAbsolutePosition(2, direction_center, 40, 80, TechnicHubPortOutputCommandEndState::_hold, false, false);
      write_result = characteristic.writeValue(&TechnicHubMessageBuilder::buffer, write_length);
      if(write_result == 0) {
        DPRINTLN("ERROR: Unable to write output to center direction");   
        return false;        
      } 
      init_direction_state = InitDirectionState::done;
      break;
    case InitDirectionState::done:
      break;
    default: break;
  }
  return true;
}

bool initDirectionLimit(bool right) {
  unsigned long currentMillis = millis();
  int8_t speed = right ? 20 : -20;
  write_length = TechnicHubMessageBuilder::createPortOutputStartSpeed(2, speed, 40, false, false);
  write_result = characteristic.writeValue(&TechnicHubMessageBuilder::buffer, write_length);

  if(write_result == 0) {
    DPRINTLN("ERROR: Unable to write output durring direction initialization");   
    return false;        
  }  
  previousMillisInitDirection = currentMillis;
  return true;
}

bool controlSpeedAndDirection() {
  unsigned long currentMillis = millis();
  float t = scaleValue(current_angle_y, -40.0f, 40.0f, -100.0f, 100.0f);
  int8_t speed = (int8_t)limitValue(silenceValue(t, 10.0), -100.0f, 100.0f);
  boolean speedChanged =  speed < (last_speed - 2) || speed > (last_speed + 2) || (speed == 0 && last_speed != 0);

  if(speedChanged) {      
    write_length = TechnicHubMessageBuilder::createPortOutputStartPower(16, -speed);
    write_result = characteristic.writeValue(&TechnicHubMessageBuilder::buffer, write_length);

    if(write_result == 0) {
      DPRINTLN("ERROR: Unable to write power");   
      return false;        
    } else if ((currentMillis - previousMillisDebugSpeed) > debugInterval) {
      previousMillisDebugSpeed = currentMillis;
      DPRINT("Wrote speed of ");
      DPRINT(speed);
      DPRINT("% with buffer size of ");
      DPRINT(write_length);
      DPRINTLN(" to hub");
      logByteBuffer(write_length);       
    } 
    last_speed = speed;  
  }
  
  t = scaleValue(current_angle_x, -60.0f, 60.0f, -100.0f, 100.0f);
  int8_t direction = (int8_t)limitValue(silenceValue(t, 10.0), -100.0f, 100.0f);
  boolean directionChanged =  direction < (last_direction - 10) || direction > (last_direction + 10);

  if(directionChanged) {    
    direction_target_position = direction_center - direction * direction_range / 200;
    write_length = TechnicHubMessageBuilder::createPortOutputGoToAbsolutePosition(2, direction_target_position, 40, 100, TechnicHubPortOutputCommandEndState::_hold, false, false);
    write_result = characteristic.writeValue(&TechnicHubMessageBuilder::buffer, write_length);
    if(write_result == 0) {
      DPRINTLN("ERROR: Unable to write absolute position");   
      return false;        
    } else if ((currentMillis - previousMillisDebugDirection) > debugInterval) {
      previousMillisDebugDirection = currentMillis;
      DPRINT("Wrote absolute position of ");
      DPRINT(direction_target_position);
      DPRINT(" with buffer size of ");
      DPRINT(write_length);
      DPRINTLN(" to hub");
      logByteBuffer(write_length);       
    }   
    last_direction = direction;
  }  

  return true;
}

float scaleValue(float v, float v_min, float v_max, float t_min, float t_max) {
  return (v - v_min) / (v_max - v_min) * (t_max - t_min) + t_min;
}

float limitValue(float v, float v_min, float v_max) {
  return (v >= v_min) ? ((v <= v_max) ? v : v_max) : v_min;
}

float silenceValue(float v, float s) {
  if(v < 0) {
    return abs(v) > s ? v + s : 0.0f;
  }
  return abs(v) > s ? v - s : 0.0f;  
}

void logByteBuffer(String title, uint8_t len) {
  DPRINTLN(title);
  logByteBuffer(len);
}

void logByteBuffer(uint8_t len) {
  DPRINT("HEX: ");
  for(int i = 0; i < len; i++)
  {
    if( i > 0 ) {
      DPRINT(':');
    }
    DPRINT(TechnicHubMessageBuilder::buffer[i], HEX);
    if( i >= len - 1) {
      DPRINTLN();
    }
  }
}
