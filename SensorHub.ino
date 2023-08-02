/*
 * SensorHub
 * 
 * Version: 1.0
 * Creator: Sebastian Ehnert
 * Date: 16.11.2022
 * 
 * Description:
 * Collect sensor data from three ultra sonic sensors (left/middle/right) and one radar sensor. 
 * The Host can retrieve the sensor data over a SPI protocol.
 */

#include "pins_arduino.h"
#include <afstandssensor.h> 
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>

#define ULTRASONIC_LEFT_TRIGGER   8
#define ULTRASONIC_LEFT_ECHO      9
#define ULTRASONIC_MIDDLE_TRIGGER 6
#define ULTRASONIC_MIDDLE_ECHO    7
#define ULTRASONIC_RIGHT_TRIGGER  4
#define ULTRASONIC_RIGHT_ECHO     5

#define TEMP_SENSOR               PD2

#define RADAR_SENSOR              PD3

// Commands:
#define READY_REPLY         0x00
#define GET_SENSOR_DATA     0x01
#define GET_ALL_SENSOR_DATA 0x02
#define CALIBRATE_DEVICE    0x03

// Devices:
#define ULTRASONIC_SENSOR_LEFT    0x00
#define ULTRASONIC_SENSOR_MIDDLE  0x01
#define ULTRASONIC_SENSOR_RIGHT   0x02
#define RADAR_SENSOR              0x03
#define All_SENSORS               0x04

// ReturnStates:
#define RETURN_SUCCESS  0x01
#define RETURN_ERROR    0x02

AfstandsSensor ultrasonicSensorLeft(ULTRASONIC_LEFT_TRIGGER, ULTRASONIC_LEFT_ECHO); 
AfstandsSensor ultrasonicSensorMiddle(ULTRASONIC_MIDDLE_TRIGGER, ULTRASONIC_MIDDLE_ECHO); 
AfstandsSensor ultrasonicSensorRight(ULTRASONIC_RIGHT_TRIGGER, ULTRASONIC_RIGHT_ECHO); 

double distanceLeft = 0;
double distanceMiddle = 0;
double distanceRight = 0;

OneWire oneWire(TEMP_SENSOR);
DallasTemperature sensors(&oneWire);

float temperature = 0.0;

int motionDetected = 0;

// SPI Communication
char spi_receive_buffer[30];
char spi_send_buffer[30];
volatile byte index;
volatile bool process_spi_command;

void setup() 
{
  // Default Values
  memset(spi_receive_buffer, 0, sizeof(spi_receive_buffer));
  memset(spi_send_buffer, 0, sizeof(spi_send_buffer));
  index = 0;
  process_spi_command = false;
  
  // SPI Slave
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE); // slave mode
  SPCR |= _BV(SPIE); // enable interrupts

  // Radar Sensor
  pinMode (RADAR_SENSOR, INPUT);

  // Temperature Sensor
  sensors.begin();

  // Test UART
  Serial.begin(9600);
}

ISR (SPI_STC_vect)
{
  byte c = SPDR; // read a byte from SPI register

  if(process_spi_command == false)
  {
    if (index < sizeof spi_receive_buffer) 
    {
      spi_receive_buffer[index] = c; 
      if (c == 0xFF) 
      {
        process_spi_command = true;
      }
    }

    c = spi_send_buffer[index];

    index++;
  }
}

void processSpiCommand()
{
  byte command = spi_receive_buffer[0];
  byte device = spi_receive_buffer[1];

  switch(command)
  {
    case READY_REPLY:
    {
      memset(spi_send_buffer, 0, sizeof(spi_send_buffer));
      break;      
    }
    case GET_SENSOR_DATA:
    {
      byte value = 0;  
      
      switch(device)
      {
        case ULTRASONIC_SENSOR_LEFT:
        {
          value = (byte)distanceLeft;
          break;
        }
        case ULTRASONIC_SENSOR_MIDDLE:
        {
          value = (byte)distanceMiddle;
          break;
        }
        case ULTRASONIC_SENSOR_RIGHT:
        {
          value = (byte)distanceRight;
          break;
        }
        default:
        {
          Serial.println("Wrong Device: " + String(device));
          break;
        }
      }

      spi_receive_buffer[0] = RETURN_SUCCESS;
      spi_receive_buffer[1] = device;
      spi_receive_buffer[2] = value;  

      break;      
    }
    case GET_ALL_SENSOR_DATA:
    {
      spi_receive_buffer[0] = RETURN_SUCCESS;
      spi_receive_buffer[1] = ULTRASONIC_SENSOR_LEFT;
      spi_receive_buffer[2] = (byte)distanceLeft;
      spi_receive_buffer[3] = ULTRASONIC_SENSOR_MIDDLE;
      spi_receive_buffer[4] = (byte)distanceMiddle;
      spi_receive_buffer[5] = ULTRASONIC_SENSOR_RIGHT;
      spi_receive_buffer[6] = (byte)distanceRight;
      spi_receive_buffer[7] = RADAR_SENSOR;
      spi_receive_buffer[8] = (byte)motionDetected;
      break;      
    }
    case CALIBRATE_DEVICE:
    {
      spi_receive_buffer[0] = RETURN_SUCCESS;
      spi_receive_buffer[1] = device;
      break;      
    }
    default:
    {
      Serial.println("Wrong Command: " + String(command));
      break;
    }
  }
}

void loop() 
{
  motionDetected = digitalRead(RADAR_SENSOR);
  
  sensors.requestTemperatures();
  temperature = sensors.getTempCByIndex(0);

  distanceLeft = ultrasonicSensorLeft.afstandCM(temperature);
  distanceMiddle = ultrasonicSensorMiddle.afstandCM(temperature);
  distanceRight = ultrasonicSensorRight.afstandCM(temperature);

  if(process_spi_command == true)
  {
    processSpiCommand();

    process_spi_command = false;
    index = 0;
  }

  // Test 
  Serial.println("Distance Left: " + String(distanceLeft) + "cm Temperature: " + temperature + "°C");
  Serial.println("Distance Middle: " + String(distanceMiddle) + "cm Temperature: " + temperature + "°C");
  Serial.println("Distance Right: " + String(distanceRight) + "cm Temperature: " + temperature + "°C");
  Serial.println("Motion Detected: " + String(motionDetected));

  delay(500);
}
