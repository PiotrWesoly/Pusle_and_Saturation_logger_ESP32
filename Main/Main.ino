/******************************************************************************************************
 *                                        Includes                                                    *
 *****************************************************************************************************/
#include <Wire.h>
#include "DFRobot_BloodOxygen_S.h"
#include "BluetoothSerial.h"
#include <string>

using namespace std;
/******************************************************************************************************
 *                                       Defines                                                      *
 *****************************************************************************************************/
#define BUFFER_LENGTH         7000
#define CURRENT_READING_MSG   1
#define HISTORY_MSG           2
#define SENT                  1
#define NOT_SENT              0
#define CONNECTED             49
#define NOT_CONNECTED         50
#define I2C_ADDRESS           0x57
#define WRONG_READING         -1

/******************************************************************************************************
 *                                      Typedefs                                                      *
 *****************************************************************************************************/
typedef struct Readings
{
  uint8_t heartRate;
  uint8_t spO2;
  uint32_t sampleTime;
} Readings_struct;

/******************************************************************************************************
 *                                      Global variables                                              *
 *****************************************************************************************************/
/********** Bluetooth **************/
BluetoothSerial SerialBT;
bool isHistorySent=false;
int incoming = 0; /* incoming message */

/********** Sensor ************/
DFRobot_BloodOxygen_S_I2C MAX30102(&Wire ,I2C_ADDRESS);

/********* Buffer ************/
uint16_t bufferIndex=0;
static int isSent[BUFFER_LENGTH] = {0};
Readings_struct readings[BUFFER_LENGTH];
uint32_t startTime;
uint32_t timeOfRecord;

/******************************************************************************************************
 *                                      Setup                                                         *
 *****************************************************************************************************/
void setup() {
  /************************* Initialize bluetooth ***********************/
  SerialBT.begin("Pulse and saturation logger");
  Serial.begin(115200);
  
  /************************* Initialize sensor ***********************/
  while (false == MAX30102.begin())
  {
    Serial.println("init fail!");
    delay(1000);
  }
  Serial.println("init success!");
  Serial.println("start measuring...");
  
  MAX30102.sensorStartCollect();

  timeOfRecord = 0;
  startTime = 0;
}

/******************************************************************************************************
 *                                      Loop                                                          *
 *****************************************************************************************************/
void loop()
{


  /****************************  Save readings to the buffer ***************************/

  MAX30102.getHeartbeatSPO2();
  
  if(MAX30102._sHeartbeatSPO2.Heartbeat != WRONG_READING)
  {
    if(bufferIndex == 0)
    {
      startTime = millis();
    }
      Serial.print(F("[VALUE]          HR="));
      Serial.print(MAX30102._sHeartbeatSPO2.Heartbeat, DEC);
      Serial.print(F(", spo2="));
      Serial.println(MAX30102._sHeartbeatSPO2.SPO2, DEC);
    
      readings[bufferIndex].heartRate = (uint8_t) MAX30102._sHeartbeatSPO2.Heartbeat;
      readings[bufferIndex].spO2 = (uint8_t) MAX30102._sHeartbeatSPO2.SPO2;
      readings[bufferIndex].sampleTime = millis() - startTime;
    }
    else
    {
      Serial.println(F("[VALUE]          wrong reading"));
    }
    delay(1000);
    
    /**************************** When connected to phone ********************************/   
    if(SerialBT.available())
    {
      incoming = SerialBT.read();
      Serial.print(F("[NEW INC]        new incoming="));
      Serial.println(incoming, DEC);
      Serial.print(F("[NEW INC]        isHistorySent="));
      Serial.println(isHistorySent, DEC);
    }

    if(incoming == CONNECTED)
    {
      uint8_t* pointer;
      uint8_t converter[4];
      uint32_t currentMillis = millis();
      if(isHistorySent == false)   /* On new connection send history first */
      {
        Serial.println("[CONN HIST]      History");
        for(uint32_t i=0; i<=bufferIndex; i++)
        {
          if(isSent[i] == NOT_SENT)
          {
            timeOfRecord = currentMillis - readings[i].sampleTime;
            pointer = (uint8_t*)&timeOfRecord;
            
            /* Endiannes swap */
            converter[0] = pointer[3];
            converter[1] = pointer[2];
            converter[2] = pointer[1];
            converter[3] = pointer[0];
            
            /* Print buffer to serial monitor */
            Serial.print("[CONN HIST]      Buffer: ");
            Serial.println(i, DEC);
      
            /* Send previous readings */
            SerialBT.write(readings[i].heartRate);
            SerialBT.write(readings[i].spO2);
            SerialBT.write(converter[0]);
            SerialBT.write(converter[1]);
            SerialBT.write(converter[2]);
            SerialBT.write(converter[3]);
            SerialBT.write((uint8_t)HISTORY_MSG);
    
            isSent[i] = SENT;
          }
        }
        isHistorySent = true;
      }
      else /* After receiving all history keep sending current reading */
      {
        Serial.println(F("[CONN CURR]      send current reading "));

        timeOfRecord = currentMillis - readings[bufferIndex].sampleTime;
        /* uint32 to uint8[4] converter */
        pointer = (uint8_t*)&timeOfRecord;
    
        /* Endiannes swap */
        converter[0] = pointer[3];
        converter[1] = pointer[2];
        converter[2] = pointer[1];
        converter[3] = pointer[0];
          
        SerialBT.write(readings[bufferIndex].heartRate);
        SerialBT.write(readings[bufferIndex].spO2);
        SerialBT.write(converter[0]);
        SerialBT.write(converter[1]);
        SerialBT.write(converter[2]);
        SerialBT.write(converter[3]);
        SerialBT.write((uint8_t)CURRENT_READING_MSG);
        isSent[bufferIndex] = SENT;
      }
    }
    else
    {
      Serial.print(F("[NOT CONN]       incoming="));
      Serial.println(incoming, DEC);
      isHistorySent = false;
    }

    /********************** Increment bufferIndex and check if it did not roll over **********************/   
    if(WRONG_READING != MAX30102._sHeartbeatSPO2.Heartbeat != WRONG_READING)
    {
      bufferIndex++;/* increment index */
    }
    
    /* If buffer rolled over */
    if(bufferIndex >= BUFFER_LENGTH) 
    {
      Serial.println("Buffer rolled over");
      bufferIndex = 0;
      for(uint32_t i=0; i<=BUFFER_LENGTH; i++)
      {
        if(isSent[i] == NOT_SENT)
        {
          readings[bufferIndex].heartRate = readings[i].heartRate;
          readings[bufferIndex].spO2 = readings[i].spO2;
          readings[bufferIndex].sampleTime = readings[i].sampleTime;
          bufferIndex++;
        }
      }
      /* Clear sent array */
      for(int i=0; i<=BUFFER_LENGTH;i++) isSent[i]=NOT_SENT;
    }
  }
