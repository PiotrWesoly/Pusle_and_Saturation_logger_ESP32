/******************************************************************************************************
 *                                        Includes                                                    *
 *****************************************************************************************************/
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <string>

using namespace std;
/******************************************************************************************************
 *                                       Defines                                                      *
 *****************************************************************************************************/
#define SERVICE_UUID        BLEUUID((uint16_t)0x1822)
#define CHARACTERISTIC_UUID BLEUUID((uint16_t)0x2A5F) 
#define BUFFER_LENGTH       11100
#define MAX_BRIGHTNESS      255

/******************************************************************************************************
 *                                      Typedefs                                                      *
 *****************************************************************************************************/
typedef struct Readings
{
  uint8_t heartRate;
  uint8_t spO2;
  uint32_t time;
} Readings_struct;

/******************************************************************************************************
 *                                      Global variables                                              *
 *****************************************************************************************************/

/********** BLE variables ************/
bool deviceConnected = false;
BLECharacteristic readCharacteristic(
  BLEUUID((uint16_t)0x1004), 
  BLECharacteristic::PROPERTY_READ| 
  BLECharacteristic::PROPERTY_NOTIFY
);

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};
/********** Sensor variables ************/
uint16_t bufferIndex=0;
bool connectedFirst=true;

//Buffer allocation
Readings_struct readings[BUFFER_LENGTH];
//Readings_struct* readings = (Readings_struct*) malloc(150000 * sizeof(Readings_struct));

MAX30105 particleSensor;

uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid


/******************************************************************************************************
 *                                      Setup                                                          *
 *****************************************************************************************************/
void setup() {
  /********************* BLE setput ***********************/
  Serial.begin(115200);
  Serial.print("Starting BLE work!");

  // Create the BLE Device
  BLEDevice::init("Pulse and spO2 logger");

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pService->addCharacteristic(&readCharacteristic);

  pServer->getAdvertising()->addServiceUUID(SERVICE_UUID);

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();

    Serial.print("Finished bel work!");

  /************************* Initialize sensor ***********************/
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
  while (Serial.available() == 0) ; //wait until user presses a key
  Serial.read();

  /********************************   Setup configurations **********************************/
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
  
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
}

/******************************************************************************************************
 *                                      Loop                                                          *
 *****************************************************************************************************/
void loop()
{
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

  /***************** Read frist 100 samples for range estimation *********************/
  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  /***************** Continously read new samples (25/reading) *********************/
  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while (1)
  {
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample
    }

    //After gathering 25 new samples recalculate HR and SP02
     maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

      /****************************  Save readings to the buffer ***************************/
     if(heartRate != -999)
     {
     Serial.print(F(", HR="));
     Serial.print(heartRate, DEC);
     Serial.print(F(", spo2="));
     Serial.println(spo2, DEC);
     readings[bufferIndex].heartRate = (uint8_t) heartRate;
     readings[bufferIndex].spO2 = (uint8_t) spo2;
     readings[bufferIndex].time = millis();
     }
     else
     {
      Serial.println(F("wrong reading"));
     }

     /**************************** When connected to phone ********************************/
    if(deviceConnected)
    {
      if(connectedFirst == true)
      {
       for(uint32_t i =0; i<bufferIndex; i++)
       {
//         string heartRateMsg((char*)&readings[i].heartRate, 1);
//         string spO2Msg((char*)&readings[i].spO2, 1);
//         string timeMsg((char*)&readings[i].time, 4);
//         Serial.print(F(", BufferIndex="));
//         Serial.println(i, DEC);
//         Serial.print(F(", BT="));
//         Serial.println(readings[i].heartRate, DEC);

          

//          Msg = (string)readings[i].heartRate+','+(string)readings[i].spO2+','+(string)readings[i].time;
//          readCharacteristic.setValue(heartRateMsg);
//          readCharacteristic.setValue(",");
//          readCharacteristic.setValue(spO2Msg);
//          readCharacteristic.setValue(",");
//          readCharacteristic.setValue(timeMsg);
//          readCharacteristic.setValue(";");

          Serial.print(readings[i].heartRate, DEC);
          Serial.print(F(","));
          Serial.print(readings[i].spO2, DEC);
          Serial.print(F(","));
          Serial.print(readings[i].time, DEC);
          Serial.println(F(";"));
          
         
         readCharacteristic.setValue( (uint8_t*)&readings[i].heartRate, 1);
         readCharacteristic.setValue( (uint8_t*)&readings[i].spO2, 1);
         readCharacteristic.setValue( (uint8_t*)&readings[i].time, 4);
       }
       connectedFirst = false;
      }
      else
      {
        readCharacteristic.setValue( (uint8_t*)&readings[bufferIndex].heartRate, 1);
        readCharacteristic.setValue( (uint8_t*)&readings[i].spO2, 1);
        readCharacteristic.setValue( (uint8_t*)&readings[i].time, 4);
      }
    }
     bufferIndex++;
     if(bufferIndex >= BUFFER_LENGTH) bufferIndex = 0;
  }
}
