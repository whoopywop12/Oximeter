#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <ArduinoBLE.h>

BLEService oximeterService("4a44b2f1-5273-4272-a288-756eaf0f7c3d");  // Custom Oximeter service
BLEIntCharacteristic oximeterCharacteristic("b6431f64-5f8e-4c53-a9de-4a63d9a0c8b7", BLERead | BLENotify);

MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
uint16_t irBuffer[100];   //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100];   //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

int32_t bufferLength = 100;  //data length
int32_t spo2;                //SPO2 value
int8_t validSPO2;            //indicator to show if the SPO2 calculation is valid
int32_t heartRate;           //heart rate value
int8_t validHeartRate;       //indicator to show if the heart rate calculation is valid
byte readLED = 13;           //Blinks with each data read

void reportMetrics() {
  if (validSPO2) {
    Serial.print(F("SPO2: "));
    Serial.println(spo2, DEC);

    BLEDevice central = BLE.central();
    if (central){
      Serial.print("Connected to central: ");
      Serial.println(central.address());
      oximeterCharacteristic.writeValue((byte) spo2);
    } else {
      Serial.println("Bluetooth device active, waiting for connections...");
      Serial.print("Device's Bluetooth Address: ");
      Serial.println(BLE.address());
    }
  }
}

void setup() {
  // Initialize Serial Communication
  Serial.begin(115200); 
  pinMode(readLED, OUTPUT);

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1)
      ;
  }

  BLE.setLocalName("Oximeter");
  BLE.setAdvertisedService(oximeterService);
  oximeterService.addCharacteristic(oximeterCharacteristic);
  BLE.addService(oximeterService);
  oximeterCharacteristic.writeValue(0);
  BLE.advertise();

  // Initialize Oximeter Sensor
  while (!particleSensor.begin(Wire, I2C_SPEED_FAST))  //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    delay(1000);
  }

  Serial.println(F("MAX30105 available!"));
  Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion!"));
  while (Serial.available() == 0) {
    Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion!"));
  }
  Serial.read();

  byte ledBrightness = 60;                                                                        //Options: 0=Off to 255=50mA
  byte sampleAverage = 4;                                                                         //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2;                                                                               //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100;                                                                          //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411;                                                                           //Options: 69, 118, 215, 411
  int adcRange = 4096;                                                                            //Options: 2048, 4096, 8192, 16384
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);  //Configure sensor with these settings

  Serial.println("Calibrating oximeter...");
  //read the first 100 samples, and determine the signal range
  for (byte i = 0; i < bufferLength; i++) {
    while (particleSensor.available() == false)  //do we have new data?
      particleSensor.check();                    //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();  //We're finished with this sample so move to next sample
  }
  Serial.println("Done Calibrating!");
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
}

void loop() {
  BLEDevice central = BLE.central();
  if (central){
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    while (central.connected()){
      for (byte i = 25; i < 100; i++) {
        redBuffer[i - 25] = redBuffer[i];
        irBuffer[i - 25] = irBuffer[i];
      }

      //take 25 sets of samples before calculating the heart rate.
      for (byte i = 75; i < 100; i++) {
        while (particleSensor.available() == false)  //do we have new data?
          particleSensor.check();                    //Check the sensor for new data

        digitalWrite(readLED, !digitalRead(readLED));  //Blink onboard LED with every data read
        redBuffer[i] = particleSensor.getRed();
        irBuffer[i] = particleSensor.getIR();
        particleSensor.nextSample();  //We're finished with this sample so move to next sample
      }

      //After gathering 25 new samples recalculate HR and SP02
      maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
      if (validSPO2) {
        Serial.print(F("SPO2: "));
        Serial.println(spo2, DEC);
        oximeterCharacteristic.writeValue((byte) spo2);
      }
    }
  } else {
    Serial.println("Bluetooth device active, waiting for connections...");
    Serial.print("Device's Bluetooth Address: ");
    Serial.println(BLE.address());
  }
}
