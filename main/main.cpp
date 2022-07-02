/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "Arduino.h"
#include "nvs_flash.h" 
#include "SHT3x.h"
#include "BLEDescriptor.h"
#include "BLECharacteristic.h"
#include "BLEDevice.h"
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include "MPU9250.h"
#include <SparkFunMPU9250-DMP.h>

MPU9250_DMP imu;
//BLE server name
#define bleServerName "BME280_ESP32"

unsigned long stepCount = 0;
unsigned long stepTime = 0;
unsigned long lastStepCount = 0;
float temp;
float tempF;
float hum;
#define SDA_PIN 27
#define SCLK_PIN 26
// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 3000;
SHT3x shtTempSensor = SHT3x();
MPU9250 mpu = MPU9250();

bool deviceConnected = false;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID "91bad492-b950-4226-aa2b-4ede9fa42f59"

// Temperature Characteristic and Descriptor
BLECharacteristic bmeTemperatureCelsiusCharacteristics("cba1d466-344c-4be3-ab3f-189f80dd7518", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor bmeTemperatureCelsiusDescriptor(BLEUUID((uint16_t)0x2902));

// Humidity Characteristic and Descriptor
BLECharacteristic bmeHumidityCharacteristics("ca73b3ba-39f6-4ab3-91ae-186dc9577d99", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor bmeHumidityDescriptor(BLEUUID((uint16_t)0x2903));

//Setup callbacks onConnect and onDisconnect
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
    };
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
    }
};

void initBME() {
    Wire.begin(SDA_PIN, SCLK_PIN);
    shtTempSensor.Begin();
}

void setup() {
    nvs_flash_init();
    // Start serial communication 
    Serial.begin(115200);
    // Init BME Sensor
    initBME();
    // Create the BLE Device
    BLEDevice::init(bleServerName);

    // Create the BLE Server
    BLEServer* pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    BLEService* bmeService = pServer->createService(SERVICE_UUID);

    // Create BLE Characteristics and Create a BLE Descriptor
    // Temperature
    bmeService->addCharacteristic(&bmeTemperatureCelsiusCharacteristics);
    bmeTemperatureCelsiusDescriptor.setValue("BME temperature Celsius");
    bmeTemperatureCelsiusCharacteristics.addDescriptor(&bmeTemperatureCelsiusDescriptor);

    // Humidity
    bmeService->addCharacteristic(&bmeHumidityCharacteristics);
    bmeHumidityDescriptor.setValue("BME humidity");
    bmeHumidityCharacteristics.addDescriptor(&bmeHumidityDescriptor);

    // Start the service
    bmeService->start();

    // Start advertising
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pServer->getAdvertising()->start();
    Serial.println("Waiting a client connection to notify...");
    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
    else
    {
        mpu.calibrateAccelGyro();
        delay(5000);
        mpu.calibrateMag();
        delay(5000);
    }
    if (imu.begin() != INV_SUCCESS)
    {
        while (1)
        {
            Serial.println("Check connections, and try again.");
            delay(5000);
        }
    }

    else
    {
        imu.dmpBegin(DMP_FEATURE_PEDOMETER);
        imu.dmpSetPedometerSteps(stepCount);
        imu.dmpSetPedometerTime(stepTime);
    }
}

void print_roll_pitch_yaw() {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu.getYaw(), 2);
    Serial.print(", ");
    Serial.print(mpu.getPitch(), 2);
    Serial.print(", ");
    Serial.println(mpu.getRoll(), 2);
}


void loop() {
    if ((millis() - lastTime) > timerDelay)
    {
        // Read temperature as Celsius (the default)
        shtTempSensor.UpdateData();
        temp = shtTempSensor.GetTemperature();
        // Read humidity
        hum = shtTempSensor.GetRelHumidity();
        //Notify temperature reading from BME sensor
        static char temperatureCTemp[6];
        dtostrf(temp, 6, 2, temperatureCTemp);
        //Set temperature Characteristic value and notify connected client
        bmeTemperatureCelsiusCharacteristics.setValue(temperatureCTemp);
        bmeTemperatureCelsiusCharacteristics.notify();
        Serial.print("Temperature Celsius: ");
        Serial.print(temp);
        Serial.print(" ºC");
        //Notify humidity reading from BME
        static char humidityTemp[6];
        dtostrf(hum, 6, 2, humidityTemp);
        //Set humidity Characteristic value and notify connected client
        bmeHumidityCharacteristics.setValue(humidityTemp);
        bmeHumidityCharacteristics.notify();
        Serial.print(" - Humidity: ");
        Serial.print(hum);
        Serial.println(" %");
        lastTime = millis();
    }
}

extern "C" void app_main(void)
{
    setup();
    while (1) {
        /*if (mpu.update()) {
            static uint32_t prev_ms = millis();
            if (millis() > prev_ms + 25) {
                print_roll_pitch_yaw();
                prev_ms = millis();
            }
        }*/
        loop();
    }
}
