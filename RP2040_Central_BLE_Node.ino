//Mark Frost, Oregon State University Intelligent Machines and Materials Lab, Summer 2023
//Part of The Tree Sensorization Data Collection Suite
//IMU Data Recieving Node
//Inspired by https://github.com/little-scale/arduino-sketches/blob/master/BLE_IMU.ino
//Derived from examples/ArduinoBLE/Central/LedControl
//Version 4 August 2, 2023
//Central Device: Arduino Nano RP2040 Connect
//Peripheral Device: Arduino Nano 33 BLE
//This script only works for ONE peripheral. Can be used as a good testing and starting point
//IMU: LSM9DS1
//    DataSheet: https://www.st.com/resource/en/datasheet/lsm9ds1.pdf
//    Standard Specification Cheat Sheet:
//      Accelerometer range is set at ±4 g with a resolution of 0.122 mg. -> converted to m/s^2 on peripheral
//      Gyroscope range is set at ±2000 dps with a resolution of 70 mdps.
//      Magnetometer range is set at ±400 uT with a resolution of 0.014 uT.
//      Accelerometer and gyrospcope output data rate is fixed at 119 Hz.
//      Magnetometer output data rate is fixed at 20 Hz.

//BEGINING OF CENTRAL SCRIPT

//Library for BLE
#include <ArduinoBLE.h>

//Establishes flag for connection and continuous data collection
bool connected = false;

void setup() {
  //Serial baud rate set to be the same as default micro-ROS environment for eventual porting 
  Serial.begin(115200);

  BLE.begin();
  Serial.println("BLE Central Node Running");

  //Sets the name of the central node
  BLE.setDeviceName("IMU Central Node Subscriber");

  //Set the local name the central node advertises
  BLE.setLocalName("IMUCentralService");

  //Start advertising as central node
  BLE.advertise();

  Serial.println("Waiting for connection...");

  //Scans for Acceleration, Gyroscope, Magnetometer, and Orientation Service UUIDs
  BLE.scanForUuid("19A10010-E8F2-537E-4F6C-D104768A1214");
  BLE.scanForUuid("20A10010-E8F2-537E-4F6C-D104768A1214");
  BLE.scanForUuid("30A10010-E8F2-537E-4F6C-D104768A1214");
  BLE.scanForUuid("40A10010-E8F2-537E-4F6C-D104768A1214");
}

void loop() {
  BLEDevice peripheral = BLE.available();
  if (peripheral) {
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();

    //If the name doesn't have "IMUPeripheral1" it is ignored
    //Here you can change the peripheral number to match the peripheral you want to test
    if (peripheral.localName().indexOf("IMUPeripheral1") < 0) {
      Serial.println("No 'IMUPeripheral1' in name");
      return;
    }
    BLE.stopScan();
    readPeripheral(peripheral);

    //Scans for Acceleration, Gyroscope, Magnetometer, and Orientation Service UUIDs
    BLE.scanForUuid("19A10010-E8F2-537E-4F6C-D104768A1214");
    BLE.scanForUuid("20A10010-E8F2-537E-4F6C-D104768A1214");
    BLE.scanForUuid("30A10010-E8F2-537E-4F6C-D104768A1214");
    BLE.scanForUuid("40A10010-E8F2-537E-4F6C-D104768A1214");
  }
}

void readPeripheral(BLEDevice& peripheral) {
  if (peripheral.connect()) {
    Serial.println("Connected to peripheral");
    connected = true;
  } else {
    Serial.println("Connection Failed!");
  }

  if (peripheral.discoverAttributes()) {
    Serial.println("Attributes discovered");
  } else {
    Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }

  //If it finds all the desired IMU services it begins reading them
  if (peripheral.hasService("19A10010-E8F2-537E-4F6C-D104768A1214") && 
      peripheral.hasService("20A10010-E8F2-537E-4F6C-D104768A1214") && 
      peripheral.hasService("30A10010-E8F2-537E-4F6C-D104768A1214") && 
      peripheral.hasService("40A10010-E8F2-537E-4F6C-D104768A1214")) {
  
    Serial.println("Found Services!");
    BLECharacteristic acceleration = peripheral.characteristic("19A10011-E8F2-537E-4F6C-D104768A1214");

    BLECharacteristic gyroscope = peripheral.characteristic("20A10011-E8F2-537E-4F6C-D104768A1214");

    BLECharacteristic magnetometer = peripheral.characteristic("30A10011-E8F2-537E-4F6C-D104768A1214");

    BLECharacteristic orientation = peripheral.characteristic("40A10011-E8F2-537E-4F6C-D104768A1214");

    //While the peripheral is connected and all services can be read, data is ingested and printed
    while (connected) {
      if (acceleration.canRead() && gyroscope.canRead() && magnetometer.canRead() && orientation.canRead()) {

        Serial.println("IMU Data: ");
        Serial.print("\tAcceleration data: ");
        Serial.println('\t');
        float aData[3];
        acceleration.readValue(aData, 12);

        Serial.print("\t\tAccel X: ");
        Serial.print(aData[0]);
        Serial.println('\t');

        Serial.print("\t\tAccel Y: ");
        Serial.print(aData[1]);
        Serial.println('\t');

        Serial.print("\t\tAccel Z: ");
        Serial.print(aData[2]);
        Serial.println('\t');

        Serial.print("\tGyroscope data: ");
        Serial.println('\t');
        float gData[3];
        gyroscope.readValue(gData, 12);

        Serial.print("\t\tGyro X: ");
        Serial.print(gData[0]);
        Serial.println('\t');

        Serial.print("\t\tGyro Y: ");
        Serial.print(gData[1]);
        Serial.println('\t');

        Serial.print("\t\tGyro Z: ");
        Serial.print(gData[2]);
        Serial.println('\t');

        Serial.print("\tMagnetometer data: ");
        Serial.println('\t');
        float mData[3];
        magnetometer.readValue(mData, 12);

        Serial.print("\t\tMag X: ");
        Serial.print(mData[0]);
        Serial.println('\t');

        Serial.print("\t\tMag Y: ");
        Serial.print(mData[1]);
        Serial.println('\t');

        Serial.print("\t\tMag Z: ");
        Serial.print(mData[2]);
        Serial.println('\t');

        Serial.print("\tOrientation data: ");
        Serial.println('\t');
        float oData[3];
        orientation.readValue(oData, 12);

        Serial.print("\t\tHeading: ");
        Serial.print(oData[0]);
        Serial.println('\t');

        Serial.print("\t\tPitch: ");
        Serial.print(oData[1]);
        Serial.println('\t');

        Serial.print("\t\tRoll: ");
        Serial.print(oData[2]);
        Serial.println('\t');
      }
    }
  } else {
    Serial.println("Attribute discovery failed!");
  }
  peripheral.disconnect();
  Serial.println("Peripheral disconnected");
  connected = false;
}
