//Mark Frost, Oregon State University Intelligent Machines and Materials Lab, Summer 2023
//Part of The Tree Sensorization Data Collection Suite
//IMU Data Collection and BLE Transmission Node
//Inspired by https://github.com/little-scale/arduino-sketches/blob/master/BLE_IMU.ino
//Derived from examples/ArduinoBLE/Central/LedControl
//Version 1 July 14, 2023
//Central Device: Arduino Nano RP2040 Connect
//Peripheral Device: Arduino Nano 33 BLE
//IMU: LSM9DS1
//    DataSheet: https://www.st.com/resource/en/datasheet/lsm9ds1.pdf
//    Standard Specification Cheat Sheet:
//      Accelerometer range is set at ±4 g with a resolution of 0.122 mg.
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
  while (!Serial)
    ;

  BLE.begin();
  Serial.println("BLE Central Node Running");

  //Sets the name of the central node
  BLE.setDeviceName("IMU Central Node Subscriber");

  //Set the local name the central node advertises
  BLE.setLocalName("IMUCentralService");

  //Start advertising as central node
  BLE.advertise();

  Serial.println("Waiting for connection...");

  //Scans for Acceleration, Gyroscope, and Magnetometer Service UUIDs
  BLE.scanForUuid("19B10010-E8F2-537E-4F6C-D104768A1214");
  BLE.scanForUuid("20B10010-E8F2-537E-4F6C-D104768A1214");
  BLE.scanForUuid("30B10010-E8F2-537E-4F6C-D104768A1214");
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

    //If the name doesn't have "IMUPeripheralService" it is ignored
    if (peripheral.localName().indexOf("IMUPeripheralService") < 0) {
      Serial.println("No 'IMUPeripheralService' in name");
      return;
    }
    BLE.stopScan();
    readPeripheral(peripheral);

    //Scans for Acceleration, Gyroscope, and Magnetometer Service UUIDs again
    BLE.scanForUuid("19B10010-E8F2-537E-4F6C-D104768A1214");
    BLE.scanForUuid("20B10010-E8F2-537E-4F6C-D104768A1214");
    BLE.scanForUuid("30B10010-E8F2-537E-4F6C-D104768A1214");
  }
}

void readPeripheral(BLEDevice peripheral) {
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
  if (peripheral.hasService("19B10010-E8F2-537E-4F6C-D104768A1214") && peripheral.hasService("20B10010-E8F2-537E-4F6C-D104768A1214") && peripheral.hasService("30B10010-E8F2-537E-4F6C-D104768A1214")) {
    Serial.println("Found Services!");
    BLECharacteristic accelX = peripheral.characteristic("19B10011-E8F2-537E-4F6C-D104768A1214");
    BLECharacteristic accelY = peripheral.characteristic("19B10012-E8F2-537E-4F6C-D104768A1214");
    BLECharacteristic accelZ = peripheral.characteristic("19B10013-E8F2-537E-4F6C-D104768A1214");

    BLECharacteristic gyroX = peripheral.characteristic("20B10011-E8F2-537E-4F6C-D104768A1214");
    BLECharacteristic gyroY = peripheral.characteristic("20B10012-E8F2-537E-4F6C-D104768A1214");
    BLECharacteristic gyroZ = peripheral.characteristic("20B10013-E8F2-537E-4F6C-D104768A1214");

    BLECharacteristic magX = peripheral.characteristic("30B10011-E8F2-537E-4F6C-D104768A1214");
    BLECharacteristic magY = peripheral.characteristic("30B10012-E8F2-537E-4F6C-D104768A1214");
    BLECharacteristic magZ = peripheral.characteristic("30B10013-E8F2-537E-4F6C-D104768A1214");

    //While the peripheral is connected and all services can be read, data is ingested and printed
    while (connected) {
      if (accelX.canRead() && accelY.canRead() && accelZ.canRead() && gyroX.canRead() && gyroY.canRead() && gyroZ.canRead() && magX.canRead() && magY.canRead() && magZ.canRead()) {

        Serial.println("IMU Data: ");
        Serial.print("\tAccelerometer data: ");
        Serial.println('\t');
        double accelXValue;
        accelX.readValue(&accelXValue, 8);
        Serial.print("\t\tAccel X: ");
        Serial.print(accelXValue);
        Serial.println('\t');

        double accelYValue;
        accelY.readValue(&accelYValue, 8);
        Serial.print("\t\tAccel Y: ");
        Serial.print(accelYValue);
        Serial.println('\t');

        double accelZValue;
        accelZ.readValue(&accelZValue, 8);
        Serial.print("\t\tAccel Z: ");
        Serial.print(accelZValue);
        Serial.println();

        Serial.print("\tGyroscope data: ");
        Serial.println('\t');
        double gyroXValue;
        gyroX.readValue(&gyroXValue, 8);
        Serial.print("\t\tGyro X: ");
        Serial.print(gyroXValue);
        Serial.println('\t');

        double gyroYValue;
        gyroY.readValue(&gyroYValue, 8);
        Serial.print("\t\tGyro Y: ");
        Serial.print(gyroYValue);
        Serial.println('\t');

        double gyroZValue;
        gyroZ.readValue(&gyroZValue, 8);
        Serial.print("\t\tGyro Z: ");
        Serial.print(gyroZValue);
        Serial.println();

        Serial.print("\tMagnetometer data: ");
        Serial.println('\t');
        double magXValue;
        magX.readValue(&magXValue, 8);
        Serial.print("\t\tMag X: ");
        Serial.print(magXValue);
        Serial.println('\t');

        double magYValue;
        magY.readValue(&magYValue, 8);
        Serial.print("\t\tMag Y: ");
        Serial.print(magYValue);
        Serial.println('\t');

        double magZValue;
        magZ.readValue(&magZValue, 8);
        Serial.print("\t\tMag Z: ");
        Serial.print(magZValue);
        Serial.println();
      }
    }
  } else {
    Serial.println("Attribute discovery failed!");
  }
  peripheral.disconnect();
  Serial.println("Peripheral disconnected");
  connected = false;
}