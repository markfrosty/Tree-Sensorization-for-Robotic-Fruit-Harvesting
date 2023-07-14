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

//BEGINING OF PERIPHERAL SCRIPT

//Libraries for BLE and IMU
#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>

//Establishment of variables for axis
//Acceleration Axis
float Ax, Ay, Az;
//Gyroscope Axis
float Gx, Gy, Gz;
//Magnetometer Axis
float Mx, My, Mz;

//Establishment of BLE Services
BLEService AccelService("19B10010-E8F2-537E-4F6C-D104768A1214");  // create service
BLEService GyroService("20B10010-E8F2-537E-4F6C-D104768A1214");   // create service
BLEService MagService("30B10010-E8F2-537E-4F6C-D104768A1214");    // create service

//Acceleration characteristic to send values of type double
BLEDoubleCharacteristic accelX("19B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEDoubleCharacteristic accelY("19B10012-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEDoubleCharacteristic accelZ("19B10013-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);

//Gyroscope characteristic to send values of type double
BLEDoubleCharacteristic gyroX("20B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEDoubleCharacteristic gyroY("20B10012-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEDoubleCharacteristic gyroZ("20B10013-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);

//Magnetometer characteristic to send values of type double
BLEDoubleCharacteristic magX("30B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEDoubleCharacteristic magY("30B10012-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEDoubleCharacteristic magZ("30B10013-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);

void setup() {
  //Serial baud rate set to be the same as default micro-ROS environment for eventual porting 
  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  if (!BLE.begin()) {
    Serial.println("Starting Bluetooth® Low Energy module failed!");
    while (1);
  }

  //Local peripheral name is defined
  BLE.setLocalName("IMUPeripheralService");

  //Sets the advertised service UUIDs for the specific services
  BLE.setAdvertisedService(AccelService);
  BLE.setAdvertisedService(GyroService);
  BLE.setAdvertisedService(MagService);

  //Adds characteristics to the services
  //These were done for each axis for ease of data extraction
  AccelService.addCharacteristic(accelX);
  AccelService.addCharacteristic(accelY);
  AccelService.addCharacteristic(accelZ);

  GyroService.addCharacteristic(gyroX);
  GyroService.addCharacteristic(gyroY);
  GyroService.addCharacteristic(gyroZ);

  MagService.addCharacteristic(magX);
  MagService.addCharacteristic(magY);
  MagService.addCharacteristic(magZ);

  //Adds services the peripheral provides
  BLE.addService(AccelService);
  BLE.addService(GyroService);
  BLE.addService(MagService);

  //Starts advertising services
  BLE.advertise();

  Serial.println("Peripheral advertising...");
}

void loop() {
  BLEDevice central = BLE.central();

  //If a central is connected to the peripheral
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    //When central is connected data collection begins
    while (central.connected()) {
      IMU.readAcceleration(Ax, Ay, Az);
      IMU.readGyroscope(Gx, Gy, Gz);
      IMU.readMagneticField(Mx, My, Mz);

      //Float to Double conversion
      double accelXValue = Ax;
      double accelYValue = Ay;
      double accelZValue = Az;

      double gyroXValue = Gx;
      double gyroYValue = Gy;
      double gyroZValue = Gz;

      double magXValue = Mx;
      double magYValue = My;
      double magZValue = Mz;

      //Writes sensor values to service characteristics
      accelX.writeValue(accelXValue);
      accelY.writeValue(accelYValue);
      accelZ.writeValue(accelZValue);

      gyroX.writeValue(gyroXValue);
      gyroY.writeValue(gyroYValue);
      gyroZ.writeValue(gyroZValue);

      magX.writeValue(magXValue);
      magY.writeValue(magYValue);
      magZ.writeValue(magZValue);
    }
  }
}