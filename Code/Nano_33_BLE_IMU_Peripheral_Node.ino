//Mark Frost, Oregon State University Intelligent Machines and Materials Lab, Summer 2023
//Part of The Tree Sensorization Data Collection Suite
//IMU Data Collection and BLE Transmission Node
//Inspired by https://github.com/little-scale/arduino-sketches/blob/master/BLE_IMU.ino
//Derived from examples/ArduinoBLE/Central/LedControl
//Version 3 July 27, 2023
//Central Device: Arduino Nano RP2040 Connect
//Peripheral Device: Arduino Nano 33 BLE
//IMU: LSM9DS1
//    DataSheet: https://www.st.com/resource/en/datasheet/lsm9ds1.pdf
//    Standard Specification Cheat Sheet:
//      Accelerometer range is set at ±4 g with a resolution of 0.122 mg. -> converted to m/s^2 on peripheral
//      Gyroscope range is set at ±2000 dps with a resolution of 70 mdps.
//      Magnetometer range is set at ±400 uT with a resolution of 0.014 uT.
//      Accelerometer and gyrospcope output data rate is fixed at 119 Hz.
//      Magnetometer output data rate is fixed at 20 Hz.

//BEGINING OF PERIPHERAL SCRIPT

//Libraries for BLE and IMU
#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>
#include <MadgwickAHRS.h>

// Madgwick
Madgwick filter;
// sensor's sample rate is fixed at 119 Hz:
const float sensorRate = 119;

//Establishment of variables for axis
//Acceleration Axis
float Ax, Ay, Az;
//Gyroscope Axis
float Gx, Gy, Gz;
//Magnetometer Axis
float Mx, My, Mz;
//Quaternion derived Orientation Variables
float roll, pitch, heading;

//Establishment of BLE Services
BLEService AccelService("19A10010-E8F2-537E-4F6C-D104768A1214");
BLEService GyroService("20A10010-E8F2-537E-4F6C-D104768A1214");
BLEService MagService("30A10010-E8F2-537E-4F6C-D104768A1214");
BLEService OrientService("40A10010-E8F2-537E-4F6C-D104768A1214");

//Acceleration characteristic to send byte array of values
BLECharacteristic acceleration("19A10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 12);
//Gyroscope characteristic to send byte array of values
BLECharacteristic gyroscope("20A10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 12);
//Magnetometer characteristic to send byte array of values
BLECharacteristic magnetometer("30A10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 12);
//Orientation characteristic to send byte array of values
BLECharacteristic orientation("40A10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 12);

void setup() {
  //Serial baud rate set to be the same as default micro-ROS environment for eventual porting 
  Serial.begin(115200);

  //Failure print statements for debugging
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  if (!BLE.begin()) {
    Serial.println("Starting Bluetooth® Low Energy module failed!");
    while (1);
  }

  //Begins the Madgwick processing filter for orientations
  filter.begin(sensorRate);

  //Local peripheral name is defined
  //CHANGE # FOR EACH BOARD AND UPLOAD
  BLE.setLocalName("IMUPeripheral1");

  BLE.setConnectionInterval(6, 6);

  //Sets the advertised service UUIDs for the specific services
  BLE.setAdvertisedService(AccelService);
  BLE.setAdvertisedService(GyroService);
  BLE.setAdvertisedService(MagService);
  BLE.setAdvertisedService(OrientService);

  //Adds characteristics to the services
  AccelService.addCharacteristic(acceleration);

  GyroService.addCharacteristic(gyroscope);

  MagService.addCharacteristic(magnetometer);

  OrientService.addCharacteristic(orientation);

  //Adds services the peripheral provides
  BLE.addService(AccelService);
  BLE.addService(GyroService);
  BLE.addService(MagService);
  BLE.addService(OrientService);

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
      filter.update(Gx, Gy, Gz, Ax, Ay, Az, -Mx, My, Mz); //for all 3
      roll = filter.getRoll();
      pitch = filter.getPitch();
      heading = filter.getYaw();

      float aData[3];
      aData[0] = Ax*9.81; //conversion from G's to m/s^2
      aData[1] = Ay*9.81;
      aData[2] = Az*9.81;

      float gData[3];
      gData[0] = Gx;
      gData[1] = Gy;
      gData[2] = Gz;

      float mData[3];
      mData[0] = Mx;
      mData[1] = My;
      mData[2] = Mz;

      float oData[3];
      oData[0] = heading;
      oData[1] = pitch;
      oData[2] = roll;

      // Write sensor values to service characteristics
      acceleration.writeValue((byte *) &aData, 12);
      
      gyroscope.writeValue((byte *) &gData, 12);

      magnetometer.writeValue((byte *) &mData, 12);

      orientation.writeValue((byte *) &oData, 12);
    }
  }
}
