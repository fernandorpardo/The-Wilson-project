### Gyroscope module
This is the SW of the Gyroscope module of the vehicle that is part of [the Wilson project](https://www.iambobot.com)

### DESCRIPTION
Wilson's GYRO module is an Arduino Nano project for controlling a GY-521. The module reports the Z axis value through the SPI interface to provide the vehicle with the orientation.

The original code is from  the [MPU6050 Arduino Library](https://www.arduinolibraries.info/libraries/mpu6050).

From there you can download the zip file ‘MPU6050-0.0.2’ and create the Arduino project taking the following 6 files from src

-	src\helper_3dmath.h
-	src\I2Cdev.cpp
-	src\I2Cdev.h
-	src\MPU6050.cpp
-	src\MPU6050.h
- src\PU6050_6Axis_MotionApps20.h

And adding the following 2
- led.h
- version.h
- wilson.h

The project is based on the example
- examples\MPU6050_DMP6\MPU6050_DMP6.ino

that I modified to add an API on the SPI interface.

Files from other projects are also published here for completeness.

### REFERENCES
Based in the work of
- [MPU6050 by Electronic Cats](https://github.com/ElectronicCats/mpu6050).  
- I2C library for the [MPU6050](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050) by Jeff Rowberg.
