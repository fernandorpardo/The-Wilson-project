/** ---------------------------------------------------------------------
 *  WILSON GYRO module
 *  Part of the Wilson project (www.iambobot.com)
 *  Fernando R
 *  
 *  GY-521 original code is at
 *  https://www.arduinolibraries.info/libraries/mpu6050
 *  in the downloadable file
 *  MPU6050-0.0.2.zip
 *  
 *  or from GitHub at
 *  https://github.com/ElectronicCats/mpu6050
 *  
 *  MPU6050 project is originally code by Jeff Rowberg <jeff@rowberg.net>
 *    https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
 *    
 *  From MPU6050-0.0.2.zip the Wilson GYRO module takes
 *    src\helper_3dmath.h
 *    src\I2Cdev.cpp
 *    src\I2Cdev.h
 *    src\MPU6050.cpp
 *    src\MPU6050.h
 *    src\PU6050_6Axis_MotionApps20.h
 *    
 * and this file that result from changes to   
 *    examples\MPU6050_DMP6\MPU6050_DMP6.ino
 *    
 *  Wilson's GYRO module is an Arduino Nano controlling a GY-521 via I2C 
 *  the module exposes an API through SPI
 *  
 *  Version: see version.h
 *  ---------------------------------------------------------------------
 **/

// ** WILSON
#include <SPI.h>
#include "led.h"
#include "version.h"
#include "wilson.h"
#define I2CDEV_IMPLEMENTATION I2CDEV_ARDUINO_WIRE
//#define DEBUG_Z
//#define DEBUG_SPI
// ** (END)

 
 // I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2019-07-08 - Added Auto Calibration and offset generator
//		   - and altered FIFO retrieval sequence to avoid using blocking code
//      2016-04-18 - Eliminated a potential infinite loop
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */



// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
// ** WILSON - commented
//#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT


#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
// ** WILSON
//#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
//bool blinkState = false;
// ** (END)

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
// ** WILSON
    // LED lights with SPI activity
    pinMode(PIN_SPI_LED, OUTPUT);
    digitalWrite(PIN_SPI_LED, LOW);

    // Configure SPI
    pinMode(MISO, OUTPUT); 
    pinMode(MOSI, INPUT);   
    SPCR |= _BV(SPE); // turn on SPI in slave mode
    
    rx_i = 0;
    SPI_comm_available = false;
    led_comm_on=  false;
    count_fail= 0;
    
    // Registers - information is provided by means of 3 registers
    // [1] data registers
    memset(&reg_gyro_data, 0, sizeof(reg_gyro_data));
    snprintf(reg_gyro_data.id, 4, "%s", command_DAT);
    reg_gyro_data.id[3]='\0';
    // [2] quadternion registers
    memset(&reg_gyro_quad, 0, sizeof(reg_gyro_quad));
    snprintf(reg_gyro_quad.id, 4, "%s", command_QUAD);
    reg_gyro_quad.id[3]='\0';
    
    // status & module information
    memset(&reg_gyro_status, 0, sizeof(reg_gyro_status));
    snprintf(reg_gyro_status.id, 4, "%s", command_STA);
    reg_gyro_status.id[3]='\0';
    reg_gyro_status.is_ready= false;
    reg_gyro_status.is_error= false;

    // name & version
    // date
    char day[16];
    char month[16];
    char year[16];
    char hour[16];
    char min[16];
    char sec[16];
    char time[32];
    // date
    sscanf(__DATE__, "%s %s %s", month, day, year);
    // time
    snprintf(time, sizeof(time),"%s",__TIME__);
    for(int i=0; i<(int)strlen(time); i++) if(time[i]==(char)':') time[i]=' ';
    sscanf(time, "%s %s %s", hour, min, sec);  
    snprintf(reg_gyro_status.name, 16, "%s", MODULE_NAME);
//    snprintf(reg_gyro_status.version, 16, "%s-%s%s-%02d%02d", MODULE_VERSION, day, month, atoi(hour), atoi(min));
    snprintf(reg_gyro_status.version, 16, "%s-%s%s-%s%s", MODULE_VERSION, day, month, hour, min);
       
 //   memset(&reg_gyro_test, 0, sizeof(reg_gyro_test));
 //   snprintf(reg_gyro_test.pattern, sizeof(reg_gyro_test), "MODULE %s v%s", MODULE_NAME, MODULE_VERSION);

    SPI.attachInterrupt(); // turn on interrupt 
// ** (END)    
  
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
// ** WILSON    
    Serial.begin(SERIAL_BAUD_RATE);
    delay(800);
    Serial.println(MODULE_NAME " v" MODULE_VERSION " (" __DATE__ " " __TIME__ ")");
    //snprintf(str, sizeof(str), "%s-%s-%s", hour, min, sec);
    //Serial.println(str);
// ** (END)

// ** WILSON - comment    
//    Serial.begin(115200);
 //   while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
// ** WILSON - commented
/*    
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
*/
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    //pinMode(LED_PIN, OUTPUT);
// ** WILSON
    leds.create(PIN_LED_TRES);
    leds.set_idle();
    
    pinMode(PIN_Z_ED, OUTPUT);
    digitalWrite(PIN_Z_ED, LOW);

    time_gyro_led= micros();
    led_on= false;
    time_gyro_to_change= TIME_GYRO_LED_PERIOD / 2;
    z_degrees= 0;
    
    fatal_error= false;
    is_working_fine= false;    
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
// ** WILSON
  
    // if programming failed, don't try to do anything
    if (!dmpReady && !fatal_error) 
    {
      fatal_error= true;
      Serial.print(F("FATAL ERROR -  dmpReady\n"));
      // return;
    }
// ** (END)

// ** WILSON - commented  
    // if programming failed, don't try to do anything
//    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .

        // ** WILSON - loop taks come here  
        // manage the 3 LED
        // [1-TASK] LED UNO show that the board is alive (sw i coming through this point
        leds.loop();
        
        // [2-TASK] LED DOS is direction led: brighter as closer to zero degress and dimming as moving towards +-180       
        if ((micros() - time_gyro_led) / 1000 > time_gyro_to_change) // mili
        {
            led_on= !led_on;
            digitalWrite(PIN_Z_ED, led_on? HIGH : LOW); 
            if(!fatal_error)
            {
                unsigned long deg= (unsigned long)z_degrees;
                if (deg>1800) deg= 1800;
                if(led_on)  time_gyro_to_change=  ((1800 - deg) * TIME_GYRO_LED_PERIOD) / 1800 ;
                else        time_gyro_to_change= (8 * deg * TIME_GYRO_LED_PERIOD) / 1800;
            }
            else 
                time_gyro_to_change= 1000;
            time_gyro_led= micros();
        }    
        // [3-TASK] LED TRES is SPI LED that toggle when command is received
        if (SPI_comm_available) 
        {
            #ifdef DEBUG_SPI   
            snprintf(str, sizeof(str),"dec= %s%u.%u flo=", (zf>=0)?" ":"-", z_degrees/10, z_degrees - 10 * (z_degrees/10));
            Serial.print(str);
            Serial.print(zf);
            Serial.print("\n");
            #endif
            SPI_comm_available = false;
            digitalWrite(PIN_SPI_LED, (led_comm_on= !led_comm_on)? LOW : HIGH);
        }
        // ** (END)
    }

// ** WILSON
    if(!is_working_fine)
    {
      is_working_fine= true;
      Serial.print("OK - got first interrupt\n");  
      if(!fatal_error) Serial.print("ALL GOOD!!! Module is working fine\n");      
    }
// ** (END)
    
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
	if(fifoCount < packetSize){
	        //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
			// This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
	}
    // check for overflow (this should never happen unless our code is too inefficient)
    else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
      //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {

        // read a packet from FIFO
	while(fifoCount >= packetSize){ // Lets catch up to NOW, someone is using the dreaded delay()!
		mpu.getFIFOBytes(fifoBuffer, packetSize);
		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;
	}

// ** WILSON
        // At this point we have in fifoBuffer the reading from
        // FIFO_R_W reg 0x74 (116)
        // The contents of the sensor data registers (Registers 59 to 96) are written into the FIFO buffer
        //  0   ACCEL_XOUT_H     0x3B
        //  1   ACCEL_XOUT_L     0x3C
        //  2   ACCEL_YOUT_H     0x3D
        //  3   ACCEL_YOUT_L     0x3E
        //  4   ACCEL_ZOUT_H     0x3F
        //  5   ACCEL_ZOUT_L     0x40
        //  6   TEMP_OUT_H       0x41
        //  7   TEMP_OUT_L       0x42
        //  8   GYRO_XOUT_H      0x43
        //  9   GYRO_XOUT_L      0x44
        //  10  GYRO_YOUT_H      0x45
        //  11  GYRO_YOUT_L      0x46
        //  12  GYRO_ZOUT_H      0x47
        //  13  GYRO_ZOUT_L      0x48
        //  14..37  EXT_SENS_DATA_00..23
// ** (END)
        
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

// ** WILSON - commented
/*
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
*/        
// ** (END)

// ** WILSON
        // ------------------------------------------------------------------------
        // DATA
        // [1] QUATERNIONS
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        reg_gyro_quad.w= (int16_t) (q.w * 1000.0);
        reg_gyro_quad.x= (int16_t) (q.x * 1000.0);
        reg_gyro_quad.y= (int16_t) (q.y * 1000.0);
        reg_gyro_quad.z= (int16_t) (q.z * 1000.0);
        
        // [2] GYRO
        //mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        // yaw: (about Z axis)
        zf= ypr[0] * 180/M_PI;
        // pitch: (nose up/down, about Y axis)
        yf= ypr[1] * 180/M_PI;
        // roll: (tilt left/right, about X axis)
        xf= ypr[2] * 180/M_PI; 
        // int16_t format
        reg_gyro_data.x= (int16_t) (xf * 10.0);       
        reg_gyro_data.y= (int16_t) (yf * 10.0);
        reg_gyro_data.z= (int16_t) (zf * 10.0);
        
        // [3] Acceleration
        // display real acceleration, adjusted to remove gravity
        //mpu.dmpGetQuaternion(&q, fifoBuffer);     
        mpu.dmpGetAccel(&aa, fifoBuffer);
        //mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        reg_gyro_data.ax= (int16_t) (aaReal.x);       
        reg_gyro_data.ay= (int16_t) (aaReal.y);
        reg_gyro_data.az= (int16_t) (aaReal.z);


        // GYRO ---------------------------------------------------------       
        // zf is the float value of GY-521 z-axis in degress
        // z_degrees is the integer from for the Z-axis value from the GY-521, multiplied x 10. Used for the LEDS and debugging    
        z_degrees= (unsigned int) ((zf>=0)? 10* zf : -10 * zf);

#ifdef DEBUG_Z
        snprintf(str, sizeof(str),"dg:%s%u.%u ", (zf>=0)?" ":"-", z_degrees/10, z_degrees - 10 * (z_degrees/10));
        Serial.print(str);
        Serial.print(zf);
        Serial.print("\n");
#endif
// ** (END)
    }
}
