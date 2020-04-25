/** 
 *  WILSON SENSOR module
 *  Part of the Wilson project (www.iambobot.com)
 *  Fernando R
 *  
 *  HC-SR04 sensor class 
 *  
 **/
// -----------------------------------------------------------------------------
// ------- Ultrasonic sensor HC-SR04  ------------------------------------------
// -----------------------------------------------------------------------------
// No compiler optimizations
//#pragma GCC optimize ("-O0")

#include <Arduino.h>
#include "hcsr.h"

void pciEnable(byte pin);
//void ISR_HCSR(void);

HCSR::HCSR(bool poll_mode, void (* f)(int, uint16_t, void *)) 
{
    poll= poll_mode;
    dataready_callback= f;
    n_devices= 0;
    error= 0;
    for(int i=0; i<N_HCSR04_DEVICES; i++) ready[i]= false;
    for(int i=0; i<N_HCSR04_DEVICES; i++) duration[i]=0;
}

void HCSR::setup(uint8_t trigger, uint8_t echo, void *ptrled) 
{
    if(n_devices>=N_HCSR04_DEVICES) return;
    pin_trigger[n_devices] = trigger;
    pin_echo[n_devices] = echo;
    ready[n_devices]= false;
    duration[n_devices]=0;	
    ptr[n_devices]= ptrled;
    device_count= -1;	
    pinMode(trigger, OUTPUT); 	// Sets the pin_trigger as an Output
    pinMode(echo, INPUT); 		// Sets the pin_echo as an Input  
    // intp
    if(!poll) pciEnable(echo);
    //	attachInterrupt(digitalPinToInterrupt(echo), ISR_HCSR, RISING );
    //	attachInterrupt(0, ISR_HCSR, CHANGE );
     n_devices++;
}

void HCSR::intp(void)
{
	unsigned int v= digitalRead(pin_echo[device]); 
	if(v)
	{
		time = micros();
		duration[device]= 0;
	}
	else 
	{
		duration[device]= micros() - time;
		ready[device]= true;
	}
}

void HCSR::start_pulse(int d)
{ 	
	if(d >= n_devices) return;
	device= d;
	// Generate pulse
	// Clears the pin_trigger
	digitalWrite(pin_trigger[device], LOW);
	delayMicroseconds(2);
	// Sets the pin_trigger on HIGH state for 10 micro seconds
	digitalWrite(pin_trigger[device], HIGH);
	delayMicroseconds(10);
	digitalWrite(pin_trigger[device], LOW);
	duration[device]= 0;
	ready[device]= false; 
}

unsigned long HCSR::get_response(void)
{
	// (2) Read pulse
	// waits for the pin to go HIGH, starts timing, then waits for the pin to go LOW and stops timing. 
	// Returns the length of the pulse in microseconds or 0 if no complete pulse was received within the timeout. 
	// timeout (optional) is set to 50.000 microseconds, 50milisecinds
	duration[device] = pulseIn(pin_echo[device], HIGH, 50000); // length of the pulse in microseconds or = if timeout
       
	if(duration[device] != 0)
	{   
		ready[device]= true;
	}
	return duration[device];
}
unsigned int HCSR::distance(int d)
{
    // Calculating the distance
    return (unsigned int) ((double) duration[d] * 0.034/2);
}
// read all sensors one after the other
void HCSR::start_round_table(void (* callback)(void)) 
{
    callback_round_table= callback;
    device_count= 0;
    if(device_count < n_devices)
    {
        start_pulse(device_count);
    }
    else device_count= -1;
}

uint8_t HCSR::loop(void)
{ 
    bool is_anyone_ready= false;
    // data ready to be reported
    for(int d=0; d<N_HCSR04_DEVICES; d++)
    {
        if(ready[d]) 
        {
            ready[d]= false;    
            // Report
            if(dataready_callback) dataready_callback(d, (uint16_t) duration[d], ptr[d]);
            is_anyone_ready= true;
        }
    }

    if(device_count>=0 && is_anyone_ready)
    {
        device_count++;
        if(device_count < n_devices)
        {
            start_pulse(device_count);
        }
        else {
          // done all the 3 sensors
          device_count= -1;
          if(callback_round_table) callback_round_table();
        }
    }

    if(error)
    {
      error= 0;
  //    Serial.print("\nWarning overflow");
    }
    return 0;
}
/* END OF FILE */
