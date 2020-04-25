/** 
 *  WILSON SENSOR module
 *  Part of the Wilson project (www.iambobot.com)
 *  Fernando R
 *  
 *  Manage an array of 3 x infrared proxmitity sensors HC-SR04
 *  and Analog input to read battery level
 *  
 *  1.0 - March 2020 - first Wilson version
 *  
 **/

// No compiler optimizations
//#pragma GCC optimize ("-O0")

//#define ENABLE_DEBUG_ON_SERIAL
//#define ENABLE_DEBUG_IC2

#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>
#include "module.h"
#include "hcsr.h"
#include "led.h"
#include "WILSON_sensor.h"

// -----------------------------------------------------------------------------
// PIN-OUT definition
// -----------------------------------------------------------------------------
// Ultrasonic sensor HC-SR04
// RIGHT
#define PIN_HCSR_ECHO_RIGHT     A2 // A2 (intp)
#define PIN_HCSR_TRIGGER_RIGHT  2  // D2  
// FRONT
#define PIN_HCSR_ECHO_FRONT     4  // D4 (intp)    
#define PIN_HCSR_TRIGGER_FRONT  9  // D9
// LEFT
#define PIN_HCSR_ECHO_LEFT      8  // D8 (intp)
#define PIN_HCSR_TRIGGER_LEFT   7  // D7

// Analog input for battery level
#define PIN_BATTERY             A7

// I2C
// SDA - Pin A4
// SCL - Pin A5

// -----------------------------------------------------------------------------
// LED's
ledbucket leds;
// REGISTERS
REG_SENSOR_DATA_struct reg_sensor_meter; 
REG_SENSOR_STATUS_struct reg_sensor_status;
REG_SENSOR_MODULE_struct reg_module;

// -----------------------------------------------------------------------------
// ------- Battery          ----------------------------------------------------
// -----------------------------------------------------------------------------
#define BATTERY_METER_PERIOD 1000 // miliseconds  ( = 1 second) 
unsigned long time_BATTERY;

void battery_init(void)
{
    analogReference(DEFAULT); // the default analog reference of 5 volts (on 5V Arduino boards) or 3.3 volts (on 3.3V Arduino boards)
    time_BATTERY= micros();
}
void battery_read(void)
{
    reg_sensor_status.battery= analogRead(PIN_BATTERY);
}
void battery_loop(void)
{
    if( ((micros()-time_BATTERY) / 1000 > BATTERY_METER_PERIOD))
    {   
          battery_read();
          time_BATTERY= micros();
    }
}
// -----------------------------------------------------------------------------
// ------- SAMPLING -------- ---------------------------------------------------
// -----------------------------------------------------------------------------
// Reading activity
#define SENSOR_SAMPLE_PERIOD 200 // miliseconds  ( 1000 = 1 second)  
#define READING_INACTIVITY_TIMEOUT 30000 // miliseconds (30 seconds)
bool is_SAMPLING_active;
unsigned long time_SAMPLING_t0;
unsigned long time_samplig_repeat;

void SensorSampling_init(void)
{
    is_SAMPLING_active= true;
    time_samplig_repeat= time_SAMPLING_t0= micros();
}
// Go IDLE = stop sampling the sensors
// - after inactivity timeout (no message from MASTER)
// - "REG_SENSOR_ACTIVATE 0" received from MASTER
void SensorSampling_go_idle(void)
{
    is_SAMPLING_active= false;
    leds.set_idle();
    memset(&reg_sensor_meter, 0, sizeof(reg_sensor_meter));
    Serial.print("IDLE\n");
}
// Go ACTIVE 
// - "REG_SENSOR_ACTIVATE 1" received from MASTER
// - if IDLE and "REG_SENSOR_METER" received from MASTER
void SensorSampling_go_active(void)
{
    memset(&reg_sensor_meter, 0, sizeof(reg_sensor_meter));
    is_SAMPLING_active = true;
    time_SAMPLING_t0= micros();
    Serial.print("ACTIVE\n");
}
// Reset timeout each time a REG_SENSOR_METER request for meassures is recevied from the MASTER
// go ACTIVE if it was IDLE
void SensorSampling_refresh(void)
{
    if(!is_SAMPLING_active) SensorSampling_go_active();
    else time_SAMPLING_t0= micros();
}

// -----------------------------------------------------------------------------
// ------- Ultrasonic sensor HC-SR04  ------------------------------------------
// -----------------------------------------------------------------------------
// Called on each individual sensor reading completion (3 times per round)
void Callback_on_single_read_done (int device, uint16_t duration, void *pv)
{
    if(!(device<N_HCSR04_DEVICES)) return;
    reg_sensor_meter.duration[device]= duration;
    if(pv!=0 && is_SAMPLING_active) ((led *)pv)->asymmetry(duration>5000? ASYMMETRY_WHEN_IDLE : duration/200);
}

// Called on each round table - all the 3 sensors been read 
bool round_table_done= false;
void Callback_on_round_table_done (void)
{
    round_table_done= true;
}

HCSR hcsr(false, Callback_on_single_read_done);

// -----------------------------------------------------------------------------
// ------- I2C -----------------------------------------------------------------
// -----------------------------------------------------------------------------
// I2C pinout (NANO)
// GND - GND
// VCC - VCC
// SDA - Pin A4
// SCL - Pin A5
bool I2C_error;
unsigned char I2C_rxbuffer[32];
unsigned int I2C_rxi;
bool I2C_data_available;
byte I2C_READING_register;

void I2C_init(void)
{
    Wire.begin(I2C_ADDR);                     // join i2c bus with address I2C_ADDR
    Wire.onRequest(I2C_Request_EventHandler); // Data requested by master
    Wire.onReceive(I2C_Receive_EventHandler); // Data received from master
    I2C_READING_register= 0;
    I2C_data_available= false;
    // initialize registers
    memset(&reg_sensor_meter, 0, sizeof(reg_sensor_meter));
    memset(&reg_sensor_status, 0, sizeof(reg_sensor_status));
}

// This function executes whenever data is received from master
// this function is passed as a parameter to Wire.onRequest
// the MASTER send a 1 byte data message with the REGISTER number
// then the SLAVE (this code) responds back with the REGISTER content (a data structure)
void I2C_Request_EventHandler()
{
    if(I2C_READING_register)
    {
        switch(I2C_READING_register)
        {
            // sensor module's SPECIFIC registers
            case REG_SENSOR_METER:
                Wire.write((uint8_t*) &reg_sensor_meter, sizeof(reg_sensor_meter));
                SensorSampling_refresh();
            break;
            // COMMON registers to all modules
            case REG_MODULE_INFO:
                Wire.write((uint8_t*) &reg_module, sizeof(reg_module));
            break;            
            case REG_STATUS:
            case REG_SENSOR_BATTERY:
                uint16_t regbattery= reg_sensor_status.battery;
                Wire.write((uint8_t*) &regbattery, sizeof(regbattery));
            break;
            case REG_SERIAL_COUNT:
            break;  
            case REG_SERIAL_READ:
            break; 
        }
        I2C_READING_register= 0;
    }
    else
    {
        Wire.write((uint8_t*) &reg_sensor_meter, sizeof(reg_sensor_meter));
        SensorSampling_refresh();
    }
}

// this function executes whenever data is received from the MASTER
// this function is passed as a parameter to Wire.onReceive
void I2C_Receive_EventHandler(int numBytes)
{
    I2C_rxi= 0;
    I2C_error= false;
    while (Wire.available() && I2C_rxi < sizeof(I2C_rxbuffer))
        I2C_rxbuffer[I2C_rxi++] = Wire.read();
    I2C_READING_register= (I2C_rxi==1)? I2C_rxbuffer[0] : 0; 
    I2C_error= ( numBytes != (int)I2C_rxi );
    I2C_data_available= true;
    switch(I2C_rxbuffer[0])
    {
        case REG_SENSOR_ACTIVATE:
          if(numBytes==2) 
          {
              if(I2C_rxbuffer[1] == 1)  SensorSampling_go_active();
              else                      SensorSampling_go_idle();
          }
        break;
    }
}

// -----------------------------------------------------------------------------
// ------- INTERRUPTIONS -------------------------------------------------------
// -----------------------------------------------------------------------------
// PCI - Pin Change Interrupt Request - VECTORS
// https://sites.google.com/site/qeewiki/books/avr-guide/external-interrupts-on-the-atmega328
// http://ee-classes.usc.edu/ee459/library/documents/avr_intr_vectors/

// ARDUINO NANO
// attachInterrupt() on NANO only works for INTP0 and INTP1 (pins 2 and 3)
// hence the alternative is ISR()

volatile uint8_t port_B_history = 0xFF;     // default is high because the pull-up
volatile uint8_t port_C_history = 0xFF;     // default is high because the pull-up
volatile uint8_t port_D_history = 0xFF;     // default is high because the pull-up

// PCICR = Pin Change Interrupt Control Register
// PCIEx = Pin Change Interrupr Enable x=0..2
//     7     6     5     4     3      2       1       0
//  |  -  |  -  |  -  |  -  |  -  | PCIE2 | PCIE1 | PCIE0 |
void pciEnable(byte pin)
{
    port_B_history = 0xFF;
    port_C_history = 0xFF;
    port_D_history = 0xFF;
    pinMode(pin, INPUT);
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}
void pciDisable(byte pin)
{
    PCICR  &= ~bit (digitalPinToPCICRbit(pin));
}

// ATmega has three ports: 
//    B (digital inputs D8..D13 pins  8..13)
//    C (analog inputs  A0..A5  pins 14..19)
//    D (digital inputs D0..D7  pins  0..7) 
// Use one ISR routine to handle each group

// Port B - handle pin change interrupt for D8 to D13 here
// D8 - LEFT
ISR (PCINT0_vect) 
{    
    uint8_t r= PINB;
    volatile uint8_t changedbits =  r ^ port_B_history;
    port_B_history = r;
    // D8 changedbits= 0x01    
    if(changedbits & 0x01) hcsr.intp();
}
// Port C - handle pin change interrupt for A0 to A5 here 
// A2 - RIGHT
ISR (PCINT1_vect) 
{
    uint8_t r= PINC;
    volatile uint8_t changedbits =  r ^ port_C_history;
    port_C_history = r; 
    // A2 changedbits= 0x04 0x0C    
    if(changedbits & 0x04) hcsr.intp();
}  
 
// Port D - handle pin change interrupt for D0 to D7 here 
// D4 - FRONT
ISR (PCINT2_vect) { 

    uint8_t r= PIND;
    volatile uint8_t changedbits =  r ^ port_D_history;
    port_D_history = r;
    // D4 changedbits= 0x10 0x12    
    if(changedbits & 0x10) hcsr.intp();
} 

// -----------------------------------------------------------------------------
// ---- SERIAL MONITOR CLI interpreter
// commands typed from Arduino's Serial Monitor (Tools -> Serial Monitor)
// -----------------------------------------------------------------------------
#define MAX_ARGN 3
#define MAX_ARGV_LENGTH 16
char argv[MAX_ARGN][MAX_ARGV_LENGTH];
unsigned int argn;
unsigned int parse(char *s)
{
  unsigned int j=0;
  unsigned int l= strlen(s);
  argn= 0; 
  argv[0][0]= '\0';
  unsigned int i;
  for(i=0; i<l && s[i]!='\n' && s[i]!=';' && argn<MAX_ARGN; i++)
  {
      if(s[i]!=' ')
      {
          if(j<(MAX_ARGV_LENGTH-1)) argv[argn][j++]= s[i]; 
      }
      else
      {
          if(j!=0)
          {
             argv[argn][j]= '\0';
             if(argn<MAX_ARGN) argn++;
          }
          j= 0;
      }
  }
  if(j!=0 && argn<MAX_ARGN)
  {
     argv[argn][j]= '\0';
     if(argn<MAX_ARGN) argn++;
  }
  // <>0 if concatenated commands
  if(s[i]=='\n' || s[i]==';') i++;
  return (i==l)? 0 : i;
}

void response_serial(char *b)
{
  if(strlen(b)) 
    Serial.print(b);
}

void interpreter(char *rbuf, void (*f)(char*))
{
  char txbuff[128];
  unsigned int j;
  for(j=0; rbuf[j]!='\0' && rbuf[j]!=';' && rbuf[j]!='\r' && rbuf[j]!='\n'; j++);
  rbuf[j]='\0';
        
  // PARSE
  parse(rbuf);
  txbuff[0]='\0';
 
  // INTERPRETER
  if(strcmp(argv[0], "ping")==0)
  {
    snprintf(txbuff, sizeof(txbuff), "OK %s;", rbuf);
  }
  else if(strcmp(argv[0], "start")==0)
  {
    SensorSampling_go_active();
  }
  else if(strcmp(argv[0], "distance")==0)
  {
    hcsr.start_pulse(argn>1? atoi(argv[1]) : 0);
    snprintf(txbuff, sizeof(txbuff), "OK %s %d;", rbuf, (unsigned int) hcsr.get_response());
  }
  // battery
  else if(strcmp(argv[0], "battery")==0)
  {
    int val = analogRead(PIN_BATTERY);
    snprintf(txbuff, sizeof(txbuff), "OK battery %d;", val);
  } 
  // Default - ERROR
  else {
    snprintf(txbuff, sizeof(txbuff), "ERROR %s;", rbuf);        
  }
  if(txbuff[0]!='\0') f(txbuff);
}
char rxbuf[32];
unsigned int rix;

void serial_setup()
{
    rix= 0;
    rxbuf[0]='\0';
}
void serial_loop()
{
    if (Serial.available() > 0)
    {
        // read the incoming byte
        char c = (char) Serial.read();        
        if(rix<(sizeof(rxbuf)-1)) rxbuf[rix++]= c;      
        if(c=='\n' || c==';')
        {
            rxbuf[rix]='\0';         
            // INTERPRETER
            interpreter(rxbuf, response_serial);           
            rix= 0;    
        }
    }
}

// -----------------------------------------------------------------------------
// ---- setup / loop
// -----------------------------------------------------------------------------
int count_sampling_timer;
// put your setup code here, to run once:
void setup() 
{
    // Param
    memset(&reg_module, 0, sizeof(reg_module));
    snprintf(reg_module.name, 16, "%s", MODULE_NAME);
    snprintf(reg_module.version, 16, "%s", MODULE_VERSION);
      
    // Init variables
    rix= 0;
    round_table_done= false;
    
    // Start serial
    Serial.begin(115200);
    serial_setup();
    delay(800);
    
    // TIME
    count_sampling_timer= 0;

    SensorSampling_init();
    
    memset(&reg_sensor_meter, 0, sizeof(reg_sensor_meter));
    hcsr.setup(PIN_HCSR_TRIGGER_RIGHT, PIN_HCSR_ECHO_RIGHT, (void*) leds.create(PIN_LED_TRES)); // 11
    hcsr.setup(PIN_HCSR_TRIGGER_FRONT, PIN_HCSR_ECHO_FRONT, (void*) leds.create(PIN_LED_DOS));  // 22
    hcsr.setup(PIN_HCSR_TRIGGER_LEFT,  PIN_HCSR_ECHO_LEFT,  (void*) leds.create(PIN_LED_UNO));  // 00

    // Battery
    battery_init();
    
    // I2C
    I2C_init();
      
    Serial.println(MODULE_NAME " v" MODULE_VERSION " (" __DATE__ " " __TIME__ ")"); 
}


// put your main code here, to run repeatedly:
void loop() 
{
    // (0) TASK - TIME
    if(count_sampling_timer<=0)
    {
        count_sampling_timer= 100;
        if( is_SAMPLING_active && ((micros()-time_samplig_repeat) / 1000 > SENSOR_SAMPLE_PERIOD))
        {    
            hcsr.start_round_table(Callback_on_round_table_done);
            time_samplig_repeat= micros();
        }

        if( is_SAMPLING_active && ((micros() - time_SAMPLING_t0) / 1000 > READING_INACTIVITY_TIMEOUT))
        {   
            SensorSampling_go_idle();
        }
    }
    count_sampling_timer --;
    
    // (1) TASK - LEDS
    leds.loop();
      
    // (2) TASK - HCSR
    hcsr.loop();

    // (3) TASK -  SERIAL
    serial_loop();

    // (4) TASK -  READING
    if(round_table_done)
    {
        round_table_done= false;    
#ifdef ENABLE_DEBUG_ON_SERIAL
        char str[128];
        Serial.print("\n");
        for(int i=0; i<N_HCSR04_DEVICES; i++)
        { 
            snprintf(str, sizeof(str), "%u (%u cm) ", (unsigned int) reg_sensor_meter.duration[i], (unsigned int) ((double) reg_sensor_meter.duration[i] * 0.034/2) );
            Serial.print(str);
        }
#endif          
    }

    // (5) TASK -  I2C data received
    if(I2C_data_available)
    {
        I2C_data_available= false;
#ifdef ENABLE_DEBUG_IC2
        char str[128];
        for(unsigned int i=0; i<I2C_rxi; i++)
        { 
            snprintf(str, sizeof(str), "0x%X ",  I2C_rxbuffer[i]);
            Serial.print(str);
        }       
        Serial.print("\n");
#endif 
    }
    
    // (6) TASK -  Battery
    battery_loop();
    
    // TASK 
    // (Others)
}

/* END OF FILE */
