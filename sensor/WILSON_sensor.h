/** 
 *  WILSON SENSOR module
 *  Part of the Wilson project (www.iambobot.com)
 *  Fernando R
 *  
 *  common code with Raspberry (MASTER)
 *  
 **/

#define I2C_ADDR  0x09

// REGISTERS
// commons to all modules
#define REG_STATUS                  0x01
#define REG_MODULE_INFO             0x02
#define REG_SERIAL_COUNT            0x03
#define REG_SERIAL_READ             0x04
// module specific
#define REG_SENSOR_METER            0x10
#define REG_SENSOR_BATTERY          0x11
#define REG_SENSOR_ACTIVATE         0x12

struct REG_SENSOR_DATA_struct
{
  uint16_t duration[N_HCSR04_DEVICES];
};

struct REG_SENSOR_STATUS_struct
{
  uint16_t battery;
};
struct REG_SENSOR_MODULE_struct
{
    char name[16];
    char version[16];
};
/* END OF FILE */
