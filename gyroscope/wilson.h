/* ---------------------------------------------------------------------
 *  WILSON GYRO module
 *  Part of the Wilson project (www.iambobot.com)
 *  Fernando R
 *  
 */

// LED's
ledbucket leds;

#define SERIAL_BAUD_RATE 115200
#define PIN_SPI_LED     PIN_LED_UNO 
#define PIN_Z_ED        PIN_LED_DOS

// Everything is working fine variables
bool fatal_error;
bool is_working_fine;


float zf, yf, xf;

char rx_buff [128];
char str[128];
bool led_comm_on;
bool led_on;

volatile byte rx_i;
volatile boolean SPI_comm_available;
unsigned int count_fail= 0;

#define TIME_GYRO_LED_PERIOD 50
unsigned long time_gyro_led;
unsigned long time_gyro_to_change;
unsigned int z_degrees; // integer from for the Z-axis value from the GY-521, multiplied x 10. Used for the LEDS and debugging
/*
struct REG_GY521_TEST_struct
{
    char pattern[32];
};
*/
struct REG_GY521_DATA_struct
{
    char id[4];
    // Gyro
    int16_t x;
    int16_t y;
    int16_t z;
    // Acceleration
    int16_t ax;
    int16_t ay;
    int16_t az;   
};
struct REG_GY521_QUAD_struct
{
    char id[4];
    int16_t w;
    int16_t x;
    int16_t y;
    int16_t z;
};
struct REG_GY521_STATUS_struct
{
    char id[4];
    bool is_ready;
    bool is_error;
    char name[16];
    char version[16];
};
//REG_GY521_TEST_struct reg_gyro_test;
REG_GY521_DATA_struct reg_gyro_data;
REG_GY521_QUAD_struct reg_gyro_quad;
REG_GY521_STATUS_struct reg_gyro_status;

const char command_DAT[] =    "DAT";
const char command_QUAD[] =   "QAD";
const char command_STA[] =    "STA";

bool strcomp(char *a, char *b)
{
    int i;
    for(i=0; a[i]!='\0' && b[i]!='\0' && a[i]==b[i]; i++);
    return (b[i] == '\0'); // true= equal 
}

// inline static void attachInterrupt() { SPCR |= _BV(SPIE); }
// http://ee-classes.usc.edu/ee459/library/documents/avr_intr_vectors/
ISR (SPI_STC_vect) // SPI interrupt routine 
{ 
    //char *p= response;
    byte c = SPDR; // read byte from SPI Data Register
    if (c!=0xFF && (rx_i < sizeof (rx_buff) -1)) 
    {
        rx_buff [rx_i++] = c; // save data in the next index in the array buff
        rx_buff [rx_i]= '\0';
        //check for the end of the word
        if (c == '\n') 
        {
            rx_buff [rx_i-1]= '\0';
            // (1) GENERATE response
            // IMPORTANT !!!
            // Time to generate the response is limited to TIME_TO_RESPOND micro seconds
            // that is defined at MASTER side. If response takes longer you need to adjust this value           
            int n= 0;
            byte *p= (byte*) &reg_gyro_data; // default           
            // SEND response
            if(strcomp(rx_buff, (char*)command_DAT))
            {
                n= sizeof(reg_gyro_data);
                p= (byte*) &reg_gyro_data;
            } 
            else if(strcomp(rx_buff, (char*)command_QUAD))
            {
                n= sizeof(reg_gyro_quad);
                p= (byte*) &reg_gyro_quad;
            }   
            else if(strcomp(rx_buff, (char*)command_STA))
            {
                n= sizeof(reg_gyro_status);
                p= (byte*) &reg_gyro_status;
            }            
            for(; n>0; n--, p++) 
            {
                SPDR = *p;
                while (!(SPSR & _BV(SPIF))) ; // wait
            }  
            
            SPI_comm_available = true; 
            rx_i=0;
        }
    }
    else rx_i= 0;
}

/* END OF FILE */
