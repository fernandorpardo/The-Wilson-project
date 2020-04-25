// -----------------------------------------------------------------------------
// ------- LEDs  ---------------------------------------------------------------
// -----------------------------------------------------------------------------
#define MAX_LEDS      3


//#define PIN_LED_UNO    2 // D2  LED 1   PIN 5 - LEFT (Tank's stand point)
//#define PIN_LED_DOS    3 // D3  LED 2   PIN 6 - MIDDLE
//#define PIN_LED_TRES   4 // D4  LED 3   PIN 7 - RIGHT

#define PIN_LED_UNO    A0 // A0 19  LED 1   
#define PIN_LED_DOS    A1 // A1 20  LED 2   
#define PIN_LED_TRES   5  // D5     LED 3  


#define ASYMMETRY_WHEN_IDLE  50
class led
{
    public:
        led(char p);
        void period(unsigned int);
        void asymmetry(unsigned int);
        void set_idle(unsigned int );
        void loop(void);
    private:
        char pin;
        bool on;
        unsigned int PERIOD;
        unsigned int period_count;
        unsigned int ASYMMETRY;
        unsigned int asymmetry_count;
};
led::led(char p)
{
    PERIOD= 0;
    period_count= 0;
    on= false;
    pin=p;
    asymmetry_count= ASYMMETRY= ASYMMETRY_WHEN_IDLE;
    pinMode(p, OUTPUT);
}
void led::period(unsigned int p)
{
    period_count= PERIOD= p;
}
void led::asymmetry(unsigned int t)
{
    if(t<ASYMMETRY) asymmetry_count= t;
    ASYMMETRY= t;
}
void led::set_idle(unsigned int percentage)
{
    ASYMMETRY= ASYMMETRY_WHEN_IDLE;
    asymmetry_count= ASYMMETRY_WHEN_IDLE - percentage;
    on= false;
    digitalWrite(pin, LOW);
}
void led::loop(void)
{
    if(period_count)
    {
        period_count --;
        if(period_count==0)
        {
            if(on)
            {
                digitalWrite(pin, LOW);
                on = false;
                asymmetry_count= ASYMMETRY;
            }           
            else
            {
                asymmetry_count --;
                if(asymmetry_count==0)
                {
                    digitalWrite(pin, HIGH);
                    on= true;
                }
            }
            period_count= PERIOD;
        }
    }
}

// ---------------------------------------------------------------------------------


#define TIME_LED_PERIOD 10 // miliseconds  ( 1000 = 1 second)  

class ledbucket
{
    public:
        ledbucket(void);
        led* create(char pin);
        led* add(led *p);
        void set_idle(void);
        void loop(void);
    private:
        led *ptr[MAX_LEDS]= {0};
        int n= 0;
        unsigned long time_leds;
        int count_leds;
};
ledbucket::ledbucket(void)
{
    n=0;
    time_leds= micros();
    count_leds= 0;
}
// A led object can be created outside the ledbucket class as follows
// led myled
// and later added to the bucket as follows
// ledbucket.add(&myled)
led* ledbucket::add(led *p)
{
    if(n>=MAX_LEDS && p) return 0;
    ptr[n]= p;
    p->period(2);
    n++;
    return p;
}
// .. or created in the bucket by the Arduino pin number
led* ledbucket::create(char pin)
{
    led *p;
    if(n>=MAX_LEDS) return 0;
    p= ptr[n]= new led(pin);
    p->period(2);
    n++;
    return p;
}
void ledbucket::set_idle(void)
{
    for(int i=0; i<n; i++)
    {
        ptr[i]->set_idle(3*i);
    }
}
void ledbucket::loop(void)
{
    if(count_leds<=0)
    {
        count_leds= 20;
        if( (micros()-time_leds) / 1000 > TIME_LED_PERIOD)
        {      
            for(int i=0; i<n; i++) ptr[i]->loop();
            time_leds= micros();
        }
    }
    count_leds --;
}
/* END OF FILE */
