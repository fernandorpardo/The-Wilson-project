// -----------------------------------------------------------------------------
// ------- Ultrasonic sensor HC-SR04  ------------------------------------------
// -----------------------------------------------------------------------------
#define N_HCSR04_DEVICES  3 // 3 devices max

class HCSR {
    public:
        HCSR(bool p=true, void (* f)(int, uint16_t, void*)= NULL); //poll mode by default
        void setup(uint8_t trigger, uint8_t echo, void *ptrled); 
        uint8_t loop(void);
        void start_pulse(int device);
        unsigned long get_response(void);
        void intp(void);
        unsigned int distance(int device);	// cm
        void start_round_table(void (* callback)(void)=NULL);   
      	int error;
        void *ptr[N_HCSR04_DEVICES];
    private:
        void (* dataready_callback)(int, uint16_t, void*)= NULL;
        void (* callback_round_table)(void)= NULL;
        unsigned long time;			//  micros()
        uint16_t duration[N_HCSR04_DEVICES];
        uint8_t pin_trigger[N_HCSR04_DEVICES];
        uint8_t pin_echo[N_HCSR04_DEVICES];	
        bool ready[N_HCSR04_DEVICES];
        bool poll= true;
        int n_devices;
        int device; // device in use
        int device_count;
};
/* END OF FILE */
