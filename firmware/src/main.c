#define INCLUDE_FROM_PWM_C
#include "main.h"

/* Scheduler Task List */
TASK_LIST
{
    { .Task = USB_USBTask          , .TaskStatus = TASK_STOP },
    { .Task = USB_ProcessPacket    , .TaskStatus = TASK_STOP },
};

/* DFU Bootloader Declarations */
uint32_t  boot_key __attribute__ ((section (".noinit")));
typedef void (*AppPtr_t)(void) __attribute__ ((noreturn));
AppPtr_t Bootloader = (AppPtr_t)0xf000;

/* Main program entry point. This routine configures the hardware required 
 * by the application, then starts the scheduler to run the USB management 
 * task.
 */
int main(void)
{
    /* After reset start bootloader? */
    if ((AVR_IS_WDT_RESET()) && (boot_key == DFU_BOOT_KEY_VAL)) {
        boot_key = 0;
        Bootloader();
    }

    /* Disable watchdog if enabled by bootloader/fuses */
    MCUSR &= ~(1 << WDRF);
    wdt_disable();

    /* Disable clock division */
    clock_prescale_set(clock_div_1);

    /* Initialize Scheduler so that it can be used */
    Scheduler_Init();

    /* Initialize USB Subsystem */
    USB_Init();

    /* Initialize I/O lines */
    IO_Init();

    /* Scheduling - routine never returns, so put this last in the main function */
    Scheduler_Start();

}

/** Event handler for the USB_Connect event. This indicates that the device is enumerating via the status LEDs and
 *  starts the library USB task to begin the enumeration and USB management process.
 */
void EVENT_USB_Connect(void)
{
    /* Start USB management task */
    Scheduler_SetTaskMode(USB_USBTask, TASK_RUN);
}

/** Event handler for the USB_Disconnect event. This indicates that the device is no longer connected to a host via
 *  the status LEDs and stops the USB management task.
 */
void EVENT_USB_Disconnect(void)
{
    /* Stop running HID reporting and USB management tasks */
    Scheduler_SetTaskMode(USB_ProcessPacket, TASK_STOP);
    Scheduler_SetTaskMode(USB_USBTask, TASK_STOP);
}

/** Event handler for the USB_ConfigurationChanged event. This is fired when the host sets the current configuration
 *  of the USB device after enumeration, and configures the generic HID device endpoints.
 */
void EVENT_USB_ConfigurationChanged(void)
{
    /* Setup USB In and Out Endpoints */
    Endpoint_ConfigureEndpoint(OUT_EPNUM, EP_TYPE_BULK,
            ENDPOINT_DIR_OUT, OUT_EPSIZE,
            ENDPOINT_BANK_SINGLE); 

    Endpoint_ConfigureEndpoint(IN_EPNUM, EP_TYPE_BULK,
            ENDPOINT_DIR_IN, IN_EPSIZE,
            ENDPOINT_BANK_SINGLE);

    /* Start ProcessPacket task */
    Scheduler_SetTaskMode(USB_ProcessPacket, TASK_RUN);
}


TASK(USB_ProcessPacket)
{
    uint8_t commandID;

    /* Check if the USB System is connected to a Host */
    if (USB_IsConnected) {

        /* Select the Data Out Endpoint */
        Endpoint_SelectEndpoint(OUT_EPNUM);

        /* Check if OUT Endpoint contains a packet */
        if (Endpoint_IsOUTReceived()) {

            /* Check to see if a command from the host has been issued */
            if (Endpoint_IsReadWriteAllowed()) {

                /* Read USB packet from the host */
                USBPacket_Read();

                /* Reset buffer for reading and writing */
                USBOut_ResetData();
                USBIn_ResetData();

                /* Get command ID from bulkout buffer */
                USBOut_GetData(&commandID,sizeof(uint8_t));

                /* Return same command ID in bulkin buffer */
                USBIn_SetData(&commandID,sizeof(uint8_t));

                /* Process USB packet */
                switch (commandID) {

                    case USB_CMD_SET_PWM_VAL:
                        {
                            uint8_t pwm_val;
                            USBOut_GetData(&pwm_val,sizeof(uint8_t));
                            Set_PWM_Val(pwm_val);
                        }
                        break;

                    case USB_CMD_GET_PWM_VAL:
                        USBIn_SetData(&sys_state.pwm_val,sizeof(uint8_t));
                        break;

                    case USB_CMD_SET_RELAY_MODE:
                        {
                            uint8_t mode;
                            USBOut_GetData(&mode,sizeof(uint8_t));
                            Set_Mode(mode);
                            USBIn_SetData(&sys_state.mode,sizeof(uint8_t));
                        }
                        break;

                    case USB_CMD_GET_RELAY_MODE:
                        USBIn_SetData(&sys_state.mode,sizeof(uint8_t));
                        break;

                    case USB_CMD_GET_THERM_VAL:
                        break;

                    case USB_CMD_SET_PWM_PERIOD:
                        {
                            float period;
                            USBOut_GetData(&period,sizeof(float));
                            Set_PWM_Period(period);
                            USBIn_SetData(&sys_state.pwm_period,sizeof(float));
                        }
                        break;

                    case USB_CMD_GET_PWM_PERIOD:
                        USBIn_SetData(&sys_state.pwm_period,sizeof(float));
                        break;
                            
                    case USB_CMD_AVR_RESET:
                        USBPacket_Write();
                        AVR_RESET();
                        break;

                    case USB_CMD_AVR_DFU_MODE:
                        USBPacket_Write();
                        boot_key = DFU_BOOT_KEY_VAL;
                        AVR_RESET();
                        break;

                    default:
                        break;
                }

                /* Write the return USB packet */
                USBPacket_Write();

                /* Indicate ready */
                LEDs_SetAllLEDs(LEDS_LED2 | LEDS_LED4);
            }
        }
    }
}

static void USBIn_ResetData(void)
{
   USBIn.Pos = 0;
    return;
}

static void USBOut_ResetData(void)
{
    USBOut.Pos = 0;
    return;
}

static uint8_t USBIn_SetData(void *data, size_t len) 
{
    uint8_t rval = FAIL;
    if (USBIn.Pos + len <= IN_EPSIZE) { 
        memcpy((void *)(USBIn.Packet.Buf + USBIn.Pos), data, len);
        USBIn.Pos += len;
        rval = PASS;
    }
    return rval;
}

static uint8_t USBOut_GetData(void *data, size_t len)
{
    uint8_t rval = FAIL;
    if (USBOut.Pos + len <= OUT_EPSIZE) {
        memcpy(data, (void *)(USBOut.Packet.Buf + USBOut.Pos), len);
        USBOut.Pos += len;
        rval = PASS;
    }
    return rval;
}
static void USBPacket_Read(void)
{
    uint8_t* USBPacketOutPtr = (uint8_t*)&USBOut.Packet;

    /* Select the Data Out endpoint */
    Endpoint_SelectEndpoint(OUT_EPNUM);

    /* Read in USB packet header */
    Endpoint_Read_Stream_LE(USBPacketOutPtr, sizeof(USBOut.Packet));

    /* Finalize the stream transfer to send the last packet */
    Endpoint_ClearOUT();
}

static void USBPacket_Write(void)
{
    uint8_t* USBPacketInPtr = (uint8_t*)&USBIn.Packet;

    /* Select the Data In endpoint */
    Endpoint_SelectEndpoint(IN_EPNUM);

    /* Wait until read/write to IN data endpoint allowed */
    while (!(Endpoint_IsReadWriteAllowed() && Endpoint_IsINReady()));

    /* Write the return data to the endpoint */
    Endpoint_Write_Stream_LE(USBPacketInPtr, sizeof(USBIn.Packet));

    /* Finalize the stream transfer to send the last packet */
    Endpoint_ClearIN();
}

// Initialize IO 
static void IO_Init(void)
{
    // Initial DIO PORT
    DIO_DDR_PORT_E = 0xff;  // set all pins to output
    DIO_PORT_E = 0x00;      // set all pins low

    // Set indicator pin high
    DIO_PORT_E |= (1 << dio_port_e_pins[0]);

    // Set TOP high
    Set_PWM_Period(DEFAULT_PWM_PERIOD); 

    // Set timer control registers, set to fast PWM mode with double buffering of TOP
    TIMER_TCCRA = 0x3; 
    TIMER_TCCRB = 0x18; 

    // Set Timer prescaler to 0
    TIMER_TCCRB |= 0x1;
    
    // Enable Timer3 overflow interrupts
    TIMER_TIMSK = 0x00; 
    TIMER_TIMSK |= (1<<TIMER_TOIE); 
    
    // Set ADC channel and voltage reference
    ADMUX = 0x0;
    ADMUX |= (1 << REFS0); // voltage reference = AVcc
    ADMUX |= (1 << MUX0);  // Channel 1

    // Enable ADC, interupt enable, prescaler 128
    ADCSRA |= (1 << ADEN);   // Enable
    ADCSRA |= (1 << ADPS0);  // Prescaler
    ADCSRA |= (1 << ADPS1);
    ADCSRA |= (1 << ADPS1);

    // Set to input and Disable digital input
    DDRF = 0x0; 
    DIDR0 = 0xff;
}

static void IO_Disconnect(void)
{
}

// Read thermocouple value
// Note, may want to use the LUFA code for this ....
static uint16_t Get_Therm_Val(void)
{
    uint16_t therm_val;
    uint8_t val_H;
    uint8_t val_L;

    // Start conversion
    ADCSRA |= (1<<ADSC);

    // Wait for reading to complete


    return therm_val;
}

// Set sys_state mode (PWM or CONSTANT)
static void Set_Mode(uint8_t mode)
{
    if ((mode == ON) || (mode == OFF) || (mode == PWM)) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            sys_state.mode = mode;
        }
    }
    return;
}

// Set sys_state.pwm_val 
static void Set_PWM_Val(uint8_t val)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        sys_state.pwm_val = val;
    }
    return;
}

// Set sys_state.pwm_period
static void Set_PWM_Period(float pwm_period)
{ uint16_t top;
    float period;
    top = Set_TimerTop(pwm_period);
    period = Get_PWM_Period(top);
    sys_state.pwm_period = period;
    return;
}

// Get the pwm period given the current top
static float Get_PWM_Period(uint16_t top)
{
    float period;
    period = 256.0*((float)top)/((float)F_CPU);
    return period;
}

// Set timer top given desired pwm period
static uint16_t Set_TimerTop(float pwm_period) 
{ 
    uint16_t top;
    top = Get_TimerTop(pwm_period);
    TIMER_TOP = top;
    return top;
}

// Computes the timer top for a given pwm period
static uint16_t Get_TimerTop(float pwm_period) 
{
    uint16_t top;
    top = (uint16_t)((pwm_period*F_CPU)/(256.0));
    top = top < TIMER_TOP_MAX ? top : TIMER_TOP_MAX; 
    top = top > TIMER_TOP_MIN ? top : TIMER_TOP_MIN;
    return top;
}

// Timer overflow interrupt  
ISR(TIMER3_OVF_vect) {
    switch(sys_state.mode) {

        case ON:
            DIO_PORT_E |= (1 << dio_port_e_pins[1]);
            break;

        case OFF:
            DIO_PORT_E &= ~(1 << dio_port_e_pins[1]);
            break;

        case PWM:
            sys_state.pwm_cnt += 1;
            if (sys_state.pwm_cnt < sys_state.pwm_val) {
                DIO_PORT_E |= (1 << dio_port_e_pins[1]);
            }
            else {
                DIO_PORT_E &= ~(1 << dio_port_e_pins[1]);
            }
            break;

        default:
            break;
    
    } // switch(sys_state.mode)

    return;
}
