/*-----------------------------------------------------------------------
reflow
Copyright (C) William Dickson, 2008.
  
wbd@caltech.edu
www.willdickson.com

Released under the LGPL Licence, Version 3

This file is part of reflow.

reflow is free software: you can redistribute it and/or modify it
under the terms of the GNU Lesser General Public License as published
by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
    
reflow is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with simple_step.  If not, see
<http://www.gnu.org/licenses/>.

------------------------------------------------------------------------

Purpose: . 

Author: William Dickson 

-----------------------------------------------------------------------*/

#ifndef _PWM_H_
#define _PWM_H_

/* Includes: */
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdbool.h>
#include <string.h>
#include "descriptors.h"
#include <LUFA/Version.h>                    // Library Version Information
#include <LUFA/Scheduler/Scheduler.h>        // Simple scheduler for task management
#include <LUFA/Drivers/USB/USB.h>            // USB Functionality
#include <LUFA/Drivers/Board/LEDs.h>         // LEDs driver
#include <LUFA/Drivers/Peripheral/ADC.h>

// USB Commands 
#define USB_CMD_SET_MODE        1
#define USB_CMD_GET_MODE        2
#define USB_CMD_SET_PWM_VAL     3
#define USB_CMD_GET_PWM_VAL     4
#define USB_CMD_GET_THERM_VAL   5
#define USB_CMD_SET_PWM_PERIOD  6
#define USB_CMD_GET_PWM_PERIOD  7

#define USB_CMD_AVR_RESET       200
#define USB_CMD_AVR_DFU_MODE    201

/* Software reset */
#define AVR_RESET() wdt_enable(WDTO_30MS); while(1) {}
#define AVR_IS_WDT_RESET()  ((MCUSR&(1<<WDRF)) ? 1:0)
#define DFU_BOOT_KEY_VAL 0xAA55AA55

// Constants
#define PASS 0
#define FAIL 1
#define DEFAULT_PWM_PERIOD 1.0

// Timer constants
#define TIMER_PRESCALER 0 
#define TIMER_TOP_MIN 625      
#define TIMER_TOP_MAX 65535   

// Timer control registers
#define TIMER_TCCRA TCCR3A
#define TIMER_TCCRB TCCR3B
#define TIMER_TOP OCR3A  // Using OCR3A gives double buffering of top  

// Timer interrupt mask register and enable
#define TIMER_TIMSK TIMSK3 // Mask register
#define TIMER_TOIE  TOIE3  // Enable

// DIO DDR register
#define DIO_DDR_PORT_E DDRE
#define DIO_DDR_PORT_E_PINS {DDE0,DDE1,DDE2,DDE3,DDE4,DDE5,DDE6,DDE7}  

// DIO PORT
#define DIO_PORT_E PORTE
#define DIO_PORT_E_PINS {PE0,PE1,PE2,PE3,PE4,PE5,PE6,PE7}

enum {OFF, ON, PWM}; 

typedef struct {
    char Buf[IN_EPSIZE];
} InPacket_t;

typedef struct {
    char Buf[OUT_EPSIZE];
} OutPacket_t;

typedef struct {
    InPacket_t Packet;
    uint8_t Pos;
} USBIn_t;

typedef struct {
    OutPacket_t Packet;
    uint8_t Pos;
} USBOut_t;

// Sytem state structure
typedef struct {
    uint8_t mode;
    uint8_t pwm_val;
    uint8_t pwm_cnt;
    float pwm_period;
} Sys_State_t;

/* Global Variables: */
const uint8_t dio_port_e_pins[] = DIO_PORT_E_PINS;
USBIn_t USBIn;
USBOut_t USBOut;
Sys_State_t sys_state = {OFF, 0, 0, DEFAULT_PWM_PERIOD};

/* Task Definitions: */
TASK(USB_ProcessPacket);

/* Function Prototypes: */
void EVENT_USB_Connect(void);
void EVENT_USB_Disconnect(void);
void EVENT_USB_ConfigurationChanged(void);
void EVENT_USB_UnhandledControlPacket(void);
static uint16_t Get_TimerTop(float pwm_period);
static uint16_t Set_TimerTop(float pwm_period);
static void USBPacket_Read(void);
static void USBPacket_Write(void);
static void IO_Init(void);
static void IO_Disconnect(void);
static uint8_t USBIn_SetData(void *data, size_t len); 
static uint8_t USBOut_GetData(void *data, size_t len);
static void USBIn_ResetData(void);
static void USBOut_ResetData(void);

static void Set_Mode(uint8_t mode);
static void Set_PWM_Val(uint8_t val);
static void Set_PWM_Period(float pwm_period);
static float Get_PWM_Period(uint16_t top);
static uint16_t Get_Therm_Val(void);

#endif
