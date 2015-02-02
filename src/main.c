#include <stdlib.h>
#include <string.h>
#include <avr/eeprom.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "croutine.h"

/* Demo file headers. */
#include "PollQ.h"
#include "integer.h"

#include "VirtualSerial.h"
#include <stdbool.h>
#include "stdlib.h"
#include "main.h"

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "iocompat.h" /* Note [1] */

#include "onewire.h"
#include "ds18x20.h"
#include "temp_sensors.h"

#define MAXSENSORS 5
#define NEWLINESTR "\r\n"

typedef enum state
{
    START,
    SET_LED,
    SET_FAN,
    SET_PUMP,
    READ_TEMP
} states;

states state = START;

/* Priority definitions for most of the tasks in the demo application.  Some
 tasks just use the idle priority. */
#define mainLED_TASK_PRIORITY			( tskIDLE_PRIORITY + 1 )
#define mainCOM_TEST_PRIORITY			( tskIDLE_PRIORITY + 2 )
#define mainQUEUE_POLL_PRIORITY			( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY			( tskIDLE_PRIORITY + 3 )

#define MAIN_TASK_PRIORITY              ( configMAX_PRIORITIES - 3 )
#define ViSe_TASK_PRIORITY              ( configMAX_PRIORITIES - 1 )    // highest priority

/* The period between executions of the check task. */
#define mainCHECK_PERIOD				( ( TickType_t ) 3000 / portTICK_PERIOD_MS  )

/* An address in the EEPROM used to count resets.  This is used to check that
 the demo application is not unexpectedly resetting. */
#define mainRESET_COUNT_ADDRESS			( ( void * ) 0x50 )

/* The number of coroutines to create. */
#define mainNUM_FLASH_COROUTINES		( 3 )

#define taskDelayPeriod                 3

/*
 * Called on boot to increment a count stored in the EEPROM.  This is used to
 * ensure the CPU does not reset unexpectedly.
 */
static void prvIncrementResetCount(void);

/*
 * The idle hook is used to scheduler co-routines.
 */
void vApplicationIdleHook(void);
static void vLedPwm(void *pvParameters);
static void VirtualSerialTask(void *pvParameters);

void ioinit(void);
void setup_usb(void);
void print_usb(char* input);
uint8_t classify_sensors(uint8_t nSensors);
void measure_temp(uint8_t nSensors, int32_t* temp_eminus4);




static uint16_t ucPwm = 0;
static uint8_t ucUp = 1;

static uint16_t ucPwmMax = 200;
static uint16_t ucPwmMin = 10;

int BytesAvailable = 0;
char buffer[100];
char end_buffer[100];
int32_t temp_eminus4[2];

USB_ClassInfo_CDC_Device_t USB_Interface =
{

{ INTERFACE_ID_CDC_CCI,

{
CDC_TX_EPADDR,
CDC_TXRX_EPSIZE, 1 },

{
CDC_RX_EPADDR,
CDC_TXRX_EPSIZE, 1 },

{
CDC_NOTIFICATION_EPADDR,
CDC_NOTIFICATION_EPSIZE, 1 } } };

/** Standard file stream for the CDC interface when set up, so that the virtual CDC COM port can be
 *  used like any regular character stream in the C APIs.
 */
//static FILE USBSerialStream;
/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
//	LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
//	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
    bool ConfigSuccess = true;

    ConfigSuccess &= CDC_Device_ConfigureEndpoints(&USB_Interface);

//	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
    CDC_Device_ProcessControlRequest(&USB_Interface);
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void setup_usb(void)
{
    /* Disable watchdog if enabled by bootloader/fuses */
    MCUSR &= ~(1 << WDRF);
    wdt_disable();

    /* Disable clock division */
    clock_prescale_set(clock_div_1);

    /* Hardware Initialization */
    USB_Init();
}

void ioinit(void) /* Note [6] */
{
    /* Timer 1 is 10-bit PWM (8-bit PWM on some ATtinys). */
    TCCR4A = _BV(COM4A0) | _BV(PWM4A) | _BV(COM4B0) | _BV(PWM4B);

    TCCR4B = _BV(CS43) | _BV(CS40);//_BV(CS42) | _BV(CS41);

    /* Set PWM value to 0. */
    OCR4A = 0;
    OCR4B = 0;

    /* enable pump PWM output pin, arduino pin 10 */
    DDRB = _BV(PB6);


    /* enable fan pwm output
     * arduino pin
     * Digital Pin 5 (PWM)
     * Atmega pin PC6
     * fan control 25khz see http://forum.arduino.cc/index.php?topic=18742.0
     * */
    TCCR3A = _BV(COM3A1);

    /* prescaler 8 */
    TCCR3B = _BV(CS31) | _BV(WGM33);
    /* TOP is MAX */
    ICR3 = 40;
    OCR3A = 0;

    DDRC = _BV(PC6) | _BV(PC7);



}

void print_usb(char* input)
{
    vTaskSuspendAll();
    CDC_Device_SendString(&USB_Interface, input);
    xTaskResumeAll();

}

void measure_temp(uint8_t nSensors, int32_t* temp_eminus4)
{
    uint8_t i = 0;
    uint8_t result = 0;

    if (DS18X20_start_meas( DS18X20_POWER_EXTERN, NULL) == DS18X20_OK)
    {
        _delay_ms(DS18B20_TCONV_12BIT);

        for (i = 0; i < nSensors; i++)
        {
//            vTaskSuspendAll();
//            result = DS18X20_read_decicelsius(&gSensorIDs[i][0], &decicelsius);
//            xTaskResumeAll();

//            if ( result == DS18X20_OK)
//            {
//
//            }
            vTaskSuspendAll();
            result = DS18X20_read_maxres(&gSensorIDs[i][0], &temp_eminus4[i]);
            xTaskResumeAll();

//            if (result == DS18X20_OK)
//            {
//
//
//            }

        }

    }


}



void handle_input(char* input)
{

    if (state == START)
    {
        if (strcmp("set fan\r", input) == 0)
        {
            print_usb("enter fan speed, 0...40 \n\r");

            /* advance state machine */
            state = SET_FAN;

        }
        else if (strcmp("set pump\r", input) == 0)
        {
            print_usb("enter pump speed, 0...200  \n\r");

            /* advance state machine */
            state = SET_PUMP;
        }
        else if (strcmp("set led\r", input) == 0)
        {
            print_usb("enter led brightness, 0...255  \n\r");

            /* advance state machine */
            state = SET_LED;
        }
        else if (strcmp("read temp\r", input) == 0)
        {
            print_usb("reading temp...  \n\r");

            /* advance state machine */
            state = READ_TEMP;
        }
        else
        {
            /* didnt understand string */
            print_usb("did not understand command \n\r");

            /* dont change state */
            state = START;
        }
    }
    else if (state == SET_LED)
    {
        static uint16_t pwm;
        char *garbage = NULL;
        pwm = strtol(input, &garbage, 0);
        OCR4A = pwm;

        print_usb("led set. \n\r");

        /* go back to start */
        state = START;
    }
    else if (state == SET_FAN)
    {
        static uint16_t pwm;
        char *garbage = NULL;
        pwm = strtol(input, &garbage, 0);
        OCR3A = pwm;

        print_usb("fan set. \n\r");

        /* go back to start */
        state = START;
    }
    else if (state == SET_PUMP)
    {
        static uint16_t pwm;
        char *garbage = NULL;
        pwm = strtol(input, &garbage, 0);
        OCR4B = pwm;

        print_usb("pump set. \n\r");

        /* go back to start */
        state = START;
    }
    else if (state == READ_TEMP)
    {
        uint8_t nSensors;

        char maxres_buffer[10];

        ow_set_bus(&PIND, &PORTD, &DDRD, PD4);

        ow_reset();

        /* search for temperature sensors */
        nSensors = search_sensors();

        /* classify sensors */
        classify_sensors(nSensors);

        /* measure temperature */
        measure_temp(nSensors, temp_eminus4);

        DS18X20_format_from_maxres(temp_eminus4[0], maxres_buffer, 10);

        print_usb("\n\r ");
        print_usb("temp max res 1: ");
        print_usb(maxres_buffer);
        print_usb("\n\r ");

        DS18X20_format_from_maxres(temp_eminus4[1], maxres_buffer, 10);

        print_usb("temp max res 2: ");
        print_usb(maxres_buffer);
        print_usb("\n\r ");

        /* go back to start */
        state = START;

    }
    else
    {
        CDC_Device_SendString(&USB_Interface, "went into unknown state!? \n\r");

        state = START;

    }

}

uint8_t classify_sensors(uint8_t nSensors)
{
    int i = 0;

    /* classify sensors */
    for (i = 0; i < nSensors; i++)
    {
        if (gSensorIDs[i][0] == DS18B20_FAMILY_CODE)
        {
            CDC_Device_SendString(&USB_Interface, "DS18B20 found ");
        }

        if (DS18X20_get_power_status(
                &gSensorIDs[i][0]) == DS18X20_POWER_PARASITE)
        {
            CDC_Device_SendString(&USB_Interface, "parasite\n\r");
        }
        else
        {
            CDC_Device_SendString(&USB_Interface, "externally\n\r");
        }
    }
}


static void vLedPwm(void *pvParameters)
{
    /* change the PWM duty cycle to get a pulsing LED */
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 100;

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        /* each cycle, increment or decrement the pwm duty cycle */
        if (ucUp == true)
        {
            /* count up */
            ucPwm = ucPwm + 30;
            if (ucPwm >= ucPwmMax)
            {
                ucUp = false;
            }

        }
        else if (ucUp == false)
        {
            /* count down */
            ucPwm = ucPwm - 20;

            if (ucPwm <= ucPwmMin)
            {
                ucUp = true;
            }

        }

        OCR4A = ucPwm;

        vTaskDelayUntil(&xLastWakeTime, xFrequency);

    }
}

static void vUserInput(void)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 100;

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    vTaskSuspendAll();
    CDC_Device_SendString(&USB_Interface, "enter command: ");
    xTaskResumeAll();

    for (;;)
    {

        int count = 0;
        buffer[0] = 0;

        /* read bytes from USB interface */
        vTaskSuspendAll();
        BytesAvailable = CDC_Device_BytesReceived(&USB_Interface);
        for (count = 0; count < BytesAvailable; count++)
        {
            buffer[count] = CDC_Device_ReceiveByte(&USB_Interface);
        }
        xTaskResumeAll();

        /* Null terminate buffer*/
        buffer[count] = 0;

        /* Buffer is not empty */
        if (buffer[0] != 0)
        {
            /* echo user input back */
            vTaskSuspendAll();
            CDC_Device_SendString(&USB_Interface, buffer);
            xTaskResumeAll();

            /* combine into buffer */
            strncat(end_buffer, buffer, 1);

            if (buffer[0] == '\r')
            {
                strncat(end_buffer, '\0', 1);

//                vTaskSuspendAll();
//                CDC_Device_SendString(&USB_Interface, end_buffer);
//                CDC_Device_SendString(&USB_Interface, "\n\r");
//                xTaskResumeAll();

                handle_input(end_buffer);

                /* 'reset' end_buffer */
                end_buffer[0] = '\0';

                vTaskSuspendAll();
                CDC_Device_SendString(&USB_Interface, "enter command: ");
                xTaskResumeAll();

            }

        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);

    }

}

static void prvIncrementResetCount(void)
{
    unsigned char ucCount;

    eeprom_read_block(&ucCount, mainRESET_COUNT_ADDRESS, sizeof(ucCount));
    ucCount++;
    eeprom_write_byte( mainRESET_COUNT_ADDRESS, ucCount);
}


void vApplicationIdleHook(void)
{
//	vCoRoutineSchedule();
}

static void VirtualSerialTask(void *pvParameters)
{

    for (;;)
    {
        // Must throw away unused bytes from the host, or it will lock up while waiting for the device
        // TODO: this causes loopback to fail
        //CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);

        // want CDC and USB functions to run without interruption but
        // with interrupts enabled so ENTER/EXIT_CRITICAL won't work
        vTaskSuspendAll();

        CDC_Device_USBTask(&USB_Interface);
        USB_USBTask();

        xTaskResumeAll();

        vTaskDelay((portTickType) taskDelayPeriod);
    }

}

int main(void)
{

    /* Setup the LED's for output. */
    ioinit();
    setup_usb();
    GlobalInterruptEnable();

    prvIncrementResetCount();

    xTaskCreate(VirtualSerialTask, (const portCHAR *) "ViSeTask",
            configMINIMAL_STACK_SIZE, NULL, ViSe_TASK_PRIORITY, NULL);

    xTaskCreate(vLedPwm, ( const portCHAR * )"Ape", configMINIMAL_STACK_SIZE,
            NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(vUserInput, ( const portCHAR * )"ui", configMINIMAL_STACK_SIZE*2,
            NULL, tskIDLE_PRIORITY + 2, NULL);

    vTaskStartScheduler();

    return 0;

}

