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
static void prvIncrementResetCount( void );

/*
 * The idle hook is used to scheduler co-routines.
 */
void vApplicationIdleHook( void );
static void vLedPwm( void *pvParameters  );
static void VirtualSerialTask(void *pvParameters);


void ioinit( void );
void setup_usb(void);



static uint16_t ucPwm = 0;
static uint8_t ucUp = 1;

static uint16_t ucPwmMax = 200;
static uint16_t ucPwmMin = 10;


USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_InterfaceR =
{

    {
        INTERFACE_ID_CDC_CCI,

        {
             CDC_TX_EPADDR,
             CDC_TXRX_EPSIZE,
             1
        },

        {
            CDC_RX_EPADDR,
            CDC_TXRX_EPSIZE,
            1
        },

        {
            CDC_NOTIFICATION_EPADDR,
            CDC_NOTIFICATION_EPSIZE,
            1
        }
    }
};


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

	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_InterfaceR);

//	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_InterfaceR);
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


int main( void )
{

	/* Setup the LED's for output. */
	ioinit();
	setup_usb();
    GlobalInterruptEnable();

	prvIncrementResetCount();

	xTaskCreate( vLedPwm, ( const portCHAR * )"Ape", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL );
	xTaskCreate( VirtualSerialTask, (const portCHAR *) "ViSeTask", configMINIMAL_STACK_SIZE, NULL, ViSe_TASK_PRIORITY, NULL );

	vTaskStartScheduler();


	return 0;

}

static void vLedPwm( void *pvParameters  )
{
	/* change the PWM duty cycle to get a pulsing LED */

	for(;;)
	{
		/* each cycle, increment or decrement the pwm duty cycle */
		if(ucUp == true)
		{
			/* count up */
			ucPwm++;
			if(ucPwm >= ucPwmMax)
			{
				ucUp = false;
			}

		}
		else if(ucUp == false)
		{
			/* count down */
			ucPwm--;

			if(ucPwm <= ucPwmMin)
			{
				ucUp = true;
			}

		}

        //CDC_Device_SendString (&VirtualSerial_CDC_InterfaceR, "fan set. \n\r");

		_delay_ms(10);
		OCR4A = ucPwm;
		portYIELD();


	}
}


static void prvIncrementResetCount( void )
{
unsigned char ucCount;

	eeprom_read_block( &ucCount, mainRESET_COUNT_ADDRESS, sizeof( ucCount ) );
	ucCount++;
	eeprom_write_byte( mainRESET_COUNT_ADDRESS, ucCount );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
//	vCoRoutineSchedule();
}


void ioinit(void) /* Note [6] */
{
    /* Timer 1 is 10-bit PWM (8-bit PWM on some ATtinys). */
    TCCR4A = _BV(COM4A0) | _BV(PWM4A); // | _BV(COM1A1)
    /*
     * Start timer 1.
     *
     * NB: TCCR1A and TCCR1B could actually be the same register, so
     * take care to not clobber it.
     */
    TCCR4B = _BV(CS42) | _BV(CS41);//| _BV(CS41) | _BV(CS40);

    /* Set PWM value to 0. */
    OCR4A = 0;
    /* Enable OC1 as output. */
//    DDRC = _BV(PC7);
    /* Enable timer 1 overflow interrupt. */
//    TIMSK4 = _BV(TOIE4);
//    sei ();


    /* enable fan pwm output
     * arduino pin
     * Digital Pin 5 (PWM)
     * Atmega pin PC6
     * fan control 25khz see http://forum.arduino.cc/index.php?topic=18742.0
     * */
//    TCCR1A = _BV(COM1B1);
//    /* set prescaler to 8 */
//    TCCR1B = _BV(CS11)| _BV(WGM13);
//    /* set TOP */
//    ICR1 = 40;//79;
//    OCR1B = 0;
//    DDRB = _BV(PB6);

    /* enable pump pwm output
     * arduino pin
     * Digital Pin 5 (PWM)
     * Atmega pin PC6
     * */
    TCCR3A = _BV(COM3A1);
    /* no prescaler */
    TCCR3B = _BV(CS31) | _BV(WGM33);
    /* TOP is MAX */
    ICR3 = 40;//0xffff;
    OCR3A = 0;
    DDRC = _BV(PC6) | _BV(PC7);

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

                        CDC_Device_USBTask(&VirtualSerial_CDC_InterfaceR);
                        USB_USBTask();

                xTaskResumeAll();

                vTaskDelay((portTickType) taskDelayPeriod );
        }

}

