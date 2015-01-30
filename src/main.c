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

int BytesAvailable = 0;
char buffer[100];
char end_buffer[100];


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


void handle_input(char* input)
{

    if(state == START)
    {
        if (strcmp("set fan\r",input) == 0)
        {
            CDC_Device_SendString (&VirtualSerial_CDC_InterfaceR, "enter fan speed, 0...40 \n\r");

            /* advance state machine */
            state = SET_FAN;


        }
        else if(strcmp("set pump\r", input)==0)
        {
            CDC_Device_SendString (&VirtualSerial_CDC_InterfaceR, "enter pump speed, 0...35  \n\r");

            /* advance state machine */
            state = SET_PUMP;
        }
        else if(strcmp("set led\r", input)==0)
        {
            CDC_Device_SendString (&VirtualSerial_CDC_InterfaceR, "enter led brightness, 0...255  \n\r");

            /* advance state machine */
            state = SET_LED;
        }
        else if(strcmp("read temp\r", input)==0)
        {
            CDC_Device_SendString (&VirtualSerial_CDC_InterfaceR, "reading temp...  \n\r");

            /* advance state machine */
            state = READ_TEMP;
        }
        else
        {
            /* didnt understand string */
            CDC_Device_SendString (&VirtualSerial_CDC_InterfaceR, "what was that??? \n\r");

            /* dont change state */
            state = START;
        }
    }
    else if(state == SET_LED)
    {
        static uint16_t pwm;
        char *garbage = NULL;
        pwm = strtol(input, &garbage, 0);
        OCR4A = pwm;

        CDC_Device_SendString (&VirtualSerial_CDC_InterfaceR, "led set. \n\r");

        /* go back to start */
        state = START;
    }
    else if(state == SET_FAN)
    {
        static uint16_t pwm;
        char *garbage = NULL;
        pwm = strtol(input, &garbage, 0);
        OCR1B = pwm;

        CDC_Device_SendString (&VirtualSerial_CDC_InterfaceR, "fan set. \n\r");

        /* go back to start */
        state = START;
    }
    else if(state == SET_PUMP)
    {
        static uint16_t pwm;
        char *garbage = NULL;
        pwm = strtol(input, &garbage, 0);
        OCR3A = pwm;

        CDC_Device_SendString (&VirtualSerial_CDC_InterfaceR, "pump set. \n\r");

        /* go back to start */
        state = START;
    }
    else if(state == READ_TEMP)
    {
        uint8_t i;
        int16_t decicelsius;
        uint8_t error;
        uint8_t id[OW_ROMCODE_SIZE];
        uint8_t diff, nSensors;

        char itoa_buffer[10];

        ow_set_bus(&PIND,&PORTD,&DDRD,PD4);

        ow_reset();

        nSensors = 0;

        nSensors = search_sensors();

        CDC_Device_SendString (&VirtualSerial_CDC_InterfaceR, "Number of sensors found: ");
        itoa(nSensors, itoa_buffer, 10);
        CDC_Device_SendString (&VirtualSerial_CDC_InterfaceR, itoa_buffer);
        CDC_Device_SendString (&VirtualSerial_CDC_InterfaceR, "\n\r");

        /* classify sensors */
        for ( i = 0; i < nSensors; i++ ) {
            if ( gSensorIDs[i][0] == DS18B20_FAMILY_CODE ) {
                CDC_Device_SendString (&VirtualSerial_CDC_InterfaceR, "DS18B20 found " );
            }

            if ( DS18X20_get_power_status( &gSensorIDs[i][0] ) == DS18X20_POWER_PARASITE ) {
                CDC_Device_SendString (&VirtualSerial_CDC_InterfaceR, "parasite\n\r" );
            } else {
                CDC_Device_SendString (&VirtualSerial_CDC_InterfaceR, "externally\n\r" );
            }
        }
//        for ( i = nSensors; i > 0; i-- )
//        {
//            if ( DS18X20_start_meas( DS18X20_POWER_EXTERN,
//                            &gSensorIDs[i-1][0] ) == DS18X20_OK )
        if ( DS18X20_start_meas( DS18X20_POWER_EXTERN, NULL )
            == DS18X20_OK)
        {
            _delay_ms(DS18B20_TCONV_12BIT);

            for ( i = 0; i < nSensors; i++ )
            {

                if ( DS18X20_read_decicelsius( &gSensorIDs[i][0], &decicelsius)
                     == DS18X20_OK )
                {
                    CDC_Device_SendString (&VirtualSerial_CDC_InterfaceR, "temp: ");
                    itoa(decicelsius, itoa_buffer,10);
                    CDC_Device_SendString (&VirtualSerial_CDC_InterfaceR, itoa_buffer);
                    CDC_Device_SendString (&VirtualSerial_CDC_InterfaceR, "\n\r ");
                }

                int32_t temp_eminus4;
                if ( DS18X20_read_maxres( &gSensorIDs[i][0], &temp_eminus4 )
                     == DS18X20_OK )
                {
                    DS18X20_format_from_maxres( temp_eminus4, itoa_buffer, 10 );

                    CDC_Device_SendString (&VirtualSerial_CDC_InterfaceR, "temp max res: ");
                    CDC_Device_SendString (&VirtualSerial_CDC_InterfaceR, itoa_buffer);
                    CDC_Device_SendString (&VirtualSerial_CDC_InterfaceR, "\n\r ");

                }

            }

        }


        /* go back to start */
        state = START;

    }
    else
    {
        CDC_Device_SendString (&VirtualSerial_CDC_InterfaceR, "went into unknown state!? \n\r");

        state = START;

    }


}



static void vLedPwm( void *pvParameters  )
{
	/* change the PWM duty cycle to get a pulsing LED */
	 TickType_t xLastWakeTime;
	 const TickType_t xFrequency = 100;

	 // Initialise the xLastWakeTime variable with the current time.
	 xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{
		/* each cycle, increment or decrement the pwm duty cycle */
		if(ucUp == true)
		{
			/* count up */
			ucPwm = ucPwm + 30;
			if(ucPwm >= ucPwmMax)
			{
				ucUp = false;
			}

		}
		else if(ucUp == false)
		{
			/* count down */
			ucPwm = ucPwm - 20;

			if(ucPwm <= ucPwmMin)
			{
				ucUp = true;
			}

		}

		OCR4A = ucPwm;

        vTaskDelayUntil( &xLastWakeTime, xFrequency );

	}
}


static void vUserInput ( void )
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 100;

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {

        int count = 0;
        buffer[0] = 0;
    	vTaskSuspendAll();
        BytesAvailable = CDC_Device_BytesReceived (&VirtualSerial_CDC_InterfaceR);
        for (count=0; count < BytesAvailable; count++) {
          buffer[count] = CDC_Device_ReceiveByte (&VirtualSerial_CDC_InterfaceR);
        }
		xTaskResumeAll();


        /* Null terminate buffer*/
        buffer[count] = 0;

        /* Buffer is not empty, print it */
        if ( buffer[0] != 0 )
        {
        	vTaskSuspendAll();
    		CDC_Device_SendString (&VirtualSerial_CDC_InterfaceR, "received something! \n\r");
    		xTaskResumeAll();

//        	CDC_Device_SendString (&VirtualSerial_CDC_InterfaceR, "received something! \n\r");

//            /* combine into buffer */
//            strncat(end_buffer, buffer, 1);
//
////            CDC_Device_SendByte (&VirtualSerial_CDC_InterfaceR, '\r');
//            CDC_Device_SendString (&VirtualSerial_CDC_InterfaceR, buffer);
//
//            if(buffer[0]=='\r')
//            {
//                CDC_Device_SendString (&VirtualSerial_CDC_InterfaceR, "\n\r");
//                strncat(end_buffer, '\0', 1);
//                CDC_Device_SendString (&VirtualSerial_CDC_InterfaceR, "\n\r");
//                CDC_Device_SendString (&VirtualSerial_CDC_InterfaceR, end_buffer);
//
////                handle_input(end_buffer);
//
//                end_buffer[0] = '\0';
//
//                CDC_Device_SendString (&VirtualSerial_CDC_InterfaceR, "enter command: ");
//
//            }
//
        }



        vTaskDelayUntil( &xLastWakeTime, xFrequency );

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


int main( void )
{

	/* Setup the LED's for output. */
	ioinit();
	setup_usb();
    GlobalInterruptEnable();

	prvIncrementResetCount();

	xTaskCreate( VirtualSerialTask, (const portCHAR *) "ViSeTask", configMINIMAL_STACK_SIZE, NULL, ViSe_TASK_PRIORITY, NULL );

	xTaskCreate( vLedPwm, ( const portCHAR * )"Ape", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL );
	xTaskCreate( vUserInput , ( const portCHAR * )"ui", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL );


	vTaskStartScheduler();


	return 0;

}



