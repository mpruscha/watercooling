/*
 * temp_sensors.c
 *
 *  Created on: Jan 28, 2015
 *      Author: martin
 */

#include "onewire.h"
#include "ds18x20.h"
#include "temp_sensors.h"


uint8_t search_sensors(void)
{
    uint8_t i;
    uint8_t id[OW_ROMCODE_SIZE];
    uint8_t diff, nSensors;

    ow_reset();

    nSensors = 0;

    diff = OW_SEARCH_FIRST;
    while ( diff != OW_LAST_DEVICE && nSensors < MAXSENSORS ) {
        DS18X20_find_sensor( &diff, &id[0] );

        if( diff == OW_PRESENCE_ERR ) {
//            CDC_Device_SendString (&VirtualSerial_CDC_InterfaceR, "No Sensor found \n\r");
            break;
        }

        if( diff == OW_DATA_ERR ) {
//            CDC_Device_SendString (&VirtualSerial_CDC_InterfaceR, "Bus Error \n\r");
            break;
        }

        for ( i=0; i < OW_ROMCODE_SIZE; i++ )
            gSensorIDs[nSensors][i] = id[i];

        nSensors++;
    }

    return nSensors;
}

