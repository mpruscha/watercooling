/*
 * temp_sensors.h
 *
 *  Created on: Jan 30, 2015
 *      Author: martin
 */

#ifndef TEMP_SENSORS_H_
#define TEMP_SENSORS_H_



#define MAXSENSORS 5
#define NEWLINESTR "\r\n"

uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];

uint8_t search_sensors(void);



#endif /* TEMP_SENSORS_H_ */
