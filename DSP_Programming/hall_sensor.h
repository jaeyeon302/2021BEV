/*
 * hall_sensor.h
 *
 *  Created on: 2021. 3. 2.
 *      Author: Jaeyeon Park
 */

#ifndef HALL_SENSOR_H_
#define HALL_SENSOR_H_
#include <F28x_Project.h>
#include "F2837xD_ecap.h"
void Init_hall_sensor_ECAP(Uint16 poll_pair);
void Start_hall_sensor_ECAP();
#endif /* HALL_SENSOR_H_ */
