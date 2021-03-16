/*
 * hall_sensor2.h
 *
 *  Created on: 2021. 3. 12.
 *      Author: Jaeyeon Park
 */

#ifndef HALL_SENSOR_H_
#define HALL_SENSOR_H_
#include <F28x_Project.h>
#include "constants.h"
typedef unsigned char byte;

typedef struct {
    byte hu:1;
    byte hv:1;
    byte hw:1;
    byte rsvd:5;
    float32 angle_E_rad;
}Hall_state;


void Init_hall_sensor(Uint16 pole_pair);
void Start_hall_sensor();

Hall_state hall_sensor_update();



#endif /* HALL_SENSOR2_C_ */
