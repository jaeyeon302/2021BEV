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
#include "constants.h"

#define TSCTR_PER_SEC 200000000
typedef unsigned char byte;

typedef struct {
    byte hu:1;
    byte hv:1;
    byte hw:1;
    byte rsvd:5;
}Commutation_state ;

void Init_hall_sensor_ECAP(Uint16 pole_pair);
void Start_hall_sensor_ECAP();
Commutation_state hall_sensor_get_commutation();

float32 hall_sensor_get_M_angle_position(); // radian
float32 hall_sensor_get_E_angle_position(); // radian
float32 hall_sensor_get_angle_speed(); // radian per second

#endif /* HALL_SENSOR_H_ */
