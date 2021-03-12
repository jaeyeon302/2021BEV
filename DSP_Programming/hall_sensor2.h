/*
 * hall_sensor2.h
 *
 *  Created on: 2021. 3. 12.
 *      Author: Jaeyeon Park
 */

#ifndef HALL_SENSOR2_C_
#define HALL_SENSOR2_C_
#include <F28x_Project.h>
typedef unsigned char byte;

typedef struct {
    byte hu:1;
    byte hv:1;
    byte hw:1;
    byte rsvd:5;
}Hall_state;


void Init_hall_sensor(Uint16 pole_pair, Uint32 sampling_frequency);
void Start_hall_sensor();

float32 hall_sensor_get_angle_speed();
Hall_state hall_sensor_update();
float32 hall_sensor_state();



#endif /* HALL_SENSOR2_C_ */
