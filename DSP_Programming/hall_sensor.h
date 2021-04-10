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
    float32 angle_E_offset_rad;
    float32 angle_E_rad;
    float64 Wr;
    int32 rotation;
}Hall_state;


void Init_hall_sensor(float32 update_period);
void Start_hall_sensor();
Hall_state get_hall_state();
Hall_state hall_sensor_update();
void hall_sensor_set_angle_offset_rad(float64 offset_rad);
float64 hall_sensor_get_E_angle_rad();
float64 hall_sensor_get_M_angle_rad();
float64 hall_sensor_get_E_angular_speed(); // rad/sec
float64 hall_sensor_get_M_angular_speed();




#endif /* HALL_SENSOR2_C_ */
