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
#include "math.h"

#define ANGLE_OBSERVER_ZETA 0.707
#define ANGLE_OBSERVER_Wn  50.0
#define ANGLE_OBSERVER_KP (2*ANGLE_OBSERVER_ZETA*ANGLE_OBSERVER_Wn)
#define ANGLE_OBSERVER_KI (ANGLE_OBSERVER_Wn*ANGLE_OBSERVER_Wn)

typedef unsigned char byte;

typedef struct {
    byte hu:1;
    byte hv:1;
    byte hw:1;
    byte rsvd:5;
    float32 angle_E_offset_rad;
    float32 angle_E_rad;
    float64 Wr;
    byte bounded;
    int32 rotation;

}Hall_state;

typedef struct{
    float32 Kp, Ki;
    float32 Wr;
    float32 angle;
}Angle_observer;

void Init_angle_observer();
float32 angle_observer_update(float32 hall_sensor_angle);
Angle_observer get_angle_observer();
float32 calibrate_angle_offset(float32 angle);
void Init_hall_sensor();
void Start_hall_sensor();
Hall_state get_hall_state();
Hall_state hall_sensor_update();
void hall_sensor_set_angle_offset_rad(float64 offset_rad);
float64 hall_sensor_get_E_angle_rad();
float64 hall_sensor_get_E_angular_speed(); // rad/sec
float64 hall_sensor_get_M_angular_speed();

#endif /* HALL_SENSOR2_C_ */
