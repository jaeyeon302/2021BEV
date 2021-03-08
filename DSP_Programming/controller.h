/*
 * controller.h
 *
 *  Created on: 2021. 3. 5.
 *      Author: Jaeyeon Park
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <F28x_Project.h>
#include "hall_sensor.h"
#include "pwm_run.h"
#include "adc_current.h"

#define CURRENT_SENSOR_VOLTAGE_SLOPE ((float32)55.0/1000.0); //wcs1800 [mV/A]
#define CURRENT_SENSOR_VOLTAGE_OFFSET_FOR_ZERO 1.65 // [V]
#define CURRENT_LIMIT_SCALE 0.33 //

#define BAT_VOLTAGE 48.0 //[V]
#define BAT_VOLTAGE_SCALER ((float32)BAT_VOLTAGE/3.3_; //
#define ADC_Voltage2Current ((float32)1.0/CURRENT_SENSOR_VOLTAGE_SLOPE) // 1.65V -> MAX_CURRENT [A]

#define FLAG_ADC_CURRENT_PHASE_U_SAMPLED  0X01 // 0000 0001
#define FLAG_ADC_CURRENT_PHASE_V_SAMPLED  0X02 // 0000 0010
#define FLAG_ADC_CURRENT_PHASE_W_SAMPLED  0X04 // 0000 0100
#define FLAG_ADC_THROTTLE_SAMPLED  0x08    // 0000 1000
#define FLAG_ADC_BATTERY_SAMPLED  0x10  // 0001 0000

void Ready_controller();
void Start_controller();

#endif /* CONTROLLER_ */
