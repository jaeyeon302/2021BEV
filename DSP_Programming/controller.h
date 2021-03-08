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

// ±è»óÈÆ Àú ) ¸ðÅÍÁ¦¾î 119, 145 page
#define Wc 100 // [Hz] control system bandwidth
#define Ls ((float32)0.104/1000) // [H]
#define Rs ((float32)0.1) //[ohm]
#define Kp_BLDC Ls*Wc
#define Ki_BLDC Rs*Wc
#define Ka_BLDC ((float32)1/Kp_BLDC);


#define CURRENT_SENSOR_VOLTAGE_SLOPE ((float32)55.0/1000.0) //wcs1800 [mV/A]
#define CURRENT_SENSOR_VOLTAGE_OFFSET_FOR_ZERO 1.65 // [V]
#define CURRENT_LIMIT_SCALE 0.33 //

#define BAT_VOLTAGE 48.0 //[V]
#define BAT_VOLTAGE_SCALER ((float32)BAT_VOLTAGE/3.3) //
#define ADC_Voltage2Current ((float32)1.0/CURRENT_SENSOR_VOLTAGE_SLOPE) // 1.65V -> MAX_CURRENT [A]

#define FLAG_ADC_CURRENT_PHASE_U_SAMPLED  0X01 // 0000 0001
#define FLAG_ADC_CURRENT_PHASE_V_SAMPLED  0X02 // 0000 0010
#define FLAG_ADC_CURRENT_PHASE_W_SAMPLED  0X04 // 0000 0100
#define FLAG_ADC_THROTTLE_SAMPLED  0x08    // 0000 1000
#define FLAG_ADC_BATTERY_SAMPLED  0x10  // 0001 0000

void Ready_controller();
void Start_controller();

enum phase{phaseU=0,phaseV=1,phaseW=2, not=3};

typedef struct _current_controller{
    float32 I_ref, I_fb, I_err;
    float32 alpha, Kp, Ki, Ka, K_ff;
    float32 V_err_integ;
    float32 V_ref;
    float32 V_ref_fb; //voltage reference feedforward
    float32 V_ref_ff; // voltage reference feedback
    float32 V_sat; // saturation voltage
    float32 V_anti; // anti windup
}Current_controller;

#endif /* CONTROLLER_ */
