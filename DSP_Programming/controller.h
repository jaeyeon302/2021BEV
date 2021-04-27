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
#include "math.h"
#include "constants.h"
/* */
#define CONTROL_FREQ 10000.0
#define CONTROL_PRD (1.0/CONTROL_FREQ)

/* motor coefficients */
#define Ls_DEFAULT ((float32)0.1/1000) //[H]
#define Rs_DEFAULT ((float32)0.1) // [Ohm]
#define flux_DEFAULT 0.0227

/* control coefficients */
// 김상훈 저 ) 모터제어 119, 145 page
#define Wc 625 // 2*PI*f
#define Kp_DEFAULT 62.5/1000//Ls_DEFAULT*Wc
#define Ki_DEFAULT 62.5//Rs_DEFAULT*Wc = 50
#define Ka_DEFAULT 1/Kp_DEFAULT
#define CURRENT_LIMIT_SCALE 0.75 // FOR SAFETY
// 0.99 -> max 36A
// 0.75 -> max 30A
// 0.66 -> max 24A
// 0.55 -> max 20A
// 0.33 -> max 12A
// 0.15 -> max 6A
// 0.11 -> max 4A
// 0.075 -> max 3A
// 0.055 -> max 2A
// 0.0325 -> max 1.5A

/* current sensor coefficients */
// #define CURRENT_SENSOR_VOLTAGE_SLOPE ((float32)55.0 / 1000.0) // [V/A] WCS1800
#define CURRENT_SENSOR_VOLTAGE_SLOPE ((float32)30.0 / 1000.0) // [V/A] WCS1700
//#define CURRENT_SENSOR_VOLTAGE_OFFSET_FOR_ZERO 1.95 //1.65 //[V]

/* Battery Coefficients */
#define BAT_DEFAULT_VOLTAGE 48.0 // [V]
#define BAT_VOLTAGE_SCALER ((float32)BAT_DEFAULT_VOLTAGE/3.3) //

/* ADC coefficients and flags */
#define MAX_OFFSET_SAMPLE_COUNT 1000
#define ADC_Voltage2Current ((float32)1.0/CURRENT_SENSOR_VOLTAGE_SLOPE) // 1.65V(오프셋 제거된거) -> MAX_CURRENT [A]
#define FLAG_ADC_CURRENT_PHASE_U_SAMPLED  0X01 // 0000 0001
#define FLAG_ADC_CURRENT_PHASE_V_SAMPLED  0X02 // 0000 0010
#define FLAG_ADC_CURRENT_PHASE_W_SAMPLED  0X04 // 0000 0100
#define FLAG_ADC_THROTTLE_SAMPLED  0x08    // 0000 1000
#define FLAG_ADC_BATTERY_SAMPLED  0x10  // 0001 0000
#define FLAG_ALIGNED 0X20 // 0010 0000

void Ready_controller();
void Start_controller();

float32* get_3phase_currents();
float32* get_dqr_currents();

void test_angle_update(float32 unit_angle);
void test_Idq_update(float32 Id_ref, float32 Iq_ref);
void test_I_update(float32 current);
enum phase{phaseU=0,phaseV=1,phaseW=2, not=3};
typedef struct _current_controller{
  float32 I_ref;
  float32 V_ref;
  float32 V_sat;
  float32 V_anti;
  float32 V_int;
  float32 V_fb;
  float32 alpha, kp, ki, ka, kff;
}Current_Controller;


#endif /* CONTROLLER_ */
