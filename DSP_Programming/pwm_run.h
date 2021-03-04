/*
 * pwm_run.h
 *
 *  Created on: 2021. 3. 3.
 *      Author: Jaeyeon Park
 */

#ifndef PWM_RUN_H_
#define PWM_RUN_H_
#include <F28x_Project.h>
typedef struct duty{
    float32 a_duty_ratio;
    float32 b_duty_ratio;
} PWM_duty;


void Init_3phase_ePWM();
void Start_3phase_ePWM();
float32 epwm_get_minimum_duty_ratio();
void epwm1_set_duty(float32 CMPA_ratio, float32 CMPB_ratio);
void epwm2_set_duty(float32 CMPA_ratio, float32 CMPB_ratio);
void epwm3_set_duty(float32 CMPA_ratio, float32 CMPB_ratio);

#endif /* PWM_H_ */
