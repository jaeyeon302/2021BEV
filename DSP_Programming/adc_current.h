/*
 * adc_current.h
 *
 *  Created on: 2021. 3. 4.
 *      Author: Jaeyeon Park
 */

#ifndef ADC_CURRENT_H_
#define ADC_CURRENT_H_
#include <F28x_Project.h>

#define NOT_SAMPLED -1
#define ADC_CLK_PRESCALE 6

#define ADC_12bit_Ain2Voltage ((float32)3.3/4096.0) //4096 -> 3v3
#define ADC_16bit_Ain2Voltage ((float32)3.3/65536.0) //65536 -> 3v3


enum ADC_RESULT_TYPE{ADCcurrentPhaseU=0,
                    ADCcurrentPhaseV=1,
                    ADCcurrentPhaseW=2,
                    ADCthrottle=3,
                    ADCbatteryLevel=4};


void Init_3current_ADC( void (*after_sample_adc)(enum ADC_RESULT_TYPE, float32)   );
void Init_misc_ADC();

void Start_3current_ADC();

#endif /* ADC_CURRENT_H_ */
