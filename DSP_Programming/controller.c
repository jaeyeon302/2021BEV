/*
 * controller.c
 *
 *  Created on: 2021. 3. 5.
 *      Author: Jaeyeon Park
 */

#include "controller.h"
Uint32 controlCycleCount = 0;
unsigned char adc_result_flag = 0x00;

#define ADC_CURRENT_PHASE_U_SAMPLED  0X01 // 0000 0001
#define ADC_CURRENT_PHASE_V_SAMPLED  0X02 // 0000 0010
#define ADC_CURRENT_PHASE_W_SAMPLED  0X04 // 0000 0100
#define ADC_THROTTLE_SAMPLED  0x08    // 0000 1000
#define ADC_BATTERY_SAMPLED  0x10  // 0001 0000
const unsigned char ADC_ALL_SAMPLED = (ADC_CURRENT_PHASE_U_SAMPLED
                                      |ADC_CURRENT_PHASE_V_SAMPLED
                                      |ADC_CURRENT_PHASE_W_SAMPLED
                                      |ADC_THROTTLE_SAMPLED
                                      |ADC_BATTERY_SAMPLED);

Uint16 phase_current_result[3]={0, 0, 0};
Uint16 throttle_result = 0;
Uint16 battery_level_result = 0;

void control(){
    Commutation_state commutation = hall_sensor_get_commutation();
}

void control_state_update(enum ADC_RESULT_TYPE type, Uint16 adc_result){
    switch(type){
    case ADCcurrentPhaseU:
        adc_result_flag |= ADC_CURRENT_PHASE_U_SAMPLED;
        phase_current_result[type] = adc_result;
        break;
    case ADCcurrentPhaseV:
        adc_result_flag |= ADC_CURRENT_PHASE_V_SAMPLED;
        phase_current_result[type] = adc_result;
        break;
    case ADCcurrentPhaseW:
        adc_result_flag |= ADC_CURRENT_PHASE_W_SAMPLED;
        phase_current_result[type] = adc_result;
        break;
    case ADCthrottle:
        adc_result_flag |= ADC_THROTTLE_SAMPLED;
        throttle_result = adc_result;
        break;
    case ADCbatteryLevel:
        adc_result_flag |= ADC_BATTERY_SAMPLED;
        battery_level_result = adc_result;
        break;
    default:
        break;
    }

    if(adc_result_flag == ADC_ALL_SAMPLED ){
        controlCycleCount++;
        control();
        adc_result_flag = 0x00; //re-initialize for next sampling
    }
}


void Ready_controller(){
    /* InitPieCtrl(), InitPieVectTable() should be called
     * before this function.
     * Also IER, IFR should be initialized by 0x0000;
     *
     * Initialize ePWM, ADC, ECAP assume pie vector table was initialized
     */


    // initialize ������ �߿��ϴ�
    // ADC, ECAP �� ePWM���� ���� init �Ǿ���Ѵ�
    // ���� �׷��� ������ ADC, ECAP�� initialize �Ǵ� ����
    // start_controller�� ���� ePWM�� ���۵ǰ� ePWM cycle (2~3��)�� �׳� ������������
    // ePWM -> ADC ȣ�� -> �����Լ� ȣ�� �����̱� ������, ADC�� ePWM���� ���� ������ �Ϸ��ؾ��Ѵ�.
    Init_3current_ADC( &control_state_update );
    Init_misc_ADC();
    Init_hall_sensor_ECAP(23);
    Init_3phase_ePWM();
}

void Start_controller(){
    Start_hall_sensor_ECAP();
    Start_3phase_ePWM();
    //Start_3current_ADC();
}
