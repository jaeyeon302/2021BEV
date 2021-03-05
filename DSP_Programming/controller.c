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

enum phase{u=0,v=1,w=2, not=3};


Uint16 phase_current_result[3]={0, 0, 0};
// 0A ~ value 2048.
// (because the current sensor make output voltage 1.65V when there is no current flow)
const Uint16 current_result_offset = 2048;
const Uint16 throttle2current_prescaler = 2;

Uint16 throttle_result = 0; // min 0, max 4096
Uint16 battery_level_result = 0;

void control(){
    Commutation_state c = hall_sensor_get_commutation(); // current commutation

    float32 cmpa[3] = {0,0,0}; // high stage switch
    float32 cmpb[3] = {0,0,0}; // low stage switch
    enum phase off_phase;
    static Uint16 last_current_result[3]={0,0,0};

    Uint16 i_cmd = throttle_result/throttle2current_prescaler;

    if(c.hu & !c.hv & c.hw){
        // 1 0 1
        off_phase = phase.w;
    }else if(c.hu & !c.hv & !c.hw){
        // 1 0 0
        off_phase = phase.v;
    }else if(c.hu & c.hv & !c.hw){
        // 1 1 0
        off_phase = phase.u;
    }else if(!c.hu & c.hv & !c.hw){
        // 0 1 0
        off_phase = phase.w;
    }else if(!c.hu & c.hv & c.hw){
        // 0 1 1
        off_phase = phase.v;
    }else if(!c.hu & !c.hv & c.hw){
        // 0 0 1
        off_phase = phase.u;
    }

    cmpa[off_phase] = 0.0;
    cmpb[off_phase] = 0.0;

    // update last_current_result
    memcpy(last_current_result, phase_current_result, sizeof(Uint16)*3);
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


    // initialize 순서가 중요하다
    // ADC, ECAP 이 ePWM보다 먼저 init 되어야한다
    // 만일 그렇지 않으면 ADC, ECAP이 initialize 되는 동안
    // start_controller에 의해 ePWM이 시작되고 ePWM cycle (2~3개)가 그냥 지나가버린다
    // ePWM -> ADC 호출 -> 제어함수 호출 구조이기 때문에, ADC를 ePWM보다 먼저 설정을 완료해야한다.
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
