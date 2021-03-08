/*
 * controller.c
 *
 *  Created on: 2021. 3. 5.
 *      Author: Jaeyeon Park
 */

#include "controller.h"
Uint32 controlCycleCount = 0;
unsigned char adc_result_flag = 0x00;
const unsigned char ADC_ALL_SAMPLED = (FLAG_ADC_CURRENT_PHASE_U_SAMPLED
                                      |FLAG_ADC_CURRENT_PHASE_V_SAMPLED
                                      |FLAG_ADC_CURRENT_PHASE_W_SAMPLED
                                      |FLAG_ADC_THROTTLE_SAMPLED
                                      |FLAG_ADC_BATTERY_SAMPLED);

#define current_limit 8// [A]

enum phase{phaseU=0,phaseV=1,phaseW=2, not=3};



float32 phase_current_result[3]={0, 0, 0}; //[Ampere]

float32 throttle_result = 0; //[Ampere]
float32 battery_level_result = 0; // 0 - 48[V]




float32 PIcontroller(enum phase p, Uint16 i_command, Uint16 i_measured){
    // i_commnad -> i[n], i_measured -> i[n-1]
    static float32 err_sum[3] = {0,0,0};
    const float32 Kp = 0.1;
    const float32 Ki = 0.1;
    float32 err = (float32)(i_command - i_measured);
    err_sum[p] += err;
    return Kp*err + Ki*err_sum[p];
}


void control(){
    Commutation_state c = hall_sensor_get_commutation(); // current commutation

    float32 cmpa_ratio[3] = {0,0,0}; // high stage switch
    float32 cmpb_ratio[3] = {0,0,0}; // low stage switch
    enum phase off_phase;

    float32 i_cmd = (throttle_result); //[A]
    float32 duty = 0;
    if(c.hu & !c.hv & c.hw){
        // 1 0 1
        // control UH->VL (Iu, Iv)
        // INV -> MTR 기준 Iu
        off_phase = phaseW;
        duty = PIcontroller(phaseU, i_cmd, phase_current_result[phaseU]);
        cmpa_ratio[phaseU] = duty;
        cmpb_ratio[phaseV] = duty;
    }else if(c.hu & !c.hv & !c.hw){
        // 1 0 0
        // control UH->WL (Iu, Iw)
        // INV -> MTR 기준 Iu
        off_phase = phaseV;
        duty = PIcontroller(phaseU,i_cmd, phase_current_result[phaseU]);
        cmpa_ratio[phaseU] = duty;
        cmpb_ratio[phaseW] = duty;
    }else if(c.hu & c.hv & !c.hw){
        // 1 1 0
        // control VH->WL (Iv, Iw)
        // INV -> MTR 기준 Iv
        off_phase = phaseU;
        duty = PIcontroller(phaseV,i_cmd, phase_current_result[phaseV]);
        cmpa_ratio[phaseV] = duty;
        cmpb_ratio[phaseW] = duty;
    }else if(!c.hu & c.hv & !c.hw){
        // 0 1 0
        // control VH->UL (Iv,Iu)
        // INV -> MTR 기준 Iv
        off_phase = phaseW;
        duty = PIcontroller(phaseV,i_cmd, phase_current_result[phaseV]);
        cmpa_ratio[phaseV] = duty;
        cmpb_ratio[phaseU] = duty;
    }else if(!c.hu & c.hv & c.hw){
        // 0 1 1
        // control WH->UL (Iw, Iu)
        // INV -> MTR 기준 Iw
        off_phase = phaseV;
        duty = PIcontroller(phaseW,i_cmd, phase_current_result[phaseW]);
        cmpa_ratio[phaseW] = duty;
        cmpb_ratio[phaseU] = duty;
    }else if(!c.hu & !c.hv & c.hw){
        // 0 0 1
        // control WH->VL (Iw, Iv)
        // INV -> MTR 기준 Iw
        off_phase = phaseU;
        duty = PIcontroller(phaseW,i_cmd, phase_current_result[phaseW]);
        cmpa_ratio[phaseW] = duty;
        cmpb_ratio[phaseV] = duty;
    }

    cmpa_ratio[off_phase] = 0.0;
    cmpb_ratio[off_phase] = 0.0;

    epwm1_set_duty(cmpa_ratio[phaseU],cmpb_ratio[phaseU]);
    epwm2_set_duty(cmpa_ratio[phaseV],cmpb_ratio[phaseV]);
    epwm3_set_duty(cmpa_ratio[phaseW],cmpb_ratio[phaseW]);
}

void control_state_update(enum ADC_RESULT_TYPE type, float32 adc_result_voltage){
    float32 current = (adc_result_voltage - CURRENT_SENSOR_VOLTAGE_OFFSET_FOR_ZERO ) * ADC_Voltage2Current; //[Ampere]

    switch(type){
    case ADCcurrentPhaseU:
        adc_result_flag |= FLAG_ADC_CURRENT_PHASE_U_SAMPLED;
        phase_current_result[type] = current;
        break;
    case ADCcurrentPhaseV:
        adc_result_flag |= FLAG_ADC_CURRENT_PHASE_V_SAMPLED;
        phase_current_result[type] = current;
        break;
    case ADCcurrentPhaseW:
        adc_result_flag |= FLAG_ADC_CURRENT_PHASE_W_SAMPLED;
        phase_current_result[type] = current;
        break;
    case ADCthrottle:
        adc_result_flag |= FLAG_ADC_THROTTLE_SAMPLED;
        throttle_result = adc_result_voltage*ADC_Voltage2Current*CURRENT_LIMIT_SCALE;
        break;
    case ADCbatteryLevel:
        adc_result_flag |= FLAG_ADC_BATTERY_SAMPLED;
        battery_level_result = adc_result_voltage*BAT_VOLTAGE_SCALER;
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
