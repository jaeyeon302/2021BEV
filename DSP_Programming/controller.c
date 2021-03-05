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

#define current_real_result_max 30 //[A] Adc result 4096일 때 흐르는 전류값
#define current_result_offset 2048
#define current_limit 8// [A]

enum phase{phaseU=0,phaseV=1,phaseW=2, not=3};



Uint16 phase_current_result[3]={0, 0, 0};
// -current_real_result_max ~ value 0.
// +current_real_result_max ~ value 4096 (expectation value based on 55mV/A, wcs1800)
// 0A ~ value 2048.
// (because the current sensor make output voltage 1.65V when there is no current flow)


// min 0 ~ max 4096
Uint16 throttle_result = 0;
Uint16 battery_level_result = 0; // 4096 <-> 3.3V <-> 48V BAT


float32 throttle2current_prescaler = 2*current_real_result_max / current_limit;


float32 adc_current_result_to_real_current_value(Uint16 adc_current_result){
    return ( ((float32)adc_current_result) - current_result_offset) * 30 / 2048.0;
}

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
    int32 current_result[3];

    float32 cmpa_ratio[3] = {0,0,0}; // high stage switch
    float32 cmpb_ratio[3] = {0,0,0}; // low stage switch
    enum phase off_phase;

    for(int i = 0; i<3; i++){
        // remove offset
        current_result[i] = (int32)(phase_current_result[i] - current_result_offset);
    }
    // current_result
    // -2048 : - current_real_result_max[A]
    // 0 : 0A
    // 2048 : + current_real_result_max[A]

    int32 i_cmd = (int32)(throttle_result/throttle2current_prescaler);
    float32 duty = 0;
    if(c.hu & !c.hv & c.hw){
        // 1 0 1
        // control UH->VL (Iu, Iv)
        // INV -> MTR 기준 Iu
        off_phase = phaseW;
        duty = PIcontroller(phaseU, i_cmd, current_result[phaseU]);
        cmpa_ratio[phaseU] = duty;
        cmpb_ratio[phaseV] = duty;
    }else if(c.hu & !c.hv & !c.hw){
        // 1 0 0
        // control UH->WL (Iu, Iw)
        // INV -> MTR 기준 Iu
        off_phase = phaseV;
        duty = PIcontroller(phaseU,i_cmd,current_result[phaseU]);
        cmpa_ratio[phaseU] = duty;
        cmpb_ratio[phaseW] = duty;
    }else if(c.hu & c.hv & !c.hw){
        // 1 1 0
        // control VH->WL (Iv, Iw)
        // INV -> MTR 기준 Iv
        off_phase = phaseU;
        duty = PIcontroller(phaseV,i_cmd,current_result[phaseV]);
        cmpa_ratio[phaseV] = duty;
        cmpb_ratio[phaseW] = duty;
    }else if(!c.hu & c.hv & !c.hw){
        // 0 1 0
        // control VH->UL (Iv,Iu)
        // INV -> MTR 기준 Iv
        off_phase = phaseW;
        duty = PIcontroller(phaseV,i_cmd,current_result[phaseV]);
        cmpa_ratio[phaseV] = duty;
        cmpb_ratio[phaseU] = duty;
    }else if(!c.hu & c.hv & c.hw){
        // 0 1 1
        // control WH->UL (Iw, Iu)
        // INV -> MTR 기준 Iw
        off_phase = phaseV;
        duty = PIcontroller(phaseW,i_cmd,current_result[phaseW]);
        cmpa_ratio[phaseW] = duty;
        cmpb_ratio[phaseU] = duty;
    }else if(!c.hu & !c.hv & c.hw){
        // 0 0 1
        // control WH->VL (Iw, Iv)
        // INV -> MTR 기준 Iw
        off_phase = phaseU;
        duty = PIcontroller(phaseW,i_cmd,current_result[phaseW]);
        cmpa_ratio[phaseW] = duty;
        cmpb_ratio[phaseV] = duty;
    }

    cmpa_ratio[off_phase] = 0.0;
    cmpb_ratio[off_phase] = 0.0;

    epwm1_set_duty(cmpa_ratio[phaseU],cmpb_ratio[phaseU]);
    epwm2_set_duty(cmpa_ratio[phaseV],cmpb_ratio[phaseV]);
    epwm3_set_duty(cmpa_ratio[phaseW],cmpb_ratio[phaseW]);
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
