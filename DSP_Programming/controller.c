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




Current_controller CCuvw[3];
Current_controller CCtmp;
float32 phase_current_result[3]={0, 0, 0}; //[Ampere]
float32 throttle_result = 0; //[Ampere]
float32 battery_level_result = 0; // 0 - 48[V]


/**********************************/
// Eunjin's code



float32 get_offset_voltage(float32 phase_a_v, float32 phase_b_v, float32 phase_c_v){

    float32 vmax = 0;
    float32 vmin = 0;
    float32 vsn = 0;
    if (phase_a_v > phase_b_v){
        vmax = phase_a_v;
        vmin = phase_b_v;
    }
    else{
        vmax = phase_b_v;
        vmin = phase_a_v;
    }
    if (phase_c_v > vmax){
        vmax = phase_c_v;
    }
    if (phase_c_v < vmin){
        vmin = phase_c_v;
    }
    vsn = 24-(vmax+vmin)/2;

    return vsn;
}

float32 Vd_anti = 0.0;
float32 Vd_ref = 0.0;
float32 Vd_int = 0.0;

float32 Vq_anti = 0.0;
float32 Vq_ref = 0.0;
float32 Vq_int = 0.0;

void control_sinusoidal_BEMF(){
    const float32 flux = 0.0021;
    const float32 ls = 0.0001;
    const float32 Vd_sat = 40;
    const float32 Vq_sat = 40;

    const float32 Kp_d = 1;//Lds*wc
    const float32 Kp_q = 1;//Lqs*wc //0.1mH
    const float32 Ki = 40;//Rc=0.1ohm, wc=(1/25)*10kHz=0.4kHz //Ki=Rc*wc=40
    const float32 Ka_d = 1;//Ka_d = 1/Kp_d;
    const float32 Ka_q = 1;//Kp_q;

    // uvw -> DQ
    float32 angle = hall_sensor_get_E_angle_position();
    float32 Id,Iq, current_u, current_v, current_w;
    current_u = phase_current_result[phaseU];
    current_v = phase_current_result[phaseV];
    current_w = phase_current_result[phaseW];
    Id=(2/3)*(cos(angle)*current_u + cos(angle-2*PI/3)*current_v + cos(angle+2*PI/3)*current_w);
    Iq=(2/3)*(-sin(angle)*current_u - sin(angle-2*PI/3)*current_v - sin(angle+2*PI/3)*current_w);


    //PI
    float32 Id_ref = 0.0;
    float32 Iq_ref = throttle_result;

    float32 Id_err = Id_ref - Id;
    Vd_anti = Vd_ref - Vd_sat;
    Vd_int += Ki*(Id_err - Ka_d*Vd_anti);
    float32 Vd_fb = Vd_int + Kp_d*Id_err;


    float32 Iq_err = Iq_ref - Iq;
    Vq_anti = Vq_ref - Vq_sat;
    Vq_int += Ki*(Id_err - Ka_q*Vq_anti);
    float32 Vq_fb = Vq_int + Kp_q*Iq_err;

    float32 wr = hall_sensor_get_angle_speed();
    float32 Vd_ff = -ls*wr*Iq;
    float32 Vq_ff = wr*ls*Id + wr*flux;

    Vd_ref = Vd_fb + Vd_ff;
    Vq_ref = Vq_fb + Vq_ff;


    // DQ -> ABC
    float32 vsn=0;
    float32 Vdc=BAT_VOLTAGE;

    float32 voltage_command[3] = {0,0,0};

    //phase_voltage_command
    voltage_command[phaseU] = cos(angle)*Vd_ref - sin(angle)*Vq_ref;
    voltage_command[phaseV] = cos(angle-2*PI/3)*Vd_ref - sin(angle-2*PI/3)*Vq_ref;
    voltage_command[phaseW] = cos(angle+2*PI/3)*Vd_ref - sin(angle+2*PI/3)*Vq_ref;

    vsn = get_offset_voltage(voltage_command[phaseU], voltage_command[phaseV], voltage_command[phaseW]);

    //voltage command
    voltage_command[phaseU] = voltage_command[phaseU] + vsn;
    voltage_command[phaseV] = voltage_command[phaseV] + vsn;
    voltage_command[phaseW] = voltage_command[phaseW] + vsn;

    //
    voltage_command[phaseU] = (voltage_command[phaseU] > Vdc) ? Vdc : ((voltage_command[phaseU] < 0) ? 0 : voltage_command[phaseU]);
    voltage_command[phaseV] = (voltage_command[phaseV] > Vdc) ? Vdc : ((voltage_command[phaseV] < 0) ? 0 : voltage_command[phaseV]);
    voltage_command[phaseW] = (voltage_command[phaseW] > Vdc) ? Vdc : ((voltage_command[phaseW] < 0) ? 0 : voltage_command[phaseW]);

    //최종 결과는 abc상 전압 duty

    // GO TO EPWM
    float32 highside_duty_ratio[3] = {0,0,0};
    float32 lowside_duty_ratio[3] = {0,0,0};

    highside_duty_ratio[phaseU] = voltage_command[phaseU]/Vdc;
    lowside_duty_ratio[phaseU] = voltage_command[phaseU]/Vdc;
    highside_duty_ratio[phaseV] = voltage_command[phaseV]/Vdc;
    lowside_duty_ratio[phaseV] = voltage_command[phaseV]/Vdc;
    highside_duty_ratio[phaseW] = voltage_command[phaseW]/Vdc;
    lowside_duty_ratio[phaseW] = voltage_command[phaseW]/Vdc;

    epwm1_set_duty(highside_duty_ratio[phaseU], lowside_duty_ratio[phaseU]);
    epwm2_set_duty(highside_duty_ratio[phaseV], lowside_duty_ratio[phaseV]);
    epwm3_set_duty(highside_duty_ratio[phaseW], lowside_duty_ratio[phaseW]);


}


/***********************************/



void Current_control_init(Current_controller* cc){
    cc->I_ref = 0;
    cc->I_fb = 0;
    cc->I_err = 0;
    cc->Kp = 0;
    cc->Ki = 0;
    cc->Ka = 0;
    cc->K_ff = 0;
    cc->V_err_integ = 0;
    cc->V_ref = 0;
    cc->V_ref_fb = 0;
    cc->V_ref_ff = 0;
    cc->V_sat = 0;
    cc->V_anti = 0;
}


void Current_control_update(Current_controller* cc,
                               float32 i_ref,
                               float32 i_fb,
                               float32 Wr_rad_per_sec){
    float32 i_err = i_ref - i_fb;
    float32 anti_windup = cc->V_ref - cc->V_sat;
    cc->V_err_integ += cc->Ki*( i_err - cc->Ka*anti_windup );
    cc->V_ref = cc->Kp*(i_err) + cc->V_err_integ - cc->Ka*anti_windup;
    cc->V_ref_ff = Wr_rad_per_sec * cc->K_ff;

    cc->V_ref += cc->V_ref_ff;
    if(cc->V_ref>BAT_VOLTAGE){
        cc->V_ref = BAT_VOLTAGE;
    }
/*
    cc->alpha = 1;
    cc->I_ref = i_ref;
    cc->I_fb = i_fb;
    cc->I_err = i_ref - i_fb;
    cc->V_anti = cc->V_ref - cc->V_sat;
    cc->V_err_integ += cc->Ki*(cc->I_err - cc->Ka*cc->V_anti);
    cc->V_ref_fb = cc->V_err_integ + (cc->alpha*cc->Kp*cc->I_err) + (1 - cc->alpha)*i_fb;
    cc->V_ref_ff = Wr_rad_per_sec*cc->K_ff;
    cc->V_ref = cc->V_ref_fb + cc->V_ref_ff;*/
}

void Current_control_update_DQ(Current_controller* ccId,
                                  Current_controller* ccIq,
                                  float32 id_ref,
                                  float32 iq_ref,
                                  float32 id_fb,
                                  float32 iq_fb,
                                  float32 Wr_rad_per_sec){

}


void test_3phase_pole_voltage(float32 duty){
    float32 highside_duty_ratio[3] = {0,0,0};
    float32 lowside_duty_ratio[3] = {0,0,0};

    if(duty>1.0) duty = 1.0;
    if(duty<0.0) duty = 0.0;

    highside_duty_ratio[phaseU] = duty;
    lowside_duty_ratio[phaseU] = 1-duty;

    highside_duty_ratio[phaseV] = duty;
    lowside_duty_ratio[phaseV] = 1- duty;

    highside_duty_ratio[phaseW] = duty;
    lowside_duty_ratio[phaseW] = 1 - duty;


    epwm1_set_duty(highside_duty_ratio[phaseU], lowside_duty_ratio[phaseU]);
    epwm2_set_duty(highside_duty_ratio[phaseV], lowside_duty_ratio[phaseV]);
    epwm3_set_duty(highside_duty_ratio[phaseW], lowside_duty_ratio[phaseW]);
}

void test_just_run(Commutation_state c){
    float32 highside_duty_ratio[3] = {0,0,0};
    float32 lowside_duty_ratio[3] = {0,0,0};
    enum phase off_phase;
    float32 duty = 0.25;
    if(c.hu&!c.hv&c.hw){
        // 1 0 1
        // UH UL VH VL WH WL
        // 0 0 1 0 0 1
        // VH -> WL
        off_phase = phaseU; // floating U
        highside_duty_ratio[phaseV] = duty;
        lowside_duty_ratio[phaseV] = 1.0-duty;
        highside_duty_ratio[phaseW] = 0;
        lowside_duty_ratio[phaseW] = 1;
    }else if(c.hu & !c.hv & !c.hw){
        // 1 0 0
        // UH UL VH VL WH WL
        // 0 1 1 0 0 0
        // VH -> UL
        off_phase = phaseW;
        highside_duty_ratio[phaseV] = duty;
        lowside_duty_ratio[phaseV] = 1.0-duty;
        highside_duty_ratio[phaseU] = 0.0;
        lowside_duty_ratio[phaseU] = 1.0;
    }else if(c.hu & c.hv & !c.hw){
        // 1 1 0
        // UH UL VH VL WH WL
        // 0 1 0 0 1 0
        // WH -> UL
        off_phase = phaseV;
        highside_duty_ratio[phaseW] = duty;
        lowside_duty_ratio[phaseW] = 1.0-duty;
        highside_duty_ratio[phaseU] = 0.0;
        lowside_duty_ratio[phaseU] = 1.0;
    }else if(!c.hu & c.hv & !c.hw){
        // 0 1 0
        // UH UL VH VL WH WL
        // 0 0 0 1 1 0
        // WH -> VL
        off_phase = phaseU;
        highside_duty_ratio[phaseW] = duty;
        lowside_duty_ratio[phaseW] = 1.0-duty;
        highside_duty_ratio[phaseV] = 0.0;
        lowside_duty_ratio[phaseV] = 1.0;
    }else if(!c.hu & c.hv & c.hw){
        // 0 1 1
        // UH UL VH VL WH WL
        // 1 0 0 1 0 0
        // UH -> VL
        off_phase = phaseW;
        highside_duty_ratio[phaseU] = duty;
        lowside_duty_ratio[phaseU] = 1.0-duty;
        highside_duty_ratio[phaseV] = 0.0;
        lowside_duty_ratio[phaseV] = 1.0;
    }else if(!c.hu & !c.hv & c.hw){
        // 0 0 1
        // UH UL VH VL WH WL
        // 1 0 0 0 0 1
        // UH -> WL
        off_phase = phaseV;
        highside_duty_ratio[phaseU] = duty;
        lowside_duty_ratio[phaseU] = 1.0-duty;
        highside_duty_ratio[phaseW] = 0.0;
        lowside_duty_ratio[phaseW] = 1.0;
    }

    // floating
    highside_duty_ratio[off_phase] = 0.0;
    lowside_duty_ratio[off_phase] = 0.0;

    epwm1_set_duty(highside_duty_ratio[phaseU], lowside_duty_ratio[phaseU]);
    epwm2_set_duty(highside_duty_ratio[phaseV], lowside_duty_ratio[phaseV]);
    epwm3_set_duty(highside_duty_ratio[phaseW], lowside_duty_ratio[phaseW]);
}
/*

void control_trapezoidal_BEMF0(){
    Commutation_state c = hall_sensor_get_commutation();

    // FOR THE TEST
    // ONLY CONTROL HIGH SIDE OF U PHASE
    // 011
    c.hu = 0; c.hv = 1; c.hw = 1; // TODO REMOVE THIS LINE TO OPERATE MOTOR

    float32 highside_duty_ratio[3] = {0,0,0};
    float32 lowside_duty_ratio[3] = {0,0,0};

    enum phase off_phase;

    float32 i_ref = throttle_result;
    float32 i_fb = 0;
    float32 wr =  0;//hall_sensor_get_angle_speed();
    float32 duty = 0;
    if(c.hu&!c.hv&c.hw){
        // 1 0 1
        // UH UL VH VL WH WL
        // 0 0 1 0 0 1
        // VH -> WL
        off_phase = phaseU; // floating U
        i_fb = phase_current_result[phaseV];
        Current_control_update(&CCtmp,i_ref,i_fb,wr);
        duty = CCtmp.V_ref/BAT_VOLTAGE;
        highside_duty_ratio[phaseV] = duty;
        lowside_duty_ratio[phaseV] = (1-duty);
        highside_duty_ratio[phaseW] = 0;
        lowside_duty_ratio[phaseW] = 1;
    }else if(c.hu & !c.hv & !c.hw){
        // 1 0 0
        // UH UL VH VL WH WL
        // 0 1 1 0 0 0
        // VH -> UL
        off_phase = phaseW;
        i_fb = phase_current_result[phaseV];
        Current_control_update(&CCtmp,i_ref,i_fb,wr);
        duty = CCtmp.V_ref/BAT_VOLTAGE;
        highside_duty_ratio[phaseV] = duty;
        lowside_duty_ratio[phaseV] = (1.0-duty);
        highside_duty_ratio[phaseU] = 0.0;
        lowside_duty_ratio[phaseU] = 1.0;
    }else if(c.hu & c.hv & !c.hw){
        // 1 1 0
        // UH UL VH VL WH WL
        // 0 1 0 0 1 0
        // WH -> UL
        off_phase = phaseV;
        i_fb = phase_current_result[phaseW];
        Current_control_update(&CCtmp, i_ref, i_fb, wr);
        duty = CCtmp.V_ref/BAT_VOLTAGE; // or battery_level_result;
        highside_duty_ratio[phaseW] = duty;
        lowside_duty_ratio[phaseW] = 1.0-duty;
        highside_duty_ratio[phaseU] = 0.0;
        lowside_duty_ratio[phaseU] = 1.0;
    }else if(!c.hu & c.hv & !c.hw){
        // 0 1 0
        // UH UL VH VL WH WL
        // 0 0 0 1 1 0
        // WH -> VL
        off_phase = phaseU;
        i_fb = phase_current_result[phaseW];
        Current_control_update(&CCtmp, i_ref, i_fb, wr);
        duty = CCtmp.V_ref/BAT_VOLTAGE; // or battery_level_result;
        highside_duty_ratio[phaseW] = duty;
        lowside_duty_ratio[phaseW] = 1.0-duty;
        highside_duty_ratio[phaseV] = 0.0;
        lowside_duty_ratio[phaseV] = 1.0;
    }else if(!c.hu & c.hv & c.hw){
        // 0 1 1
        // UH UL VH VL WH WL
        // 1 0 0 1 0 0
        // UH -> VL
        off_phase = phaseW;
        i_fb = phase_current_result[phaseU];
        Current_control_update(&CCtmp, i_ref, i_fb, wr);
        duty = CCtmp.V_ref/BAT_VOLTAGE; // or battery_level_result;
        highside_duty_ratio[phaseU] = duty;
        lowside_duty_ratio[phaseU] = 1.0-duty;
        highside_duty_ratio[phaseV] = 0.0;
        lowside_duty_ratio[phaseV] = 1.0;
    }else if(!c.hu & !c.hv & c.hw){
        // 0 0 1
        // UH UL VH VL WH WL
        // 1 0 0 0 0 1
        // UH -> WL
        off_phase = phaseV;
        i_fb = phase_current_result[phaseU];
        Current_control_update(&CCtmp, i_ref, i_fb, wr);
        duty = CCtmp.V_ref/BAT_VOLTAGE; // or battery_level_result;
        highside_duty_ratio[phaseU] = duty;
        lowside_duty_ratio[phaseU] = 1.0-duty;
        highside_duty_ratio[phaseW] = 0.0;
        lowside_duty_ratio[phaseW] = 1.0;
    }

    // floating
    highside_duty_ratio[off_phase] = 0.0;
    lowside_duty_ratio[off_phase] = 0.0;

    epwm1_set_duty(highside_duty_ratio[phaseU], lowside_duty_ratio[phaseU]);
    epwm2_set_duty(highside_duty_ratio[phaseV], lowside_duty_ratio[phaseV]);
    epwm3_set_duty(highside_duty_ratio[phaseW], lowside_duty_ratio[phaseW]);
}*/

void control_state_update(enum ADC_RESULT_TYPE type, float32 adc_result_voltage){
    //float32 current = (adc_result_voltage - CURRENT_SENSOR_VOLTAGE_OFFSET_FOR_ZERO ) * ADC_Voltage2Current; //[Ampere]


    float32 current = adc_result_voltage/100.0; // TODO FOR THE TEST. REMOVE THIS LINE TO OPERATE MOTOR

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
        //FOR TEST
        throttle_result = 0.015/4; //TODO FOR THE TEST. REMOVE THIS LINE TO OPERATE THE MOTOR
        //throttle_result = adc_result_voltage*ADC_Voltage2Current*CURRENT_LIMIT_SCALE;
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

        //control_sinusoidal_BEMF();  // Q CURRENT CONTROL - PMSM
        //control_trapezoidal_BEMF(); // 3 CURRENT CONTROLLER 3 PHASE DC
        //control_trapezoidal_BEMF0(); // 1 CURRENT CCONTROLLER 3 PHASE DC
        //test_just_run();//
        //test_3phase_pole_voltage(0.25); // test Pole Voltage with duty
        throttle_result = 3; //[A]
        control_sinusoidal_BEMF();
        adc_result_flag = 0x00; //CLEAR FLAG for next sampling
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
    Current_control_init(&CCuvw[phaseU]);
    Current_control_init(&CCuvw[phaseV]);
    Current_control_init(&CCuvw[phaseW]);
    Current_control_init(&CCtmp);

    CCtmp.Kp = Kp_BLDC;
    CCtmp.Ki = Ki_BLDC;
    CCtmp.Ka = Ka_BLDC;
    CCtmp.alpha = 1; // PI CONTROLLER
    CCtmp.V_sat = BAT_VOLTAGE;


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
