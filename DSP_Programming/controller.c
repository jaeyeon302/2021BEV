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

float32 offset_voltage[4] = {0, 0, 0, 0};
float32 phase_current_result[3]={0, 0, 0}; //[Ampere]
float32 throttle_result = 0; //[Ampere]
float32 battery_level_result = 0; // 0 - 48[V]

Current_Controller CCd,CCq,CCtest;

float32 testduty = 0.1;

/**********************************/
// Functions for DQ PI control with SVPWM
void Init_CC(Current_Controller* cc, float32 v_sat ){
    cc->I_ref = 0.0;
    cc->V_sat = v_sat;
    cc->V_anti = 0.0;
    cc->V_int = 0.0;
    cc->V_ref = 0.0;
    cc->V_fb = 0.0;
    cc->alpha = 1;
    cc->kp = Kp_DEFAULT;
    cc->ki = Ki_DEFAULT;
    cc->ka = Ka_DEFAULT;
    cc->kff = 0.0;
}
void abc2dq(float32 a, float32 b, float32 c, float32 angle_rad,float32* d, float32* q){
    *d=(2/3)*(cos(angle_rad)*a + cos(angle_rad-2*PI/3)*b + cos(angle_rad+2*PI/3)*c);
    *q=(2/3)*(-sin(angle_rad)*a - sin(angle_rad-2*PI/3)*b - sin(angle_rad+2*PI/3)*c);
}
void dq2abc(float32 d, float32 q, float32 angle_rad, float32* a, float32* b ,float32* c){
    *a = cos(angle_rad)*d - sin(angle_rad)*q;
    *b = cos(angle_rad - 2*PI/3)*d - sin(angle_rad - 2*PI/3)*q;
    *c = cos(angle_rad + 2*PI/3)*d - sin(angle_rad + 2*PI/3)*q;
}
float32 SVPWM_offset_voltage(float32 phase_a_v,float32 phase_b_v, float32 phase_c_v){
    float32 vmax = 0;
    float32 vmin = 0;
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
    return -(vmax+vmin)/2;
}


void control_DQ(float32* phase_current,
                Current_Controller* ccId, Current_Controller* ccIq,
                float32 angle, float32 angular_speed,
                float32 max_v_pole,
                float32* duty_out){
    // angle: rad
    // angle_speed : rad/sec
    float32 Ia_fb = phase_current[phaseU];
    float32 Ib_fb = phase_current[phaseV];
    float32 Ic_fb = phase_current[phaseW];
    float32 Id_fb, Iq_fb;
    float32 I_err = 0.0;

    Id_fb = 0.0;
    Iq_fb = 0.0;
    float32 va,vb,vc, vsn, vdc, voffset_pole;

    abc2dq(Ia_fb, Ib_fb, Ic_fb, angle, &Id_fb, &Iq_fb);

    // control D
    I_err = ccId->I_ref - Id_fb;
    ccId->V_anti = ccId->V_ref - ccId->V_sat;
    ccId->V_int += ccId->ki*(I_err - ccId->ka*ccId->V_anti);
    ccId->V_fb = ccId->V_int + ccId->kp*I_err;

    // control Q
    I_err = ccIq->I_ref - Iq_fb;
    ccIq->V_anti = ccIq->V_ref - ccIq->V_sat;
    ccIq->V_int += ccIq->ki*(I_err - ccIq->ka*ccIq->V_anti);
    ccIq->V_fb = ccIq->V_int + ccIq->kp*I_err;

    ccId->V_ref = ccId->V_fb + (-Ls_DEFAULT*angular_speed*Iq_fb); // add Vd feeadforward
    ccIq->V_ref = ccIq->V_fb + (angular_speed*Ls_DEFAULT*Id_fb + angular_speed*flux_DEFAULT); // Vq feedforward



    dq2abc(ccId->V_ref, ccIq->V_ref, angle, &va, &vb, &vc);
    vsn = SVPWM_offset_voltage(va,vb,vc);

    vdc = max_v_pole;
    voffset_pole = vdc/2;

    vsn += voffset_pole;

    va += vsn;
    vb += vsn;
    vc += vsn;

    va = (va > vdc) ? vdc : ((va < 0)? 0: va);
    vb = (vb > vdc) ? vdc : ((vb < 0)? 0: vb);
    vc = (vc > vdc) ? vdc : ((vc < 0)? 0: vc);

    duty_out[phaseU] = va/vdc;
    duty_out[phaseV] = vb/vdc;
    duty_out[phaseW] = vc/vdc;

}
void control_sinusoidal_BEMF(){
    float32 duty[3]={0,0,0};
    float32 angle = hall_sensor_get_E_angle_rad();
    float32 angular_speed = hall_sensor_get_E_angular_speed();   //0; // TODO update needed

    // calculate Iabc->DQ->PI->DQ->Vabc
    control_DQ(phase_current_result,
               &CCd, &CCq, angle, angular_speed,
               BAT_DEFAULT_VOLTAGE,
               duty);


    epwm1_set_duty(duty[phaseU], 1-duty[phaseU]);
    epwm2_set_duty(duty[phaseV], 1-duty[phaseV]);
    epwm3_set_duty(duty[phaseW], 1-duty[phaseW]);
}


/*************TEST CODE******************/
float32 test_angle = 0;
void test_angle_update(float32 unit_angle){
    test_angle += unit_angle;
    if(test_angle >= 2*PI){
        test_angle = 0;
    }
}
void test_I_update(float32 current){
    CCtest.I_ref = current;
}
void test_poll_voltage(float32 duty){
    epwm1_set_duty(duty, 1-duty);
    epwm2_set_duty(duty, 1-duty);
    epwm3_set_duty(duty, 1-duty);
}

void test_V_DQ(float32 vdr, float32 angle_rad){
    float32 va, vb, vc, vsn;
    float32 duty_out[3];

    dq2abc(vdr,0,angle_rad, &va, &vb, &vc);
    vsn = SVPWM_offset_voltage(va,vb,vc);
    va += (vsn+24);
    vb += (vsn+24);
    vc += (vsn+24);

    va = (va > vdc) ? vdc : ((va < 0)? 0: va);
    vb = (vb > vdc) ? vdc : ((vb < 0)? 0: vb);
    vc = (vc > vdc) ? vdc : ((vc < 0)? 0: vc);

    duty_out[phaseU] = va/vdc;
    duty_out[phaseV] = vb/vdc;
    duty_out[phaseW] = vc/vdc;

    epwm1_set_duty(duty_out[phaseU], 1-duty_out[phaseU]);
    epwm2_set_duty(duty_out[phaseV], 1-duty_out[phaseV]);
    epwm3_set_duty(duty_out[phaseW], 1-duty_out[phaseW]);
}


void test_Id_DQ(float32 vdc,  float32 Id_fb, float32 angle_rad, Current_Controller* ccId){
    float32 I_err = 0.0;
    float32 va,vb,vc,vsn;
    float32 duty_out[3];

    I_err = ccId->I_ref - I_fb;
    ccId->V_int += (ccId->ki*I_err)/10000.0; // 10kHz time duration = 1/0000 secs

    vdr = cc->V_fb;

    dq2abc(vdr,0,angle_rad, &va, &vb, &vc);
    vsn = SVPWM_offset_voltage(va,vb,vc);
    va += (vsn+24);
    vb += (vsn+24);
    vc += (vsn+24);

    va = (va > vdc) ? vdc : ((va < 0)? 0: va);
    vb = (vb > vdc) ? vdc : ((vb < 0)? 0: vb);
    vc = (vc > vdc) ? vdc : ((vc < 0)? 0: vc);

    duty_out[phaseU] = va/vdc;
    duty_out[phaseV] = vb/vdc;
    duty_out[phaseW] = vc/vdc;

    epwm1_set_duty(duty_out[phaseU], 1-duty_out[phaseU]);
    epwm2_set_duty(duty_out[phaseV], 1-duty_out[phaseV]);
    epwm3_set_duty(duty_out[phaseW], 1-duty_out[phaseW]);
}

void test_control_1phase(float32 vdc, float32 I_fb, Current_Controller* cc){
    float32 I_err = 0.0;
    float32 va;
    float32 duty = 0.0;

    I_err = cc->I_ref - I_fb;
    // anti windup�� Vref�� 48V�� �������� ������ ����Ǵ� ���̴�.
    //cc->V_anti = cc->V_ref - cc->V_sat;
    //cc->V_int += (cc->ki*(I_err - cc->ka*cc->V_anti))/10000.0;
    cc->V_int += (cc->ki*I_err)/10000.0; // 10kHz time duration = 1/0000 secs
    cc->V_fb = cc->V_int + cc->kp*I_err;

    va = cc->V_fb;
    va = (va > vdc) ? vdc : ((va < 0)? 0: va);
    duty = va/vdc;

    epwm1_set_duty(duty, 1-duty);
    //epwm2_set_duty(duty, 1-duty);
    //epwm3_set_duty(duty, 1-duty);
}

/******************************************/

int offset_voltage_update(enum ADC_RESULT_TYPE type, float32 adc_result_voltage){
    static Uint32 update_count_u = 0;
    static Uint32 update_count_v = 0;
    static Uint32 update_count_w = 0;
    static Uint32 update_count_throttle = 0;
    if(update_count_u < MAX_OFFSET_SAMPLE_COUNT ||
       update_count_v < MAX_OFFSET_SAMPLE_COUNT ||
       update_count_w < MAX_OFFSET_SAMPLE_COUNT ||
       update_count_throttle < MAX_OFFSET_SAMPLE_COUNT){
        switch(type){
        case ADCcurrentPhaseU:
            offset_voltage[ADCcurrentPhaseU] += adc_result_voltage;
            update_count_u++;
            break;
        case ADCcurrentPhaseV:
            offset_voltage[ADCcurrentPhaseV] += adc_result_voltage;
            update_count_v++;
            break;
        case ADCcurrentPhaseW:
            offset_voltage[ADCcurrentPhaseW] += adc_result_voltage;
            update_count_w++;
            break;
        case ADCthrottle:
            offset_voltage[ADCthrottle] += adc_result_voltage;
            update_count_throttle++;
            break;
        default:
            break;
        }
        return 1;
    }
    if(update_count_u == MAX_OFFSET_SAMPLE_COUNT){
        offset_voltage[ADCcurrentPhaseU] = offset_voltage[ADCcurrentPhaseU]/((float32)MAX_OFFSET_SAMPLE_COUNT);
        update_count_u++;
    }
    if(update_count_v == MAX_OFFSET_SAMPLE_COUNT){
        offset_voltage[ADCcurrentPhaseV] = offset_voltage[ADCcurrentPhaseV]/((float32)MAX_OFFSET_SAMPLE_COUNT);
        update_count_v++;
    }
    if(update_count_w == MAX_OFFSET_SAMPLE_COUNT){
        offset_voltage[ADCcurrentPhaseW] = offset_voltage[ADCcurrentPhaseW]/((float32)MAX_OFFSET_SAMPLE_COUNT);
        update_count_w++;
    }
    if(update_count_throttle == MAX_OFFSET_SAMPLE_COUNT){
        offset_voltage[ADCthrottle] = offset_voltage[ADCthrottle]/((float32)MAX_OFFSET_SAMPLE_COUNT);
        update_count_throttle++;
    }

    return 0;
}


void control_state_update(enum ADC_RESULT_TYPE type, float32 adc_result_voltage){
    float32 current = 0.0;
    int on_calculating_offset = offset_voltage_update(type, adc_result_voltage);
    if(on_calculating_offset) return;

    switch(type){
    case ADCcurrentPhaseU:
        adc_result_flag |= FLAG_ADC_CURRENT_PHASE_U_SAMPLED;
        current = (adc_result_voltage - offset_voltage[type])*ADC_Voltage2Current;
        phase_current_result[type] = current;
        break;
    case ADCcurrentPhaseV:
        adc_result_flag |= FLAG_ADC_CURRENT_PHASE_V_SAMPLED;
        current = (adc_result_voltage - offset_voltage[type])*ADC_Voltage2Current;
        phase_current_result[type] = current;
        break;
    case ADCcurrentPhaseW:
        adc_result_flag |= FLAG_ADC_CURRENT_PHASE_W_SAMPLED;
        current = (adc_result_voltage - offset_voltage[type])*ADC_Voltage2Current;
        phase_current_result[type] = current;
        break;
    case ADCthrottle:
        adc_result_flag |= FLAG_ADC_THROTTLE_SAMPLED;
        throttle_result = (adc_result_voltage - offset_voltage[type])*ADC_Voltage2Current*CURRENT_LIMIT_SCALE;
        //FOR TEST
        break;
    case ADCbatteryLevel:
        adc_result_flag |= FLAG_ADC_BATTERY_SAMPLED;
        battery_level_result = adc_result_voltage*BAT_VOLTAGE_SCALER;
        break;
    default:
        break;
    }

    if(adc_result_flag == ADC_ALL_SAMPLED ){
        hall_sensor_update();
        controlCycleCount++;
        /* ���� �ڵ�� ���⼭ ȣ��Ǿ�� �Ѵ� */

        //throttle_result = 3; //[A]
        //control_sinusoidal_BEMF();
        //test_run_DQ();
        //test_control_1phase(48, phase_current_result[phaseU], &CCtest);
        test_poll_voltage(testduty);
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

    // initialize ������ �߿��ϴ�
    // ADC, ECAP �� ePWM���� ���� init �Ǿ���Ѵ�
    // ���� �׷��� ������ ADC, ECAP�� initialize �Ǵ� ����
    // start_controller�� ���� ePWM�� ���۵ǰ� ePWM cycle (2~3��)�� �׳� ������������
    // ePWM -> ADC ȣ�� -> �����Լ� ȣ�� �����̱� ������, ADC�� ePWM���� ���� ������ �Ϸ��ؾ��Ѵ�.
    Init_CC(&CCd, 40); // Vd_sat 40
    Init_CC(&CCq, 40); // Vq_sat 40
    Init_CC(&CCtest, 48);

    Init_3current_ADC( &control_state_update );
    Init_misc_ADC();
    Init_3phase_ePWM();
    Init_hall_sensor();
}

void Start_controller(){
    Start_3phase_ePWM();
    Start_3current_ADC();
    Start_hall_sensor();
}
