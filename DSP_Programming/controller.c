/*
 * controller.c
 *
 *  Created on: 2021. 3. 5.
 *      Author: Jaeyeon Park
 */

#include "controller.h"
#define NUM_OF_THROTTLE_MOVING_AVG_SAMPLES 10
Uint32 controlCycleCount = 0;

unsigned char adc_result_flag = 0x00;
const unsigned char ADC_ALL_SAMPLED = (FLAG_ADC_CURRENT_PHASE_U_SAMPLED
                                      |FLAG_ADC_CURRENT_PHASE_V_SAMPLED
                                      |FLAG_ADC_CURRENT_PHASE_W_SAMPLED
                                      |FLAG_ADC_THROTTLE_SAMPLED
                                      |FLAG_ADC_BATTERY_SAMPLED);


float32 offset_voltage[4] = {0, 0, 0, 0};
float32 phase_current_result[3]={0, 0, 0}; //[Ampere]
float32 dq_current_result[2] = {0,0};
float32 duty_out[3]={0,0,0};
float32 throttle_result_samples[NUM_OF_THROTTLE_MOVING_AVG_SAMPLES];
float32 throttle_result_temp_average=0;
float32 throttle_result = 0; //[Ampere]
float32 battery_level_result = 0; // 0 - 48[V]

Current_Controller CCd,CCq,CCtest;


float32 testduty0 = 0.5;
float32 testduty1 = 0.5;
float32 testduty2 = 0.5;

float32 testangle = 0;
float32 testAlignId = 2;
float32 freq = 1;
float32 angle_E_rad=0;
float32 angular_E_speed=0;


Uint32 t_count = 0;

/****** controller constants *********/
float32 VDC = 48.0;
float32 CURRENT_FAULT_LIMIT = 40.0;
float32 CURRENT_LIMIT = 35.0;
byte FLAG_RUN = 0;
byte EMERGENCY_FLAG = 0;


float32* get_3phase_currents(){
    return phase_current_result;
}
float32* get_dqr_currents(){
    return dq_current_result;
}

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
    *d=(2.0/3.0)*(cos(angle_rad)*a + cos(angle_rad-2.0*PI/3.0)*b + cos(angle_rad+2.0*PI/3.0)*c);
    *q=(2.0/3.0)*(-sin(angle_rad)*a - sin(angle_rad-2.0*PI/3.0)*b - sin(angle_rad+2.0*PI/3.0)*c);
}
void dq2abc(float32 d, float32 q, float32 angle_rad, float32* a, float32* b ,float32* c){
    *a = cos(angle_rad)*d - sin(angle_rad)*q;
    *b = cos(angle_rad - 2.0*PI/3.0)*d - sin(angle_rad - 2.0*PI/3.0)*q;
    *c = cos(angle_rad + 2.0*PI/3.0)*d - sin(angle_rad + 2.0*PI/3.0)*q;
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

//
//void control_DQ(float32* phase_current,
//                Current_Controller* ccId, Current_Controller* ccIq,
//                float32 angle, float32 angular_speed,
//                float32 max_v_pole,
//                float32* duty_out){
//    // angle: rad
//    // angle_speed : rad/sec
//    float32 Ia_fb = phase_current[phaseU];
//    float32 Ib_fb = phase_current[phaseV];
//    float32 Ic_fb = phase_current[phaseW];
//    float32 Id_fb, Iq_fb;
//    float32 I_err = 0.0;
//
//    Id_fb = 0.0;
//    Iq_fb = 0.0;
//    float32 va,vb,vc, vsn, vdc, voffset_pole;
//
//    abc2dq(Ia_fb, Ib_fb, Ic_fb, angle, &Id_fb, &Iq_fb);
//
//    // control D
//    I_err = ccId->I_ref - Id_fb;
//    ccId->V_anti = ccId->V_ref - ccId->V_sat;
//    ccId->V_int += ccId->ki*(I_err - ccId->ka*ccId->V_anti);
//    ccId->V_fb = ccId->V_int + ccId->kp*I_err;
//
//    // control Q
//    I_err = ccIq->I_ref - Iq_fb;
//    ccIq->V_anti = ccIq->V_ref - ccIq->V_sat;
//    ccIq->V_int += ccIq->ki*(I_err - ccIq->ka*ccIq->V_anti);
//    ccIq->V_fb = ccIq->V_int + ccIq->kp*I_err;
//
//    ccId->V_ref = ccId->V_fb + (-Ls_DEFAULT*angular_speed*Iq_fb); // add Vd feeadforward
//    ccIq->V_ref = ccIq->V_fb + (angular_speed*Ls_DEFAULT*Id_fb + angular_speed*flux_DEFAULT); // Vq feedforward
//
//    dq2abc(ccId->V_ref, ccIq->V_ref, angle, &va, &vb, &vc);
//    vsn = SVPWM_offset_voltage(va,vb,vc);
//
//    vdc = max_v_pole;
//    voffset_pole = vdc/2;
//
//    vsn += voffset_pole;
//
//    va += vsn;
//    vb += vsn;
//    vc += vsn;
//
//    va = (va > vdc) ? vdc : ((va < 0)? 0: va);
//    vb = (vb > vdc) ? vdc : ((vb < 0)? 0: vb);
//    vc = (vc > vdc) ? vdc : ((vc < 0)? 0: vc);
//
//    duty_out[phaseU] = va/vdc;
//    duty_out[phaseV] = vb/vdc;
//    duty_out[phaseW] = vc/vdc;
//
//}
//void control_sinusoidal_BEMF(){
//    float32 duty[3]={0,0,0};
//    float32 angle = hall_sensor_get_E_angle_rad();
//    float32 angular_speed = hall_sensor_get_E_angular_speed();   //0; // TODO update needed
//
//    // calculate Iabc->DQ->PI->DQ->Vabc
//    control_DQ(phase_current_result,
//               &CCd, &CCq, angle, angular_speed,
//               BAT_DEFAULT_VOLTAGE,
//               duty);
//
//
//    epwm1_set_duty(duty[phaseU], 1-duty[phaseU]);
//    epwm2_set_duty(duty[phaseV], 1-duty[phaseV]);
//    epwm3_set_duty(duty[phaseW], 1-duty[phaseW]);
//}
/*************TEST CODE******************/
void test_angle_update(float32 unit_angle){
    testangle += unit_angle;
    if(testangle >= 2*PI){
        testangle = 0;
    }else if(testangle <= -2*PI){
        testangle = 0;
    }
}
void test_Idq_update(float32 Id_ref, float32 Iq_ref){
    CCd.I_ref = Id_ref;
    CCq.I_ref = Iq_ref;
}
void test_I_update(float32 current){CCtest.I_ref = current;}
void test_poll_voltage(float32 duty){
    // 폴 전압이 원하는 듀티로 잘 변경되고 있음을 확인함.
    epwm1_set_duty(testduty0, 1-testduty0);
    epwm2_set_duty(testduty1, 1-testduty1);
    epwm3_set_duty(testduty2, 1-testduty2);
}

//void test_V_DQ(float32 vdr, float32 vqr, float32 angle_rad, float32 vdc){
//    // 3상 DQ 변환이 잘 제어되고 있음을 확인함
//
//    float32 va, vb, vc, vsn;
//
//    dq2abc(vdr,vqr,angle_rad, &va, &vb, &vc);
//    vsn = SVPWM_offset_voltage(va,vb,vc);
//    va += (vsn+24);
//    vb += (vsn+24);
//    vc += (vsn+24);
//
//    va = (va > vdc) ? vdc : ((va < 0)? 0: va);
//    vb = (vb > vdc) ? vdc : ((vb < 0)? 0: vb);
//    vc = (vc > vdc) ? vdc : ((vc < 0)? 0: vc);
//
//    duty_out[phaseU] = va/vdc;
//    duty_out[phaseV] = vb/vdc;
//    duty_out[phaseW] = vc/vdc;
//
//    epwm1_set_duty(duty_out[phaseU], 1-duty_out[phaseU]);
//    epwm2_set_duty(duty_out[phaseV], 1-duty_out[phaseV]);
//    epwm3_set_duty(duty_out[phaseW], 1-duty_out[phaseW]);
//}

void test_vect_I_DQ(float32 vdc, float32 angle_E_rad, float32 angular_E_speed, Current_Controller* ccId, Current_Controller* ccIq){

    float32 Id_err = 0.0;
    float32 Iq_err = 0.0;
    float32 Id_fb = 0.0;
    float32 Iq_fb = 0.0;
    float32 Ia_fb = phase_current_result[phaseU];
    float32 Ib_fb = phase_current_result[phaseV];
    float32 Ic_fb = -(Ia_fb + Ib_fb);//phase_current_result[phaseW];

    float32 va,vb,vc,vsn;
    abc2dq(Ia_fb, Ib_fb, Ic_fb, angle_E_rad, &Id_fb, &Iq_fb);

    dq_current_result[0] = Id_fb;
    dq_current_result[1] = Iq_fb;


    Id_err = ccId->I_ref - Id_fb;
    ccId->V_int += (ccId->ki*Id_err)/10000.0; // 10kHz time duration = 1/0000 secs
    ccId->V_fb = ccId->V_int + ccId->kp*Id_err;

    Iq_err = ccIq->I_ref - Iq_fb;
    ccIq->V_int += (ccIq->ki*Iq_err)/10000.0;
    ccIq->V_fb = ccIq->V_int + ccIq->kp*Iq_err;

    ccId->V_ref = ccId->V_fb; //+ (-Ls_DEFAULT*angular_E_speed*Iq_fb);
    ccIq->V_ref = ccIq->V_fb; //+ (Ls_DEFAULT*angular_E_speed*Id_fb + angular_E_speed*flux_DEFAULT);

    if(angular_E_speed<2000){
        ccId->V_ref += (-Ls_DEFAULT*angular_E_speed*Iq_fb);
        ccIq->V_ref += (Ls_DEFAULT*angular_E_speed*Id_fb + angular_E_speed*flux_DEFAULT);
    }

    dq2abc(ccId->V_ref, ccIq->V_ref, angle_E_rad, &va, &vb, &vc);

    vsn = SVPWM_offset_voltage(va,vb,vc) + vdc/2;
    //vsn = vdc/2.0;
    va += vsn;
    vb += vsn;
    vc += vsn;

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

void test_I_DQ(float32 vdc, float32 angle_rad, Current_Controller* ccId, Current_Controller* ccIq){
    float32 duty_out[3]={0,0,0};
    float32 Id_err = 0.0;
    float32 Iq_err = 0.0;
    float32 Id_fb = 0.0;
    float32 Iq_fb = 0.0;
    float32 Ia_fb = phase_current_result[phaseU];
    float32 Ib_fb = phase_current_result[phaseV];
    float32 Ic_fb = phase_current_result[phaseW];

    float32 va,vb,vc,vsn;

    abc2dq(Ia_fb, Ib_fb, Ic_fb, angle_rad, &Id_fb, &Iq_fb);

    Id_err = ccId->I_ref - Id_fb;
    ccId->V_int += (ccId->ki*Id_err)/10000.0; // 10kHz time duration = 1/0000 secs
    ccId->V_fb = ccId->V_int + ccId->kp*Id_err;

    Iq_err = ccIq->I_ref - Iq_fb;
    ccIq->V_int += (ccIq->ki*Iq_err)/10000.0;
    ccIq->V_fb = ccIq->V_int + ccIq->kp*Iq_err;

    // for the motor
    // feedforward term is needed
    /*
     *  ccId->V_ref = ccId->V_fb + (-Ls_DEFAULT*angular_speed*Iq_fb); // add Vd feeadforward
     *  ccIq->V_ref = ccIq->V_fb + (angular_speed*Ls_DEFAULT*Id_fb + angular_speed*flux_DEFAULT); // Vq feedforward
     *
     */

    dq2abc(ccId->V_fb, ccIq->V_fb, angle_rad, &va, &vb, &vc);

    vsn = SVPWM_offset_voltage(va,vb,vc) + vdc/2;
    va += vsn;
    vb += vsn;
    vc += vsn;

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

//void is_aligned(float32 vdc, Current_Controller* ccId, Current_Controller* ccIq){
//    float32 aligned_time = 5; //seconds
//    float32 aligned_counter_max = aligned_time*CONTROL_FREQ;
//    static float32 aligned_counter = 0;
//
//    if(aligned_counter < aligned_counter_max){
//        ccId->I_ref = testAlignId;
//        ccIq->I_ref = 0.0;
//        test_I_DQ(vdc, 0, ccId, ccIq);
//        aligned_counter++;
//        return false;
//    }else if(aligned_counter == aligned_counter_max){
//        float32 angle_E_rad_offset = hall_sensor_get_E_angle_rad();
//        hall_sensor_set_angle_offset_rad(angle_E_rad_offset);
//        return false;
//    }else{
//        return true;
//    }
//}

/*
void test_control_1phase(float32 vdc, Current_Controller* cc){
    // 이 함수는 테스트를 모두 통과한 함수입니다.
    // 단상 전류 제어가 1A에서 오프셋 차이 없이 잘 되었음을 확인함.

    float32 I_fb =  phase_current_result[phaseU];
    float32 I_err = 0.0;
    float32 va;
    float32 duty = 0.0;

    I_err = cc->I_ref - I_fb;
    // anti windup은 Vref가 48V를 돌파했을 때에만 적용되는 것이다.
    //cc->V_anti = cc->V_ref - cc->V_sat;
    //cc->V_int += (cc->ki*(I_err - cc->ka*cc->V_anti))/10000.0;
    cc->V_int += (cc->ki*I_err)/10000.0; // 10kHz time duration = 1/0000 secs
    cc->V_fb = cc->V_int + cc->kp*I_err;

    va = cc->V_fb;
    va = (va > vdc) ? vdc : ((va < 0)? 0: va);
    duty = va/vdc;

    epwm1_set_duty(duty, 1-duty); // U상
    //epwm2_set_duty(duty, 1-duty); // V상
    //epwm3_set_duty(duty, 1-duty); // W상
}*/

/******************************************/

int offset_voltage_update(enum ADC_RESULT_TYPE type, float32 adc_result_voltage){
    static Uint32 update_count_u = 0;
    static Uint32 update_count_v = 0;
    static Uint32 update_count_w = 0;
    static Uint32 update_count_throttle = 0;

    if(update_count_u < MAX_OFFSET_SAMPLE_COUNT && type == ADCcurrentPhaseU){
        offset_voltage[ADCcurrentPhaseU] += adc_result_voltage;
        update_count_u++;
        return 1;
    }else if(update_count_u == MAX_OFFSET_SAMPLE_COUNT){
        offset_voltage[ADCcurrentPhaseU] = offset_voltage[ADCcurrentPhaseU]/((float32)MAX_OFFSET_SAMPLE_COUNT);
        update_count_u++;
    }

    if(update_count_v < MAX_OFFSET_SAMPLE_COUNT && type == ADCcurrentPhaseV){
        offset_voltage[ADCcurrentPhaseV] += adc_result_voltage;
        update_count_v++;
        return 1;
    }else if(update_count_v == MAX_OFFSET_SAMPLE_COUNT){
        offset_voltage[ADCcurrentPhaseV] = offset_voltage[ADCcurrentPhaseV]/((float32)MAX_OFFSET_SAMPLE_COUNT);
        update_count_v++;
    }

    if(update_count_w < MAX_OFFSET_SAMPLE_COUNT && type == ADCcurrentPhaseW){
        offset_voltage[ADCcurrentPhaseW] += adc_result_voltage;
        update_count_w++;
        return 1;
    }else if(update_count_w == MAX_OFFSET_SAMPLE_COUNT){
        offset_voltage[ADCcurrentPhaseW] = offset_voltage[ADCcurrentPhaseW]/((float32)MAX_OFFSET_SAMPLE_COUNT);
        update_count_w++;
    }

    if(update_count_throttle < MAX_OFFSET_SAMPLE_COUNT && type == ADCthrottle){
        offset_voltage[ADCthrottle] += adc_result_voltage;
        update_count_throttle++;
    }else if(update_count_throttle == MAX_OFFSET_SAMPLE_COUNT){
        offset_voltage[ADCthrottle] = offset_voltage[ADCthrottle]/((float32)MAX_OFFSET_SAMPLE_COUNT);
        update_count_throttle++;
    }

    return 0;
}


void control_state_update(enum ADC_RESULT_TYPE type, float32 adc_result_voltage){
    float32 current = 0.0;
    static int throttle_idx = 0;
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
        /* 제어 코드는 여기서 호출되어야 한다 */
        //test_angle_update(2*PI*CONTROL_PRD*freq);
        //test_I_DQ(20, testangle, &CCd, &CCq);

        //test_vect_I_DQ(float32 vdc, float32 angle_E_rad, float32 angular_E_speed, Current_Controller* ccId, Current_Controller* ccIq){
        //angle_E_rad = hall_sensor_get_E_angle_rad();

        angle_E_rad = hall_sensor_get_E_angle_rad();//get_hall_state().angle_E_rad - get_hall_state().angle_E_offset_rad;

        float32 observer_angle = angle_observer_update(angle_E_rad);
        float32 observer_speed = get_angle_observer().Wr;
        observer_angle = calibrate_angle_offset(observer_angle);

        FLAG_RUN = GpioDataRegs.GPCDAT.bit.GPIO66;

        if( (offset_voltage[phaseU] < 1.2)
            || (offset_voltage[phaseV] < 1.2)
             ){
            /*|| (offset_voltage[phaseW] < 1.2)*/
            EMERGENCY_FLAG = 1;
        }

        if( (phase_current_result[phaseU] > CURRENT_FAULT_LIMIT)
            || (phase_current_result[phaseV] > CURRENT_FAULT_LIMIT)
            ){
            /*|| (phase_current_result[phaseW] > CURRENT_FAULT_LIMIT)*/
            EMERGENCY_FLAG = 1;
        }

        if( (phase_current_result[phaseU] < -CURRENT_FAULT_LIMIT)
            || (phase_current_result[phaseV] < -CURRENT_FAULT_LIMIT)
           ){
            /* || (phase_current_result[phaseW] < -CURRENT_FAULT_LIMIT) */
            EMERGENCY_FLAG = 1;
        }

        if(EMERGENCY_FLAG){
            GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;
        }



        if(FLAG_RUN && !EMERGENCY_FLAG){
            float32 throttle_result_avg = 0.0;
            throttle_result_samples[throttle_idx] = throttle_result;
            for(int i = 0; i < NUM_OF_THROTTLE_MOVING_AVG_SAMPLES;i++){
                throttle_result_avg += throttle_result_samples[throttle_idx];
            }
            throttle_result_avg = throttle_result_avg / ((float32)NUM_OF_THROTTLE_MOVING_AVG_SAMPLES);

            throttle_idx += 1;
            throttle_idx = throttle_idx % NUM_OF_THROTTLE_MOVING_AVG_SAMPLES;

            throttle_result_temp_average = throttle_result_avg;
            //angular_E_speed = hall_sensor_get_E_angular_speed();

            EPwm1Regs.AQCTLA.bit.CAU = 2;
            EPwm1Regs.AQCTLA.bit.CAD = 1;
            EPwm1Regs.DBCTL.bit.OUTSWAP = 0;

            EPwm2Regs.AQCTLA.bit.CAU = 2;
            EPwm2Regs.AQCTLA.bit.CAD = 1;
            EPwm2Regs.DBCTL.bit.OUTSWAP = 0;

            EPwm3Regs.AQCTLA.bit.CAU = 2;
            EPwm3Regs.AQCTLA.bit.CAD = 1;
            EPwm3Regs.DBCTL.bit.OUTSWAP = 0;

            if(throttle_result_avg>CURRENT_LIMIT){
                throttle_result_avg = CURRENT_LIMIT;
            }else if(throttle_result_avg < 0.2){
                throttle_result_avg = 0;
            }

            CCq.I_ref = throttle_result_avg;
            test_vect_I_DQ(VDC, observer_angle, observer_speed, &CCd, &CCq);
            //test_control_1phase(CCtest.V_sat, &CCtest);
            //test_poll_voltage(0);
           // test_V_DQ(testvd,testvq, testangle,48);

        }else{


            Init_CC(&CCd, 20); // Vd_sat 40
            Init_CC(&CCq, 20); // Vq_sat 40
            duty_out[0]= 0;
            duty_out[1]= 0;
            duty_out[2]= 0;

            for(int i = 0; i< NUM_OF_THROTTLE_MOVING_AVG_SAMPLES; i++){
                throttle_result_samples[i] = 0.0;
            }

            CCq.I_ref = 0.0;

            EPwm1Regs.AQCTLA.bit.CAU = 2;
            EPwm1Regs.AQCTLA.bit.CAD = 2;
            EPwm1Regs.DBCTL.bit.OUTSWAP = 2;

            EPwm2Regs.AQCTLA.bit.CAU = 2;
            EPwm2Regs.AQCTLA.bit.CAD = 2;
            EPwm2Regs.DBCTL.bit.OUTSWAP = 2;

            EPwm3Regs.AQCTLA.bit.CAU = 2;
            EPwm3Regs.AQCTLA.bit.CAD = 2;
            EPwm3Regs.DBCTL.bit.OUTSWAP = 2;
        }

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
    Init_CC(&CCd, 20); // Vd_sat 40
    Init_CC(&CCq, 20); // Vq_sat 40
//    Init_CC(&CCtest, 20);

    Init_3current_ADC( &control_state_update );
    Init_misc_ADC();
    Init_3phase_ePWM();
    Init_hall_sensor();
    Init_angle_observer();


    // ready FLAG_RUN switch
    GpioCtrlRegs.GPCGMUX1.bit.GPIO66 = 0; // use GPIO60 as gpio
    GpioCtrlRegs.GPCMUX1.bit.GPIO66 = 0;
    GpioCtrlRegs.GPCDIR.bit.GPIO66 = 0; // GPIO FOR INPUT
    GpioCtrlRegs.GPCQSEL1.bit.GPIO66 = 0x0; // Input Qualification type - Sync


}

void Start_controller(){
//    Start_3phase_ePWM();
    Start_3current_ADC();
    Start_hall_sensor();
}
