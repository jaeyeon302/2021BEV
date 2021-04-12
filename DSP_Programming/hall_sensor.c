/*
 * hall_sensor2.c
 *
 *  Created on: 2021. 3. 12.
 *      Author: ok6530
 */

#include <hall_sensor.h>

#define POLE_PAIR 23
Hall_state prev_state;
Hall_state hall_state;
Angle_observer angle_observer;

float32 time1=0;
Uint32 t_counter = 0;
float32 update_period = 1/10e3;

Hall_state get_hall_state(){
    return hall_state;
}

void Init_hall_sensor(){

    EALLOW;
    // H_U
    GpioCtrlRegs.GPBGMUX2.bit.GPIO60 = 0; // use GPIO60 as gpio
    GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO60 = 0; // GPIO FOR INPUT
    GpioCtrlRegs.GPBQSEL2.bit.GPIO60 = 0x0; // Input Qualification type - Sync
    GpioCtrlRegs.GPBPUD.bit.GPIO60 = 0; // PULL UP
    // H_V
    GpioCtrlRegs.GPAGMUX2.bit.GPIO22 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO22 = 0;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO22 = 0x0;
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;
    // H_W
    GpioCtrlRegs.GPDGMUX1.bit.GPIO105 = 0;
    GpioCtrlRegs.GPDMUX1.bit.GPIO105 = 0;
    GpioCtrlRegs.GPDDIR.bit.GPIO105 = 0;
    GpioCtrlRegs.GPDQSEL1.bit.GPIO105 = 0X0;
    GpioCtrlRegs.GPDPUD.bit.GPIO105 = 0;
}
void Start_hall_sensor(){
    hall_state = hall_sensor_update(); // Get initial state
    prev_state = hall_state;

    //start_ecap

    EINT;
}

Angle_observer get_angle_observer(){
    return angle_observer;
}

void Init_angle_observer(){
    angle_observer.Kp = ANGLE_OBSERVER_KP;
    angle_observer.Ki = ANGLE_OBSERVER_KI;
    angle_observer.Wr = 0.0;
    angle_observer.angle = 0.0;
}

float32 angle_observer_update(float32 hall_sensor_angle){
    float32 angle_err = hall_sensor_angle - angle_observer.angle;
    angle_observer.Wr+= angle_observer.Ki*angle_err*update_period;
    angle_observer.angle += (angle_observer.Wr + angle_observer.Kp*angle_err)*update_period;

    return angle_observer.angle;
}

float32 calibrate_angle_offset(float32 angle){
    return angle - hall_state.angle_E_offset_rad;
}


Hall_state hall_sensor_update(){ // Hwigon Kim
    Hall_state state;


    state.hu = GpioDataRegs.GPBDAT.bit.GPIO60;
    state.hv = GpioDataRegs.GPADAT.bit.GPIO22;
    state.hw = GpioDataRegs.GPDDAT.bit.GPIO105;
    state.angle_E_offset_rad = PI/3;
    hall_state.bounded = false;

    if(state.hu & !state.hv & state.hw){
        // 1 0 1
        // 5*pi/3 - 6*pi/3
        hall_state.angle_E_rad = 5*Deg_60_IN_RAD;
    }else if(state.hu & !state.hv & !state.hw){
        // 1 0 0
        // 0 - pi/3
        hall_state.bounded = true;
        hall_state.angle_E_rad = 0;
    }else if(state.hu & state.hv & !state.hw){
        // 1 1 0
        // pi/3 - 2*pi/3
        hall_state.angle_E_rad = Deg_60_IN_RAD;
    }else if(!state.hu & state.hv & !state.hw){
        // 0 1 0
        // 2*pi/3 - 3*pi/3
        hall_state.angle_E_rad = 2*Deg_60_IN_RAD;
    }else if(!state.hu & state.hv & state.hw){
        // 0 1 1
        // 3*pi/3 - 4*pi/3
        hall_state.angle_E_rad = 3*Deg_60_IN_RAD;
    }else if(!state.hu & !state.hv & state.hw){
        // 0 0 1
        // 4*pi/3 - 5*pi/3
        hall_state.angle_E_rad = 4*Deg_60_IN_RAD;
    }else{
        hall_state.angle_E_rad = state.angle_E_offset_rad;
    }



    if(state.hu != hall_state.hu ||
       state.hv != hall_state.hv ||
       state.hw != hall_state.hw){
       state.Wr = PI/3.0/(t_counter*update_period);
       hall_state.Wr = state.Wr;
       t_counter = 0;
       hall_state.hu = state.hu;
       hall_state.hv = state.hv;
       hall_state.hw = state.hw;
    }else if(t_counter >= 1.0/update_period){
        hall_state.Wr = 0;
        state.Wr = 0;
        t_counter = 0;
    }else{
        t_counter++;
    }

    return state;
}

void hall_sensor_set_angle_offset_rad(float64 offset_rad){
    hall_state.angle_E_offset_rad = offset_rad;
}

float64 hall_sensor_get_E_angle_rad(){
    float64 time2 = ECap1Regs.TSCTR;
    byte hu = hall_state.hu;
    byte hv = hall_state.hv;
    byte hw = hall_state.hw;

    float64 angle;
    angle = hall_state.angle_E_rad + (t_counter*update_period)*hall_state.Wr;
    if (angle > hall_state.angle_E_rad+PI/3.0){
        angle = hall_state.angle_E_rad+PI/3.0;
    }
    if (angle < hall_state.angle_E_rad-PI/3.0){
        angle = hall_state.angle_E_rad-PI/3.0;
    }

    return (angle);
}

float64 hall_sensor_get_M_angle_rad(){
    float64 electrical_angle = hall_sensor_get_E_angle_rad();
    float64 mechanical_angle = (electrical_angle+(TWOPI*hall_state.rotation))/POLE_PAIR;
    if(mechanical_angle > TWOPI){
        // angle = angle - 2PI*R
        // R = number of forward rotation
        mechanical_angle -= ( (int64)(mechanical_angle/TWOPI))*(TWOPI);
    }else if(mechanical_angle < 0){
        // angle = angle + 2PI*(R+1)
        // R = number of backward rotation
        mechanical_angle += ( (int64)((-mechanical_angle)/TWOPI) + 1)*(TWOPI);
    }
    return mechanical_angle;
}
float64 hall_sensor_get_E_angular_speed(){
    return hall_state.Wr;
}
float64 hall_sensor_get_M_angular_speed(){
    return hall_state.Wr/POLE_PAIR;
}
