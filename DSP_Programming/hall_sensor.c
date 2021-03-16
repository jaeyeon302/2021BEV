/*
 * hall_sensor2.c
 *
 *  Created on: 2021. 3. 12.
 *      Author: ok6530
 */

#include <hall_sensor.h>
Hall_state hall_state;
float32 POLE_PAIR_ = 1;
Uint32 SAMPLE_FREQ = 1;

void Init_hall_sensor(Uint16 pole_pair){
    POLE_PAIR_ = pole_pair;

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
    hall_state = hall_sensor_update();
    //hall_state.hu = GpioDataRegs.GPBDAT.bit.GPIO60;
    //hall_state.hv = GpioDataRegs.GPADAT.bit.GPIO22;
    //hall_state.hw = GpioDataRegs.GPDDAT.bit.GPIO105;
}
double mech_angle = 0.0;
Hall_state hall_sensor_update(){
    Hall_state state;

    state.hu = GpioDataRegs.GPBDAT.bit.GPIO60;
    state.hv = GpioDataRegs.GPADAT.bit.GPIO22;
    state.hw = GpioDataRegs.GPDDAT.bit.GPIO105;

    if(state.hu & !state.hv & state.hw){
        // 1 0 1
        // 5*pi/3 - 6*pi/3
        state.angle_E_rad = 5*Deg_60_IN_RAD;
        if(!hall_state.hu & !hall_state.hv & hall_state.hw){
            // 0 0 1
            // backward
            hall_state.angle_E_rad -= Deg_60_IN_RAD;
        }else if(hall_state.hu & !hall_state.hv & !hall_state.hw){
            // 1 0 0
            // forward
            hall_state.angle_E_rad += Deg_60_IN_RAD;
        }
    }else if(state.hu & !state.hv & !state.hw){
        // 1 0 0
        // 0 - pi/3
        state.angle_E_rad = 0;
        if(hall_state.hu & !hall_state.hv & hall_state.hw){
            // 1 0 1
            // backward
            hall_state.angle_E_rad -= Deg_60_IN_RAD;
        }else if(hall_state.hu & hall_state.hv & !hall_state.hw){
            // 1 1 0
            // forward
            hall_state.angle_E_rad += Deg_60_IN_RAD;
        }
    }else if(state.hu & state.hv & !state.hw){
        // 1 1 0
        // pi/3 - 2*pi/3
        state.angle_E_rad = Deg_60_IN_RAD;
        if(hall_state.hu & !hall_state.hv & !hall_state.hw){
            // 1 0 0
            // backward
            hall_state.angle_E_rad -= Deg_60_IN_RAD;
        }else if(!hall_state.hu & hall_state.hv & !hall_state.hw){
            // 0 1 0
            // forward
            hall_state.angle_E_rad += Deg_60_IN_RAD;
        }
    }else if(!state.hu & state.hv & !state.hw){
        // 0 1 0
        // 2*pi/3 - 3*pi/3
        state.angle_E_rad = 2*Deg_60_IN_RAD;
        if(hall_state.hu & hall_state.hv & !hall_state.hw){
            // 1 1 0
            // backward
            hall_state.angle_E_rad -= Deg_60_IN_RAD;
        }else if(!hall_state.hu & hall_state.hv & hall_state.hw){
            // 0 1 1
            // forward
            hall_state.angle_E_rad += Deg_60_IN_RAD;
        }
    }else if(!state.hu & state.hv & state.hw){
        // 0 1 1
        // 3*pi/3 - 4*pi/3
        state.angle_E_rad = 3*Deg_60_IN_RAD;
        if(!hall_state.hu & state.hv & !hall_state.hw){
            // 0 1 0
            // backward
            hall_state.angle_E_rad -= Deg_60_IN_RAD;
        }else if(!hall_state.hu & !hall_state.hv & hall_state.hw){
            // 0 0 1
            // forward
            hall_state.angle_E_rad += Deg_60_IN_RAD;
        }
    }else if(!state.hu & !state.hv & state.hw){
        // 0 0 1
        // 4*pi/3 - 5*pi/3
        state.angle_E_rad = 4*Deg_60_IN_RAD;
        if(!hall_state.hu & hall_state.hv & hall_state.hw){
            // 0 1 1
            // backward
            hall_state.angle_E_rad -= Deg_60_IN_RAD;
        }else if(hall_state.hu & !hall_state.hv & hall_state.hw){
            // 1 0 1
            // forward
            hall_state.angle_E_rad += Deg_60_IN_RAD;
        }
    }
    hall_state.hu = state.hu;
    hall_state.hv = state.hv;
    hall_state.hw = state.hw;
    mech_angle = hall_state.angle_E_rad/23;
    return state;
}


