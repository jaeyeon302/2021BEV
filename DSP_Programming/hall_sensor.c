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

void Init_hall_sensor(Uint16 pole_pair, Uint32 sampling_frequency){
    POLE_PAIR_ = pole_pair;
    SAMPLE_FREQ = sampling_frequency;

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
    hall_state.hu = GpioDataRegs.GPBDAT.bit.GPIO60;
    hall_state.hv = GpioDataRegs.GPADAT.bit.GPIO22;
    hall_state.hw = GpioDataRegs.GPDDAT.bit.GPIO105;
}

Hall_state hall_sensor_update(){
    Hall_state state;

    state.hu = GpioDataRegs.GPBDAT.bit.GPIO60;
    state.hv = GpioDataRegs.GPADAT.bit.GPIO22;
    state.hw = GpioDataRegs.GPDDAT.bit.GPIO105;

    hall_state = state;
    return state;
}


