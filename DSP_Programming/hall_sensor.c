/*
 * hall_sensor2.c
 *
 *  Created on: 2021. 3. 12.
 *      Author: ok6530
 */

#include <hall_sensor.h>

#define POLE_PAIR 23

Hall_state hall_state;
float64 time1 = 0;
interrupt void ecap1_isr_Wr_update(void){
    ECap1Regs.ECCLR.bit.INT = 1; //global interrupt status clear. clears INT flag and enables interrupt by CEVTx flag
    ECap1Regs.ECCLR.bit.CEVT1 = 1; //CEVT1 flag clear
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP4;
    Uint32 T = ECap1Regs.CAP1;
    hall_state.Wr=2*PI*1000000000/5/T; //2PI/(5ns*#ofTSCount), tsclk=sysclk=200MHz !!!!!!!!!!!!!!!!!!!

    time1 = ECap1Regs.TSCTR;
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


    // configure eCAP
    // 1. input X-bar
    EALLOW;
    InputXbarRegs.INPUT7SELECT = 60; // connect GPIO60(hall_sensor_u) input to INPUT-X-BAR line no.7 -> eCAP1
    EDIS;

    // 2. Connect Interrupt
    EALLOW;
    PieVectTable.ECAP1_INT = &ecap1_isr_Wr_update;
    EDIS;
    IER |= M_INT4;
    PieCtrlRegs.PIEIER4.bit.INTx1 = 1; // enable ECAP1 PIE Interrupt

    // 2. configure eCAP
    ECap1Regs.ECEINT.all = 0x0000; // Disable All ECAP Interrupt

    ECap1Regs.ECCLR.all = 0xFFFF; // clear All ECAP interrupt flag

    ECap1Regs.ECCTL1.bit.CAPLDEN = 0;       // Start configuration. Disable CAP1x register loading (capture event가 레지스터에 loading이 안 되게 한다)
    ECap1Regs.ECCTL2.bit.TSCTRSTOP = 0;     // Stop time stamp counter

    ECap1Regs.ECCTL1.bit.PRESCALE = 0; // no prescale
    ECap1Regs.ECCTL1.bit.CAP1POL = 1; // capture event1 enable on rising edge

    ECap1Regs.ECCTL1.bit.CTRRST1 = 1; // difference mode operation: reset tscounter when capture event occurs. CAP1 register has counter value till CEVT1

    ECap1Regs.ECCTL1.bit.CAPLDEN = 1; // CAPx register loading enable

    ECap1Regs.ECCTL2.bit.CAP_APWM = 0; // capture mode
    ECap1Regs.ECCTL2.bit.CONT_ONESHT = 0; // one-shot mode
    ECap1Regs.ECCTL2.bit.SYNCO_SEL = 0; // pass the syncInput to syncOutput
    ECap1Regs.ECCTL2.bit.SYNCI_EN = 0; // Disable SyncIn. APWM mode로 사용하지 않으므로, CTRPHS값이 TSCTR에 loading될 필요가 없음.

}
void Start_hall_sensor(){
    hall_state = hall_sensor_update(); // Get initial state

    //start_ecap
    ECap1Regs.ECCTL2.bit.TSCTRSTOP = 1; // configuration is done. start eCAP_RUN;
    ECap1Regs.ECCTL2.bit.STOP_WRAP = 0; //wrap around after CEVT1
    ECap1Regs.ECEINT.bit.CEVT1 = 1;//enable CEVT1 as interrupt source
    EINT;
}

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
            // forward
            hall_state.angle_E_rad += Deg_60_IN_RAD;
        }else if(hall_state.hu & !hall_state.hv & !hall_state.hw){
            // 1 0 0
            // backward
            hall_state.angle_E_rad -= Deg_60_IN_RAD;
        }
    }else if(state.hu & !state.hv & !state.hw){
        // 1 0 0
        // 0 - pi/3
        state.angle_E_rad = 0;
        if(hall_state.hu & !hall_state.hv & hall_state.hw){
            // 1 0 1
            // forward
            hall_state.angle_E_rad += Deg_60_IN_RAD;
        }else if(hall_state.hu & hall_state.hv & !hall_state.hw){
            // 1 1 0
            // backward
            hall_state.angle_E_rad -= Deg_60_IN_RAD;
        }
    }else if(state.hu & state.hv & !state.hw){
        // 1 1 0
        // pi/3 - 2*pi/3
        state.angle_E_rad = Deg_60_IN_RAD;
        if(hall_state.hu & !hall_state.hv & !hall_state.hw){
            // 1 0 0
            // forward
            hall_state.angle_E_rad += Deg_60_IN_RAD;
        }else if(!hall_state.hu & hall_state.hv & !hall_state.hw){
            // 0 1 0
            // backward
            hall_state.angle_E_rad -= Deg_60_IN_RAD;
        }
    }else if(!state.hu & state.hv & !state.hw){
        // 0 1 0
        // 2*pi/3 - 3*pi/3
        state.angle_E_rad = 2*Deg_60_IN_RAD;
        if(hall_state.hu & hall_state.hv & !hall_state.hw){
            // 1 1 0
            // forward
            hall_state.angle_E_rad += Deg_60_IN_RAD;
        }else if(!hall_state.hu & hall_state.hv & hall_state.hw){
            // 0 1 1
            // backward
            hall_state.angle_E_rad -= Deg_60_IN_RAD;
        }
    }else if(!state.hu & state.hv & state.hw){
        // 0 1 1
        // 3*pi/3 - 4*pi/3
        state.angle_E_rad = 3*Deg_60_IN_RAD;
        if(!hall_state.hu & state.hv & !hall_state.hw){
            // 0 1 0
            // forward
            hall_state.angle_E_rad += Deg_60_IN_RAD;
        }else if(!hall_state.hu & !hall_state.hv & hall_state.hw){
            // 0 0 1
            // backward
            hall_state.angle_E_rad -= Deg_60_IN_RAD;
        }
    }else if(!state.hu & !state.hv & state.hw){
        // 0 0 1
        // 4*pi/3 - 5*pi/3
        state.angle_E_rad = 4*Deg_60_IN_RAD;
        if(!hall_state.hu & hall_state.hv & hall_state.hw){
            // 0 1 1
            // forward
            hall_state.angle_E_rad += Deg_60_IN_RAD;
        }else if(hall_state.hu & !hall_state.hv & hall_state.hw){
            // 1 0 1
            // backward
            hall_state.angle_E_rad -= Deg_60_IN_RAD;
        }
    }

    if(ECap1Regs.TSCTR >= 200000000){
        // No signal from ECap1 during 1 second
        state.Wr = 0;
        hall_state.Wr = 0;
    }
    state.rotation = 0;

    hall_state.hu = state.hu;
    hall_state.hv = state.hv;
    hall_state.hw = state.hw;

    // restrict electrical angle between 0 to 2PI
    if(hall_state.angle_E_rad >= TWOPI){
        hall_state.rotation += 1;
        hall_state.angle_E_rad -= TWOPI;
    }
    if(hall_state.angle_E_rad < 0){
        hall_state.rotation -= 1;
        hall_state.angle_E_rad += TWOPI;
    }

    return state;
}

float64 hall_sensor_get_E_angle_rad(){
    float64 time2 = ECap1Regs.TSCTR;
    byte hu = hall_state.hu;
    byte hv = hall_state.hv;
    byte hw = hall_state.hw;
    float64 Wr = hall_state.Wr;

    float64 angle;
    if (hu==1 & hv==0 & hw==1){
        angle = hall_state.angle_E_rad + (time2)*(5/1000000000)*Wr;
        if (angle > PI/3){
            angle = PI/3;
        }
        if (angle < 0){
            angle = 0;
        }
    }
    else if (hu==1 & hv==0 & hw==0){
        angle = hall_state.angle_E_rad + (time2-time1/6)*(5/1000000000)*Wr;
        if (angle > PI*2/3){
            angle = PI*2/3;
        }
        if (angle < PI/3){
            angle = PI/3;
        }
    }
    else if (hu==1 & hv==1 & hw==0){
        angle = hall_state.angle_E_rad + (time2-time1*2/6)*(5/1000000000)*Wr;
        if (angle > PI){
            angle = PI;
        }
        if (angle < PI*2/3){
            angle = PI*2/3;
        }
    }
    else if (hu==0 & hv==1 & hw==0){
        angle = hall_state.angle_E_rad + (time2-time1*3/6)*(5/1000000000)*Wr;
        if (angle > PI*4/3){
            angle = PI*4/3;
        }
        if (angle < PI){
            angle = PI;
        }
    }
    else if (hu==0 & hv==1 & hw==1){
        angle = hall_state.angle_E_rad + (time2-time1*4/6)*(5/1000000000)*Wr;
        if (angle > PI*5/3){
            angle = PI*5/3;
        }
        if (angle < PI*4/3){
            angle = PI*4/3;
        }
    }
    else if (hu==0 & hv==0 & hw==1){
        angle = hall_state.angle_E_rad + (time2-time1*5/6)*(5/1000000000)*Wr;
        if (angle > TWOPI){
            angle = TWOPI;
        }
        if (angle < PI*5/3){
            angle = PI*5/3;
        }
    }
    return angle;
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
