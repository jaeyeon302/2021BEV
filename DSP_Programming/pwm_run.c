/*
 * pwm_run.c
 *
 *  Created on: 2021. 3. 3.
 *      Author: Jaeyeon Park
 */
#include "pwm_run.h"

typedef struct CMPV{
    Uint16 CMPA;
    Uint16 CMPB;
} PWM_CMP_Val;

PWM_CMP_Val pwm_cmp[3];
Uint32 TB_PRD = 0;
Uint32 tmp = 0;

void epwm_set_duty(volatile struct EPWM_REGS* epwm,
                   float32 a_duty_ratio,
                   float32 b_duty_ratio){
    // a_duty_ratio, b_duty_ratio should be 0<=cmp_ratio<=1;
    // Assumed up & down count mode

    if(TB_PRD==0) return;// not set CMPA & CMPB if TB_PRD is not set
    // protection
    if(a_duty_ratio > 0.0 && b_duty_ratio > 0.0) return;

    Uint16 cmpa = 0;
    Uint16 cmpb = 0;
    if(a_duty_ratio == 0.0){
        cmpa = TB_PRD+1; // turn off
    }else if(a_duty_ratio == 1.0){
        cmpa = 0;
    }else if(a_duty_ratio>0.0 && a_duty_ratio<1.0){
        cmpa = (Uint16)(TB_PRD - a_duty_ratio*TB_PRD); // bye bye Decimal places
    }

    if(b_duty_ratio == 0.0){
        cmpb = TB_PRD+1; // turn off
    }else if(b_duty_ratio == 1.0){
        cmpb = 0;
    }else if(b_duty_ratio > 0.0 && b_duty_ratio <1.0){
        cmpb = (Uint16)(TB_PRD - b_duty_ratio*TB_PRD);
    }
    epwm->CMPA.bit.CMPA = cmpa;
    epwm->CMPB.bit.CMPB = cmpb;
}
float32 epwm_get_minimum_duty_ratio(){
    return 1/((float32)TB_PRD);
}

void epwm1_set_duty(float32 CMPA_ratio, float32 CMPB_ratio){
    epwm_set_duty(&EPwm1Regs, CMPA_ratio, CMPB_ratio);
}
void epwm2_set_duty(float32 CMPA_ratio, float32 CMPB_ratio){
    epwm_set_duty(&EPwm2Regs, CMPA_ratio, CMPB_ratio);
}
void epwm3_set_duty(float32 CMPA_ratio, float32 CMPB_ratio){
    epwm_set_duty(&EPwm3Regs, CMPA_ratio, CMPB_ratio);
}
void end_of_ePWM_INT(volatile struct EPWM_REGS* epwm){
    epwm->ETCLR.bit.INT = 1;
    epwm->ETCLR.bit.SOCA = 1;
    epwm->ETCLR.bit.SOCB = 1;
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP3;
}

interrupt void epwm1_isr(){
    tmp++;
    end_of_ePWM_INT(&EPwm1Regs);
}
interrupt void epwm2_isr(){
    tmp++;
    end_of_ePWM_INT(&EPwm2Regs);
}
interrupt void epwm3_isr(){
    tmp++;
    end_of_ePWM_INT(&EPwm3Regs);
}

void configure_ePWM_INT(){
    EALLOW;
    PieVectTable.EPWM1_INT = epwm1_isr;
    PieVectTable.EPWM2_INT = epwm2_isr;
    PieVectTable.EPWM3_INT = epwm3_isr;

    IER |= M_INT3;

    // this interrupt will call the control logic loop
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;

    // why should we turn on all the interrupt?
    //PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
    //PieCtrlRegs.PIEIER3.bit.INTx3 = 1;
    EDIS;
}

void configure_ePWM(volatile struct EPWM_REGS* epwm){
    // PWM CLOCK CONFIGURATION
    // FOR LAUNCHXL_F28379D
    // external clock is 10MHz oscillator
    // PLLSYSCLK = 10M * 40/2 = 200MHz
    // refer to 27page of https://www.ti.com/lit/ug/sprui77c/sprui77c.pdf?ts=1613144312872&ref_url=https%253A%252F%252Fwww.google.com%252F

    // EPWMCLKDIV
    // 0 : EPWMCLK = PLLSYSCLK = 200MHz
    // 1 : EPWMCLK = PLLSYSCLK/2 = 100MHz
    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1;

    // TBCLK = EPWMCLK/(HSPCLKDIV x CLKDIV)
    // refer to TI Technical doc 1983page
    // CLKDIV -> 0 : clkdiv = 1
    // HSPCLKDIV -> 1 : hspclkdiv = 2
    // wiht upper config, TBCLK = 50MHz
    epwm->TBCTL.bit.CLKDIV = 0;
    epwm->TBCTL.bit.HSPCLKDIV = 1;
    /* END of configuration of TBCLK frequency */


    // Target PWM frequency : 10kHz
    // TBCLK = 50MHz -> 50 000 000 Counter Per Second(if there is no limit)
    // To get PWM frequency 10kHz, TBPRD should be set
    // the maximum counter value shoule be = 5000 (= 50,000,000/10,000)
    // HOWEVER
    // counter mode is up and down for the symmetric pwm pulse
    // because of this up&down counter mode, the twice of TBPRD should be matched
    // there fore TBPRD = 2500
    epwm->TBCTL.bit.CTRMODE = 2; // counter up and down for the symmetric pwm
    epwm->TBCTL.bit.PRDLD = 0; // enable shadow register of TB module
    epwm->TBPHS.bit.TBPHS = 0x0000; // phase is 0
    epwm->TBPRD = 2500; // 10kHz
    TB_PRD = epwm->TBPRD;

    // Setup TBCTL SYNC
    epwm->TBCTL.bit.SWFSYNC = 0;
    epwm->TBCTL.bit.SYNCOSEL = 0; // transfer SYNCIN to SYNCOUT directly
    epwm->TBCTL.bit.PHSEN = 1; // ENABLE SYNC FROM OUTSIDE

    // Setup shadowing
    // enable shadow mode for CMPA and CMPB
    epwm->CMPCTL.bit.SHDWAMODE = 0;
    epwm->CMPCTL.bit.SHDWBMODE = 0;
    // Define the timing when the DSP transfer the data
    // from shadow register to the actual active register
    epwm->CMPCTL.bit.LOADAMODE=0; //Load on TBCTR=Zero
    epwm->CMPCTL.bit.LOADBMODE=0;

    // Set actions
    epwm->AQCTLA.bit.CAU = 2; // set PWM1A(HIGH) on event A, up count
    epwm->AQCTLA.bit.CAD = 1; // clear PWM1A (LOW) on event A, down count
    epwm->AQCTLB.bit.CBU = 2;
    epwm->AQCTLB.bit.CBD = 1;

    // Set Default Compare Values
    epwm->CMPA.bit.CMPA = TB_PRD+1; // turn off
    epwm->CMPB.bit.CMPB = TB_PRD+1; // turn off
    // WTFWTFWTF 왜 CMPA 이벤트를 발생하게 하면 Carrier 주파수가 바뀌는가?????????
    // CMPA 값을 바꾸면 Carrier wave의 주파수가 바뀜
    // 문제해결 :
    // InputXbarRegs로 들어오는 EPWM1의 EXTSYNCIN을 GPIO와 연결한 뒤 제대로 pull down해주지 않아서였음.
    // pulldown 해주니 CMPA가 바뀌더라도 carrier wave가 바뀌지 않음
    // 왜 그런지는 내부 회로가 어떻게 되어있는지를 몰라서...


    // Register Interrupt where we will change the Compare Values
    // enable event time-base counter equal to period
    // refer to Technical document 2073 page
    epwm->ETSEL.bit.INTSEL = 1; // event TBCTR = 0x00 or TBPRD
    epwm->ETSEL.bit.INTEN = 1; // enable interrupt

    // generate Interrupt on every TBCTR = ZERO event
    // it manages the control loop frequency
    // INTPRD=1 -> control loop frequency = 10kHz/1;
    epwm->ETPS.bit.INTPRD = 1;

    // generate EPWMxSOCA Interrupt on every TBCTR = ZERO event
    // all epwm generate SOCA signal
    epwm->ETSEL.bit.SOCASEL = 1;
    epwm->ETSEL.bit.SOCAEN = 1;
    epwm->ETPS.bit.SOCAPRD = 1;
    epwm->ETCLR.bit.SOCA = 1;

    // generate EPWMxSOCB Interrupt on every TBCTR = TBPRD event
    // all epwm generate SOCB signal
    epwm->ETSEL.bit.SOCBSEL = 2;
    epwm->ETSEL.bit.SOCBEN = 1;
    epwm->ETPS.bit.SOCBPRD = 1;
    epwm->ETCLR.bit.SOCB = 1;
}


void Init_3phase_ePWM(){
    EALLOW;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0=0;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO1=0;
    InitEPwm1Gpio();
    GpioCtrlRegs.GPAGMUX1.bit.GPIO2=0;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO3=0;
    InitEPwm2Gpio();
    GpioCtrlRegs.GPAGMUX1.bit.GPIO4=0;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO5=0;
    InitEPwm3Gpio();


    // turn on the clock for epwm modules
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM3 = 1;
    // TB Clocks of all ePWM modules are stopped
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    // Configure ePWM SYNC
    // it is the same pin with the pin of hall_sensor.c
    // initialize unused GPIO PIN to synchronize EPWM only with SW sync
    GpioCtrlRegs.GPBGMUX2.bit.GPIO61 = 0; // use GPIO60 as gpio
    GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO61 = 1; // GPIO FOR OUTPUT
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1; // WRITE 0 TO GPIO 61

    InputXbarRegs.INPUT5SELECT = 61; // XINT1;

    configure_ePWM_INT();
    configure_ePWM(&EPwm1Regs);
    configure_ePWM(&EPwm2Regs);
    configure_ePWM(&EPwm3Regs);
}
void Start_3phase_ePWM(){
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC=1; //all TBCLK start counting
    EDIS;

    EPwm1Regs.TBCTL.bit.SWFSYNC = 1;

    // disable sync in future
    EPwm1Regs.TBCTL.bit.PHSEN = 0;
    EPwm2Regs.TBCTL.bit.PHSEN = 0;
    EPwm3Regs.TBCTL.bit.PHSEN = 0;
}
