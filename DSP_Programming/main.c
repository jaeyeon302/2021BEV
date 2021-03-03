#include <F28x_Project.h>

/**
 * main.c
 *
 * 2021 BEV
 * @author Jaeyeon Park
 *
 */
#include "hall_sensor.h"
#include "pwm_run.h"
/*
interrupt void epwm1_isr(void){
    EPwm1Regs.ETCLR.bit.INT=1; //clear
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP3;
}*/

interrupt void timer0_isr(){
    CpuTimer0.InterruptCount++;
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1; //flip the led state
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // clear PIEACK.1 == PIEACK switch closed
}

int main(void){
    InitSysCtrl();
    InitGpio();
/*
    // EPWM1 CONFIGURATION
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1; // enable clock for EPWM1

    DINT;
    InitPieCtrl();

    IER = 0x0000;
    IFR = 0x0000;

    InitPieVectTable();
    EALLOW;
    PieVectTable.EPWM1_INT=&epwm1_isr;
    EDIS;
    IER |= M_INT3;
    PieCtrlRegs.PIEIER3.bit.INTx1=1; //EPWM1_INT

    InitEPwm1Gpio();
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0=0;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO1=0;

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC=0;// time base clk of all ePWM modules is stopped
    EDIS;

    // initialize EPWM1
    //Setup TBCLK
    EPwm1Regs.TBPRD=20000;
    EPwm1Regs.TBPHS.bit.TBPHS=0x0000; //phase is 0
    EPwm1Regs.TBCTR=0x0000; //clear counter
    //Set Compare values
    EPwm1Regs.CMPA.bit.CMPA=10000; //set compare A value
    EPwm1Regs.CMPB.bit.CMPB=1000; //set compare B value
    //Setup counter mode
    EPwm1Regs.TBCTL.bit.CTRMODE=2; //Count up and down
    EPwm1Regs.TBCTL.bit.PHSEN=0; //Disable phase loading (TBPHS value)
    EPwm1Regs.TBCTL.bit.HSPCLKDIV=0; //tbclk prescale. TBCLK=EPWMCLK
    EPwm1Regs.TBCTL.bit.CLKDIV=0;//
    //Setup shadowing
    EPwm1Regs.CMPCTL.bit.SHDWAMODE=0; //Shadow mode
    EPwm1Regs.CMPCTL.bit.SHDWBMODE=0;
    EPwm1Regs.CMPCTL.bit.LOADAMODE=0; //Load on TBCTR=Zero
    EPwm1Regs.CMPCTL.bit.LOADBMODE=0;
    //Set actions
    EPwm1Regs.AQCTLA.bit.CAU=2; //Set PWM1A on event A, up count
    EPwm1Regs.AQCTLA.bit.CAD=1; //Clear PWM1A on event A, down count
    EPwm1Regs.AQCTLB.bit.CBU=2; //Set PWM1B on event B, up count
    EPwm1Regs.AQCTLB.bit.CBD=1; //Clear PWM1B on event B, down count
    //Interrupt where we will change the compare values
    EPwm1Regs.ETSEL.bit.INTSEL=1; //TBCTR=Zero event is trigger for EPWM1_INT
    EPwm1Regs.ETSEL.bit.INTEN=1; //Enable INT
    EPwm1Regs.ETPS.bit.INTPRD=3; //Generate INT on every 3rd TBCTR=Zero event

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC=1; //all TBCLK start counting
    EDIS;

    */
    // END OF EPWM CONFIGURATION
    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;

    InitPieVectTable();


    Init_3phase_ePWM();
    Init_hall_sensor_ECAP(23);




    EALLOW;
    // enable clock for timers
    CpuSysRegs.PCLKCR0.bit.CPUTIMER0 = 1;
    CpuSysRegs.PCLKCR0.bit.CPUTIMER1 = 1;
    InitCpuTimers(); // initialize global variable CpuTimer0, CpuTimer1, CpuTimer2
    ConfigCpuTimer(&CpuTimer0,200,1000000); // timer, 200MHz(PLL CLK), 1000000us
    IER |= (M_INT13 | M_INT1); // M_INT13 = CPU TIMER1 interrupt, M_INT1 = PIE1 (to enable timer0 interrupt)


    PieCtrlRegs.PIECTRL.bit.ENPIE = 1; // set total PIE Enable Register
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1; // allow TIMER0 interrupt signal

    PieVectTable.TIMER0_INT = timer0_isr; // connect the isr

    EDIS;
    EINT;
    ERTM;


    Start_3phase_ePWM();
    Start_hall_sensor_ECAP();
    //EPwm1Regs.CMPA.bit.CMPA = 2500/(100-50);

    CpuTimer0.RegsAddr->TCR.bit.TSS = 0; //StartCpuTimer0();

    while(1){

    }
}
