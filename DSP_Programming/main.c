#include <F28x_Project.h>

/**
 * main.c
 *
 * 2021 BEV
 * Author: Jaeyeon Park
 *
 */
#include "controller.h"

int flag = 0;
interrupt void timer0_isr(){
    if(flag==0){
        test_I_update(10);
        flag = 1;
    }
    CpuTimer0.InterruptCount++;
    test_angle_update(PI/180);
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1; //flip the led state0
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // clear PIEACK.1 == PIEACK switch closed
}

byte test;
float32 test_float;

int main(void){
    InitSysCtrl();
    InitGpio();

    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

    Ready_controller();

    EALLOW;
    // enable clock for timers
    // debugging timer
    // GPIO SETUP
    GpioCtrlRegs.GPAGMUX2.bit.GPIO31 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1; // GPIO FOR OUTPUT


    CpuSysRegs.PCLKCR0.bit.CPUTIMER0 = 1;
    CpuSysRegs.PCLKCR0.bit.CPUTIMER1 = 1;
    InitCpuTimers(); // initialize global variable CpuTimer0, CpuTimer1, CpuTimer2
    ConfigCpuTimer(&CpuTimer0,200,5000000); // timer, 200MHz(PLL CLK), 5000us
    IER |= (M_INT13 | M_INT1); // M_INT13 = CPU TIMER1 interrupt, M_INT1 = PIE1 (to enable timer0 interrupt)

    PieCtrlRegs.PIECTRL.bit.ENPIE = 1; // set total PIE Enable Register
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1; // allow TIMER0 interrupt signal
    PieVectTable.TIMER0_INT = timer0_isr; // connect the isr
    EDIS;

    EINT;
    ERTM;


    Start_controller();

    CpuTimer0.RegsAddr->TCR.bit.TSS = 0; //StartCpuTimer0();

    while(1){

    }
}
