#include <F28x_Project.h>

/**
 * main.c
 *
 * 2021 BEV
 * Author: Jaeyeon Park
 *
 */
#include "controller.h"
#define INITIAL_DELAY 10000000 // MICROSECONDS
#define TIMER0_DURATION 1000000 // microseconds
#define NUM_OF_RECORDS 300
float32 testcurrentD=0;
float32 testcurrentQ=0;
//float32 angles[NUM_OF_RECORDS];
//float32 samples[NUM_OF_RECORDS];
//float32 phase_currents[NUM_OF_RECORDS];
//float32 d_currents[NUM_OF_RECORDS];
//float32 q_currents[NUM_OF_RECORDS];
Uint32 i = 0;
Uint32 record = 0;

interrupt void timer0_debugging_isr(){
    CpuTimer0.InterruptCount++;
    test_Idq_update(testcurrentD,testcurrentQ);
   if(record){
       if(i < NUM_OF_RECORDS){
           //angles[i] = get_angle_observer().angle;//hall_sensor_get_E_angle_rad();//get_hall_state().angle_E_rad;
           //phase_currents[i] = get_3phase_currents()[phaseU];
           //d_currents[i] = get_dqr_currents()[0]; // 0: d-axis, 1: q-axis
           //q_currents[i] = get_dqr_currents()[1];
           i++;
       }
   }

    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1; //flip the led state0
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // clear PIEACK.1 == PIEACK switch closed
}


int main(void){
    InitSysCtrl();
    InitGpio();

    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

    DELAY_US(INITIAL_DELAY);
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    Ready_controller();
    EALLOW;

    /* SETTING TIMER0 FOR DEBUGGING */
    // enable clock for timers
    // debugging timer
    // GPIO SETUP
    // BUILT IN LED
    GpioCtrlRegs.GPAGMUX2.bit.GPIO31 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1; // GPIO FOR OUTPUT

    CpuSysRegs.PCLKCR0.bit.CPUTIMER0 = 1;
    CpuSysRegs.PCLKCR0.bit.CPUTIMER1 = 1;
    InitCpuTimers(); // initialize global variable CpuTimer0, CpuTimer1, CpuTimer2
    ConfigCpuTimer(&CpuTimer0,200, TIMER0_DURATION); // timer, 200MHz(PLL CLK),
    IER |= (M_INT13 | M_INT1); // M_INT13 = CPU TIMER1 interrupt, M_INT1 = PIE1 (to enable timer0 interrupt)

    PieCtrlRegs.PIECTRL.bit.ENPIE = 1; // set total PIE Enable Register
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1; // allow TIMER0 interrupt signal
    PieVectTable.TIMER0_INT = timer0_debugging_isr; // connect the isr
    EDIS;
    EINT;
    ERTM;


    Start_controller();

    CpuTimer0.RegsAddr->TCR.bit.TSS = 0; //StartCpuTimer0();

    //test_I_update(1);
    while(1){

    }
}
