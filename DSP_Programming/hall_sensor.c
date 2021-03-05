/*
 * hall_sensor.c
 *
 *  Created on: 2021. 3. 2.
 *      Author: Jaeyeon Park
 */

#include "hall_sensor.h"
Commutation_state commutation;

enum edge_event{rising=0, falling=1};

void end_of_eCAP_INT(volatile struct ECAP_REGS* eCAP){
    eCAP->ECCLR.bit.INT = 1; // CLEAR INTERRUPT EVENT
    eCAP->ECCLR.bit.CEVT1 = 1; //CLEAR EVENT FROM CAP1
    eCAP->ECCTL2.bit.REARM = 1; //RE-ARM
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP4;
}


interrupt void ecap1_isr(){
    // HU rising
    volatile struct ECAP_REGS* ecap = &ECap1Regs;
    commutation.hu = 1;
    end_of_eCAP_INT(ecap);
}
interrupt void ecap2_isr(){
    // HV rising
    volatile struct ECAP_REGS* ecap = &ECap2Regs;
    commutation.hv = 1;
    end_of_eCAP_INT(ecap);
}
interrupt void ecap3_isr(){
    // HW rising
    volatile struct ECAP_REGS* ecap = &ECap3Regs;
    commutation.hw = 1;
    end_of_eCAP_INT(ecap);
}
interrupt void ecap4_isr(){
    // HU falling
    volatile struct ECAP_REGS* ecap = &ECap4Regs;
    commutation.hu = 0;
    end_of_eCAP_INT(ecap);
}
interrupt void ecap5_isr(){
    // HV falling
    volatile struct ECAP_REGS* ecap = &ECap5Regs;
    commutation.hv = 0;
    end_of_eCAP_INT(ecap);
}
interrupt void ecap6_isr(){
    // HW falling
    volatile struct ECAP_REGS* ecap = &ECap6Regs;
    commutation.hw = 0;
    end_of_eCAP_INT(ecap);
}
void configure_eCAP_INT(){
    // enroll interrupt service routine for eCAP
    EALLOW;
    // rising edge capture
    PieVectTable.ECAP1_INT = ecap1_isr;
    PieVectTable.ECAP2_INT = ecap2_isr;
    PieVectTable.ECAP3_INT = ecap3_isr;
    // falling edge capture
    PieVectTable.ECAP4_INT = ecap4_isr;
    PieVectTable.ECAP5_INT = ecap5_isr;
    PieVectTable.ECAP6_INT = ecap6_isr;

    //enable ECAP 1,2,3 interrupt
    IER |= M_INT4;
    // rising edge capture
    PieCtrlRegs.PIEIER4.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER4.bit.INTx2 = 1;
    PieCtrlRegs.PIEIER4.bit.INTx3 = 1;
    // falling edge capture
    PieCtrlRegs.PIEIER4.bit.INTx4 = 1;
    PieCtrlRegs.PIEIER4.bit.INTx5 = 1;
    PieCtrlRegs.PIEIER4.bit.INTx6 = 1;
    EDIS;

}

void configure_eCAP(volatile struct ECAP_REGS* eCAP, int edge_event){
    eCAP->ECEINT.all = 0x0000;
    eCAP->ECCLR.all = 0xFFFF;
    eCAP->ECCTL1.bit.CAPLDEN = 0; // Disable CAP1-CAP4 register loads
    eCAP->ECCTL2.bit.TSCTRSTOP = 0; // Make sure the counter is stopped

    eCAP->ECCTL1.bit.PRESCALE = 0; //ECAP input signal bypasses prescaler
    eCAP->ECCTL1.bit.CAP1POL = (edge_event==rising? 0:1); // rising edge eventl

    eCAP->ECCTL1.bit.CTRRST1 = 0; // no reset counter when the event happens
    eCAP->ECCTL1.bit.CTRRST2 = 0; //
    eCAP->ECCTL1.bit.CTRRST3 = 0; //
    eCAP->ECCTL1.bit.CTRRST4 = 0; //

    eCAP->ECCTL1.bit.CAPLDEN = 1;  // save CAP result in register
    eCAP->ECCTL2.bit.CAP_APWM = 0; // eCAP capture mode

    eCAP->ECCTL2.bit.CONT_ONESHT = 0; //continuous mode
    eCAP->ECCTL2.bit.SYNCO_SEL = 0; // pass the sync input signal to sync output
    eCAP->ECCTL2.bit.SYNCI_EN = 1; // enable SYNCIN

    // eCAP is stopped after configuration
}

void start_eCAP(volatile struct ECAP_REGS* eCAP){
    eCAP->ECCTL2.bit.TSCTRSTOP = 1; // start eCAP time stamp counter;
    eCAP->ECCTL2.bit.REARM = 1;
    eCAP->ECEINT.bit.CEVT1 = 1; // only allow CEVT 1. 1 interrupt = 1 event = 1 rising edge from hall sensor
    eCAP->ECEINT.bit.CEVT2 = 0;
    eCAP->ECEINT.bit.CEVT4 = 0;
    eCAP->ECEINT.bit.CEVT3 = 0;
}

void Init_hall_sensor_ECAP(Uint16 poll_pair){
    // Set GPIO pins for Hall sensor(UVW) as input ports
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

    // Connect GPIO to eCAP module
    // rising edge capture
    InputXbarRegs.INPUT7SELECT = 60; // eCAP1 to HU
    InputXbarRegs.INPUT8SELECT = 22; // eCAP2 to HV
    InputXbarRegs.INPUT9SELECT = 105; // eCAP3 to HW
    // falling edge capture
    InputXbarRegs.INPUT10SELECT = 60;// eCAP4 to HU
    InputXbarRegs.INPUT11SELECT = 22;// eCAP5 to HV
    InputXbarRegs.INPUT12SELECT = 105;// eCAP6 to HW

    // Configure eCAP
    //initialize unused GPIO PIN to synchronize ECAP
    GpioCtrlRegs.GPBGMUX2.bit.GPIO59 = 0; // use GPIO60 as gpio
    GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO59 = 1; // GPIO FOR OUTPUT
    GpioDataRegs.GPBDAT.bit.GPIO59 = 0; // WRIE 0 TO GPIO 61

    InputXbarRegs.INPUT6SELECT = 59;

    SyncSocRegs.SYNCSELECT.bit.ECAP1SYNCIN = 6; // 6: EXTSYNCIN2 is selected for SYNCIN of ECAP1
    SyncSocRegs.SYNCSELECT.bit.ECAP4SYNCIN = 4; // 6: EXTSYNCIN2 is selected for SYNCIN of ECAP4

    configure_eCAP_INT();
    configure_eCAP(&ECap1Regs,rising);
    configure_eCAP(&ECap2Regs,rising);
    configure_eCAP(&ECap3Regs,rising);

    configure_eCAP(&ECap4Regs,falling);
    configure_eCAP(&ECap5Regs,falling);
    configure_eCAP(&ECap6Regs,falling);
}
void Start_hall_sensor_ECAP(){
    start_eCAP(&ECap1Regs);
    start_eCAP(&ECap2Regs);
    start_eCAP(&ECap3Regs);
    start_eCAP(&ECap4Regs);
    start_eCAP(&ECap5Regs);
    start_eCAP(&ECap6Regs);
    // synchronize eCAP 1,2,3 and 4,5,6
    // SYNCIN of eCAP 2,3 is the SYNCOUT of eCAP1
    // SYNCIN OF eCAP 5,6 is the SYNCOUT of eCAP1
    ECap1Regs.ECCTL2.bit.SWSYNC = 1;

    // disable sync in future
    ECap1Regs.ECCTL2.bit.SYNCI_EN = 0;
    ECap2Regs.ECCTL2.bit.SYNCI_EN = 0;
    ECap3Regs.ECCTL2.bit.SYNCI_EN = 0;
    ECap4Regs.ECCTL2.bit.SYNCI_EN = 0;
    ECap5Regs.ECCTL2.bit.SYNCI_EN = 0;
    ECap6Regs.ECCTL2.bit.SYNCI_EN = 0;
}
Commutation_state hall_sensor_get_commutation(){
    return commutation;
}


