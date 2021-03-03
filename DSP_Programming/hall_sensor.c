/*
 * hall_sensor.c
 *
 *  Created on: 2021. 3. 2.
 *      Author: Jaeyeon Park
 */

#include <hall_sensor.h>

#define TSCTR_PER_SEC 200000000
typedef char byte;

typedef struct {
    byte hu;
    byte hv;
    byte hw;
}Commutation_state ;
Commutation_state commutation;

enum edge_event{falling=0, rising=1};
enum hall_state{hu,hv,hw};
enum hall_state last_hall_state;

void end_of_eCAP_INT(volatile struct ECAP_REGS* eCAP){

    eCAP->ECEINT.bit.CEVT1 = eCAP->ECEINT.bit.CEVT1? 0:1;
    eCAP->ECEINT.bit.CEVT2 = eCAP->ECEINT.bit.CEVT2? 0:1;

    eCAP->ECCLR.bit.INT = 1;

    eCAP->ECCLR.bit.CEVT1 = 1;
    eCAP->ECCLR.bit.CEVT2 = 1;
    eCAP->ECCLR.bit.CEVT3 = 1;
    eCAP->ECCLR.bit.CEVT4 = 1;

    eCAP->ECCTL2.bit.REARM = 1;
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP4;
}
byte set_commutaition(volatile struct ECAP_REGS* ecap){
    if(ecap->ECEINT.bit.CEVT1)
        return 1;
    else if(ecap->ECEINT.bit.CEVT2)
        return 0;
}

interrupt void ecap1_isr(){
    // HU rising
    volatile struct ECAP_REGS* ecap = &ECap1Regs;
    commutation.hu = set_commutaition(ecap);

    end_of_eCAP_INT(ecap);
}
interrupt void ecap2_isr(){
    // HU falling
    volatile struct ECAP_REGS* ecap = &ECap2Regs;
    commutation.hv = set_commutation(ecap);

    end_of_eCAP_INT(ecap);
}
interrupt void ecap3_isr(){
    // HV rising
    volatile struct ECAP_REGS* ecap = &ECap3Regs;
    commutation.hw = set_commutation(ecap);

    end_of_eCAP_INT(ecap);
}

void configure_eCAP(volatile struct ECAP_REGS* eCAP){
    eCAP->ECEINT.all = 0x0000;
    eCAP->ECCLR.all = 0xFFFF;
    eCAP->ECCTL1.bit.CAPLDEN = 0; // Disable CAP1-CAP4 register loads
    eCAP->ECCTL2.bit.TSCTRSTOP = 0; // Make sure the counter is stopped

    eCAP->ECCTL1.bit.PRESCALE = 0; //ECAP input signal bypasses prescaler
    eCAP->ECCTL1.bit.CAP1POL = 1; // rising edge eventl
    eCAP->ECCTL1.bit.CAP2POL = 0; // falling edge eventl

    eCAP->ECCTL1.bit.CTRRST1 = 0; // no reset counter when the event happens
    eCAP->ECCTL1.bit.CTRRST2 = 0; //
    eCAP->ECCTL1.bit.CTRRST3 = 1; //
    eCAP->ECCTL1.bit.CTRRST4 = 1; //

    eCAP->ECCTL1.bit.CAPLDEN = 1;  // save CAP result in register
    eCAP->ECCTL2.bit.CAP_APWM = 0; // eCAP capture mode

    eCAP->ECEINT.bit.CEVT4 = 0;
    eCAP->ECEINT.bit.CEVT3 = 0;

    eCAP->ECCTL2.bit.CONT_ONESHT = 0; //continuous mode
    eCAP->ECCTL2.bit.SYNCO_SEL = 0; // pass the sync input signal to sync output
    eCAP->ECCTL2.bit.SYNCI_EN = 1; // enable SYNCIN
    // eCAP is stopped after configuration
}

void start_eCAP(volatile struct ECAP_REGS* eCAP){
    eCAP->ECCTL2.bit.TSCTRSTOP = 1; // start eCAP time stamp counter;
    eCAP->ECCTL2.bit.REARM = 1;
    eCAP->ECEINT.bit.CEVT1 = 1; // only allow CEVT 1. 1 interrupt = 1 event = 1 rising edge from hall sensor
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
    InputXbarRegs.INPUT7SELECT = 60; // eCAP1 to HU
    InputXbarRegs.INPUT8SELECT = 22; // eCAP2 to HU
    InputXbarRegs.INPUT9SELECT = 105; // eCAP3 to HV

    // enroll interrupt service routine for eCAP
    DINT;
    EALLOW;
    PieVectTable.ECAP1_INT = ecap1_isr;
    PieVectTable.ECAP2_INT = ecap2_isr;
    PieVectTable.ECAP3_INT = ecap3_isr;
    EDIS;

    //enable ECAP 1,2,3 interrupt
    IER |= M_INT4;
    PieCtrlRegs.PIEIER4.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER4.bit.INTx2 = 1;
    PieCtrlRegs.PIEIER4.bit.INTx3 = 1;

    // Configure eCAP1
    configure_eCAP(&ECap1Regs);
    configure_eCAP(&ECap2Regs);
    configure_eCAP(&ECap3Regs);

    EINT;
    ERTM;
}
void Start_hall_sensor_ECAP(){
    start_eCAP(&ECap1Regs);
    start_eCAP(&ECap2Regs);
    start_eCAP(&ECap3Regs);
    // synchronize eCAP 1,2,3
    // SYNCIN of eCAP 2,3 is the SYNCOUT of eCAP1
    // SYNCOUT of eCAP1 is the SYNCIN of eCAP1 (SYNCO_SEL = 0)
    ECap1Regs.ECCTL2.bit.SWSYNC = 1;
}


