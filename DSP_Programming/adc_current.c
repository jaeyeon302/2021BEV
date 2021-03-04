/*
 * adc_current.c
 *
 *  Created on: 2021. 3. 4.
 *      Author: ok6530
 */

#include "adc_current.h"
enum signal_mode{single=0,differential=1};

// sysclock = 200MHz ( Tsys = 5ns)
// ADCclock = 200MHz / prescaler. refer to tech doc 1597 page
// 0 : ADCClock = sysclock frequency = 200MHz ( Tadc = 5ns)
// 6 : ADCClock = sysclck/4 = 50MHz ( Tadc = 20ns)
const unsigned int ADC_PRESCALE = 0;

void end_of_ADCINT1(volatile struct ADC_REGS* adc){
    adc->ADCINTFLGCLR.bit.ADCINT1 = 1;
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP1;
}

interrupt void ADCAINT1_isr(){
    volatile struct ADC_REGS* adc = &AdcaRegs;
    end_of_ADCINT1(adc);
}
interrupt void ADCBINT1_isr(){
    volatile struct ADC_REGS* adc = &AdcbRegs;
    end_of_ADCINT1(adc);
}
interrupt void ADCCINT1_isr(){
    volatile struct ADC_REGS* adc = &AdccRegs;
    end_of_ADCINT1(adc);
}
interrupt void ADCDINT1_isr(){
    volatile struct ADC_REGS* adc = &AdcdRegs;
    end_of_ADCINT1(adc);
}


void configure_ADC(Uint16 adc_num,
                   int channel,
                   int mode,
                   unsigned int trigsel){
    // module clock should be enabled at the outside of this scope
    // - adc : ADC_ADCA or ADC_ADCB or ADC_ADCC or ADC_ADCD
    // - channel : interger -> see the docs
    // - mode : ADC_SIGNALMODE_SINGLE or ADC_SIGNALMODE_DIFFERENTIAL

    /* ADC CLOCK
     *  In 16-bit mode, the core requires approximately 29.5 ADCCLK cycles to process a voltage into a
     *  conversion result, while in 12-bit mode, this process requires approximately 10.5 ADCCLK cycles. The
     *  choice of resolution will also determine the necessary duration of the acquisition window, see
     *  Section 11.15.2.
     */
    volatile struct ADC_REGS* adc;
    EALLOW;
    switch(adc_num){
        case ADC_ADCA:
            CpuSysRegs.PCLKCR13.bit.ADC_A = 1;
            adc = &AdcaRegs;
            break;
        case ADC_ADCB:
            CpuSysRegs.PCLKCR13.bit.ADC_B = 1;
            adc = &AdcbRegs;
            break;
        case ADC_ADCC:
            CpuSysRegs.PCLKCR13.bit.ADC_C = 1;
            adc = &AdccRegs;
            break;
        case ADC_ADCD:
            CpuSysRegs.PCLKCR13.bit.ADC_D = 1;
            adc = &AdcdRegs;
            break;
        default:
            break;
    }
    EDIS;

    EALLOW;
    //AdcSetMode(adc_num, ADC_RESOLUTION_12BIT , mode);
    adc->ADCCTL2.bit.RESOLUTION = ADC_RESOLUTION_12BIT;
    adc->ADCCTL2.bit.SIGNALMODE = mode;

    /*
     * ADC CLOCK and sample&hold time should be configured to meet the required
     * specification of control logic
     *
     * In this SNU Graduation project, the control frequency is 10kHz
     * ADC sampling, conversion, and calculation for control
     * should be done in 100us(=1/10,000 sec = 100,000 ns)
     */

    // sysclock = 200MHz (=> 1 sysclk cycle = 5ns)
    // ADCclock = 200MHz / prescaler. refer to tech doc 1597 page
    // EOC Interrupt after the end of the conversion
    //    = 10.5 ADC clock cycle (12 bit)
    //    = 29.5 ADC clock cycle (16 bit)
    // refer to tech doc 1581 page (the unit of time t is the the number of sysclk cycles)
    adc->ADCCTL2.bit.PRESCALE = ADC_PRESCALE;


    //POWER UP
    adc->ADCCTL1.bit.ADCPWDNZ = 1;
    DELAY_US(1E3);
    // delay 1000us.
    // The delay is needed after power up the adc module.
    // refer to technical reference 1574p

    // Sample&Hold time = (ACQPS + 1) SYSCLK cycles
    // SH time > 1 ADCCLK cycle (must)
    // at ADC_PRESCALE = 6, 1 ADCCLK cycle = 4 SYSCLK cycles
    // at ADC_PRESCALE = 0, 1 ADCCLK cycle = 1 SYSCLK cycle
    adc->ADCSOC0CTL.bit.ACQPS = 63; // 64 sysclk cycle = 320ns

    // set the timing when the EOC signal is generated. 0(end of acq windows), 1(end of conversion)
    // refer to tech doc 1581 page
    adc->ADCCTL1.bit.INTPULSEPOS = 1;

    // configure SOC
    adc->ADCSOC0CTL.bit.CHSEL = channel;
    adc->ADCSOC0CTL.bit.TRIGSEL = trigsel;    // ePWM1 pulls the trigger of ADCSOC (1637 page tech doc)
    adc->ADCINTSOCSEL1.bit.SOC0 = 0;    // disable all ADCINT for trigger

    // configure EOC interrupt
    // each ADC module has 4 ADC interrupt
    // ex) AdcaRegs-> ADCINTA1, ADCINTA2, ADCINTA3, ADCINT4
    // In this project, we only use ADCxINT1 (the highest priority between ADC Interrupts)
    // refer to tech doc 1569 page
    adc->ADCINTSEL1N2.bit.INT1E = 1; // ADCINT1 interrupt ENABLE
    adc->ADCINTSEL1N2.bit.INT1SEL = 0; // EOC0 IS trigger for ADCINT1 (SOCn <-> EOCn).
                                       // the return signal for SOC0 start signal, is come from EOC1


    adc->ADCINTFLGCLR.bit.ADCINT1 = 1; // clear the adcintflg latch
    adc->ADCINTSEL1N2.bit.INT1CONT = 0;
    EDIS;
}

void configure_ADC_INT(){
    IER |= M_INT1;
    EALLOW;
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;

    PieCtrlRegs.PIEIER1.bit.INTx1 = 1; //enable ADCINTA1
    PieCtrlRegs.PIEIER1.bit.INTx2 = 1; //enable ADCINTB1
    PieCtrlRegs.PIEIER1.bit.INTx3 = 1; //enable ADCINTC1
    PieCtrlRegs.PIEIER1.bit.INTx6 = 1; //enable ADCINTD1
    PieVectTable.ADCA1_INT = ADCAINT1_isr;
    PieVectTable.ADCB1_INT = ADCBINT1_isr;
    PieVectTable.ADCC1_INT = ADCCINT1_isr;
    PieVectTable.ADCD1_INT = ADCDINT1_isr;
    EDIS;
}

void Init_3current_ADC(){

    configure_ADC_INT();
    configure_ADC(ADC_ADCA, 2, ADC_SIGNALMODE_SINGLE, 5); // W
    configure_ADC(ADC_ADCB, 2, ADC_SIGNALMODE_SINGLE, 7); // V
    configure_ADC(ADC_ADCC, 2, ADC_SIGNALMODE_SINGLE, 9); // U

}

void Start_3current_ADC(){
    AdcaRegs.ADCSOCFRC1.bit.SOC0 = 1; // start ADC(software trigger)
    AdcbRegs.ADCSOCFRC1.bit.SOC0 = 1; // start ADC(software trigger)
    AdccRegs.ADCSOCFRC1.bit.SOC0 = 1; // start ADC(software trigger)
    return;
}
