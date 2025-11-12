// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file pwm.c
 *
 * @brief This module configures and enables the PWM Module 
 * 
 * Definitions in this file are for dsPIC33AK512MC510
 *
 * Component: PWM
 *
 */
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Disclaimer ">

/*******************************************************************************
* SOFTWARE LICENSE AGREEMENT
* 
* © [2024] Microchip Technology Inc. and its subsidiaries
* 
* Subject to your compliance with these terms, you may use this Microchip 
* software and any derivatives exclusively with Microchip products. 
* You are responsible for complying with third party license terms applicable to
* your use of third party software (including open source software) that may 
* accompany this Microchip software.
* 
* Redistribution of this Microchip software in source or binary form is allowed 
* and must include the above terms of use and the following disclaimer with the
* distribution and accompanying materials.
* 
* SOFTWARE IS "AS IS." NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
* APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,
* MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL 
* MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR 
* CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO
* THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE 
* POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY
* LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL
* NOT EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR THIS
* SOFTWARE
*
* You agree that you are solely responsible for testing the code and
* determining its suitability.  Microchip has no obligation to modify, test,
* certify, or support the code.
*
*******************************************************************************/
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <xc.h>
#include <stdint.h>

#include "pwm.h"
#include "delay.h"

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

/**
* <B> Function: InitPWMGenerators()    </B>
*
* @brief Function initializes the registers common for all PWM modules and 
* configures individual PWM module. The function enables the PWM module 
* and completes the boot strap charging sequence
*        
* @param none.
* @return none.
* 
* @example
* <CODE> InitPWMGenerators();     </CODE>
*
*/
void InitPWMGenerators(void)
{

    PCLKCON      = 0x0000;
    /* PWM Clock Divider Selection bits
       0b11 = 1:16 ; 0b10 = 1:8 ;0b01 = 1:4 ; 0b00 = 1:2*/
    PCLKCONbits.DIVSEL = 0;
    /* PWM Master Clock Selection bits
       0b11 = AFPLLO ; 0b10 = FPLLO ; 0b01 = AFVCO/2 ; 0b00 = FOSC */
    PCLKCONbits.MCLKSEL = 1;
    /* Lock bit: 0 = Write-protected registers and bits are unlocked   */
    PCLKCONbits.LOCK = 0;

    /* Initialize Master Phase Register */
    MPHASE       = 0x0000;
    /* Initialize Master Duty Cycle */
    MDC          = 0x0000;
    /* Initialize Master Period Register */
    MPER         = 0x0000;
    
    /* Initialize FREQUENCY SCALE REGISTER*/
    FSCL          = 0x0000;
    /* Initialize FREQUENCY SCALING MINIMUM PERIOD REGISTER */
    FSMINPER     = 0x0000;
    /* Initialize Linear Feedback Shift Register */
    LFSR         = 0x0000;
    /* Initialize Combinational Trigger Register */
    CMBTRIG     = 0x0000;
    /* Initialize LOGIC CONTROL REGISTER 1 */
    LOGCONA     = 0x0000;
    /* Initialize LOGIC CONTROL REGISTER 1 */
    LOGCONB     = 0x0000;
    /* Initialize LOGIC CONTROL REGISTER 2 */
    LOGCONC     = 0x0000;
    /* Initialize LOGIC CONTROL REGISTER 2 */
    LOGCOND     = 0x0000;
    /* Initialize LOGIC CONTROL REGISTER 3 */
    LOGCONE     = 0x0000;
    /* Initialize LOGIC CONTROL REGISTER 3 */
    LOGCONF     = 0x0000;
    /* PWM EVENT OUTPUT CONTROL REGISTER A */
    PWMEVTA     = 0x0000;     
    /* PWM EVENT OUTPUT CONTROL REGISTER B */
    PWMEVTB     = 0x0000;
    /* PWM EVENT OUTPUT CONTROL REGISTER C */
    PWMEVTC     = 0x0000;
    /* PWM EVENT OUTPUT CONTROL REGISTER D */
    PWMEVTD     = 0x0000;
    /* PWM EVENT OUTPUT CONTROL REGISTER E */
    PWMEVTE     = 0x0000;
    /* PWM EVENT OUTPUT CONTROL REGISTER F */
    PWMEVTF     = 0x0000;
    
    /* Generating PWM Event A from PG5 to trigger PG1,PG2 and PG3
       The raising edge of PG5 PWM output triggers the PG1,2,3 for MC1.
       The falling edge of PG5 PWM output triggers the APG1,2,3 for MC3
       The EOC of PG5 PWM generator triggers the PG6,7,8 for MC2 */
    /* PWM Event Output Enable bit  
       1 = Event output signal is output on PWMEy pin
       0 = Event output signal is internal only */
    PWMEVTAbits.EVTAOEN  = 0;
    /* EVTyPOL: PWM Event Output Polarity bit
       1 = Event output signal is active-low
       0 = Event output signal is active-high */
    PWMEVTAbits.EVTAPOL  = 0;
    /* EVTySTRD: PWM Event Output Stretch Disable bit
       1 = Event output signal pulse width is not stretched
       0 = Event output signal is stretched to eight PWM clock cycles minimum */
    PWMEVTAbits.EVTASTRD = 1; 
    /* EVTySYNC: PWM Event Output Sync bit
       1 = Event output signal is synchronized to the system clock
       0 = Event output is not synchronized to the system clock
       Event output signal pulse will be two system clocks when this bit is set and EVTySTRD = 1 */
    PWMEVTAbits.EVTASYNC = 1;
    /* EVTASEL: PWM Event Selection
       00001 = PWM Generator output signal*/
    PWMEVTAbits.EVTASEL  = 0b00001;
    /* EVTyPGS[2:0]: PWM Event Source Selection bits
       111 = PWM Generator 8
       100 = PWM Generator 5
       ...
       000 = PWM Generator 1 */
    PWMEVTAbits.EVTAPGS  = 4;

    /* Function call to Initialize individual PWM modules*/
    InitPWMGenerator5 ();
    
    InitPWMGenerator1 ();
    InitPWMGenerator2 ();
    InitPWMGenerator3 ();
    
    InitPWMGenerator6 ();
    InitPWMGenerator7 ();
    InitPWMGenerator8 ();
    
    InitAuxPWMGenerator1 ();
    InitAuxPWMGenerator2 ();
    InitAuxPWMGenerator3 ();
    
    InitDutyPWM123Generators();
    InitDutyPWM678Generators();
    InitDutyAPWM123Generators();

    /* Clearing and disabling PWM Interrupt */
    _PWM1IF = 0;
    _PWM1IE = 0;
    _PWM1IP = 7;
    
    _PWM6IF = 0;
    _PWM6IE = 0;
    _PWM6IP = 7;
    
    _APWM1IF = 0;
    _APWM1IE = 0;
    _APWM1IP = 7;
    
    /* Enable the PWM modules after initializing generators*/
	PG1CONbits.ON = 1;    
    PG2CONbits.ON = 1;      
    PG3CONbits.ON = 1;
    
    PG6CONbits.ON = 1;      
    PG7CONbits.ON = 1;      
    PG8CONbits.ON = 1; 
    
    APG1CONbits.ON = 1; 
    APG2CONbits.ON = 1; 
    APG3CONbits.ON = 1;

    PG5CONbits.ON = 1;

}

/**
* <B> Function: InitDutyPWM123Generators()    </B>
*
* @brief Function initialize the duty to zero by overriding and 
* reset the duty registers for Boot strap charging
*        
* @param none.
* @return none.
* 
* @example
* <CODE> InitDutyPWM123Generators();     </CODE>
*
*/
void InitDutyPWM123Generators(void)
{
    
    /** Set Override Data on all PWM outputs */
    /* 0b00 = State for PWM3H,L, if Override is Enabled */
    PG3IOCON2bits.OVRDAT = 0;
    /* 0b00 = State for PWM2H,L, if Override is Enabled */
    PG2IOCON2bits.OVRDAT = 0; 
    /* 0b00 = State for PWM1H,L, if Override is Enabled */
    PG1IOCON2bits.OVRDAT = 0; 

    /* 1 = OVRDAT<1> provides data for output on PWM3H */
    PG3IOCON2bits.OVRENH = 1; 
    /* 1 = OVRDAT<0> provides data for output on PWM3L */
    PG3IOCON2bits.OVRENL = 1; 
    
    /* 1 = OVRDAT<1> provides data for output on PWM2H */
    PG2IOCON2bits.OVRENH = 1;
    /* 1 = OVRDAT<0> provides data for output on PWM2L */
    PG2IOCON2bits.OVRENL = 1; 
    
    /* 1 = OVRDAT<1> provides data for output on PWM1H */
    PG1IOCON2bits.OVRENH = 1;  
    /* 1 = OVRDAT<0> provides data for output on PWM1L */
    PG1IOCON2bits.OVRENL = 1;  

    /* Set all PWM Duty to zero */
    MC1_PWM_PDC3 = 0;
    MC1_PWM_PDC2 = 0;      
    MC1_PWM_PDC1 = 0;

}
/**
* <B> Function: InitDutyPWM678Generators()    </B>
*
* @brief Function initialize the duty to zero by overriding and 
* reset the duty registers for Boot strap charging
*        
* @param none.
* @return none.
* 
* @example
* <CODE> InitDutyPWM678Generators();     </CODE>
*
*/
void InitDutyPWM678Generators(void)
{
    
    /** Set Override Data on all PWM outputs */
    /* 0b00 = State for PWM8H,L, if Override is Enabled */
    PG8IOCON2bits.OVRDAT = 0;
    /* 0b00 = State for PWM7H,L, if Override is Enabled */
    PG7IOCON2bits.OVRDAT = 0; 
    /* 0b00 = State for PWM6H,L, if Override is Enabled */
    PG6IOCON2bits.OVRDAT = 0; 

    /* 1 = OVRDAT<1> provides data for output on PWM8H */
    PG8IOCON2bits.OVRENH = 1; 
    /* 1 = OVRDAT<0> provides data for output on PWM8L */
    PG8IOCON2bits.OVRENL = 1; 
    
    /* 1 = OVRDAT<1> provides data for output on PWM7H */
    PG7IOCON2bits.OVRENH = 1;
    /* 1 = OVRDAT<0> provides data for output on PWM7L */
    PG7IOCON2bits.OVRENL = 1; 
    
    /* 1 = OVRDAT<1> provides data for output on PWM6H */
    PG6IOCON2bits.OVRENH = 1;  
    /* 1 = OVRDAT<0> provides data for output on PWM6L */
    PG6IOCON2bits.OVRENL = 1;  

    /* Set all PWM Duty to zero */
    MC2_PWM_PDC3 = 0;
    MC2_PWM_PDC2 = 0;      
    MC2_PWM_PDC1 = 0;

}
/**
* <B> Function: InitDutyAPWM123Generators()    </B>
*
* @brief Function initialize the duty to zero by overriding and 
* reset the duty registers for Boot strap charging
*        
* @param none.
* @return none.
* 
* @example
* <CODE> InitDutyAPWM123Generators();     </CODE>
*
*/
void InitDutyAPWM123Generators(void)
{
    
    /** Set Override Data on all PWM outputs */
    /* 0b00 = State for APWM3H,L, if Override is Enabled */
    APG3IOCON2bits.OVRDAT = 0;
    /* 0b00 = State for APWM2H,L, if Override is Enabled */
    APG2IOCON2bits.OVRDAT = 0; 
    /* 0b00 = State for APWM1H,L, if Override is Enabled */
    APG1IOCON2bits.OVRDAT = 0; 

    /* 1 = OVRDAT<1> provides data for output on APWM3H */
    APG3IOCON2bits.OVRENH = 1; 
    /* 1 = OVRDAT<0> provides data for output on APWM3L */
    APG3IOCON2bits.OVRENL = 1; 
    
    /* 1 = OVRDAT<1> provides data for output on APWM2H */
    APG2IOCON2bits.OVRENH = 1;
    /* 1 = OVRDAT<0> provides data for output on APWM2L */
    APG2IOCON2bits.OVRENL = 1; 
    
    /* 1 = OVRDAT<1> provides data for output on APWM1H */
    APG1IOCON2bits.OVRENH = 1;  
    /* 1 = OVRDAT<0> provides data for output on APWM1L */
    APG1IOCON2bits.OVRENL = 1;  

    /* Set all PWM Duty to zero */
    MC3_PWM_PDC3 = 0;
    MC3_PWM_PDC2 = 0;      
    MC3_PWM_PDC1 = 0;

}

/**
* <B> Function: InitPWMGenerator1()    </B>
*
* @brief Function to configure PWM Module # 1
*        
* @param none.
* @return none.
* 
* @example
* <CODE> InitPWMGenerator1();     </CODE>
*
*/
void InitPWMGenerator1 (void)
{

    /* Initialize PWM GENERATOR 1 CONTROL REGISTER */
    PG1CON      = 0x0000;
    /* PWM Generator 1 Enable bit : 1 = Is enabled, 0 = Is not enabled */
    /* Ensuring PWM Generator is disabled prior to configuring module */
    PG1CONbits.ON = 0;
    /* Clock Selection bits
       0b01 = Macro uses Master clock selected by the PCLKCON.MCLKSEL bits*/
    PG1CONbits.CLKSEL = 1;
    /* PWM Mode Selection bits
     * 110 = Dual Edge Center-Aligned PWM mode (interrupt/register update once per cycle)
       100 = Center-Aligned PWM mode(interrupt/register update once per cycle)*/
    PG1CONbits.MODSEL = 4;
    /* Trigger Count Select bits
       000 = PWM Generator produces 1 PWM cycle after triggered */
    PG1CONbits.TRGCNT = 0;
    
    /* Initialize PWM GENERATOR 1 CONTROL REGISTER */
    /* Master Duty Cycle Register Select bit
       1 = Macro uses the MDC register instead of PG1DC
       0 = Macro uses the PG1DC register*/
    PG1CONbits.MDCSEL = 0;
    /* Master Period Register Select bit
       1 = Macro uses the MPER register instead of PG1PER
       0 = Macro uses the PG1PER register */
    PG1CONbits.MPERSEL = 0;
    /* MPHSEL: Master Phase Register Select bit
       1 = Macro uses the MPHASE register instead of PG1PHASE
       0 = Macro uses the PG1PHASE register */
    PG1CONbits.MPHSEL = 0;
    /* Master Update Enable bit
       1 = PWM Generator broadcasts software set/clear of UPDATE status bit and 
           EOC signal to other PWM Generators
       0 = PWM Generator does not broadcast UPDATE status bit or EOC signal */
    PG1CONbits.MSTEN = 0;
    /* PWM Buffer Update Mode Selection bits 
       Update Data registers at start of next PWM cycle if UPDATE = 1. */
    PG1CONbits.UPDMOD = 0;
    /* PWM Generator Trigger Mode Selection bits
       0b00 = PWM Generator operates in Single Trigger mode
       0b01 = PWM Generator operates in Retriggerable mode */
    PG1CONbits.TRGMOD = 1;
    /* Start of Cycle Selection bits
       1111 = TRIG bit or PCI Sync function only (no hardware trigger source is selected)
       0000 = Local EOC*/
    PG1CONbits.SOCS = 0b1111;
    
    /* Clear PWM GENERATOR 1 STATUS REGISTER*/
    PG1STAT      = 0x0000;
    /* Initialize PWM GENERATOR 1 I/O CONTROL REGISTER */
    PG1IOCON2    = 0x0000;

    /* Current Limit Mode Select bit
       0 = If PCI current limit is active, then the CLDAT<1:0> bits define 
       the PWM output levels */
    PG1IOCON2bits.CLMOD = 0;
    /* Swap PWM Signals to PWM1H and PWM1L Device Pins bit 
       0 = PWM1H/L signals are mapped to their respective pins */
    PG1IOCON1bits.SWAP = 0;
    /* User Override Enable for PWM1H Pin bit
       0 = PWM Generator provides data for the PWM1H pin*/
    PG1IOCON2bits.OVRENH = 0;
    /* User Override Enable for PWM1L Pin bit
       0 = PWM Generator provides data for the PWM1L pin*/
    PG1IOCON2bits.OVRENL = 0;
    /* Data for PWM1H/PWM1L Pins if Override is Enabled bits
       If OVERENH = 1, then OVRDAT<1> provides data for PWM1H.
       If OVERENL = 1, then OVRDAT<0> provides data for PWM1L */
    PG1IOCON2bits.OVRDAT = 0;
    /* User Output Override Synchronization Control bits
       00 = User output overrides via the OVRENL/H and OVRDAT<1:0> bits are 
       synchronized to the local PWM time base (next start of cycle)*/
    PG1IOCON2bits.OSYNC = 0;
    /* Data for PWM1H/PWM1L Pins if FLT Event is Active bits
       If Fault is active, then FLTDAT<1> provides data for PWM1H.
       If Fault is active, then FLTDAT<0> provides data for PWM1L.*/
    PG1IOCON2bits.FLT1DAT = 0;
    /* Data for PWM1H/PWM1L Pins if CLMT Event is Active bits
       If current limit is active, then CLDAT<1> provides data for PWM1H.
       If current limit is active, then CLDAT<0> provides data for PWM1L.*/
    PG1IOCON2bits.CLDAT = 0;
    /* Data for PWM1H/PWM1L Pins if Feed-Forward Event is Active bits
       If feed-forward is active, then FFDAT<1> provides data for PWM1H.
       If feed-forward is active, then FFDAT<0> provides data for PWM1L.*/
    PG1IOCON2bits.FFDAT = 0;
    /* Data for PWM1H/PWM1L Pins if Debug Mode is Active and PTFRZ = 1 bits
       If Debug mode is active and PTFRZ=1,then DBDAT<1> provides PWM1H data.
       If Debug mode is active and PTFRZ=1,then DBDAT<0> provides PWM1L data. */
    PG1IOCON2bits.DBDAT = 0;
    
    /* Initialize PWM GENERATOR 1 I/O CONTROL REGISTER */    

    /* Time Base Capture Source Selection bits
       000 = No hardware source selected for time base capture ? software only*/
    PG1IOCON1bits.CAPSRC = 0;
    /* Dead-Time Compensation Select bit 
       0 = Dead-time compensation is controlled by PCI Sync logic */
    PG1IOCON1bits.DTCMPSEL = 0;
    /* PWM Generator Output Mode Selection bits
       00 = PWM Generator outputs operate in Complementary mode*/
    PG1IOCON1bits.PMOD = 0;
    /* PWM1H Output Port Enable bit
       1 = PWM Generator controls the PWM1H output pin
       0 = PWM Generator does not control the PWM1H output pin */
    PG1IOCON1bits.PENH = 1;
    /* PWM1L Output Port Enable bit
       1 = PWM Generator controls the PWM1L output pin
       0 = PWM Generator does not control the PWM1L output pin */
    PG1IOCON1bits.PENL = 1;
    /* PWM1H Output Polarity bit
       1 = Output pin is active-low
       0 = Output pin is active-high*/
    PG1IOCON1bits.POLH = 0;
    /* PWM1L Output Polarity bit
       1 = Output pin is active-low
       0 = Output pin is active-high*/
    PG1IOCON1bits.POLL = 0;
    /* Peripheral Pin Select Enable bit
       1 = Peripheral pin select enabled
       0 = Peripheral pin select disabled, as a result, PWM outputs are hard-mapped to pins*/
    PG1IOCON1bits.PPSEN = 0;
    
    /* Initialize PWM GENERATOR 1 EVENT REGISTER */
    PG1EVT1      = 0x0000;
    /* ADC Trigger 1 Post-scaler Selection bits
       00000 = 1:1 */
    PG1EVT1bits.ADTR1PS = 0;
    /* ADC Trigger 1 Source is PG1TRIGC Compare Event Enable bit
       0 = PG1TRIGC register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    PG1EVT1bits.ADTR1EN3  = 0;
    /* ADC Trigger 1 Source is PG1TRIGB Compare Event Enable bit
       0 = PG1TRIGB register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    PG1EVT1bits.ADTR1EN2 = 0;    
    /* ADC Trigger 1 Source is PG1TRIGA Compare Event Enable bit
       1 = PG1TRIGA register compare event is enabled as trigger source for 
           ADC Trigger 1 */
    PG1EVT1bits.ADTR1EN1 = 1;
    /* Update Trigger Select bits
       01 = A write of the PG1DC register automatically sets the UPDATE bit*/
    PG1EVT1bits.UPDTRG = 1;
    /* PWM Generator Trigger Output Selection bits
       000 = EOC event is the PWM Generator trigger*/
    PG1EVT1bits.PGTRGSEL = 0;
    
    /* Initialize PWM GENERATOR 1 EVENT REGISTER */
    /* FLTIEN: PCI Fault Interrupt Enable bit
       1 = Fault interrupt is enabled */
    PG1EVT1bits.FLT1IEN = 1;
    /* PCI Current Limit Interrupt Enable bit
       0 = Current limit interrupt is disabled */
    PG1EVT1bits.CLIEN = 0;
    /* PCI Feed-Forward Interrupt Enable bit
       0 = Feed-forward interrupt is disabled */
    PG1EVT1bits.FFIEN = 0;
    /* PCI Sync Interrupt Enable bit
       0 = Sync interrupt is disabled */
    PG1EVT1bits.SIEN = 0;
    /* Interrupt Event Selection bits
       00 = Interrupts CPU at EOC
       01 = Interrupts CPU at TRIGA compare event
       10 = Interrupts CPU at ADC Trigger 1 event
       11 = Time base interrupts are disabled */
    PG1EVT1bits.IEVTSEL = 3;

        /* ADC Trigger 2 Source is PG1TRIGC Compare Event Enable bit
       0 = PG1TRIGC register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    PG1EVT2bits.ADTR2EN3 = 0;
    /* ADC Trigger 2 Source is PG1TRIGB Compare Event Enable bit
       0 = PG1TRIGB register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    PG1EVT2bits.ADTR2EN2 = 0;
    /* ADC Trigger 2 Source is PG1TRIGA Compare Event Enable bit
       0 = PG1TRIGA register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    PG1EVT2bits.ADTR2EN1 = 0;
    /* ADC Trigger 1 Offset Selection bits
       00000 = No offset */
    PG1EVT1bits.ADTR1OFS = 0;
    
#ifndef ENABLE_PWM_FAULT_PCI_MC1
    /* PWM GENERATOR 1 Fault PCI REGISTER */
    PG1F1PCI1     = 0x0000;
#else
       /* PWM GENERATOR 1 Fault PCI REGISTER */
    PG1F1PCI1     = 0x0000;
    /* Termination Synchronization Disable bit
       1 = Termination of latched PCI occurs immediately
       0 = Termination of latched PCI occurs at PWM EOC */
    PG1F1PCI1bits.TSYNCDIS = 0;
    /* Termination Event Selection bits
       001 = Auto-Terminate: Terminate when PCI source transitions from 
             active to inactive */
    PG1F1PCI1bits.TERM = 1;
    /* Acceptance Qualifier Polarity Select bit: 0 = Not inverted 1 = Inverted*/
    PG1F1PCI1bits.AQPS = 0;
    /* Acceptance Qualifier Source Selection bits
       111 = SWPCI control bit only (qualifier forced to 0)
       110 = Selects PCI Source #9
       101 = Selects PCI Source #8
       100 = Selects PCI Source #1 (PWM Generator output selected by the PWMPCI<2:0> bits)
       011 = PWM Generator is triggered
       010 = LEB is active
       001 = Duty cycle is active (base PWM Generator signal)        
       000 = No acceptance qualifier is used (qualifier forced to 1) */
    PG1F1PCI1bits.AQSS = 0;
    /* PCI Synchronization Control bit
       1 = PCI source is synchronized to PWM EOC
       0 = PCI source is not synchronized to PWM EOC*/
    PG1F1PCI1bits.PSYNC = 0;
    /* PCI Polarity Select bit 0 = Not inverted 1 = Inverted */
    PG1F1PCI1bits.PPS = 1;
    /* PCI Source Selection bits
       31 : CLC1 output
       30 : Comparator 3 output(0x40000000)
       ? ?
       8 : PPS PCI8R in RPINR20 register input(0x0100)
       ? ?
       1 : Internally connected to the output of PWMPCI[2:0] bits in PGxEVT1 registers
       0 : Tied to '0' */
    PG1F1PCI2 = 0x0100;
    /* PCI Bypass Enable bit
       0 = PCI function is not bypassed */
    PG1F1PCI1bits.BPEN   = 0;
    /* PCI Bypass Source Selection bits(1)
       000 = PCI control is sourced from PG1 PCI logic when BPEN = 1 */
    PG1F1PCI1bits.BPSEL   = 0;
    /* PCI Termination Polarity Select bits 
       0 = Inverter, 1 = Non Inverted */
    PG1F1PCI1bits.TERMPS = 0;
    /* PCI Acceptance Criteria Selection bits
       101 = Latched any edge(2)
       100 = Latched rising edge
       011 = Latched
       010 = Any edge
       001 = Rising edge
       000 = Level-sensitive*/
    PG1F1PCI1bits.ACP   = 3;
    /* PCI SR Latch Mode bit
       1 = SR latch is Reset-dominant in Latched Acceptance modes
       0 = SR latch is Set-dominant in Latched Acceptance modes*/
    PG1F1PCI1bits.LATMOD  = 0;
    /* Termination Qualifier Polarity Select bit 1 = Inverted 0 = Not inverted*/
    PG1F1PCI1bits.TQPS   = 0;
    /* Termination Qualifier Source Selection bits
       111 = SWPCI control bit only (qualifier forced to '0')(3)
       110 = Selects PCI Source #9 (pwm_pci[9] input port)
       101 = Selects PCI Source #8 (pwm_pci[8] input port)
       100 = Selects PCI Source #1 (PWM Generator output selected by the PWMPCI<2:0> bits)
       011 = PWM Generator is triggered
       010 = LEB is active
       001 = Duty cycle is active (base PWM Generator signal)
       000 = No termination qualifier used (qualifier forced to '1')(3)*/
    PG1F1PCI1bits.TQSS  = 0;
#endif    

    /* PWM GENERATOR 1 Current Limit PCI REGISTER */
    PG1CLPCI1    = 0x0000;
    /* PWM GENERATOR 1 Feed Forward PCI REGISTER  */
    PG1FFPCI1    = 0x0000;
    
    /* For PWM Synchronization
     PWM Gen 1 is triggered by the Event A of PWM Gen 5*/
    /* PWM GENERATOR 1 Sync PCI REGISTER */
    PG1SPCI1    = 0x0000;
    /* PCI Polarity Select bit 
       1 = Inverted
       0 = Not inverted*/
    PG1SPCI1bits.PPS = 0;
    /* PWM PCI Source Selection
       100 = PWM Generator #5 output used as PCI signal */
    PG1EVT1bits.PWMPCI = 0b100;
    /* PWM GENERATOR 1 Sync PCI Source 
       23 = PWM event A*/
    PG1SPCI2 = 0x00800000;
    
    /* Initialize PWM GENERATOR 1 LEADING-EDGE BLANKING REGISTER */
    PG1LEB      = 0x0000;
    
    /* Initialize PWM GENERATOR 1 PHASE REGISTER */
    PG1PHASEbits.PHASE     = MC1_MIN_DUTY;
    /* Initialize PWM GENERATOR 1 DUTY CYCLE REGISTER */
    PG1DCbits.DC           = MC1_MIN_DUTY;
    /* Initialize PWM GENERATOR 1 DUTY CYCLE ADJUSTMENT REGISTER */
    PG1DCA       = 0x0000;
    /* Initialize PWM GENERATOR 1 PERIOD REGISTER */
    PG1PER       = MC1_LOOPTIME_TCY;
    /* Initialize PWM GENERATOR 1 DEAD-TIME REGISTER */
    PG1DTbits.DTH = MC1_DEADTIME;
    /* Initialize PWM GENERATOR 1 DEAD-TIME REGISTER */
    PG1DTbits.DTL = MC1_DEADTIME;

    /* Initialize PWM GENERATOR 1 TRIGGER A REGISTER */
    PG1TRIGAbits.CAHALF = 0;
    PG1TRIGAbits.TRIGA  = MC1_ADC_SAMPLING_POINT;
    /* Initialize PWM GENERATOR 1 TRIGGER B REGISTER */
    PG1TRIGB     = 0x0000;
    /* Initialize PWM GENERATOR 1 TRIGGER C REGISTER */
    PG1TRIGC     = 0x0000;
    
} 

/**
* <B> Function: InitPWMGenerator2()    </B>
*
* @brief Function to configure PWM Module # 2
*        
* @param none.
* @return none.
* 
* @example
* <CODE> InitPWMGenerator2();     </CODE>
*
*/
void InitPWMGenerator2 (void)
{

    /* Initialize PWM GENERATOR 2 CONTROL REGISTER */
    PG2CON      = 0x0000;
    /* PWM Generator 2 Enable bit : 1 = Is enabled, 0 = Is not enabled */
    /* PWM Generator is disabled prior to configuring module */
    PG2CONbits.ON = 0;
    /* Clock Selection bits
       0b01 = Macro uses Master clock selected by the PCLKCON.MCLKSEL bits*/
    PG2CONbits.CLKSEL = 1;
    /* PWM Mode Selection bits
     * 110 = Dual Edge Center-Aligned PWM mode (interrupt/register update once per cycle)
       100 = Center-Aligned PWM mode(interrupt/register update once per cycle)*/
    PG2CONbits.MODSEL = 4;
    /* Trigger Count Select bits
       000 = PWM Generator produces 1 PWM cycle after triggered */
    PG2CONbits.TRGCNT = 0;
    
    /* Initialize PWM GENERATOR 2 CONTROL REGISTER */
    /* Master Duty Cycle Register Select bit
       1 = Macro uses the MDC register instead of PG2DC
       0 = Macro uses the PG2DC register*/
    PG2CONbits.MDCSEL = 0;
    /* Master Period Register Select bit
       1 = Macro uses the MPER register instead of PG2PER
       0 = Macro uses the PG2PER register */
    PG2CONbits.MPERSEL = 0;
    /* MPHSEL: Master Phase Register Select bit
       1 = Macro uses the MPHASE register instead of PG2PHASE
       0 = Macro uses the PG2PHASE register */
    PG2CONbits.MPHSEL = 0;
    /* Master Update Enable bit
       1 = PWM Generator broadcasts software set/clear of UPDATE status bit and 
           EOC signal to other PWM Generators
       0 = PWM Generator does not broadcast UPDATE status bit or EOC signal */
    PG2CONbits.MSTEN = 0;
     /* PWM Buffer Update Mode Selection bits 
       0b010 = Slaved SOC Update Data registers at start of next cycle if a 
       master update request is received. A master update request will be 
       transmitted if MSTEN = 1 and UPDATE = 1 for the requesting PWM
       Generator.. */
	PG2CONbits.UPDMOD = 0;
    /* PWM Generator Trigger Mode Selection bits
       0b00 = PWM Generator operates in Single Trigger mode
       0b01 = PWM Generator operates in Retriggerable mode */
    PG2CONbits.TRGMOD = 1;
    /* Start of Cycle Selection bits
       1111 = TRIG bit or PCI Sync function only (no hardware trigger source is selected)
       0000 = Local EOC*/
    PG2CONbits.SOCS = 0b1111;
    
    /* Clear PWM GENERATOR 2 STATUS REGISTER*/
    PG2STAT      = 0x0000;
    /* Initialize PWM GENERATOR 2 I/O CONTROL REGISTER */
    PG2IOCON2    = 0x0000;

    /* Current Limit Mode Select bit
       0 = If PCI current limit is active, then the CLDAT<1:0> bits define 
       the PWM output levels */
    PG2IOCON2bits.CLMOD = 0;
    /* Swap PWM Signals to PWM2H and PWM2L Device Pins bit 
       0 = PWM2H/L signals are mapped to their respective pins */
    PG2IOCON1bits.SWAP = 0;
    /* User Override Enable for PWM2H Pin bit
       0 = PWM Generator provides data for the PWM2H pin*/
    PG2IOCON2bits.OVRENH = 0;
    /* User Override Enable for PWM2L Pin bit
       0 = PWM Generator provides data for the PWM2L pin*/
    PG2IOCON2bits.OVRENL = 0;
    /* Data for PWM2H/PWM2L Pins if Override is Enabled bits
       If OVERENH = 1, then OVRDAT<1> provides data for PWM2H.
       If OVERENL = 1, then OVRDAT<0> provides data for PWM2L */
    PG2IOCON2bits.OVRDAT = 0;
    /* User Output Override Synchronization Control bits
       00 = User output overrides via the OVRENL/H and OVRDAT<1:0> bits are 
       synchronized to the local PWM time base (next start of cycle)*/
    PG2IOCON2bits.OSYNC = 0;
    /* Data for PWM2H/PWM2L Pins if FLT Event is Active bits
       If Fault is active, then FLTDAT<1> provides data for PWM2H.
       If Fault is active, then FLTDAT<0> provides data for PWM2L.*/
    PG2IOCON2bits.FLT1DAT = 0;
    /* Data for PWM2H/PWM2L Pins if CLMT Event is Active bits
       If current limit is active, then CLDAT<1> provides data for PWM2H.
       If current limit is active, then CLDAT<0> provides data for PWM2L.*/
    PG2IOCON2bits.CLDAT = 0;
    /* Data for PWM2H/PWM2L Pins if Feed-Forward Event is Active bits
       If feed-forward is active, then FFDAT<1> provides data for PWM2H.
       If feed-forward is active, then FFDAT<0> provides data for PWM2L.*/
    PG2IOCON2bits.FFDAT = 0;
    /* Data for PWM2H/PWM2L Pins if Debug Mode is Active and PTFRZ = 1 bits
       If Debug mode is active and PTFRZ=1,then DBDAT<1> provides PWM2H data.
       If Debug mode is active and PTFRZ=1,then DBDAT<0> provides PWM2L data. */
    PG2IOCON2bits.DBDAT = 0;
    
    /* Initialize PWM GENERATOR 2 I/O CONTROL REGISTER */    
    /* Time Base Capture Source Selection bits
       000 = No hardware source selected for time base capture ? software only*/
    PG2IOCON1bits.CAPSRC = 0;
    /* Dead-Time Compensation Select bit 
       0 = Dead-time compensation is controlled by PCI Sync logic */
    PG2IOCON1bits.DTCMPSEL = 0;
    /* PWM Generator Output Mode Selection bits
       00 = PWM Generator outputs operate in Complementary mode*/
    PG2IOCON1bits.PMOD = 0;
    /* PWM2H Output Port Enable bit
       1 = PWM Generator controls the PWM2H output pin
       0 = PWM Generator does not control the PWM2H output pin */
    PG2IOCON1bits.PENH = 1;
    /* PWM2L Output Port Enable bit
       1 = PWM Generator controls the PWM2L output pin
       0 = PWM Generator does not control the PWM2L output pin */
    PG2IOCON1bits.PENL = 1;
    /* PWM2H Output Polarity bit
       1 = Output pin is active-low
       0 = Output pin is active-high*/
    PG2IOCON1bits.POLH = 0;
    /* PWM2L Output Polarity bit
       1 = Output pin is active-low
       0 = Output pin is active-high*/
    PG2IOCON1bits.POLL = 0;
    /* Peripheral Pin Select Enable bit
       1 = Peripheral pin select enabled
       0 = Peripheral pin select disabled, as a result, PWM outputs are hard-mapped to pins*/
    PG2IOCON1bits.PPSEN = 0;
    
    /* Initialize PWM GENERATOR 2 EVENT REGISTER */
    PG2EVT1      = 0x0000;
    /* ADC Trigger 1 Post-scaler Selection bits
       00000 = 1:1 */
    PG2EVT1bits.ADTR1PS = 0;
    /* ADC Trigger 1 Source is PG2TRIGC Compare Event Enable bit
       0 = PG2TRIGC register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    PG2EVT1bits.ADTR1EN3  = 0;
    /* ADC Trigger 1 Source is PG2TRIGB Compare Event Enable bit
       0 = PG2TRIGB register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    PG2EVT1bits.ADTR1EN2 = 0;
    /* ADC Trigger 1 Source is PG2TRIGA Compare Event Enable bit
       0 = PG2TRIGA register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    PG2EVT1bits.ADTR1EN1 = 0;
    /* Update Trigger Select bits
       01 = A write of the PG2DC register automatically sets the UPDATE bit
       00 = User must set the UPDATE bit manually*/
    PG2EVT1bits.UPDTRG = 1;
    /* PWM Generator Trigger Output Selection bits
       000 = EOC event is the PWM Generator trigger*/
    PG2EVT1bits.PGTRGSEL = 0;
    
    /* Initialize PWM GENERATOR 2 EVENT REGISTER */
    /* FLTIEN: PCI Fault Interrupt Enable bit
       0 = Fault interrupt is disabled */
    PG2EVT1bits.FLT1IEN = 0;
    /* PCI Current Limit Interrupt Enable bit
       0 = Current limit interrupt is disabled */
    PG2EVT1bits.CLIEN = 0;
    /* PCI Feed-Forward Interrupt Enable bit
       0 = Feed-forward interrupt is disabled */
    PG2EVT1bits.FFIEN = 0;
    /* PCI Sync Interrupt Enable bit
       0 = Sync interrupt is disabled */
    PG2EVT1bits.SIEN = 0;
    /* Interrupt Event Selection bits
       00 = Interrupts CPU at EOC
       01 = Interrupts CPU at TRIGA compare event
       10 = Interrupts CPU at ADC Trigger 1 event
       11 = Time base interrupts are disabled */
    PG2EVT1bits.IEVTSEL = 3;
    /* ADC Trigger 2 Source is PG2TRIGC Compare Event Enable bit
       0 = PG2TRIGC register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    PG2EVT2bits.ADTR2EN3 = 0;
    /* ADC Trigger 2 Source is PG2TRIGB Compare Event Enable bit
       0 = PG2TRIGB register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    PG2EVT2bits.ADTR2EN2 = 0;
    /* ADC Trigger 2 Source is PG2TRIGA Compare Event Enable bit
       0 = PG2TRIGA register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    PG2EVT2bits.ADTR2EN1 = 0;
    /* ADC Trigger 1 Offset Selection bits
       00000 = No offset */
    PG2EVT1bits.ADTR1OFS = 0;
    
#ifndef ENABLE_PWM_FAULT_PCI_MC1
    /* PWM GENERATOR 2 Fault PCI REGISTER */
    PG2F1PCI1     = 0x0000;
#else
       /* PWM GENERATOR 2 Fault PCI REGISTER */
    PG2F1PCI1     = 0x0000;
    /* Termination Synchronization Disable bit
       1 = Termination of latched PCI occurs immediately
       0 = Termination of latched PCI occurs at PWM EOC */
    PG2F1PCI1bits.TSYNCDIS = 0;
    /* Termination Event Selection bits
       001 = Auto-Terminate: Terminate when PCI source transitions from 
             active to inactive */
    PG2F1PCI1bits.TERM = 1;
    /* Acceptance Qualifier Polarity Select bit: 0 = Not inverted 1 = Inverted*/
    PG2F1PCI1bits.AQPS = 0;
    /* Acceptance Qualifier Source Selection bits
       111 = SWPCI control bit only (qualifier forced to 0)
       110 = Selects PCI Source #9
       101 = Selects PCI Source #8
       100 = Selects PCI Source #1 (PWM Generator output selected by the PWMPCI<2:0> bits)
       011 = PWM Generator is triggered
       010 = LEB is active
       001 = Duty cycle is active (base PWM Generator signal)        
       000 = No acceptance qualifier is used (qualifier forced to 1) */
    PG2F1PCI1bits.AQSS = 0;
    /* PCI Synchronization Control bit
       1 = PCI source is synchronized to PWM EOC
       0 = PCI source is not synchronized to PWM EOC*/
    PG2F1PCI1bits.PSYNC = 0;
    /* PCI Polarity Select bit 0 = Not inverted 1 = Inverted */
    PG2F1PCI1bits.PPS = 1;
    /* PCI Source Selection bits
       31 : CLC1 output
       30 : Comparator 3 output(0x40000000)
       ? ?
       8 : PPS PCI8R in RPINR20 register input(0x0100)
       ? ?
       1 : Internally connected to the output of PWMPCI[2:0] bits in PGxEVT1 registers
       0 : Tied to '0' */
    PG2F1PCI2 = 0x0100;
    /* PCI Bypass Enable bit
       0 = PCI function is not bypassed */
    PG2F1PCI1bits.BPEN   = 0;
    /* PCI Bypass Source Selection bits(1)
       000 = PCI control is sourced from PG1 PCI logic when BPEN = 1 */
    PG2F1PCI1bits.BPSEL   = 0;
    /* PCI Termination Polarity Select bits 
       0 = Inverter, 1 = Non Inverted */
    PG2F1PCI1bits.TERMPS = 0;
    /* PCI Acceptance Criteria Selection bits
       101 = Latched any edge(2)
       100 = Latched rising edge
       011 = Latched
       010 = Any edge
       001 = Rising edge
       000 = Level-sensitive*/
    PG2F1PCI1bits.ACP   = 3;
    /* PCI SR Latch Mode bit
       1 = SR latch is Reset-dominant in Latched Acceptance modes
       0 = SR latch is Set-dominant in Latched Acceptance modes*/
    PG2F1PCI1bits.LATMOD  = 0;
    /* Termination Qualifier Polarity Select bit 1 = Inverted 0 = Not inverted*/
    PG2F1PCI1bits.TQPS   = 0;
    /* Termination Qualifier Source Selection bits
       111 = SWPCI control bit only (qualifier forced to '0')(3)
       110 = Selects PCI Source #9 (pwm_pci[9] input port)
       101 = Selects PCI Source #8 (pwm_pci[8] input port)
       100 = Selects PCI Source #1 (PWM Generator output selected by the PWMPCI<2:0> bits)
       011 = PWM Generator is triggered
       010 = LEB is active
       001 = Duty cycle is active (base PWM Generator signal)
       000 = No termination qualifier used (qualifier forced to '1')(3)*/
    PG2F1PCI1bits.TQSS  = 0;
#endif  
    
    /* PWM GENERATOR 2 Current Limit PCI REGISTER */
    PG2CLPCI1    = 0x0000;
    /* PWM GENERATOR 2 Feed Forward PCI REGISTER */
    PG2FFPCI1    = 0x0000;
    
    /* For PWM Synchronization
     PWM Gen 2 is triggered by the Event A of PWM Gen 5*/
    /* PWM GENERATOR 2 Sync PCI REGISTER */
    PG2SPCI1    = 0x0000;
    /* PCI Polarity Select bit 
       1 = Inverted
       0 = Not inverted*/
    PG2SPCI1bits.PPS = 0;
    /* PWM PCI Source Selection
       100 = PWM Generator #5 output used as PCI signal */
    PG2EVT1bits.PWMPCI = 0b100;
    /* PWM GENERATOR 2 Sync PCI Source 
       23 = PWM event A*/
    PG2SPCI2 = 0x00800000;
    
    /* Initialize PWM GENERATOR 2 LEADING-EDGE BLANKING REGISTER */
    PG2LEB      = 0x0000;
    
    /* Initialize PWM GENERATOR 2 PHASE REGISTER */
    PG2PHASEbits.PHASE     = MC1_MIN_DUTY;
    /* Initialize PWM GENERATOR 2 DUTY CYCLE REGISTER */
    PG2DCbits.DC           = MC1_MIN_DUTY;
    /* Initialize PWM GENERATOR 2 DUTY CYCLE ADJUSTMENT REGISTER */
    PG2DCA       = 0x0000;
    /* Initialize PWM GENERATOR 2 PERIOD REGISTER */
    PG2PER       = MC1_LOOPTIME_TCY;
    /* Initialize PWM GENERATOR 2 DEAD-TIME REGISTER */
    PG2DTbits.DTH = MC1_DEADTIME;
    /* Initialize PWM GENERATOR 2 DEAD-TIME REGISTER */
    PG2DTbits.DTL = MC1_DEADTIME;

    /* Initialize PWM GENERATOR 2 TRIGGER A REGISTER */
    PG2TRIGA     = 0x0000;
    /* Initialize PWM GENERATOR 2 TRIGGER B REGISTER */
    PG2TRIGB     = 0x0000;
    /* Initialize PWM GENERATOR 2 TRIGGER C REGISTER */
    PG2TRIGC     = 0x0000;
    
}

/**
* <B> Function: InitPWMGenerator3()    </B>
*
* @brief Function to configure PWM Module # 3
*        
* @param none.
* @return none.
* 
* @example
* <CODE> InitPWMGenerator3();     </CODE>
*
*/
void InitPWMGenerator3 (void)
{

    /* Initialize PWM GENERATOR 3 CONTROL REGISTER */
    PG3CON      = 0x0000;
    /* PWM Generator 3 Enable bit : 1 = Is enabled, 0 = Is not enabled */
    /* PWM Generator is disabled prior to configuring module */
    PG3CONbits.ON = 0;
    /* Clock Selection bits
       0b01 = Macro uses Master clock selected by the PCLKCON.MCLKSEL bits*/
    PG3CONbits.CLKSEL = 1;
    /* PWM Mode Selection bits
     * 110 = Dual Edge Center-Aligned PWM mode (interrupt/register update once per cycle)
       100 = Center-Aligned PWM mode(interrupt/register update once per cycle)*/
    PG3CONbits.MODSEL = 4;    
    /* Trigger Count Select bits
       000 = PWM Generator produces 1 PWM cycle after triggered */
    PG3CONbits.TRGCNT = 0;
    
    /* Initialize PWM GENERATOR 3 CONTROL REGISTER */
    /* Master Duty Cycle Register Select bit
       1 = Macro uses the MDC register instead of PG3DC
       0 = Macro uses the PG3DC register*/
    PG3CONbits.MDCSEL = 0;
    /* Master Period Register Select bit
       1 = Macro uses the MPER register instead of PG3PER
       0 = Macro uses the PG3PER register */
    PG3CONbits.MPERSEL = 0;
    /* MPHSEL: Master Phase Register Select bit
       1 = Macro uses the MPHASE register instead of PG3PHASE
       0 = Macro uses the PG3PHASE register */
    PG3CONbits.MPHSEL = 0;
    /* Master Update Enable bit
       1 = PWM Generator broadcasts software set/clear of UPDATE status bit and 
           EOC signal to other PWM Generators
       0 = PWM Generator does not broadcast UPDATE status bit or EOC signal */
    PG3CONbits.MSTEN = 0;
    /* PWM Buffer Update Mode Selection bits 
       0b010 = Slaved SOC Update Data registers at start of next cycle if a 
       master update request is received. A master update request will be 
       transmitted if MSTEN = 1 and UPDATE = 1 for the requesting PWM
       Generator.. */
	PG3CONbits.UPDMOD = 0;
    /* PWM Generator Trigger Mode Selection bits
       0b00 = PWM Generator operates in Single Trigger mode
       0b01 = PWM Generator operates in Retriggerable mode */
    PG3CONbits.TRGMOD = 1;
    /* Start of Cycle Selection bits
       1111 = TRIG bit or PCI Sync function only (no hardware trigger source is selected)
       0000 = Local EOC*/
    PG3CONbits.SOCS = 0b1111;
    
    /* Clear PWM GENERATOR 3 STATUS REGISTER*/
    PG3STAT      = 0x0000;
    /* Initialize PWM GENERATOR 3 I/O CONTROL REGISTER */
    PG3IOCON2    = 0x0000;

    /* Current Limit Mode Select bit
       0 = If PCI current limit is active, then the CLDAT<1:0> bits define 
       the PWM output levels */
    PG3IOCON2bits.CLMOD = 0;
    /* Swap PWM Signals to PWM3H and PWM3L Device Pins bit 
       0 = PWM3H/L signals are mapped to their respective pins */
    PG3IOCON1bits.SWAP = 0;
    /* User Override Enable for PWM3H Pin bit
       0 = PWM Generator provides data for the PWM3H pin*/
    PG3IOCON2bits.OVRENH = 0;
    /* User Override Enable for PWM3L Pin bit
       0 = PWM Generator provides data for the PWM3L pin*/
    PG3IOCON2bits.OVRENL = 0;
    /* Data for PWM3H/PWM3L Pins if Override is Enabled bits
       If OVERENH = 1, then OVRDAT<1> provides data for PWM3H.
       If OVERENL = 1, then OVRDAT<0> provides data for PWM3L */
    PG3IOCON2bits.OVRDAT = 0;
    /* User Output Override Synchronization Control bits
       00 = User output overrides via the OVRENL/H and OVRDAT<1:0> bits are 
       synchronized to the local PWM time base (next start of cycle)*/
    PG3IOCON2bits.OSYNC = 0;
    /* Data for PWM3H/PWM3L Pins if FLT Event is Active bits
       If Fault is active, then FLTDAT<1> provides data for PWM3H.
       If Fault is active, then FLTDAT<0> provides data for PWM3L.*/
    PG3IOCON2bits.FLT1DAT = 0;
    /* Data for PWM3H/PWM3L Pins if CLMT Event is Active bits
       If current limit is active, then CLDAT<1> provides data for PWM3H.
       If current limit is active, then CLDAT<0> provides data for PWM3L.*/
    PG3IOCON2bits.CLDAT = 0;
    /* Data for PWM3H/PWM3L Pins if Feed-Forward Event is Active bits
       If feed-forward is active, then FFDAT<1> provides data for PWM3H.
       If feed-forward is active, then FFDAT<0> provides data for PWM3L.*/
    PG3IOCON2bits.FFDAT = 0;
    /* Data for PWM3H/PWM3L Pins if Debug Mode is Active and PTFRZ = 1 bits
       If Debug mode is active and PTFRZ=1,then DBDAT<1> provides PWM3H data.
       If Debug mode is active and PTFRZ=1,then DBDAT<0> provides PWM3L data. */
    PG3IOCON2bits.DBDAT = 0;
    
    /* Initialize PWM GENERATOR 3 I/O CONTROL REGISTER */    
    /* Time Base Capture Source Selection bits
       000 = No hardware source selected for time base capture ? software only*/
    PG3IOCON1bits.CAPSRC = 0;
    /* Dead-Time Compensation Select bit 
       0 = Dead-time compensation is controlled by PCI Sync logic */
    PG3IOCON1bits.DTCMPSEL = 0;
    /* PWM Generator Output Mode Selection bits
       00 = PWM Generator outputs operate in Complementary mode*/
    PG3IOCON1bits.PMOD = 0;
    /* PWM3H Output Port Enable bit
       1 = PWM Generator controls the PWM3H output pin
       0 = PWM Generator does not control the PWM3H output pin */
    PG3IOCON1bits.PENH = 1;
    /* PWM3L Output Port Enable bit
       1 = PWM Generator controls the PWM3L output pin
       0 = PWM Generator does not control the PWM3L output pin */
    PG3IOCON1bits.PENL = 1;
    /* PWM3H Output Polarity bit
       1 = Output pin is active-low
       0 = Output pin is active-high*/
    PG3IOCON1bits.POLH = 0;
    /* PWM3L Output Polarity bit
       1 = Output pin is active-low
       0 = Output pin is active-high*/
    PG3IOCON1bits.POLL = 0;
    /* Peripheral Pin Select Enable bit
       1 = Peripheral pin select enabled
       0 = Peripheral pin select disabled, as a result, PWM outputs are hard-mapped to pins*/
    PG3IOCON1bits.PPSEN = 0;
    
    /* Initialize PWM GENERATOR 3 EVENT REGISTER */
    PG3EVT1      = 0x0000;
    /* ADC Trigger 1 Post-scaler Selection bits
       00000 = 1:1 */
    PG3EVT1bits.ADTR1PS = 0;
    /* ADC Trigger 1 Source is PG3TRIGC Compare Event Enable bit
       0 = PG3TRIGC register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    PG3EVT1bits.ADTR1EN3  = 0;
    /* ADC Trigger 1 Source is PG3TRIGB Compare Event Enable bit
       0 = PG3TRIGB register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    PG3EVT1bits.ADTR1EN2 = 0;
    /* ADC Trigger 1 Source is PG3TRIGA Compare Event Enable bit
       0 = PG3TRIGA register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    PG3EVT1bits.ADTR1EN1 = 0;
    /* Update Trigger Select bits
       01 = A write of the PG3DC register automatically sets the UPDATE bit
       00 = User must set the UPDATE bit manually*/
    PG3EVT1bits.UPDTRG = 1;
    /* PWM Generator Trigger Output Selection bits
       000 = EOC event is the PWM Generator trigger*/
    PG3EVT1bits.PGTRGSEL = 0;
    
    /* Initialize PWM GENERATOR 3 EVENT REGISTER */
    /* FLTIEN: PCI Fault Interrupt Enable bit
       0 = Fault interrupt is disabled */
    PG3EVT1bits.FLT1IEN = 0;
    /* PCI Current Limit Interrupt Enable bit
       0 = Current limit interrupt is disabled */
    PG3EVT1bits.CLIEN = 0;
    /* PCI Feed-Forward Interrupt Enable bit
       0 = Feed-forward interrupt is disabled */
    PG3EVT1bits.FFIEN = 0;
    /* PCI Sync Interrupt Enable bit
       0 = Sync interrupt is disabled */
    PG3EVT1bits.SIEN = 0;
    /* Interrupt Event Selection bits
       00 = Interrupts CPU at EOC
       01 = Interrupts CPU at TRIGA compare event
       10 = Interrupts CPU at ADC Trigger 1 event
       11 = Time base interrupts are disabled */
    PG3EVT1bits.IEVTSEL = 3;
    /* ADC Trigger 3 Source is PG3TRIGC Compare Event Enable bit
       0 = PG3TRIGC register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    PG3EVT2bits.ADTR2EN3 = 0;
    /* ADC Trigger 2 Source is PG3TRIGB Compare Event Enable bit
       0 = PG3TRIGB register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    PG3EVT2bits.ADTR2EN2 = 0;
    /* ADC Trigger 2 Source is PG3TRIGA Compare Event Enable bit
       0 = PG3TRIGA register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    PG3EVT2bits.ADTR2EN1 = 0;
    /* ADC Trigger 1 Offset Selection bits
       00000 = No offset */
    PG3EVT1bits.ADTR1OFS = 0;
    
    /* PWM GENERATOR 3 Fault PCI REGISTER */
#ifndef ENABLE_PWM_FAULT_PCI_MC1
    /* PWM GENERATOR 3 Fault PCI REGISTER */
    PG3F1PCI1    = 0x0000;
#else
       /* PWM GENERATOR 3 Fault PCI REGISTER  */
    PG3F1PCI1     = 0x0000;
    /* Termination Synchronization Disable bit
       1 = Termination of latched PCI occurs immediately
       0 = Termination of latched PCI occurs at PWM EOC */
    PG3F1PCI1bits.TSYNCDIS = 0;
    /* Termination Event Selection bits
       001 = Auto-Terminate: Terminate when PCI source transitions from 
             active to inactive */
    PG3F1PCI1bits.TERM = 1;
    /* Acceptance Qualifier Polarity Select bit: 0 = Not inverted 1 = Inverted*/
    PG3F1PCI1bits.AQPS = 0;
    /* Acceptance Qualifier Source Selection bits
       111 = SWPCI control bit only (qualifier forced to 0)
       110 = Selects PCI Source #9
       101 = Selects PCI Source #8
       100 = Selects PCI Source #1 (PWM Generator output selected by the PWMPCI<2:0> bits)
       011 = PWM Generator is triggered
       010 = LEB is active
       001 = Duty cycle is active (base PWM Generator signal)        
       000 = No acceptance qualifier is used (qualifier forced to 1) */
    PG3F1PCI1bits.AQSS = 0;
    /* PCI Synchronization Control bit
       1 = PCI source is synchronized to PWM EOC
       0 = PCI source is not synchronized to PWM EOC*/
    PG3F1PCI1bits.PSYNC = 0;
    /* PCI Polarity Select bit 0 = Not inverted 1 = Inverted */
    PG3F1PCI1bits.PPS = 1;
    /* PCI Source Selection bits
       31 : CLC1 output
       30 : Comparator 3 output(0x40000000)
       ? ?
       8 : PPS PCI8R in RPINR20 register input(0x0100)
       ? ?
       1 : Internally connected to the output of PWMPCI[2:0] bits in PGxEVT1 registers
       0 : Tied to '0' */
    PG3F1PCI2 = 0x0100;
    /* PCI Bypass Enable bit
       0 = PCI function is not bypassed */
    PG3F1PCI1bits.BPEN   = 0;
    /* PCI Bypass Source Selection bits(1)
       000 = PCI control is sourced from PG1 PCI logic when BPEN = 1 */
    PG3F1PCI1bits.BPSEL   = 0;
    /* PCI Termination Polarity Select bits 
       0 = Inverter, 1 = Non Inverted */
    PG3F1PCI1bits.TERMPS = 0;
    /* PCI Acceptance Criteria Selection bits
       101 = Latched any edge(2)
       100 = Latched rising edge
       011 = Latched
       010 = Any edge
       001 = Rising edge
       000 = Level-sensitive*/
    PG3F1PCI1bits.ACP   = 3;
    /* PCI SR Latch Mode bit
       1 = SR latch is Reset-dominant in Latched Acceptance modes
       0 = SR latch is Set-dominant in Latched Acceptance modes*/
    PG3F1PCI1bits.LATMOD  = 0;
    /* Termination Qualifier Polarity Select bit 1 = Inverted 0 = Not inverted*/
    PG3F1PCI1bits.TQPS   = 0;
    /* Termination Qualifier Source Selection bits
       111 = SWPCI control bit only (qualifier forced to '0')(3)
       110 = Selects PCI Source #9 (pwm_pci[9] input port)
       101 = Selects PCI Source #8 (pwm_pci[8] input port)
       100 = Selects PCI Source #1 (PWM Generator output selected by the PWMPCI<2:0> bits)
       011 = PWM Generator is triggered
       010 = LEB is active
       001 = Duty cycle is active (base PWM Generator signal)
       000 = No termination qualifier used (qualifier forced to '1')(3)*/
    PG3F1PCI1bits.TQSS  = 0;
#endif  
    
    /* PWM GENERATOR 3 Current Limit PCI REGISTER */
    PG3CLPCI1    = 0x0000;
    /* PWM GENERATOR 3 Feed Forward PCI REGISTER */
    PG3FFPCI1    = 0x0000;
    
    /* For PWM Synchronization
     PWM Gen 3 is triggered by the Event A of PWM Gen 5*/
    /* PWM GENERATOR 3 Sync PCI REGISTER */
    PG3SPCI1    = 0x0000;
    /* PCI Polarity Select bit 
       1 = Inverted
       0 = Not inverted*/
    PG3SPCI1bits.PPS = 0;
    /* PWM PCI Source Selection
       100 = PWM Generator #5 output used as PCI signal */
    PG3EVT1bits.PWMPCI = 0b100;
    /* PWM GENERATOR 3 Sync PCI Source 
       23 = PWM event A*/
    PG3SPCI2 = 0x00800000;    
    
    /* Initialize PWM GENERATOR 3 LEADING-EDGE BLANKING REGISTER */
    PG3LEB      = 0x0000;
    
    /* Initialize PWM GENERATOR 3 PHASE REGISTER */
    PG3PHASEbits.PHASE     = MC1_MIN_DUTY;
    /* Initialize PWM GENERATOR 3 DUTY CYCLE REGISTER */
    PG3DCbits.DC           = MC1_MIN_DUTY;
    /* Initialize PWM GENERATOR 3 DUTY CYCLE ADJUSTMENT REGISTER */
    PG3DCA       = 0x0000;
    /* Initialize PWM GENERATOR 3 PERIOD REGISTER */
    PG3PER       = MC1_LOOPTIME_TCY;
    /* Initialize PWM GENERATOR 3 DEAD-TIME REGISTER */
    PG3DTbits.DTH = MC1_DEADTIME;
    /* Initialize PWM GENERATOR 3 DEAD-TIME REGISTER */
    PG3DTbits.DTL = MC1_DEADTIME;

    /* Initialize PWM GENERATOR 3 TRIGGER A REGISTER */
    PG3TRIGA     = 0x0000;
    /* Initialize PWM GENERATOR 3 TRIGGER B REGISTER */
    PG3TRIGB     = 0x0000;
    /* Initialize PWM GENERATOR 3 TRIGGER C REGISTER */
    PG3TRIGC     = 0x0000;
    
}

/**
* <B> Function: InitPWMGenerator5()    </B>
*
* @brief Function to configure PWM Module # 5
*        
* @param none.
* @return none.
* 
* @example
* <CODE> InitPWMGenerator5();     </CODE>
*
*/
void InitPWMGenerator5 (void)
{

    /* Initialize PWM GENERATOR 5 CONTROL REGISTER */
    PG5CON      = 0x0000;
    /* PWM Generator 5 Enable bit : 1 = Is enabled, 0 = Is not enabled */
    /* Ensuring PWM Generator is disabled prior to configuring module */
    PG5CONbits.ON = 0;
    /* Clock Selection bits
       0b01 = Macro uses Master clock selected by the PCLKCON.MCLKSEL bits*/
    PG5CONbits.CLKSEL = 1;
    /* PWM Mode Selection bits
     * 110 = Dual Edge Center-Aligned PWM mode (interrupt/register update once per cycle)
       100 = Center-Aligned PWM mode(interrupt/register update once per cycle)*/
    PG5CONbits.MODSEL = 4; 
    /* Trigger Count Select bits
       000 = PWM Generator produces 1 PWM cycle after triggered */
    PG5CONbits.TRGCNT = 0;
    
    /* Initialize PWM GENERATOR 1 CONTROL REGISTER */
    /* Master Duty Cycle Register Select bit
       1 = Macro uses the MDC register instead of PG5DC
       0 = Macro uses the PG5DC register*/
    PG5CONbits.MDCSEL = 0;
    /* Master Period Register Select bit
       1 = Macro uses the MPER register instead of PG5PER
       0 = Macro uses the PG5PER register */
    PG5CONbits.MPERSEL = 0;
    /* MPHSEL: Master Phase Register Select bit
       1 = Macro uses the MPHASE register instead of PG5PHASE
       0 = Macro uses the PG5PHASE register */
    PG5CONbits.MPHSEL = 0;
    /* Master Update Enable bit
       1 = PWM Generator broadcasts software set/clear of UPDATE status bit and 
           EOC signal to other PWM Generators
       0 = PWM Generator does not broadcast UPDATE status bit or EOC signal */
    PG5CONbits.MSTEN = 0;
    /* PWM Buffer Update Mode Selection bits 
       Update Data registers at start of next PWM cycle if UPDATE = 1. */
    PG5CONbits.UPDMOD = 0;
    /* PWM Generator Trigger Mode Selection bits
       0b00 = PWM Generator operates in Single Trigger mode */
    PG5CONbits.TRGMOD = 0;
    /* Start of Cycle Selection bits
       0000 = Local EOC*/
    PG5CONbits.SOCS = 0;
    
    /* Clear PWM GENERATOR 5 STATUS REGISTER*/
    PG5STAT      = 0x0000;
    /* Initialize PWM GENERATOR 5 I/O CONTROL REGISTER */
    PG5IOCON2    = 0x0000;

    /* Current Limit Mode Select bit
       0 = If PCI current limit is active, then the CLDAT<1:0> bits define 
       the PWM output levels */
    PG5IOCON2bits.CLMOD = 0;
    /* Swap PWM Signals to PWM1H and PWM1L Device Pins bit 
       0 = PWM1H/L signals are mapped to their respective pins */
    PG5IOCON1bits.SWAP = 0;
    /* User Override Enable for PWM1H Pin bit
       0 = PWM Generator provides data for the PWM1H pin*/
    PG5IOCON2bits.OVRENH = 0;
    /* User Override Enable for PWM1L Pin bit
       0 = PWM Generator provides data for the PWM1L pin*/
    PG5IOCON2bits.OVRENL = 0;
    /* Data for PWM1H/PWM1L Pins if Override is Enabled bits
       If OVERENH = 1, then OVRDAT<1> provides data for PWM1H.
       If OVERENL = 1, then OVRDAT<0> provides data for PWM1L */
    PG5IOCON2bits.OVRDAT = 0;
    /* User Output Override Synchronization Control bits
       00 = User output overrides via the OVRENL/H and OVRDAT<1:0> bits are 
       synchronized to the local PWM time base (next start of cycle)*/
    PG5IOCON2bits.OSYNC = 0;
    /* Data for PWM5H/PWM5L Pins if FLT Event is Active bits
       If Fault is active, then FLTDAT<1> provides data for PWM5H.
       If Fault is active, then FLTDAT<0> provides data for PWM5L.*/
    PG5IOCON2bits.FLT1DAT = 0;
    /* Data for PWM5H/PWM5L Pins if CLMT Event is Active bits
       If current limit is active, then CLDAT<1> provides data for PWM5H.
       If current limit is active, then CLDAT<0> provides data for PWM5L.*/
    PG5IOCON2bits.CLDAT = 0;
    /* Data for PWM5H/PWM5L Pins if Feed-Forward Event is Active bits
       If feed-forward is active, then FFDAT<1> provides data for PWM5H.
       If feed-forward is active, then FFDAT<0> provides data for PWM5L.*/
    PG5IOCON2bits.FFDAT = 0;
    /* Data for PWM5H/PWM5L Pins if Debug Mode is Active and PTFRZ = 1 bits
       If Debug mode is active and PTFRZ=1,then DBDAT<1> provides PWM5H data.
       If Debug mode is active and PTFRZ=1,then DBDAT<0> provides PWM5L data. */
    PG5IOCON2bits.DBDAT = 0;
    
    /* Initialize PWM GENERATOR 5 I/O CONTROL REGISTER */    

    /* Time Base Capture Source Selection bits
       000 = No hardware source selected for time base capture ? software only*/
    PG5IOCON1bits.CAPSRC = 0;
    /* Dead-Time Compensation Select bit 
       0 = Dead-time compensation is controlled by PCI Sync logic */
    PG5IOCON1bits.DTCMPSEL = 0;
    /* PWM Generator Output Mode Selection bits
       00 = PWM Generator outputs operate in Complementary mode*/
    PG5IOCON1bits.PMOD = 0;
    /* PWM1H Output Port Enable bit
       1 = PWM Generator controls the PWM5H output pin
       0 = PWM Generator does not control the PWM5H output pin */
    PG5IOCON1bits.PENH = 0;
    /* PWM1L Output Port Enable bit
       1 = PWM Generator controls the PWM5L output pin
       0 = PWM Generator does not control the PWM5L output pin */
    PG5IOCON1bits.PENL = 0;
    /* PWM1H Output Polarity bit
       1 = Output pin is active-low
       0 = Output pin is active-high*/
    PG5IOCON1bits.POLH = 0;
    /* PWM1L Output Polarity bit
       1 = Output pin is active-low
       0 = Output pin is active-high*/
    PG5IOCON1bits.POLL = 0;
    /* Peripheral Pin Select Enable bit
       1 = Peripheral pin select enabled
       0 = Peripheral pin select disabled, as a result, PWM outputs are hard-mapped to pins*/
    PG5IOCON1bits.PPSEN = 0;
    
    /* Initialize PWM GENERATOR 5 EVENT REGISTER */
    PG5EVT1      = 0x0000;
    /* ADC Trigger 1 Post-scaler Selection bits
       00000 = 1:1 */
    PG5EVT1bits.ADTR1PS = 0;
    /* ADC Trigger 1 Source is PG5TRIGC Compare Event Enable bit
       0 = PG5TRIGC register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    PG5EVT1bits.ADTR1EN3  = 0;
    /* ADC Trigger 1 Source is PG5TRIGB Compare Event Enable bit
       0 = PG5TRIGB register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    PG5EVT1bits.ADTR1EN2 = 0;    
    /* ADC Trigger 1 Source is PG5TRIGA Compare Event Enable bit
       1 = PG5TRIGA register compare event is enabled as trigger source for 
           ADC Trigger 1 */
    PG5EVT1bits.ADTR1EN1 = 1;
    /* Update Trigger Select bits
       01 = A write of the PG5DC register automatically sets the UPDATE bit*/
    PG5EVT1bits.UPDTRG = 1;
    /* PWM Generator Trigger Output Selection bits
       000 = EOC event is the PWM Generator trigger*/
    PG5EVT1bits.PGTRGSEL = 0;
    
    /* Initialize PWM GENERATOR 1 EVENT REGISTER */
    /* FLTIEN: PCI Fault Interrupt Enable bit
       1 = Fault interrupt is enabled */
    PG5EVT1bits.FLT1IEN = 1;
    /* PCI Current Limit Interrupt Enable bit
       0 = Current limit interrupt is disabled */
    PG5EVT1bits.CLIEN = 0;
    /* PCI Feed-Forward Interrupt Enable bit
       0 = Feed-forward interrupt is disabled */
    PG5EVT1bits.FFIEN = 0;
    /* PCI Sync Interrupt Enable bit
       0 = Sync interrupt is disabled */
    PG5EVT1bits.SIEN = 0;
    /* Interrupt Event Selection bits
       00 = Interrupts CPU at EOC
       01 = Interrupts CPU at TRIGA compare event
       10 = Interrupts CPU at ADC Trigger 1 event
       11 = Time base interrupts are disabled */
    PG5EVT1bits.IEVTSEL = 3;

    /* ADC Trigger 2 Source is PG5TRIGC Compare Event Enable bit
       0 = PG5TRIGC register compare event is disabled as 
       trigger source for ADC Trigger 2 */
    PG5EVT2bits.ADTR2EN3 = 0;
    /* ADC Trigger 2 Source is PG5TRIGB Compare Event Enable bit
       0 = PG5TRIGB register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    PG5EVT2bits.ADTR2EN2 = 0;
    /* ADC Trigger 2 Source is PG5TRIGA Compare Event Enable bit
       0 = PG5TRIGA register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    PG5EVT2bits.ADTR2EN1 = 0;
    /* ADC Trigger 1 Offset Selection bits
       00000 = No offset */
    PG5EVT1bits.ADTR1OFS = 0;

    /* PWM GENERATOR 5 Current Limit PCI REGISTER */
    PG5CLPCI1    = 0x0000;
    /* PWM GENERATOR 5 Feed Forward PCI REGISTER  */
    PG5FFPCI1    = 0x0000;
    /* PWM GENERATOR 5 Sync PCI REGISTER */
    PG5SPCI1    = 0x0000;
    
    /* Initialize PWM GENERATOR 5 LEADING-EDGE BLANKING REGISTER */
    PG5LEB      = 0x0000;
    
    /* Initialize PWM GENERATOR 5 PHASE REGISTER */
    PG5PHASEbits.PHASE  = MC1_MIN_DUTY;
    /* Initialize PWM GENERATOR 5 DUTY CYCLE REGISTER */
    PG5DCbits.DC        = (MC1_LOOPTIME_TCY/3);
    /* Initialize PWM GENERATOR 5 DUTY CYCLE ADJUSTMENT REGISTER */
    PG5DCA       = 0x0000;
    /* Initialize PWM GENERATOR 5 PERIOD REGISTER */
    PG5PERbits.PER      = MC1_LOOPTIME_TCY;
    /* Initialize PWM GENERATOR 5 DEAD-TIME REGISTER */
    PG5DTbits.DTH = MC1_DEADTIME;
    /* Initialize PWM GENERATOR 5 DEAD-TIME REGISTER */
    PG5DTbits.DTL = MC1_DEADTIME;

    /* Initialize PWM GENERATOR 5 TRIGGER A REGISTER */
    PG5TRIGAbits.TRIGA     = MC1_ADC_SAMPLING_POINT;
    /* Initialize PWM GENERATOR 5 TRIGGER B REGISTER */
    PG5TRIGB     = 0x0000;
    /* Initialize PWM GENERATOR 5 TRIGGER C REGISTER */
    PG5TRIGC     = 0x0000;   
}

/**
* <B> Function: InitPWMGenerator6()    </B>
*
* @brief Function to configure PWM Module # 6
*        
* @param none.
* @return none.
* 
* @example
* <CODE> InitPWMGenerator6();     </CODE>
*
*/
void InitPWMGenerator6 (void)
{

    /* Initialize PWM GENERATOR 6 CONTROL REGISTER */
    PG6CON      = 0x0000;
    /* PWM Generator 6 Enable bit : 1 = Is enabled, 0 = Is not enabled */
    /* Ensuring PWM Generator is disabled prior to configuring module */
    PG6CONbits.ON = 0;
    /* Clock Selection bits
       0b01 = Macro uses Master clock selected by the PCLKCON.MCLKSEL bits*/
    PG6CONbits.CLKSEL = 1;
    /* PWM Mode Selection bits
     * 110 = Dual Edge Center-Aligned PWM mode (interrupt/register update once per cycle)
       100 = Center-Aligned PWM mode(interrupt/register update once per cycle)*/
    PG6CONbits.MODSEL = 4;
    /* Trigger Count Select bits
       000 = PWM Generator produces 1 PWM cycle after triggered */
    PG6CONbits.TRGCNT = 0;
    
    /* Initialize PWM GENERATOR 6 CONTROL REGISTER */
    /* Master Duty Cycle Register Select bit
       1 = Macro uses the MDC register instead of PG6DC
       0 = Macro uses the PG6DC register*/
    PG6CONbits.MDCSEL = 0;
    /* Master Period Register Select bit
       1 = Macro uses the MPER register instead of PG6PER
       0 = Macro uses the PG6PER register */
    PG6CONbits.MPERSEL = 0;
    /* MPHSEL: Master Phase Register Select bit
       1 = Macro uses the MPHASE register instead of PG6PHASE
       0 = Macro uses the PG6PHASE register */
    PG6CONbits.MPHSEL = 0;
    /* Master Update Enable bit
       1 = PWM Generator broadcasts software set/clear of UPDATE status bit and 
           EOC signal to other PWM Generators
       0 = PWM Generator does not broadcast UPDATE status bit or EOC signal */
    PG6CONbits.MSTEN = 0;
    /* PWM Buffer Update Mode Selection bits 
       Update Data registers at start of next PWM cycle if UPDATE = 1. */
    PG6CONbits.UPDMOD = 0;
    /* PWM Generator Trigger Mode Selection bits
       0b00 = PWM Generator operates in Single Trigger mode
       0b01 = PWM Generator operates in Retriggerable mode */
    PG6CONbits.TRGMOD = 1;
    /* Start of Cycle Selection bits
       1111 = TRIG bit or PCI Sync function only (no hardware trigger source is selected)
       0000 = Local EOC*/
    /* For PWM Synchronization
     PWM Generator 6 is in sync with PWM Generator 5*/
    PG6CONbits.SOCS = 0b0101;
    
    /* Clear PWM GENERATOR 6 STATUS REGISTER*/
    PG6STAT      = 0x0000;
    /* Initialize PWM GENERATOR 6 I/O CONTROL REGISTER */
    PG6IOCON2    = 0x0000;

    /* Current Limit Mode Select bit
       0 = If PCI current limit is active, then the CLDAT<1:0> bits define 
       the PWM output levels */
    PG6IOCON2bits.CLMOD = 0;
    /* Swap PWM Signals to PWM6H and PWM6L Device Pins bit 
       0 = PWM6H/L signals are mapped to their respective pins */
    PG6IOCON1bits.SWAP = 0;
    /* User Override Enable for PWM6H Pin bit
       0 = PWM Generator provides data for the PWM6H pin*/
    PG6IOCON2bits.OVRENH = 0;
    /* User Override Enable for PWM6L Pin bit
       0 = PWM Generator provides data for the PWM6L pin*/
    PG6IOCON2bits.OVRENL = 0;
    /* Data for PWM6H/PWM6L Pins if Override is Enabled bits
       If OVERENH = 1, then OVRDAT<1> provides data for PWM6H.
       If OVERENL = 1, then OVRDAT<0> provides data for PWM6L */
    PG6IOCON2bits.OVRDAT = 0;
    /* User Output Override Synchronization Control bits
       00 = User output overrides via the OVRENL/H and OVRDAT<1:0> bits are 
       synchronized to the local PWM time base (next start of cycle)*/
    PG6IOCON2bits.OSYNC = 0;
    /* Data for PWM6H/PWM6L Pins if FLT Event is Active bits
       If Fault is active, then FLTDAT<1> provides data for PWM6H.
       If Fault is active, then FLTDAT<0> provides data for PWM6L.*/
    PG6IOCON2bits.FLT1DAT = 0;
    /* Data for PWM6H/PWM6L Pins if CLMT Event is Active bits
       If current limit is active, then CLDAT<1> provides data for PWM6H.
       If current limit is active, then CLDAT<0> provides data for PWM6L.*/
    PG6IOCON2bits.CLDAT = 0;
    /* Data for PWM6H/PWM6L Pins if Feed-Forward Event is Active bits
       If feed-forward is active, then FFDAT<1> provides data for PWM6H.
       If feed-forward is active, then FFDAT<0> provides data for PWM6L.*/
    PG6IOCON2bits.FFDAT = 0;
    /* Data for PWM6H/PWM6L Pins if Debug Mode is Active and PTFRZ = 1 bits
       If Debug mode is active and PTFRZ=1,then DBDAT<1> provides PWM6H data.
       If Debug mode is active and PTFRZ=1,then DBDAT<0> provides PWM6L data. */
    PG6IOCON2bits.DBDAT = 0;
    
    /* Initialize PWM GENERATOR 6 I/O CONTROL REGISTER */    

    /* Time Base Capture Source Selection bits
       000 = No hardware source selected for time base capture ? software only*/
    PG6IOCON1bits.CAPSRC = 0;
    /* Dead-Time Compensation Select bit 
       0 = Dead-time compensation is controlled by PCI Sync logic */
    PG6IOCON1bits.DTCMPSEL = 0;
    /* PWM Generator Output Mode Selection bits
       00 = PWM Generator outputs operate in Complementary mode*/
    PG6IOCON1bits.PMOD = 0;
    /* PWM6H Output Port Enable bit
       1 = PWM Generator controls the PWM6H output pin
       0 = PWM Generator does not control the PWM6H output pin */
    PG6IOCON1bits.PENH = 1;
    /* PWM6L Output Port Enable bit
       1 = PWM Generator controls the PWM6L output pin
       0 = PWM Generator does not control the PWM6L output pin */
    PG6IOCON1bits.PENL = 1;
    /* PWM6H Output Polarity bit
       1 = Output pin is active-low
       0 = Output pin is active-high*/
    PG6IOCON1bits.POLH = 0;
    /* PWM6L Output Polarity bit
       1 = Output pin is active-low
       0 = Output pin is active-high*/
    PG6IOCON1bits.POLL = 0;
    /* Peripheral Pin Select Enable bit
       1 = Peripheral pin select enabled
       0 = Peripheral pin select disabled, as a result, PWM outputs are hard-mapped to pins*/
    PG6IOCON1bits.PPSEN = 0;
    
    /* Initialize PWM GENERATOR 6 EVENT REGISTER */
    PG6EVT1      = 0x0000;
    /* ADC Trigger 1 Post-scaler Selection bits
       00000 = 1:1 */
    PG6EVT1bits.ADTR1PS = 0;
    /* ADC Trigger 1 Source is PG6TRIGC Compare Event Enable bit
       0 = PG6TRIGC register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    PG6EVT1bits.ADTR1EN3  = 0;
    /* ADC Trigger 1 Source is PG6TRIGB Compare Event Enable bit
       0 = PG6TRIGB register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    PG6EVT1bits.ADTR1EN2 = 0;    
    /* ADC Trigger 1 Source is PG6TRIGA Compare Event Enable bit
       1 = PG6TRIGA register compare event is enabled as trigger source for 
           ADC Trigger 1 */
    PG6EVT1bits.ADTR1EN1 = 1;
    /* Update Trigger Select bits
       01 = A write of the PG6DC register automatically sets the UPDATE bit*/
    PG6EVT1bits.UPDTRG = 1;
    /* PWM Generator Trigger Output Selection bits
       000 = EOC event is the PWM Generator trigger*/
    PG6EVT1bits.PGTRGSEL = 0;
    
    /* Initialize PWM GENERATOR 6 EVENT REGISTER */
    /* FLTIEN: PCI Fault Interrupt Enable bit
       1 = Fault interrupt is enabled */
    PG6EVT1bits.FLT1IEN = 1;
    /* PCI Current Limit Interrupt Enable bit
       0 = Current limit interrupt is disabled */
    PG6EVT1bits.CLIEN = 0;
    /* PCI Feed-Forward Interrupt Enable bit
       0 = Feed-forward interrupt is disabled */
    PG6EVT1bits.FFIEN = 0;
    /* PCI Sync Interrupt Enable bit
       0 = Sync interrupt is disabled */
    PG6EVT1bits.SIEN = 0;
    /* Interrupt Event Selection bits
       00 = Interrupts CPU at EOC
       01 = Interrupts CPU at TRIGA compare event
       10 = Interrupts CPU at ADC Trigger 1 event
       11 = Time base interrupts are disabled */
    PG6EVT1bits.IEVTSEL = 3;

        /* ADC Trigger 2 Source is PG6TRIGC Compare Event Enable bit
       0 = PG6TRIGC register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    PG6EVT2bits.ADTR2EN3 = 0;
    /* ADC Trigger 2 Source is PG6TRIGB Compare Event Enable bit
       0 = PG6TRIGB register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    PG6EVT2bits.ADTR2EN2 = 0;
    /* ADC Trigger 2 Source is PG6TRIGA Compare Event Enable bit
       0 = PG6TRIGA register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    PG6EVT2bits.ADTR2EN1 = 0;
    /* ADC Trigger 1 Offset Selection bits
       00000 = No offset */
    PG6EVT1bits.ADTR1OFS = 0;
    
#ifndef ENABLE_PWM_FAULT_PCI_MC2
    /* PWM GENERATOR 6 Fault PCI REGISTER */
    PG6F1PCI1     = 0x0000;
#else
       /* PWM GENERATOR 6 Fault PCI REGISTER  */
    PG6F1PCI1     = 0x0000;
    /* Termination Synchronization Disable bit
       1 = Termination of latched PCI occurs immediately
       0 = Termination of latched PCI occurs at PWM EOC */
    PG6F1PCI1bits.TSYNCDIS = 0;
    /* Termination Event Selection bits
       001 = Auto-Terminate: Terminate when PCI source transitions from 
             active to inactive */
    PG6F1PCI1bits.TERM = 1;
    /* Acceptance Qualifier Polarity Select bit: 0 = Not inverted 1 = Inverted*/
    PG6F1PCI1bits.AQPS = 0;
    /* Acceptance Qualifier Source Selection bits
       111 = SWPCI control bit only (qualifier forced to 0)
       110 = Selects PCI Source #9
       101 = Selects PCI Source #8
       100 = Selects PCI Source #1 (PWM Generator output selected by the PWMPCI<2:0> bits)
       011 = PWM Generator is triggered
       010 = LEB is active
       001 = Duty cycle is active (base PWM Generator signal)        
       000 = No acceptance qualifier is used (qualifier forced to 1) */
    PG6F1PCI1bits.AQSS = 0;
    /* PCI Synchronization Control bit
       1 = PCI source is synchronized to PWM EOC
       0 = PCI source is not synchronized to PWM EOC*/
    PG6F1PCI1bits.PSYNC = 0;
    /* PCI Polarity Select bit 0 = Not inverted 1 = Inverted */
    PG6F1PCI1bits.PPS = 1;
    /* PCI Source Selection bits
       31 : CLC1 output
       30 : Comparator 3 output(0x40000000)
       ? ?
       8 : PPS PCI8R in RPINR20 register input(0x0100)
       ? ?
       1 : Internally connected to the output of PWMPCI[2:0] bits in PGxEVT1 registers
       0 : Tied to '0' */
    PG6F1PCI2 = 0x0100;
    /* PCI Bypass Enable bit
       0 = PCI function is not bypassed */
    PG6F1PCI1bits.BPEN   = 0;
    /* PCI Bypass Source Selection bits(1)
       000 = PCI control is sourced from PG6 PCI logic when BPEN = 1 */
    PG6F1PCI1bits.BPSEL   = 0;
    /* PCI Termination Polarity Select bits 
       0 = Inverter, 1 = Non Inverted */
    PG6F1PCI1bits.TERMPS = 0;
    /* PCI Acceptance Criteria Selection bits
       101 = Latched any edge(2)
       100 = Latched rising edge
       011 = Latched
       010 = Any edge
       001 = Rising edge
       000 = Level-sensitive*/
    PG6F1PCI1bits.ACP   = 3;
    /* PCI SR Latch Mode bit
       1 = SR latch is Reset-dominant in Latched Acceptance modes
       0 = SR latch is Set-dominant in Latched Acceptance modes*/
    PG6F1PCI1bits.LATMOD  = 0;
    /* Termination Qualifier Polarity Select bit 1 = Inverted 0 = Not inverted*/
    PG6F1PCI1bits.TQPS   = 0;
    /* Termination Qualifier Source Selection bits
       111 = SWPCI control bit only (qualifier forced to '0')(3)
       110 = Selects PCI Source #9 (pwm_pci[9] input port)
       101 = Selects PCI Source #8 (pwm_pci[8] input port)
       100 = Selects PCI Source #1 (PWM Generator output selected by the PWMPCI<2:0> bits)
       011 = PWM Generator is triggered
       010 = LEB is active
       001 = Duty cycle is active (base PWM Generator signal)
       000 = No termination qualifier used (qualifier forced to '1')(3)*/
    PG6F1PCI1bits.TQSS  = 0;
#endif    

    /* PWM GENERATOR 6 Current Limit PCI REGISTER */
    PG6CLPCI1    = 0x0000;
    /* PWM GENERATOR 6 Feed Forward PCI REGISTER  */
    PG6FFPCI1    = 0x0000;

    /* PWM GENERATOR 6 Sync PCI REGISTER */
    PG6SPCI1    = 0x0000;
    /* PCI Polarity Select bit 
       1 = Inverted
       0 = Not inverted*/
    PG6SPCI1bits.PPS = 0;
    /* PWM GENERATOR 6 Sync PCI Source 
       0 = Tied to ?0? */
    PG6SPCI2 = 0x0000;
    
    /* Initialize PWM GENERATOR 6 LEADING-EDGE BLANKING REGISTER */
    PG6LEB      = 0x0000;
    
    /* Initialize PWM GENERATOR 6 PHASE REGISTER */
    PG6PHASEbits.PHASE     = MC2_MIN_DUTY;
    /* Initialize PWM GENERATOR 6 DUTY CYCLE REGISTER */
    PG6DCbits.DC           = MC2_MIN_DUTY;
    /* Initialize PWM GENERATOR 6 DUTY CYCLE ADJUSTMENT REGISTER */
    PG6DCA       = 0x0000;
    /* Initialize PWM GENERATOR 6 PERIOD REGISTER */
    PG6PER       = MC2_LOOPTIME_TCY;
    /* Initialize PWM GENERATOR 6 DEAD-TIME REGISTER */
    PG6DTbits.DTH = MC2_DEADTIME;
    /* Initialize PWM GENERATOR 6 DEAD-TIME REGISTER */
    PG6DTbits.DTL = MC2_DEADTIME;

    /* Initialize PWM GENERATOR 6 TRIGGER A REGISTER */
    PG6TRIGAbits.CAHALF = 0;
    PG6TRIGAbits.TRIGA  = MC2_ADC_SAMPLING_POINT;
    /* Initialize PWM GENERATOR 6 TRIGGER B REGISTER */
    PG6TRIGB     = 0x0000;
    /* Initialize PWM GENERATOR 6 TRIGGER C REGISTER */
    PG6TRIGC     = 0x0000;
    
}

/**
* <B> Function: InitPWMGenerator7()    </B>
*
* @brief Function to configure PWM Module # 7
*        
* @param none.
* @return none.
* 
* @example
* <CODE> InitPWMGenerator7();     </CODE>
*
*/
void InitPWMGenerator7 (void)
{

    /* Initialize PWM GENERATOR 7 CONTROL REGISTER */
    PG7CON      = 0x0000;
    /* PWM Generator 7 Enable bit : 1 = Is enabled, 0 = Is not enabled */
    /* Ensuring PWM Generator is disabled prior to configuring module */
    PG7CONbits.ON = 0;
    /* Clock Selection bits
       0b01 = Macro uses Master clock selected by the PCLKCON.MCLKSEL bits*/
    PG7CONbits.CLKSEL = 1;
    /* PWM Mode Selection bits
     * 110 = Dual Edge Center-Aligned PWM mode (interrupt/register update once per cycle)
       100 = Center-Aligned PWM mode(interrupt/register update once per cycle)*/
    PG7CONbits.MODSEL = 4;
    /* Trigger Count Select bits
       000 = PWM Generator produces 1 PWM cycle after triggered */
    PG7CONbits.TRGCNT = 0;
    
    /* Initialize PWM GENERATOR 7 CONTROL REGISTER */
    /* Master Duty Cycle Register Select bit
       1 = Macro uses the MDC register instead of PG7DC
       0 = Macro uses the PG7DC register*/
    PG7CONbits.MDCSEL = 0;
    /* Master Period Register Select bit
       1 = Macro uses the MPER register instead of PG7PER
       0 = Macro uses the PG7PER register */
    PG7CONbits.MPERSEL = 0;
    /* MPHSEL: Master Phase Register Select bit
       1 = Macro uses the MPHASE register instead of PG7PHASE
       0 = Macro uses the PG7PHASE register */
    PG7CONbits.MPHSEL = 0;
    /* Master Update Enable bit
       1 = PWM Generator broadcasts software set/clear of UPDATE status bit and 
           EOC signal to other PWM Generators
       0 = PWM Generator does not broadcast UPDATE status bit or EOC signal */
    PG7CONbits.MSTEN = 0;
    /* PWM Buffer Update Mode Selection bits 
       Update Data registers at start of next PWM cycle if UPDATE = 1. */
    PG7CONbits.UPDMOD = 0;
    /* PWM Generator Trigger Mode Selection bits
       0b00 = PWM Generator operates in Single Trigger mode
       0b01 = PWM Generator operates in Retriggerable mode */
    PG7CONbits.TRGMOD = 1;
    /* Start of Cycle Selection bits
       1111 = TRIG bit or PCI Sync function only (no hardware trigger source is selected)
       0000 = Local EOC*/
    /* For PWM Synchronization
     PWM Generator 6 is in sync with PWM Generator 5*/
    PG7CONbits.SOCS = 0b0101;
    
    /* Clear PWM GENERATOR 7 STATUS REGISTER*/
    PG7STAT      = 0x0000;
    /* Initialize PWM GENERATOR 7 I/O CONTROL REGISTER */
    PG7IOCON2    = 0x0000;

    /* Current Limit Mode Select bit
       0 = If PCI current limit is active, then the CLDAT<1:0> bits define 
       the PWM output levels */
    PG7IOCON2bits.CLMOD = 0;
    /* Swap PWM Signals to PWM7H and PWM7L Device Pins bit 
       0 = PWM7H/L signals are mapped to their respective pins */
    PG7IOCON1bits.SWAP = 0;
    /* User Override Enable for PWM7H Pin bit
       0 = PWM Generator provides data for the PWM7H pin*/
    PG7IOCON2bits.OVRENH = 0;
    /* User Override Enable for PWM7L Pin bit
       0 = PWM Generator provides data for the PWM7L pin*/
    PG7IOCON2bits.OVRENL = 0;
    /* Data for PWM7H/PWM7L Pins if Override is Enabled bits
       If OVERENH = 1, then OVRDAT<1> provides data for PWM7H.
       If OVERENL = 1, then OVRDAT<0> provides data for PWM7L */
    PG7IOCON2bits.OVRDAT = 0;
    /* User Output Override Synchronization Control bits
       00 = User output overrides via the OVRENL/H and OVRDAT<1:0> bits are 
       synchronized to the local PWM time base (next start of cycle)*/
    PG7IOCON2bits.OSYNC = 0;
    /* Data for PWM7H/PWM7L Pins if FLT Event is Active bits
       If Fault is active, then FLTDAT<1> provides data for PWM7H.
       If Fault is active, then FLTDAT<0> provides data for PWM7L.*/
    PG7IOCON2bits.FLT1DAT = 0;
    /* Data for PWM7H/PWM7L Pins if CLMT Event is Active bits
       If current limit is active, then CLDAT<1> provides data for PWM7H.
       If current limit is active, then CLDAT<0> provides data for PWM7L.*/
    PG7IOCON2bits.CLDAT = 0;
    /* Data for PWM7H/PWM7L Pins if Feed-Forward Event is Active bits
       If feed-forward is active, then FFDAT<1> provides data for PWM7H.
       If feed-forward is active, then FFDAT<0> provides data for PWM7L.*/
    PG7IOCON2bits.FFDAT = 0;
    /* Data for PWM7H/PWM7L Pins if Debug Mode is Active and PTFRZ = 1 bits
       If Debug mode is active and PTFRZ=1,then DBDAT<1> provides PWM7H data.
       If Debug mode is active and PTFRZ=1,then DBDAT<0> provides PWM7L data. */
    PG7IOCON2bits.DBDAT = 0;
    
    /* Initialize PWM GENERATOR 7 I/O CONTROL REGISTER */    

    /* Time Base Capture Source Selection bits
       000 = No hardware source selected for time base capture ? software only*/
    PG7IOCON1bits.CAPSRC = 0;
    /* Dead-Time Compensation Select bit 
       0 = Dead-time compensation is controlled by PCI Sync logic */
    PG7IOCON1bits.DTCMPSEL = 0;
    /* PWM Generator Output Mode Selection bits
       00 = PWM Generator outputs operate in Complementary mode*/
    PG7IOCON1bits.PMOD = 0;
    /* PWM7H Output Port Enable bit
       1 = PWM Generator controls the PWM7H output pin
       0 = PWM Generator does not control the PWM7H output pin */
    PG7IOCON1bits.PENH = 1;
    /* PWM7L Output Port Enable bit
       1 = PWM Generator controls the PWM7L output pin
       0 = PWM Generator does not control the PWM7L output pin */
    PG7IOCON1bits.PENL = 1;
    /* PWM7H Output Polarity bit
       1 = Output pin is active-low
       0 = Output pin is active-high*/
    PG7IOCON1bits.POLH = 0;
    /* PWM7L Output Polarity bit
       1 = Output pin is active-low
       0 = Output pin is active-high*/
    PG7IOCON1bits.POLL = 0;
    /* Peripheral Pin Select Enable bit
       1 = Peripheral pin select enabled
       0 = Peripheral pin select disabled, as a result, PWM outputs are hard-mapped to pins*/
    PG7IOCON1bits.PPSEN = 1;
    
    /* Initialize PWM GENERATOR 7 EVENT REGISTER */
    PG7EVT1      = 0x0000;
    /* ADC Trigger 1 Post-scaler Selection bits
       00000 = 1:1 */
    PG7EVT1bits.ADTR1PS = 0;
    /* ADC Trigger 1 Source is PG7TRIGC Compare Event Enable bit
       0 = PG7TRIGC register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    PG7EVT1bits.ADTR1EN3  = 0;
    /* ADC Trigger 1 Source is PG7TRIGB Compare Event Enable bit
       0 = PG7TRIGB register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    PG7EVT1bits.ADTR1EN2 = 0;    
    /* ADC Trigger 1 Source is PG7TRIGA Compare Event Enable bit
       0 = PG7TRIGA register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    PG7EVT1bits.ADTR1EN1 = 0;
    /* Update Trigger Select bits
       01 = A write of the PG7DC register automatically sets the UPDATE bit*/
    PG7EVT1bits.UPDTRG = 1;
    /* PWM Generator Trigger Output Selection bits
       000 = EOC event is the PWM Generator trigger*/
    PG7EVT1bits.PGTRGSEL = 0;
    
    /* Initialize PWM GENERATOR 7 EVENT REGISTER */
    /* FLTIEN: PCI Fault Interrupt Enable bit
       1 = Fault interrupt is enabled */
    PG7EVT1bits.FLT1IEN = 1;
    /* PCI Current Limit Interrupt Enable bit
       0 = Current limit interrupt is disabled */
    PG7EVT1bits.CLIEN = 0;
    /* PCI Feed-Forward Interrupt Enable bit
       0 = Feed-forward interrupt is disabled */
    PG7EVT1bits.FFIEN = 0;
    /* PCI Sync Interrupt Enable bit
       0 = Sync interrupt is disabled */
    PG7EVT1bits.SIEN = 0;
    /* Interrupt Event Selection bits
       00 = Interrupts CPU at EOC
       01 = Interrupts CPU at TRIGA compare event
       10 = Interrupts CPU at ADC Trigger 1 event
       11 = Time base interrupts are disabled */
    PG7EVT1bits.IEVTSEL = 3;

        /* ADC Trigger 2 Source is PG7TRIGC Compare Event Enable bit
       0 = PG7TRIGC register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    PG7EVT2bits.ADTR2EN3 = 0;
    /* ADC Trigger 2 Source is PG7TRIGB Compare Event Enable bit
       0 = PG7TRIGB register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    PG7EVT2bits.ADTR2EN2 = 0;
    /* ADC Trigger 2 Source is PG7TRIGA Compare Event Enable bit
       0 = PG7TRIGA register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    PG7EVT2bits.ADTR2EN1 = 0;
    /* ADC Trigger 1 Offset Selection bits
       00000 = No offset */
    PG7EVT1bits.ADTR1OFS = 0;
    
#ifndef ENABLE_PWM_FAULT_PCI_MC2
    /* PWM GENERATOR 7 Fault PCI REGISTER */
    PG7F1PCI1     = 0x0000;
#else
       /* PWM GENERATOR 7 Fault PCI REGISTER */
    PG7F1PCI1     = 0x0000;
    /* Termination Synchronization Disable bit
       1 = Termination of latched PCI occurs immediately
       0 = Termination of latched PCI occurs at PWM EOC */
    PG7F1PCI1bits.TSYNCDIS = 0;
    /* Termination Event Selection bits
       001 = Auto-Terminate: Terminate when PCI source transitions from 
             active to inactive */
    PG7F1PCI1bits.TERM = 1;
    /* Acceptance Qualifier Polarity Select bit: 0 = Not inverted 1 = Inverted*/
    PG7F1PCI1bits.AQPS = 0;
    /* Acceptance Qualifier Source Selection bits
       111 = SWPCI control bit only (qualifier forced to 0)
       110 = Selects PCI Source #9
       101 = Selects PCI Source #8
       100 = Selects PCI Source #1 (PWM Generator output selected by the PWMPCI<2:0> bits)
       011 = PWM Generator is triggered
       010 = LEB is active
       001 = Duty cycle is active (base PWM Generator signal)        
       000 = No acceptance qualifier is used (qualifier forced to 1) */
    PG7F1PCI1bits.AQSS = 0;
    /* PCI Synchronization Control bit
       1 = PCI source is synchronized to PWM EOC
       0 = PCI source is not synchronized to PWM EOC*/
    PG7F1PCI1bits.PSYNC = 0;
    /* PCI Polarity Select bit 0 = Not inverted 1 = Inverted */
    PG7F1PCI1bits.PPS = 1;
    /* PCI Source Selection bits
       31 : CLC1 output
       30 : Comparator 3 output(0x40000000)
       ? ?
       8 : PPS PCI8R in RPINR20 register input(0x0100)
       ? ?
       1 : Internally connected to the output of PWMPCI[2:0] bits in PGxEVT1 registers
       0 : Tied to '0' */
    PG7F1PCI2 = 0x0100;
    /* PCI Bypass Enable bit
       0 = PCI function is not bypassed */
    PG7F1PCI1bits.BPEN   = 0;
    /* PCI Bypass Source Selection bits(1)
       000 = PCI control is sourced from PG7 PCI logic when BPEN = 1 */
    PG7F1PCI1bits.BPSEL   = 0;
    /* PCI Termination Polarity Select bits 
       0 = Inverter, 1 = Non Inverted */
    PG7F1PCI1bits.TERMPS = 0;
    /* PCI Acceptance Criteria Selection bits
       101 = Latched any edge(2)
       100 = Latched rising edge
       011 = Latched
       010 = Any edge
       001 = Rising edge
       000 = Level-sensitive*/
    PG7F1PCI1bits.ACP   = 3;
    /* PCI SR Latch Mode bit
       1 = SR latch is Reset-dominant in Latched Acceptance modes
       0 = SR latch is Set-dominant in Latched Acceptance modes*/
    PG7F1PCI1bits.LATMOD  = 0;
    /* Termination Qualifier Polarity Select bit 1 = Inverted 0 = Not inverted*/
    PG7F1PCI1bits.TQPS   = 0;
    /* Termination Qualifier Source Selection bits
       111 = SWPCI control bit only (qualifier forced to '0')(3)
       110 = Selects PCI Source #9 (pwm_pci[9] input port)
       101 = Selects PCI Source #8 (pwm_pci[8] input port)
       100 = Selects PCI Source #1 (PWM Generator output selected by the PWMPCI<2:0> bits)
       011 = PWM Generator is triggered
       010 = LEB is active
       001 = Duty cycle is active (base PWM Generator signal)
       000 = No termination qualifier used (qualifier forced to '1')(3)*/
    PG7F1PCI1bits.TQSS  = 0;
#endif    

    /* PWM GENERATOR 7 Current Limit PCI REGISTER */
    PG7CLPCI1    = 0x0000;
    /* PWM GENERATOR 7 Feed Forward PCI REGISTER  */
    PG7FFPCI1    = 0x0000;

    /* PWM GENERATOR 7 Sync PCI REGISTER */
    PG7SPCI1    = 0x0000;
    /* PCI Polarity Select bit 
       1 = Inverted
       0 = Not inverted*/
    PG7SPCI1bits.PPS = 0;
    /* PWM GENERATOR 7 Sync PCI Source 
       0 = Tied to ?0? */
    PG7SPCI2 = 0x0000;
    
    /* Initialize PWM GENERATOR 7 LEADING-EDGE BLANKING REGISTER */
    PG7LEB      = 0x0000;
    
    /* Initialize PWM GENERATOR 7 PHASE REGISTER */
    PG7PHASEbits.PHASE     = MC2_MIN_DUTY;
    /* Initialize PWM GENERATOR 7 DUTY CYCLE REGISTER */
    PG7DCbits.DC           = MC2_MIN_DUTY;
    /* Initialize PWM GENERATOR 7 DUTY CYCLE ADJUSTMENT REGISTER */
    PG7DCA       = 0x0000;
    /* Initialize PWM GENERATOR 7 PERIOD REGISTER */
    PG7PER       = MC2_LOOPTIME_TCY;
    /* Initialize PWM GENERATOR 7 DEAD-TIME REGISTER */
    PG7DTbits.DTH = MC2_DEADTIME;
    /* Initialize PWM GENERATOR 7 DEAD-TIME REGISTER */
    PG7DTbits.DTL = MC2_DEADTIME;

    /* Initialize PWM GENERATOR 7 TRIGGER A REGISTER */
    PG7TRIGA     = 0x0000;
    /* Initialize PWM GENERATOR 7 TRIGGER B REGISTER */
    PG7TRIGB     = 0x0000;
    /* Initialize PWM GENERATOR 7 TRIGGER C REGISTER */
    PG7TRIGC     = 0x0000;
    
}

/**
* <B> Function: InitPWMGenerator8()    </B>
*
* @brief Function to configure PWM Module # 8
*        
* @param none.
* @return none.
* 
* @example
* <CODE> InitPWMGenerator8();     </CODE>
*
*/
void InitPWMGenerator8 (void)
{

    /* Initialize PWM GENERATOR 8 CONTROL REGISTER */
    PG8CON      = 0x0000;
    /* PWM Generator 8 Enable bit : 1 = Is enabled, 0 = Is not enabled */
    /* Ensuring PWM Generator is disabled prior to configuring module */
    PG8CONbits.ON = 0;
    /* Clock Selection bits
       0b01 = Macro uses Master clock selected by the PCLKCON.MCLKSEL bits*/
    PG8CONbits.CLKSEL = 1;
    /* PWM Mode Selection bits
     * 110 = Dual Edge Center-Aligned PWM mode (interrupt/register update once per cycle)
       100 = Center-Aligned PWM mode(interrupt/register update once per cycle)*/
    PG8CONbits.MODSEL = 4;
    /* Trigger Count Select bits
       000 = PWM Generator produces 1 PWM cycle after triggered */
    PG8CONbits.TRGCNT = 0;
    
    /* Initialize PWM GENERATOR 8 CONTROL REGISTER */
    /* Master Duty Cycle Register Select bit
       1 = Macro uses the MDC register instead of PG8DC
       0 = Macro uses the PG8DC register*/
    PG8CONbits.MDCSEL = 0;
    /* Master Period Register Select bit
       1 = Macro uses the MPER register instead of PG8PER
       0 = Macro uses the PG8PER register */
    PG8CONbits.MPERSEL = 0;
    /* MPHSEL: Master Phase Register Select bit
       1 = Macro uses the MPHASE register instead of PG8PHASE
       0 = Macro uses the PG8PHASE register */
    PG8CONbits.MPHSEL = 0;
    /* Master Update Enable bit
       1 = PWM Generator broadcasts software set/clear of UPDATE status bit and 
           EOC signal to other PWM Generators
       0 = PWM Generator does not broadcast UPDATE status bit or EOC signal */
    PG8CONbits.MSTEN = 0;
    /* PWM Buffer Update Mode Selection bits 
       Update Data registers at start of next PWM cycle if UPDATE = 1. */
    PG8CONbits.UPDMOD = 0;
    /* PWM Generator Trigger Mode Selection bits
       0b00 = PWM Generator operates in Single Trigger mode
       0b01 = PWM Generator operates in Retriggerable mode */
    PG8CONbits.TRGMOD = 1;
    /* Start of Cycle Selection bits
       1111 = TRIG bit or PCI Sync function only (no hardware trigger source is selected)
       0000 = Local EOC*/
    /* For PWM Synchronization
     PWM Generator 6 is in sync with PWM Generator 5*/
    PG8CONbits.SOCS = 0b0101;
    
    /* Clear PWM GENERATOR 8 STATUS REGISTER*/
    PG8STAT      = 0x0000;
    /* Initialize PWM GENERATOR 8 I/O CONTROL REGISTER */
    PG8IOCON2    = 0x0000;

    /* Current Limit Mode Select bit
       0 = If PCI current limit is active, then the CLDAT<1:0> bits define 
       the PWM output levels */
    PG8IOCON2bits.CLMOD = 0;
    /* Swap PWM Signals to PWM8H and PWM8L Device Pins bit 
       0 = PWM8H/L signals are mapped to their respective pins */
    PG8IOCON1bits.SWAP = 0;
    /* User Override Enable for PWM8H Pin bit
       0 = PWM Generator provides data for the PWM8H pin*/
    PG8IOCON2bits.OVRENH = 0;
    /* User Override Enable for PWM8L Pin bit
       0 = PWM Generator provides data for the PWM8L pin*/
    PG8IOCON2bits.OVRENL = 0;
    /* Data for PWM8H/PWM8L Pins if Override is Enabled bits
       If OVERENH = 1, then OVRDAT<1> provides data for PWM8H.
       If OVERENL = 1, then OVRDAT<0> provides data for PWM8L */
    PG8IOCON2bits.OVRDAT = 0;
    /* User Output Override Synchronization Control bits
       00 = User output overrides via the OVRENL/H and OVRDAT<1:0> bits are 
       synchronized to the local PWM time base (next start of cycle)*/
    PG8IOCON2bits.OSYNC = 0;
    /* Data for PWM8H/PWM8L Pins if FLT Event is Active bits
       If Fault is active, then FLTDAT<1> provides data for PWM8H.
       If Fault is active, then FLTDAT<0> provides data for PWM8L.*/
    PG8IOCON2bits.FLT1DAT = 0;
    /* Data for PWM8H/PWM8L Pins if CLMT Event is Active bits
       If current limit is active, then CLDAT<1> provides data for PWM8H.
       If current limit is active, then CLDAT<0> provides data for PWM8L.*/
    PG8IOCON2bits.CLDAT = 0;
    /* Data for PWM8H/PWM8L Pins if Feed-Forward Event is Active bits
       If feed-forward is active, then FFDAT<1> provides data for PWM8H.
       If feed-forward is active, then FFDAT<0> provides data for PWM8L.*/
    PG8IOCON2bits.FFDAT = 0;
    /* Data for PWM8H/PWM8L Pins if Debug Mode is Active and PTFRZ = 1 bits
       If Debug mode is active and PTFRZ=1,then DBDAT<1> provides PWM8H data.
       If Debug mode is active and PTFRZ=1,then DBDAT<0> provides PWM8L data. */
    PG8IOCON2bits.DBDAT = 0;
    
    /* Initialize PWM GENERATOR 8 I/O CONTROL REGISTER */    

    /* Time Base Capture Source Selection bits
       000 = No hardware source selected for time base capture ? software only*/
    PG8IOCON1bits.CAPSRC = 0;
    /* Dead-Time Compensation Select bit 
       0 = Dead-time compensation is controlled by PCI Sync logic */
    PG8IOCON1bits.DTCMPSEL = 0;
    /* PWM Generator Output Mode Selection bits
       00 = PWM Generator outputs operate in Complementary mode*/
    PG8IOCON1bits.PMOD = 0;
    /* PWM8H Output Port Enable bit
       1 = PWM Generator controls the PWM8H output pin
       0 = PWM Generator does not control the PWM8H output pin */
    PG8IOCON1bits.PENH = 1;
    /* PWM8L Output Port Enable bit
       1 = PWM Generator controls the PWM8L output pin
       0 = PWM Generator does not control the PWM8L output pin */
    PG8IOCON1bits.PENL = 1;
    /* PWM8H Output Polarity bit
       1 = Output pin is active-low
       0 = Output pin is active-high*/
    PG8IOCON1bits.POLH = 0;
    /* PWM8L Output Polarity bit
       1 = Output pin is active-low
       0 = Output pin is active-high*/
    PG8IOCON1bits.POLL = 0;
    /* Peripheral Pin Select Enable bit
       1 = Peripheral pin select enabled
       0 = Peripheral pin select disabled, as a result, PWM outputs are hard-mapped to pins*/
    PG8IOCON1bits.PPSEN = 0;
    
    /* Initialize PWM GENERATOR 8 EVENT REGISTER */
    PG8EVT1      = 0x0000;
    /* ADC Trigger 1 Post-scaler Selection bits
       00000 = 1:1 */
    PG8EVT1bits.ADTR1PS = 0;
    /* ADC Trigger 1 Source is PG8TRIGC Compare Event Enable bit
       0 = PG8TRIGC register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    PG8EVT1bits.ADTR1EN3  = 0;
    /* ADC Trigger 1 Source is PG8TRIGB Compare Event Enable bit
       0 = PG8TRIGB register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    PG8EVT1bits.ADTR1EN2 = 0;    
    /* ADC Trigger 1 Source is PG8TRIGA Compare Event Enable bit
       0 = PG8TRIGA register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    PG8EVT1bits.ADTR1EN1 = 0;
    /* Update Trigger Select bits
       01 = A write of the PG8DC register automatically sets the UPDATE bit*/
    PG8EVT1bits.UPDTRG = 1;
    /* PWM Generator Trigger Output Selection bits
       000 = EOC event is the PWM Generator trigger*/
    PG8EVT1bits.PGTRGSEL = 0;
    
    /* Initialize PWM GENERATOR 8 EVENT REGISTER */
    /* FLTIEN: PCI Fault Interrupt Enable bit
       1 = Fault interrupt is enabled */
    PG8EVT1bits.FLT1IEN = 1;
    /* PCI Current Limit Interrupt Enable bit
       0 = Current limit interrupt is disabled */
    PG8EVT1bits.CLIEN = 0;
    /* PCI Feed-Forward Interrupt Enable bit
       0 = Feed-forward interrupt is disabled */
    PG8EVT1bits.FFIEN = 0;
    /* PCI Sync Interrupt Enable bit
       0 = Sync interrupt is disabled */
    PG8EVT1bits.SIEN = 0;
    /* Interrupt Event Selection bits
       00 = Interrupts CPU at EOC
       01 = Interrupts CPU at TRIGA compare event
       10 = Interrupts CPU at ADC Trigger 1 event
       11 = Time base interrupts are disabled */
    PG8EVT1bits.IEVTSEL = 3;

        /* ADC Trigger 2 Source is PG8TRIGC Compare Event Enable bit
       0 = PG8TRIGC register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    PG8EVT2bits.ADTR2EN3 = 0;
    /* ADC Trigger 2 Source is PG8TRIGB Compare Event Enable bit
       0 = PG8TRIGB register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    PG8EVT2bits.ADTR2EN2 = 0;
    /* ADC Trigger 2 Source is PG8TRIGA Compare Event Enable bit
       0 = PG8TRIGA register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    PG8EVT2bits.ADTR2EN1 = 0;
    /* ADC Trigger 1 Offset Selection bits
       00000 = No offset */
    PG8EVT1bits.ADTR1OFS = 0;
    
#ifndef ENABLE_PWM_FAULT_PCI_MC2
    /* PWM GENERATOR 8 Fault PCI REGISTER */
    PG8F1PCI1     = 0x0000;
#else
       /* PWM GENERATOR 8 Fault PCI REGISTER */
    PG8F1PCI1     = 0x0000;
    /* Termination Synchronization Disable bit
       1 = Termination of latched PCI occurs immediately
       0 = Termination of latched PCI occurs at PWM EOC */
    PG8F1PCI1bits.TSYNCDIS = 0;
    /* Termination Event Selection bits
       001 = Auto-Terminate: Terminate when PCI source transitions from 
             active to inactive */
    PG8F1PCI1bits.TERM = 1;
    /* Acceptance Qualifier Polarity Select bit: 0 = Not inverted 1 = Inverted*/
    PG8F1PCI1bits.AQPS = 0;
    /* Acceptance Qualifier Source Selection bits
       111 = SWPCI control bit only (qualifier forced to 0)
       110 = Selects PCI Source #9
       101 = Selects PCI Source #8
       100 = Selects PCI Source #1 (PWM Generator output selected by the PWMPCI<2:0> bits)
       011 = PWM Generator is triggered
       010 = LEB is active
       001 = Duty cycle is active (base PWM Generator signal)        
       000 = No acceptance qualifier is used (qualifier forced to 1) */
    PG8F1PCI1bits.AQSS = 0;
    /* PCI Synchronization Control bit
       1 = PCI source is synchronized to PWM EOC
       0 = PCI source is not synchronized to PWM EOC*/
    PG8F1PCI1bits.PSYNC = 0;
    /* PCI Polarity Select bit 0 = Not inverted 1 = Inverted */
    PG8F1PCI1bits.PPS = 1;
    /* PCI Source Selection bits
       31 : CLC1 output
       30 : Comparator 3 output(0x40000000)
       ? ?
       8 : PPS PCI8R in RPINR20 register input(0x0100)
       ? ?
       1 : Internally connected to the output of PWMPCI[2:0] bits in PGxEVT1 registers
       0 : Tied to '0' */
    PG8F1PCI2 = 0x0100;
    /* PCI Bypass Enable bit
       0 = PCI function is not bypassed */
    PG8F1PCI1bits.BPEN   = 0;
    /* PCI Bypass Source Selection bits(1)
       000 = PCI control is sourced from PG8 PCI logic when BPEN = 1 */
    PG8F1PCI1bits.BPSEL   = 0;
    /* PCI Termination Polarity Select bits 
       0 = Inverter, 1 = Non Inverted */
    PG8F1PCI1bits.TERMPS = 0;
    /* PCI Acceptance Criteria Selection bits
       101 = Latched any edge(2)
       100 = Latched rising edge
       011 = Latched
       010 = Any edge
       001 = Rising edge
       000 = Level-sensitive*/
    PG8F1PCI1bits.ACP   = 3;
    /* PCI SR Latch Mode bit
       1 = SR latch is Reset-dominant in Latched Acceptance modes
       0 = SR latch is Set-dominant in Latched Acceptance modes*/
    PG8F1PCI1bits.LATMOD  = 0;
    /* Termination Qualifier Polarity Select bit 1 = Inverted 0 = Not inverted*/
    PG8F1PCI1bits.TQPS   = 0;
    /* Termination Qualifier Source Selection bits
       111 = SWPCI control bit only (qualifier forced to '0')(3)
       110 = Selects PCI Source #9 (pwm_pci[9] input port)
       101 = Selects PCI Source #8 (pwm_pci[8] input port)
       100 = Selects PCI Source #1 (PWM Generator output selected by the PWMPCI<2:0> bits)
       011 = PWM Generator is triggered
       010 = LEB is active
       001 = Duty cycle is active (base PWM Generator signal)
       000 = No termination qualifier used (qualifier forced to '1')(3)*/
    PG8F1PCI1bits.TQSS  = 0;
#endif    

    /* PWM GENERATOR 8 Current Limit PCI REGISTER */
    PG8CLPCI1    = 0x0000;
    /* PWM GENERATOR 8 Feed Forward PCI REGISTER  */
    PG8FFPCI1    = 0x0000;

    /* PWM GENERATOR 8 Sync PCI REGISTER */
    PG8SPCI1    = 0x0000;
    /* PCI Polarity Select bit 
       1 = Inverted
       0 = Not inverted*/
    PG8SPCI1bits.PPS = 0;
    /* PWM GENERATOR 8 Sync PCI Source 
       0 = Tied to ?0? */
    PG8SPCI2 = 0x0000;
    
    /* Initialize PWM GENERATOR 8 LEADING-EDGE BLANKING REGISTER */
    PG8LEB      = 0x0000;
    
    /* Initialize PWM GENERATOR 8 PHASE REGISTER */
    PG8PHASEbits.PHASE     = MC2_MIN_DUTY;
    /* Initialize PWM GENERATOR 8 DUTY CYCLE REGISTER */
    PG8DCbits.DC           = MC2_MIN_DUTY;
    /* Initialize PWM GENERATOR 8 DUTY CYCLE ADJUSTMENT REGISTER */
    PG8DCA       = 0x0000;
    /* Initialize PWM GENERATOR 8 PERIOD REGISTER */
    PG8PER       = MC2_LOOPTIME_TCY;
    /* Initialize PWM GENERATOR 8 DEAD-TIME REGISTER */
    PG8DTbits.DTH = MC2_DEADTIME;
    /* Initialize PWM GENERATOR 8 DEAD-TIME REGISTER */
    PG8DTbits.DTL = MC2_DEADTIME;

    /* Initialize PWM GENERATOR 8 TRIGGER A REGISTER */
    PG8TRIGA     = 0x0000;
    /* Initialize PWM GENERATOR 8 TRIGGER B REGISTER */
    PG8TRIGB     = 0x0000;
    /* Initialize PWM GENERATOR 8 TRIGGER C REGISTER */
    PG8TRIGC     = 0x0000;
    
}

/**
* <B> Function: InitAuxPWMGenerator1()    </B>
*
* @brief Function to configure Aux PWM Module # 1
*        
* @param none.
* @return none.
* 
* @example
* <CODE> InitAuxPWMGenerator1();     </CODE>
*
*/
void InitAuxPWMGenerator1 (void)
{

    /* Initialize AUX PWM GENERATOR 1 CONTROL REGISTER */
    APG1CON      = 0x0000;
    /* Aux PWM Generator 1 Enable bit : 1 = Is enabled, 0 = Is not enabled */
    /* Ensuring Aux PWM Generator is disabled prior to configuring module */
    APG1CONbits.ON = 0;
    /* Clock Selection bits
       0b01 = Macro uses Master clock selected by the PCLKCON.MCLKSEL bits*/
    APG1CONbits.CLKSEL = 1;
    /* PWM Mode Selection bits
     * 110 = Dual Edge Center-Aligned PWM mode (interrupt/register update once per cycle)
       100 = Center-Aligned PWM mode(interrupt/register update once per cycle)*/
    APG1CONbits.MODSEL = 4;
    /* Trigger Count Select bits
       000 = Aux PWM Generator produces 1 PWM cycle after triggered */
    APG1CONbits.TRGCNT = 0;
    
    /* Initialize AUX PWM GENERATOR 1 CONTROL REGISTER */
    /* Master Duty Cycle Register Select bit
       1 = Macro uses the MDC register instead of APG1DC
       0 = Macro uses the APG1DC register*/
    APG1CONbits.MDCSEL = 0;
    /* Master Period Register Select bit
       1 = Macro uses the MPER register instead of APG1PER
       0 = Macro uses the APG1PER register */
    APG1CONbits.MPERSEL = 0;
    /* MPHSEL: Master Phase Register Select bit
       1 = Macro uses the MPHASE register instead of APG1PHASE
       0 = Macro uses the APG1PHASE register */
    APG1CONbits.MPHSEL = 0;
    /* Master Update Enable bit
       1 = Aux PWM Generator broadcasts software set/clear of UPDATE status bit and 
           EOC signal to other Aux PWM Generators
       0 = Aux PWM Generator does not broadcast UPDATE status bit or EOC signal */
    APG1CONbits.MSTEN = 0;
    /* PWM Buffer Update Mode Selection bits 
       Update Data registers at start of next PWM cycle if UPDATE = 1. */
    APG1CONbits.UPDMOD = 0;
    /* Aux PWM Generator Trigger Mode Selection bits
       0b00 = Aux PWM Generator operates in Single Trigger mode
       0b01 = Aux PWM Generator operates in Retriggerable mode */
    APG1CONbits.TRGMOD = 1;
    /* Start of Cycle Selection bits
       1111 = TRIG bit or PCI Sync function only (no hardware trigger source is selected)
       0000 = Local EOC*/
    APG1CONbits.SOCS = 0b1111;
    
    /* Clear AUX PWM GENERATOR 1 STATUS REGISTER*/
    APG1STAT      = 0x0000;
    /* Initialize AUX PWM GENERATOR 1 I/O CONTROL REGISTER */
    APG1IOCON2    = 0x0000;

    /* Current Limit Mode Select bit
       0 = If PCI current limit is active, then the CLDAT<1:0> bits define 
       the PWM output levels */
    APG1IOCON2bits.CLMOD = 0;
    /* Swap PWM Signals to APWM1H and APWM1L Device Pins bit 
       0 = APWM1H/L signals are mapped to their respective pins */
    APG1IOCON1bits.SWAP = 0;
    /* User Override Enable for APWM1H Pin bit
       0 = Aux PWM Generator provides data for the APWM1H pin*/
    APG1IOCON2bits.OVRENH = 0;
    /* User Override Enable for APWM1L Pin bit
       0 = Aux PWM Generator provides data for the APWM1L pin*/
    APG1IOCON2bits.OVRENL = 0;
    /* Data for APWM1H/APWM1L Pins if Override is Enabled bits
       If OVERENH = 1, then OVRDAT<1> provides data for APWM1H.
       If OVERENL = 1, then OVRDAT<0> provides data for APWM1L */
    APG1IOCON2bits.OVRDAT = 0;
    /* User Output Override Synchronization Control bits
       00 = User output overrides via the OVRENL/H and OVRDAT<1:0> bits are 
       synchronized to the local PWM time base (next start of cycle)*/
    APG1IOCON2bits.OSYNC = 0;
    /* Data for APWM1H/APWM1L Pins if FLT Event is Active bits
       If Fault is active, then FLTDAT<1> provides data for APWM1H.
       If Fault is active, then FLTDAT<0> provides data for APWM1L.*/
    APG1IOCON2bits.FLT1DAT = 0;
    /* Data for APWM1H/APWM1L Pins if CLMT Event is Active bits
       If current limit is active, then CLDAT<1> provides data for APWM1H.
       If current limit is active, then CLDAT<0> provides data for APWM1L.*/
    APG1IOCON2bits.CLDAT = 0;
    /* Data for APWM1H/APWM1L Pins if Feed-Forward Event is Active bits
       If feed-forward is active, then FFDAT<1> provides data for APWM1H.
       If feed-forward is active, then FFDAT<0> provides data for APWM1L.*/
    APG1IOCON2bits.FFDAT = 0;
    /* Data for APWM1H/APWM1L Pins if Debug Mode is Active and PTFRZ = 1 bits
       If Debug mode is active and PTFRZ=1,then DBDAT<1> provides APWM1H data.
       If Debug mode is active and PTFRZ=1,then DBDAT<0> provides APWM1L data. */
    APG1IOCON2bits.DBDAT = 0;
    
    /* Initialize AUX PWM GENERATOR 1 I/O CONTROL REGISTER */    

    /* Time Base Capture Source Selection bits
       000 = No hardware source selected for time base capture ? software only*/
    APG1IOCON1bits.CAPSRC = 0;
    /* Dead-Time Compensation Select bit 
       0 = Dead-time compensation is controlled by PCI Sync logic */
    APG1IOCON1bits.DTCMPSEL = 0;
    /* Aux PWM Generator Output Mode Selection bits
       00 = Aux PWM Generator outputs operate in Complementary mode*/
    APG1IOCON1bits.PMOD = 0;
    /* APWM1H Output Port Enable bit
       1 = Aux PWM Generator controls the APWM1H output pin
       0 = Aux PWM Generator does not control the APWM1H output pin */
    APG1IOCON1bits.PENH = 1;
    /* APWM1L Output Port Enable bit
       1 = Aux PWM Generator controls the APWM1L output pin
       0 = Aux PWM Generator does not control the APWM1L output pin */
    APG1IOCON1bits.PENL = 1;
    /* APWM1H Output Polarity bit
       1 = Output pin is active-low
       0 = Output pin is active-high*/
    APG1IOCON1bits.POLH = 0;
    /* APWM1L Output Polarity bit
       1 = Output pin is active-low
       0 = Output pin is active-high*/
    APG1IOCON1bits.POLL = 0;
    /* Peripheral Pin Select Enable bit
       1 = Peripheral pin select enabled
       0 = Peripheral pin select disabled, as a result, PWM outputs are hard-mapped to pins*/
    APG1IOCON1bits.PPSEN = 1;
    
    /* Initialize AUX PWM GENERATOR 1 EVENT REGISTER */
    APG1EVT1      = 0x0000;
    /* ADC Trigger 1 Post-scaler Selection bits
       00000 = 1:1 */
    APG1EVT1bits.ADTR1PS = 0;
    /* ADC Trigger 1 Source is APG1TRIGC Compare Event Enable bit
       0 = APG1TRIGC register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    APG1EVT1bits.ADTR1EN3  = 0;
    /* ADC Trigger 1 Source is APG1TRIGB Compare Event Enable bit
       0 = APG1TRIGB register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    APG1EVT1bits.ADTR1EN2 = 0;    
    /* ADC Trigger 1 Source is APG1TRIGA Compare Event Enable bit
       1 = APG1TRIGA register compare event is enabled as trigger source for 
           ADC Trigger 1 */
    APG1EVT1bits.ADTR1EN1 = 1;
    /* Update Trigger Select bits
       01 = A write of the APG1DC register automatically sets the UPDATE bit*/
    APG1EVT1bits.UPDTRG = 1;
    /* Aux PWM Generator Trigger Output Selection bits
       000 = EOC event is the Aux PWM Generator trigger*/
    APG1EVT1bits.PGTRGSEL = 0;
    
    /* Initialize AUX PWM GENERATOR 1 EVENT REGISTER */
    /* FLTIEN: PCI Fault Interrupt Enable bit
       1 = Fault interrupt is enabled */
    APG1EVT1bits.FLT1IEN = 1;
    /* PCI Current Limit Interrupt Enable bit
       0 = Current limit interrupt is disabled */
    APG1EVT1bits.CLIEN = 0;
    /* PCI Feed-Forward Interrupt Enable bit
       0 = Feed-forward interrupt is disabled */
    APG1EVT1bits.FFIEN = 0;
    /* PCI Sync Interrupt Enable bit
       0 = Sync interrupt is disabled */
    APG1EVT1bits.SIEN = 0;
    /* Interrupt Event Selection bits
       00 = Interrupts CPU at EOC
       01 = Interrupts CPU at TRIGA compare event
       10 = Interrupts CPU at ADC Trigger 1 event
       11 = Time base interrupts are disabled */
    APG1EVT1bits.IEVTSEL = 3;

        /* ADC Trigger 2 Source is APG1TRIGC Compare Event Enable bit
       0 = APG1TRIGC register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    APG1EVT2bits.ADTR2EN3 = 0;
    /* ADC Trigger 2 Source is APG1TRIGB Compare Event Enable bit
       0 = APG1TRIGB register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    APG1EVT2bits.ADTR2EN2 = 0;
    /* ADC Trigger 2 Source is APG1TRIGA Compare Event Enable bit
       0 = APG1TRIGA register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    APG1EVT2bits.ADTR2EN1 = 0;
    /* ADC Trigger 1 Offset Selection bits
       00000 = No offset */
    APG1EVT1bits.ADTR1OFS = 0;
    
#ifndef ENABLE_PWM_FAULT_PCI_MC3
    /* AUX PWM GENERATOR 1 Fault PCI REGISTER */
    APG1F1PCI1     = 0x0000;
#else
       /* AUX PWM GENERATOR 1 Fault PCI REGISTER */
    APG1F1PCI1     = 0x0000;
    /* Termination Synchronization Disable bit
       1 = Termination of latched PCI occurs immediately
       0 = Termination of latched PCI occurs at PWM EOC */
    APG1F1PCI1bits.TSYNCDIS = 0;
    /* Termination Event Selection bits
       001 = Auto-Terminate: Terminate when PCI source transitions from 
             active to inactive */
    APG1F1PCI1bits.TERM = 1;
    /* Acceptance Qualifier Polarity Select bit: 0 = Not inverted 1 = Inverted*/
    APG1F1PCI1bits.AQPS = 0;
    /* Acceptance Qualifier Source Selection bits
       111 = SWPCI control bit only (qualifier forced to 0)
       110 = Selects PCI Source #9
       101 = Selects PCI Source #8
       100 = Selects PCI Source #1 (Aux PWM Generator output selected by the PWMPCI<2:0> bits)
       011 = Aux PWM Generator is triggered
       010 = LEB is active
       001 = Duty cycle is active (base Aux PWM Generator signal)        
       000 = No acceptance qualifier is used (qualifier forced to 1) */
    APG1F1PCI1bits.AQSS = 0;
    /* PCI Synchronization Control bit
       1 = PCI source is synchronized to PWM EOC
       0 = PCI source is not synchronized to PWM EOC*/
    APG1F1PCI1bits.PSYNC = 0;
    /* PCI Polarity Select bit 0 = Not inverted 1 = Inverted */
    APG1F1PCI1bits.PPS = 1;
    /* PCI Source Selection bits
       31 : CLC1 output
       30 : Comparator 3 output(0x40000000)
       ? ?
       8 : PPS PCI8R in RPINR20 register input(0x0100)
       ? ?
       1 : Internally connected to the output of PWMPCI[2:0] bits in PGxEVT1 registers
       0 : Tied to '0' */
    APG1F1PCI2 = 0x0100;
    /* PCI Bypass Enable bit
       0 = PCI function is not bypassed */
    APG1F1PCI1bits.BPEN   = 0;
    /* PCI Bypass Source Selection bits(1)
       000 = PCI control is sourced from APG1 PCI logic when BPEN = 1 */
    APG1F1PCI1bits.BPSEL   = 0;
    /* PCI Termination Polarity Select bits 
       0 = Inverter, 1 = Non Inverted */
    APG1F1PCI1bits.TERMPS = 0;
    /* PCI Acceptance Criteria Selection bits
       101 = Latched any edge(2)
       100 = Latched rising edge
       011 = Latched
       010 = Any edge
       001 = Rising edge
       000 = Level-sensitive*/
    APG1F1PCI1bits.ACP   = 3;
    /* PCI SR Latch Mode bit
       1 = SR latch is Reset-dominant in Latched Acceptance modes
       0 = SR latch is Set-dominant in Latched Acceptance modes*/
    APG1F1PCI1bits.LATMOD  = 0;
    /* Termination Qualifier Polarity Select bit 1 = Inverted 0 = Not inverted*/
    APG1F1PCI1bits.TQPS   = 0;
    /* Termination Qualifier Source Selection bits
       111 = SWPCI control bit only (qualifier forced to '0')(3)
       110 = Selects PCI Source #9 (pwm_pci[9] input port)
       101 = Selects PCI Source #8 (pwm_pci[8] input port)
       100 = Selects PCI Source #1 (Aux PWM Generator output selected by the PWMPCI<2:0> bits)
       011 = Aux PWM Generator is triggered
       010 = LEB is active
       001 = Duty cycle is active (base Aux PWM Generator signal)
       000 = No termination qualifier used (qualifier forced to '1')(3)*/
    APG1F1PCI1bits.TQSS  = 0;
#endif    

    /* AUX PWM GENERATOR 1 Current Limit PCI REGISTER */
    APG1CLPCI1    = 0x0000;
    /* AUX PWM GENERATOR 1 Feed Forward PCI REGISTER  */
    APG1FFPCI1    = 0x0000;
    
    /* For PWM Synchronization
     PWM Gen 1 is triggered by the Event A of PWM Gen 5*/
    /* AUX PWM GENERATOR 1 Sync PCI REGISTER */
    APG1SPCI1    = 0x0000;
    /* PCI Polarity Select bit 
       1 = Inverted
       0 = Not inverted*/
    APG1SPCI1bits.PPS = 1;
    /* PWM PCI Source Selection
       100 = PWM Generator #5 output used as PCI signal */
    APG1EVT1bits.PWMPCI = 0b100;
    /* AUX PWM GENERATOR 1 Sync PCI Source 
       23 = PWM event A*/
    APG1SPCI2 = 0x00800000;
    
    /* Initialize AUX PWM GENERATOR 1 LEADING-EDGE BLANKING REGISTER */
    APG1LEB      = 0x0000;
    
    /* Initialize AUX PWM GENERATOR 1 PHASE REGISTER */
    APG1PHASEbits.PHASE     = MC3_MIN_DUTY;
    /* Initialize AUX PWM GENERATOR 1 DUTY CYCLE REGISTER */
    APG1DCbits.DC           = MC3_MIN_DUTY;
    /* Initialize AUX PWM GENERATOR 1 DUTY CYCLE ADJUSTMENT REGISTER */
    APG1DCA       = 0x0000;
    /* Initialize AUX PWM GENERATOR 1 PERIOD REGISTER */
    APG1PERbits.PER       = MC3_LOOPTIME_TCY;
    /* Initialize AUX PWM GENERATOR 1 DEAD-TIME REGISTER */
    APG1DTbits.DTH = MC3_DEADTIME;
    /* Initialize AUX PWM GENERATOR 1 DEAD-TIME REGISTER */
    APG1DTbits.DTL = MC3_DEADTIME;

    /* Initialize AUX PWM GENERATOR 1 TRIGGER A REGISTER */
    APG1TRIGAbits.CAHALF = 0;
    APG1TRIGAbits.TRIGA  = MC3_ADC_SAMPLING_POINT;
    /* Initialize AUX PWM GENERATOR 1 TRIGGER B REGISTER */
    APG1TRIGB     = 0x0000;
    /* Initialize AUX PWM GENERATOR 1 TRIGGER C REGISTER */
    APG1TRIGC     = 0x0000;
    
}

/**
* <B> Function: InitAuxPWMGenerator2()    </B>
*
* @brief Function to configure Aux PWM Module # 2
*        
* @param none.
* @return none.
* 
* @example
* <CODE> InitAuxPWMGenerator2();     </CODE>
*
*/
void InitAuxPWMGenerator2 (void)
{

    /* Initialize AUX PWM GENERATOR 2 CONTROL REGISTER */
    APG2CON      = 0x0000;
    /* Aux PWM Generator 2 Enable bit : 1 = Is enabled, 0 = Is not enabled */
    /* Ensuring Aux PWM Generator is disabled prior to configuring module */
    APG2CONbits.ON = 0;
    /* Clock Selection bits
       0b01 = Macro uses Master clock selected by the PCLKCON.MCLKSEL bits*/
    APG2CONbits.CLKSEL = 1;
    /* PWM Mode Selection bits
     * 110 = Dual Edge Center-Aligned PWM mode (interrupt/register update once per cycle)
       100 = Center-Aligned PWM mode(interrupt/register update once per cycle)*/
    APG2CONbits.MODSEL = 4;
    /* Trigger Count Select bits
       000 = Aux PWM Generator produces 1 PWM cycle after triggered */
    APG2CONbits.TRGCNT = 0;
    
    /* Initialize AUX PWM GENERATOR 2 CONTROL REGISTER */
    /* Master Duty Cycle Register Select bit
       1 = Macro uses the MDC register instead of APG2DC
       0 = Macro uses the APG2DC register*/
    APG2CONbits.MDCSEL = 0;
    /* Master Period Register Select bit
       1 = Macro uses the MPER register instead of APG2PER
       0 = Macro uses the APG2PER register */
    APG2CONbits.MPERSEL = 0;
    /* MPHSEL: Master Phase Register Select bit
       1 = Macro uses the MPHASE register instead of APG2PHASE
       0 = Macro uses the APG2PHASE register */
    APG2CONbits.MPHSEL = 0;
    /* Master Update Enable bit
       1 = Aux PWM Generator broadcasts software set/clear of UPDATE status bit and 
           EOC signal to other Aux PWM Generators
       0 = Aux PWM Generator does not broadcast UPDATE status bit or EOC signal */
    APG2CONbits.MSTEN = 0;
    /* PWM Buffer Update Mode Selection bits 
       Update Data registers at start of next PWM cycle if UPDATE = 1. */
    APG2CONbits.UPDMOD = 0;
    /* Aux PWM Generator Trigger Mode Selection bits
       0b00 = Aux PWM Generator operates in Single Trigger mode
       0b01 = Aux PWM Generator operates in Retriggerable mode */
    APG2CONbits.TRGMOD = 1;
    /* Start of Cycle Selection bits
       1111 = TRIG bit or PCI Sync function only (no hardware trigger source is selected)
       0000 = Local EOC*/
    APG2CONbits.SOCS = 0b1111;
    
    /* Clear AUX PWM GENERATOR 2 STATUS REGISTER*/
    APG2STAT      = 0x0000;
    /* Initialize AUX PWM GENERATOR 2 I/O CONTROL REGISTER */
    APG2IOCON2    = 0x0000;

    /* Current Limit Mode Select bit
       0 = If PCI current limit is active, then the CLDAT<1:0> bits define 
       the PWM output levels */
    APG2IOCON2bits.CLMOD = 0;
    /* Swap PWM Signals to APWM2H and APWM2L Device Pins bit 
       0 = APWM2H/L signals are mapped to their respective pins */
    APG2IOCON1bits.SWAP = 0;
    /* User Override Enable for APWM2H Pin bit
       0 = Aux PWM Generator provides data for the APWM2H pin*/
    APG2IOCON2bits.OVRENH = 0;
    /* User Override Enable for APWM2L Pin bit
       0 = Aux PWM Generator provides data for the APWM2L pin*/
    APG2IOCON2bits.OVRENL = 0;
    /* Data for APWM2H/APWM2L Pins if Override is Enabled bits
       If OVERENH = 1, then OVRDAT<1> provides data for APWM2H.
       If OVERENL = 1, then OVRDAT<0> provides data for APWM2L */
    APG2IOCON2bits.OVRDAT = 0;
    /* User Output Override Synchronization Control bits
       00 = User output overrides via the OVRENL/H and OVRDAT<1:0> bits are 
       synchronized to the local PWM time base (next start of cycle)*/
    APG2IOCON2bits.OSYNC = 0;
    /* Data for APWM2H/APWM2L Pins if FLT Event is Active bits
       If Fault is active, then FLTDAT<1> provides data for APWM2H.
       If Fault is active, then FLTDAT<0> provides data for APWM2L.*/
    APG2IOCON2bits.FLT1DAT = 0;
    /* Data for APWM2H/APWM2L Pins if CLMT Event is Active bits
       If current limit is active, then CLDAT<1> provides data for APWM2H.
       If current limit is active, then CLDAT<0> provides data for APWM2L.*/
    APG2IOCON2bits.CLDAT = 0;
    /* Data for APWM2H/APWM2L Pins if Feed-Forward Event is Active bits
       If feed-forward is active, then FFDAT<1> provides data for APWM2H.
       If feed-forward is active, then FFDAT<0> provides data for APWM2L.*/
    APG2IOCON2bits.FFDAT = 0;
    /* Data for APWM2H/APWM2L Pins if Debug Mode is Active and PTFRZ = 1 bits
       If Debug mode is active and PTFRZ=1,then DBDAT<1> provides APWM2H data.
       If Debug mode is active and PTFRZ=1,then DBDAT<0> provides APWM2L data. */
    APG2IOCON2bits.DBDAT = 0;
    
    /* Initialize AUX PWM GENERATOR 2 I/O CONTROL REGISTER */    

    /* Time Base Capture Source Selection bits
       000 = No hardware source selected for time base capture ? software only*/
    APG2IOCON1bits.CAPSRC = 0;
    /* Dead-Time Compensation Select bit 
       0 = Dead-time compensation is controlled by PCI Sync logic */
    APG2IOCON1bits.DTCMPSEL = 0;
    /* Aux PWM Generator Output Mode Selection bits
       00 = Aux PWM Generator outputs operate in Complementary mode*/
    APG2IOCON1bits.PMOD = 0;
    /* APWM2H Output Port Enable bit
       1 = Aux PWM Generator controls the APWM2H output pin
       0 = Aux PWM Generator does not control the APWM2H output pin */
    APG2IOCON1bits.PENH = 1;
    /* APWM2L Output Port Enable bit
       1 = Aux PWM Generator controls the APWM2L output pin
       0 = Aux PWM Generator does not control the APWM2L output pin */
    APG2IOCON1bits.PENL = 1;
    /* APWM2H Output Polarity bit
       1 = Output pin is active-low
       0 = Output pin is active-high*/
    APG2IOCON1bits.POLH = 0;
    /* APWM2L Output Polarity bit
       1 = Output pin is active-low
       0 = Output pin is active-high*/
    APG2IOCON1bits.POLL = 0;
    /* Peripheral Pin Select Enable bit
       1 = Peripheral pin select enabled
       0 = Peripheral pin select disabled, as a result, PWM outputs are hard-mapped to pins*/
    APG2IOCON1bits.PPSEN = 1;
    
    /* Initialize AUX PWM GENERATOR 2 EVENT REGISTER */
    APG2EVT1      = 0x0000;
    /* ADC Trigger 1 Post-scaler Selection bits
       00000 = 1:1 */
    APG2EVT1bits.ADTR1PS = 0;
    /* ADC Trigger 1 Source is APG2TRIGC Compare Event Enable bit
       0 = APG2TRIGC register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    APG2EVT1bits.ADTR1EN3  = 0;
    /* ADC Trigger 1 Source is APG2TRIGB Compare Event Enable bit
       0 = APG2TRIGB register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    APG2EVT1bits.ADTR1EN2 = 0;    
    /* ADC Trigger 1 Source is APG2TRIGA Compare Event Enable bit
       1 = APG2TRIGA register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    APG2EVT1bits.ADTR1EN1 = 0;
    /* Update Trigger Select bits
       01 = A write of the APG2DC register automatically sets the UPDATE bit*/
    APG2EVT1bits.UPDTRG = 1;
    /* Aux PWM Generator Trigger Output Selection bits
       000 = EOC event is the Aux PWM Generator trigger*/
    APG2EVT1bits.PGTRGSEL = 0;
    
    /* Initialize AUX PWM GENERATOR 2 EVENT REGISTER */
    /* FLTIEN: PCI Fault Interrupt Enable bit
       1 = Fault interrupt is enabled */
    APG2EVT1bits.FLT1IEN = 1;
    /* PCI Current Limit Interrupt Enable bit
       0 = Current limit interrupt is disabled */
    APG2EVT1bits.CLIEN = 0;
    /* PCI Feed-Forward Interrupt Enable bit
       0 = Feed-forward interrupt is disabled */
    APG2EVT1bits.FFIEN = 0;
    /* PCI Sync Interrupt Enable bit
       0 = Sync interrupt is disabled */
    APG2EVT1bits.SIEN = 0;
    /* Interrupt Event Selection bits
       00 = Interrupts CPU at EOC
       01 = Interrupts CPU at TRIGA compare event
       10 = Interrupts CPU at ADC Trigger 1 event
       11 = Time base interrupts are disabled */
    APG2EVT1bits.IEVTSEL = 3;

        /* ADC Trigger 2 Source is APG2TRIGC Compare Event Enable bit
       0 = APG2TRIGC register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    APG2EVT2bits.ADTR2EN3 = 0;
    /* ADC Trigger 2 Source is APG2TRIGB Compare Event Enable bit
       0 = APG2TRIGB register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    APG2EVT2bits.ADTR2EN2 = 0;
    /* ADC Trigger 2 Source is APG2TRIGA Compare Event Enable bit
       0 = APG2TRIGA register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    APG2EVT2bits.ADTR2EN1 = 0;
    /* ADC Trigger 1 Offset Selection bits
       00000 = No offset */
    APG2EVT1bits.ADTR1OFS = 0;
    
#ifndef ENABLE_PWM_FAULT_PCI_MC3
    /* AUX PWM GENERATOR 2 Fault PCI REGISTER */
    APG2F1PCI1     = 0x0000;
#else
       /* AUX PWM GENERATOR 2 Fault PCI REGISTER */
    APG2F1PCI1     = 0x0000;
    /* Termination Synchronization Disable bit
       1 = Termination of latched PCI occurs immediately
       0 = Termination of latched PCI occurs at PWM EOC */
    APG2F1PCI1bits.TSYNCDIS = 0;
    /* Termination Event Selection bits
       001 = Auto-Terminate: Terminate when PCI source transitions from 
             active to inactive */
    APG2F1PCI1bits.TERM = 1;
    /* Acceptance Qualifier Polarity Select bit: 0 = Not inverted 1 = Inverted*/
    APG2F1PCI1bits.AQPS = 0;
    /* Acceptance Qualifier Source Selection bits
       111 = SWPCI control bit only (qualifier forced to 0)
       110 = Selects PCI Source #9
       101 = Selects PCI Source #8
       100 = Selects PCI Source #1 (Aux PWM Generator output selected by the PWMPCI<2:0> bits)
       011 = Aux PWM Generator is triggered
       010 = LEB is active
       001 = Duty cycle is active (base Aux PWM Generator signal)        
       000 = No acceptance qualifier is used (qualifier forced to 1) */
    APG2F1PCI1bits.AQSS = 0;
    /* PCI Synchronization Control bit
       1 = PCI source is synchronized to PWM EOC
       0 = PCI source is not synchronized to PWM EOC*/
    APG2F1PCI1bits.PSYNC = 0;
    /* PCI Polarity Select bit 0 = Not inverted 1 = Inverted */
    APG2F1PCI1bits.PPS = 1;
    /* PCI Source Selection bits
       31 : CLC1 output
       30 : Comparator 3 output(0x40000000)
       ? ?
       8 : PPS PCI8R in RPINR20 register input(0x0100)
       ? ?
       1 : Internally connected to the output of PWMPCI[2:0] bits in PGxEVT1 registers
       0 : Tied to '0' */
    APG2F1PCI2 = 0x0100;
    /* PCI Bypass Enable bit
       0 = PCI function is not bypassed */
    APG2F1PCI1bits.BPEN   = 0;
    /* PCI Bypass Source Selection bits(1)
       000 = PCI control is sourced from APG2 PCI logic when BPEN = 1 */
    APG2F1PCI1bits.BPSEL   = 0;
    /* PCI Termination Polarity Select bits 
       0 = Inverter, 1 = Non Inverted */
    APG2F1PCI1bits.TERMPS = 0;
    /* PCI Acceptance Criteria Selection bits
       101 = Latched any edge(2)
       100 = Latched rising edge
       011 = Latched
       010 = Any edge
       001 = Rising edge
       000 = Level-sensitive*/
    APG2F1PCI1bits.ACP   = 3;
    /* PCI SR Latch Mode bit
       1 = SR latch is Reset-dominant in Latched Acceptance modes
       0 = SR latch is Set-dominant in Latched Acceptance modes*/
    APG2F1PCI1bits.LATMOD  = 0;
    /* Termination Qualifier Polarity Select bit 1 = Inverted 0 = Not inverted*/
    APG2F1PCI1bits.TQPS   = 0;
    /* Termination Qualifier Source Selection bits
       111 = SWPCI control bit only (qualifier forced to '0')(3)
       110 = Selects PCI Source #9 (pwm_pci[9] input port)
       101 = Selects PCI Source #8 (pwm_pci[8] input port)
       100 = Selects PCI Source #1 (Aux PWM Generator output selected by the PWMPCI<2:0> bits)
       011 = Aux PWM Generator is triggered
       010 = LEB is active
       001 = Duty cycle is active (base Aux PWM Generator signal)
       000 = No termination qualifier used (qualifier forced to '1')(3)*/
    APG2F1PCI1bits.TQSS  = 0;
#endif    

    /* AUX PWM GENERATOR 2 Current Limit PCI REGISTER */
    APG2CLPCI1    = 0x0000;
    /* AUX PWM GENERATOR 2 Feed Forward PCI REGISTER  */
    APG2FFPCI1    = 0x0000;
    
    /* For PWM Synchronization
     PWM Gen 1 is triggered by the Event A of PWM Gen 5*/
    /* AUX PWM GENERATOR 2 Sync PCI REGISTER */
    APG2SPCI1    = 0x0000;
    /* PCI Polarity Select bit 
       1 = Inverted
       0 = Not inverted*/
    APG2SPCI1bits.PPS = 1;
    /* PWM PCI Source Selection
       100 = PWM Generator #5 output used as PCI signal */
    APG2EVT1bits.PWMPCI = 0b100;
    /* AUX PWM GENERATOR 2 Sync PCI Source 
       23 = PWM event A*/
    APG2SPCI2 = 0x00800000;
    
    /* Initialize AUX PWM GENERATOR 2 LEADING-EDGE BLANKING REGISTER */
    APG2LEB      = 0x0000;
    
    /* Initialize AUX PWM GENERATOR 2 PHASE REGISTER */
    APG2PHASEbits.PHASE     = MC3_MIN_DUTY;
    /* Initialize AUX PWM GENERATOR 2 DUTY CYCLE REGISTER */
    APG2DCbits.DC           = MC3_MIN_DUTY;
    /* Initialize AUX PWM GENERATOR 2 DUTY CYCLE ADJUSTMENT REGISTER */
    APG2DCA       = 0x0000;
    /* Initialize AUX PWM GENERATOR 2 PERIOD REGISTER */
    APG2PERbits.PER       = MC3_LOOPTIME_TCY;
    /* Initialize AUX PWM GENERATOR 2 DEAD-TIME REGISTER */
    APG2DTbits.DTH = MC3_DEADTIME;
    /* Initialize AUX PWM GENERATOR 2 DEAD-TIME REGISTER */
    APG2DTbits.DTL = MC3_DEADTIME;

    /* Initialize AUX PWM GENERATOR 2 TRIGGER A REGISTER */
    APG2TRIGB     = 0x0000;
    /* Initialize AUX PWM GENERATOR 2 TRIGGER B REGISTER */
    APG2TRIGB     = 0x0000;
    /* Initialize AUX PWM GENERATOR 2 TRIGGER C REGISTER */
    APG2TRIGC     = 0x0000;
    
}

/**
* <B> Function: InitAuxPWMGenerator3()    </B>
*
* @brief Function to configure Aux PWM Module # 3
*        
* @param none.
* @return none.
* 
* @example
* <CODE> InitAuxPWMGenerator3();     </CODE>
*
*/
void InitAuxPWMGenerator3 (void)
{

    /* Initialize AUX PWM GENERATOR 3 CONTROL REGISTER */
    APG3CON      = 0x0000;
    /* Aux PWM Generator 3 Enable bit : 1 = Is enabled, 0 = Is not enabled */
    /* Ensuring Aux PWM Generator is disabled prior to configuring module */
    APG3CONbits.ON = 0;
    /* Clock Selection bits
       0b01 = Macro uses Master clock selected by the PCLKCON.MCLKSEL bits*/
    APG3CONbits.CLKSEL = 1;
    /* PWM Mode Selection bits
     * 110 = Dual Edge Center-Aligned PWM mode (interrupt/register update once per cycle)
       100 = Center-Aligned PWM mode(interrupt/register update once per cycle)*/
    APG3CONbits.MODSEL = 4;
    /* Trigger Count Select bits
       000 = Aux PWM Generator produces 1 PWM cycle after triggered */
    APG3CONbits.TRGCNT = 0;
    
    /* Initialize AUX PWM GENERATOR 3 CONTROL REGISTER */
    /* Master Duty Cycle Register Select bit
       1 = Macro uses the MDC register instead of APG3DC
       0 = Macro uses the APG3DC register*/
    APG3CONbits.MDCSEL = 0;
    /* Master Period Register Select bit
       1 = Macro uses the MPER register instead of APG3PER
       0 = Macro uses the APG3PER register */
    APG3CONbits.MPERSEL = 0;
    /* MPHSEL: Master Phase Register Select bit
       1 = Macro uses the MPHASE register instead of APG3PHASE
       0 = Macro uses the APG3PHASE register */
    APG3CONbits.MPHSEL = 0;
    /* Master Update Enable bit
       1 = Aux PWM Generator broadcasts software set/clear of UPDATE status bit and 
           EOC signal to other Aux PWM Generators
       0 = Aux PWM Generator does not broadcast UPDATE status bit or EOC signal */
    APG3CONbits.MSTEN = 0;
    /* PWM Buffer Update Mode Selection bits 
       Update Data registers at start of next PWM cycle if UPDATE = 1. */
    APG3CONbits.UPDMOD = 0;
    /* Aux PWM Generator Trigger Mode Selection bits
       0b00 = Aux PWM Generator operates in Single Trigger mode
       0b01 = Aux PWM Generator operates in Retriggerable mode */
    APG3CONbits.TRGMOD = 1;
    /* Start of Cycle Selection bits
       1111 = TRIG bit or PCI Sync function only (no hardware trigger source is selected)
       0000 = Local EOC*/
    APG3CONbits.SOCS = 0b1111;
    
    /* Clear AUX PWM GENERATOR 3 STATUS REGISTER*/
    APG3STAT      = 0x0000;
    /* Initialize AUX PWM GENERATOR 3 I/O CONTROL REGISTER */
    APG3IOCON2    = 0x0000;

    /* Current Limit Mode Select bit
       0 = If PCI current limit is active, then the CLDAT<1:0> bits define 
       the PWM output levels */
    APG3IOCON2bits.CLMOD = 0;
    /* Swap PWM Signals to APWM3H and APWM3L Device Pins bit 
       0 = APWM3H/L signals are mapped to their respective pins */
    APG3IOCON1bits.SWAP = 0;
    /* User Override Enable for APWM3H Pin bit
       0 = Aux PWM Generator provides data for the APWM3H pin*/
    APG3IOCON2bits.OVRENH = 0;
    /* User Override Enable for APWM3L Pin bit
       0 = Aux PWM Generator provides data for the APWM3L pin*/
    APG3IOCON2bits.OVRENL = 0;
    /* Data for APWM3H/APWM3L Pins if Override is Enabled bits
       If OVERENH = 1, then OVRDAT<1> provides data for APWM3H.
       If OVERENL = 1, then OVRDAT<0> provides data for APWM3L */
    APG3IOCON2bits.OVRDAT = 0;
    /* User Output Override Synchronization Control bits
       00 = User output overrides via the OVRENL/H and OVRDAT<1:0> bits are 
       synchronized to the local PWM time base (next start of cycle)*/
    APG3IOCON2bits.OSYNC = 0;
    /* Data for APWM3H/APWM3L Pins if FLT Event is Active bits
       If Fault is active, then FLTDAT<1> provides data for APWM3H.
       If Fault is active, then FLTDAT<0> provides data for APWM3L.*/
    APG3IOCON2bits.FLT1DAT = 0;
    /* Data for APWM3H/APWM3L Pins if CLMT Event is Active bits
       If current limit is active, then CLDAT<1> provides data for APWM3H.
       If current limit is active, then CLDAT<0> provides data for APWM3L.*/
    APG3IOCON2bits.CLDAT = 0;
    /* Data for APWM3H/APWM3L Pins if Feed-Forward Event is Active bits
       If feed-forward is active, then FFDAT<1> provides data for APWM3H.
       If feed-forward is active, then FFDAT<0> provides data for APWM3L.*/
    APG3IOCON2bits.FFDAT = 0;
    /* Data for APWM3H/APWM3L Pins if Debug Mode is Active and PTFRZ = 1 bits
       If Debug mode is active and PTFRZ=1,then DBDAT<1> provides APWM3H data.
       If Debug mode is active and PTFRZ=1,then DBDAT<0> provides APWM3L data. */
    APG3IOCON2bits.DBDAT = 0;
    
    /* Initialize AUX PWM GENERATOR 3 I/O CONTROL REGISTER */    

    /* Time Base Capture Source Selection bits
       000 = No hardware source selected for time base capture ? software only*/
    APG3IOCON1bits.CAPSRC = 0;
    /* Dead-Time Compensation Select bit 
       0 = Dead-time compensation is controlled by PCI Sync logic */
    APG3IOCON1bits.DTCMPSEL = 0;
    /* Aux PWM Generator Output Mode Selection bits
       00 = Aux PWM Generator outputs operate in Complementary mode*/
    APG3IOCON1bits.PMOD = 0;
    /* APWM3H Output Port Enable bit
       1 = Aux PWM Generator controls the APWM3H output pin
       0 = Aux PWM Generator does not control the APWM3H output pin */
    APG3IOCON1bits.PENH = 1;
    /* APWM3L Output Port Enable bit
       1 = Aux PWM Generator controls the APWM3L output pin
       0 = Aux PWM Generator does not control the APWM3L output pin */
    APG3IOCON1bits.PENL = 1;
    /* APWM3H Output Polarity bit
       1 = Output pin is active-low
       0 = Output pin is active-high*/
    APG3IOCON1bits.POLH = 0;
    /* APWM3L Output Polarity bit
       1 = Output pin is active-low
       0 = Output pin is active-high*/
    APG3IOCON1bits.POLL = 0;
    /* Peripheral Pin Select Enable bit
       1 = Peripheral pin select enabled
       0 = Peripheral pin select disabled, as a result, PWM outputs are hard-mapped to pins*/
    APG3IOCON1bits.PPSEN = 1;
    
    /* Initialize AUX PWM GENERATOR 3 EVENT REGISTER */
    APG3EVT1      = 0x0000;
    /* ADC Trigger 1 Post-scaler Selection bits
       00000 = 1:1 */
    APG3EVT1bits.ADTR1PS = 0;
    /* ADC Trigger 1 Source is APG3TRIGC Compare Event Enable bit
       0 = APG3TRIGC register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    APG3EVT1bits.ADTR1EN3  = 0;
    /* ADC Trigger 1 Source is APG3TRIGB Compare Event Enable bit
       0 = APG3TRIGB register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    APG3EVT1bits.ADTR1EN2 = 0;    
    /* ADC Trigger 1 Source is APG3TRIGA Compare Event Enable bit
       1 = APG3TRIGA register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    APG3EVT1bits.ADTR1EN1 = 0;
    /* Update Trigger Select bits
       01 = A write of the APG3DC register automatically sets the UPDATE bit*/
    APG3EVT1bits.UPDTRG = 1;
    /* Aux PWM Generator Trigger Output Selection bits
       000 = EOC event is the Aux PWM Generator trigger*/
    APG3EVT1bits.PGTRGSEL = 0;
    
    /* Initialize AUX PWM GENERATOR 3 EVENT REGISTER */
    /* FLTIEN: PCI Fault Interrupt Enable bit
       1 = Fault interrupt is enabled */
    APG3EVT1bits.FLT1IEN = 1;
    /* PCI Current Limit Interrupt Enable bit
       0 = Current limit interrupt is disabled */
    APG3EVT1bits.CLIEN = 0;
    /* PCI Feed-Forward Interrupt Enable bit
       0 = Feed-forward interrupt is disabled */
    APG3EVT1bits.FFIEN = 0;
    /* PCI Sync Interrupt Enable bit
       0 = Sync interrupt is disabled */
    APG3EVT1bits.SIEN = 0;
    /* Interrupt Event Selection bits
       00 = Interrupts CPU at EOC
       01 = Interrupts CPU at TRIGA compare event
       10 = Interrupts CPU at ADC Trigger 1 event
       11 = Time base interrupts are disabled */
    APG3EVT1bits.IEVTSEL = 3;

        /* ADC Trigger 2 Source is APG3TRIGC Compare Event Enable bit
       0 = APG3TRIGC register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    APG3EVT2bits.ADTR2EN3 = 0;
    /* ADC Trigger 2 Source is APG3TRIGB Compare Event Enable bit
       0 = APG3TRIGB register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    APG3EVT2bits.ADTR2EN2 = 0;
    /* ADC Trigger 2 Source is APG3TRIGA Compare Event Enable bit
       0 = APG3TRIGA register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    APG3EVT2bits.ADTR2EN1 = 0;
    /* ADC Trigger 1 Offset Selection bits
       00000 = No offset */
    APG3EVT1bits.ADTR1OFS = 0;
    
#ifndef ENABLE_PWM_FAULT_PCI_MC3
    /* AUX PWM GENERATOR 3 Fault PCI REGISTER */
    APG3F1PCI1     = 0x0000;
#else
       /* AUX PWM GENERATOR 3 Fault PCI REGISTER */
    APG3F1PCI1     = 0x0000;
    /* Termination Synchronization Disable bit
       1 = Termination of latched PCI occurs immediately
       0 = Termination of latched PCI occurs at PWM EOC */
    APG3F1PCI1bits.TSYNCDIS = 0;
    /* Termination Event Selection bits
       001 = Auto-Terminate: Terminate when PCI source transitions from 
             active to inactive */
    APG3F1PCI1bits.TERM = 1;
    /* Acceptance Qualifier Polarity Select bit: 0 = Not inverted 1 = Inverted*/
    APG3F1PCI1bits.AQPS = 0;
    /* Acceptance Qualifier Source Selection bits
       111 = SWPCI control bit only (qualifier forced to 0)
       110 = Selects PCI Source #9
       101 = Selects PCI Source #8
       100 = Selects PCI Source #1 (Aux PWM Generator output selected by the PWMPCI<2:0> bits)
       011 = Aux PWM Generator is triggered
       010 = LEB is active
       001 = Duty cycle is active (base Aux PWM Generator signal)        
       000 = No acceptance qualifier is used (qualifier forced to 1) */
    APG3F1PCI1bits.AQSS = 0;
    /* PCI Synchronization Control bit
       1 = PCI source is synchronized to PWM EOC
       0 = PCI source is not synchronized to PWM EOC*/
    APG3F1PCI1bits.PSYNC = 0;
    /* PCI Polarity Select bit 0 = Not inverted 1 = Inverted */
    APG3F1PCI1bits.PPS = 1;
    /* PCI Source Selection bits
       31 : CLC1 output
       30 : Comparator 3 output(0x40000000)
       ? ?
       8 : PPS PCI8R in RPINR20 register input(0x0100)
       ? ?
       1 : Internally connected to the output of PWMPCI[2:0] bits in PGxEVT1 registers
       0 : Tied to '0' */
    APG3F1PCI2 = 0x0100;
    /* PCI Bypass Enable bit
       0 = PCI function is not bypassed */
    APG3F1PCI1bits.BPEN   = 0;
    /* PCI Bypass Source Selection bits(1)
       000 = PCI control is sourced from APG3 PCI logic when BPEN = 1 */
    APG3F1PCI1bits.BPSEL   = 0;
    /* PCI Termination Polarity Select bits 
       0 = Inverter, 1 = Non Inverted */
    APG3F1PCI1bits.TERMPS = 0;
    /* PCI Acceptance Criteria Selection bits
       101 = Latched any edge(2)
       100 = Latched rising edge
       011 = Latched
       010 = Any edge
       001 = Rising edge
       000 = Level-sensitive*/
    APG3F1PCI1bits.ACP   = 3;
    /* PCI SR Latch Mode bit
       1 = SR latch is Reset-dominant in Latched Acceptance modes
       0 = SR latch is Set-dominant in Latched Acceptance modes*/
    APG3F1PCI1bits.LATMOD  = 0;
    /* Termination Qualifier Polarity Select bit 1 = Inverted 0 = Not inverted*/
    APG3F1PCI1bits.TQPS   = 0;
    /* Termination Qualifier Source Selection bits
       111 = SWPCI control bit only (qualifier forced to '0')(3)
       110 = Selects PCI Source #9 (pwm_pci[9] input port)
       101 = Selects PCI Source #8 (pwm_pci[8] input port)
       100 = Selects PCI Source #1 (Aux PWM Generator output selected by the PWMPCI<2:0> bits)
       011 = Aux PWM Generator is triggered
       010 = LEB is active
       001 = Duty cycle is active (base Aux PWM Generator signal)
       000 = No termination qualifier used (qualifier forced to '1')(3)*/
    APG3F1PCI1bits.TQSS  = 0;
#endif    

    /* AUX PWM GENERATOR 3 Current Limit PCI REGISTER */
    APG3CLPCI1    = 0x0000;
    /* AUX PWM GENERATOR 3 Feed Forward PCI REGISTER  */
    APG3FFPCI1    = 0x0000;
    
    /* For PWM Synchronization
     PWM Gen 1 is triggered by the Event A of PWM Gen 5*/
    /* AUX PWM GENERATOR 3 Sync PCI REGISTER */
    APG3SPCI1    = 0x0000;
    /* PCI Polarity Select bit 
       1 = Inverted
       0 = Not inverted*/
    APG3SPCI1bits.PPS = 1;
    /* PWM PCI Source Selection
       100 = PWM Generator #5 output used as PCI signal */
    APG3EVT1bits.PWMPCI = 0b100;
    /* AUX PWM GENERATOR 3 Sync PCI Source 
       23 = PWM event A*/
    APG3SPCI2 = 0x00800000;
    
    /* Initialize AUX PWM GENERATOR 3 LEADING-EDGE BLANKING REGISTER */
    APG3LEB      = 0x0000;
    
    /* Initialize AUX PWM GENERATOR 3 PHASE REGISTER */
    APG3PHASEbits.PHASE     = MC3_MIN_DUTY;
    /* Initialize AUX PWM GENERATOR 3 DUTY CYCLE REGISTER */
    APG3DCbits.DC           = MC3_MIN_DUTY;
    /* Initialize AUX PWM GENERATOR 3 DUTY CYCLE ADJUSTMENT REGISTER */
    APG3DCA       = 0x0000;
    /* Initialize AUX PWM GENERATOR 3 PERIOD REGISTER */
    APG3PERbits.PER       = MC3_LOOPTIME_TCY;
    /* Initialize AUX PWM GENERATOR 3 DEAD-TIME REGISTER */
    APG3DTbits.DTH = MC3_DEADTIME;
    /* Initialize AUX PWM GENERATOR 3 DEAD-TIME REGISTER */
    APG3DTbits.DTL = MC3_DEADTIME;

    /* Initialize AUX PWM GENERATOR 3 TRIGGER A REGISTER */
    APG3TRIGB     = 0x0000;
    /* Initialize AUX PWM GENERATOR 3 TRIGGER B REGISTER */
    APG3TRIGB     = 0x0000;
    /* Initialize AUX PWM GENERATOR 3 TRIGGER C REGISTER */
    APG3TRIGC     = 0x0000;
    
}
// </editor-fold>