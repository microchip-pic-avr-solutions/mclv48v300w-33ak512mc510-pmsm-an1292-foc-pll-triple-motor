// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file port_config.c
 *
 * @brief This module initializes the GPIO pins as analog/digital,input or 
 * output etc. Also to PPS functionality to Re-mappable input or output pins.
 * 
 * Definitions in this file are for dsPIC33AK512MC510
 *
 * Component: PORTS
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

#include "port_config.h"

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

/**
* <B> Function: SetupGPIOPorts() </B>
*
* @brief Function initialize all ports as input and digital pins
*        
* @param none.
* @return none.
* 
* @example
* <CODE> SetupGPIOPorts(); </CODE>
*
*/
void SetupGPIOPorts(void)
{
    /* Reset all PORTx register (all inputs) */
    
    #ifdef TRISA
        TRISA = 0xFFFF;
        LATA  = 0x0000;
    #endif
    #ifdef ANSELA
        ANSELA = 0x0000;
    #endif

    #ifdef TRISB
        TRISB = 0xFFFF;
        LATB  = 0x0000;
    #endif
    #ifdef ANSELB
        ANSELB = 0x0000;
    #endif

    #ifdef TRISC
        TRISC = 0xFFFF;
        LATC  = 0x0000;
    #endif
    #ifdef ANSELC
        ANSELC = 0x0000;
    #endif

    #ifdef TRISD
        TRISD = 0xFFFF;
        LATD  = 0x0000;
    #endif
    #ifdef ANSELD
        ANSELD = 0x0000;
    #endif

    #ifdef TRISE
        TRISE = 0xFFFF;
        LATE  = 0x0000;
    #endif
    #ifdef ANSELE
        ANSELE = 0x0000;
    #endif

    #ifdef TRISF
        TRISF = 0xFFFF;
        LATF  = 0x0000;
    #endif
    #ifdef ANSELF
        ANSELF = 0x0000;
    #endif

    #ifdef TRISG
        TRISG = 0xFFFF;
        LATG  = 0x0000;
    #endif
    #ifdef ANSELG
        ANSELG = 0x0000;
    #endif

    MapGPIOHWFunction();

}
/**
* <B> Function: MapGPIOHWFunction() </B>
*
* @brief Function maps port pins as input or output, analog or digital
*        
* @param none.
* @return none.
* 
* @example
* <CODE> MapGPIOHWFunction(); </CODE>
*
*/
void MapGPIOHWFunction(void)
{  
    /***** Internal Amplifier pin configuration for IA, IB and IBUS with MC1 ******/
    /* Amplifier input and output pins */
    /* MC1 IA : OA1
     * Positive Input   : DIM:013 - PIN23: OA1IN+/AD1AN1/AD5AN4/CVDAN4/CMP1B/RP5/RA4
     * Negative Input   : DIM:015 - PIN22: OA1IN-/AD1ANN2/AD1AN2/CVDAN3/CMPCN/CMP1C/RP4/RA3
     * Amplifier Output : DIM:017 - PIN21: OA1OUT/AD1AN0/CVDAN2/CMP1A/RP3/RA2       */
    ANSELAbits.ANSELA4 = 1;
    TRISAbits.TRISA4 = 1;   
    ANSELAbits.ANSELA3 = 1;
    TRISAbits.TRISA3 = 1;   
    ANSELAbits.ANSELA2 = 1;
    TRISAbits.TRISA2 = 0;
    
    /* MC1 IB : OA2
     * Positive Input   : DIM:021 - PIN33: OA2IN+/AD2AN1/CVDAN18/CVDTX2/CMP2B/RP19/RB2
     * Negative Input   : DIM:023 - PIN32: TMS/OA2IN-/AD2ANN2/AD2AN2/CVDAN17/CVDTX1/CMP2C/RP18/RB1
     * Amplifier Output : DIM:025 - PIN31: OA2OUT/AD2AN0/CVDAN16/CVDTX0/CMP2A/RP17/RB0   */
    ANSELBbits.ANSELB2 = 1;
    TRISBbits.TRISB2 = 1;   
    ANSELBbits.ANSELB1 = 1;
    TRISBbits.TRISB1 = 1;     
    ANSELBbits.ANSELB0 = 1;
    TRISBbits.TRISB0 = 0;
    
    /* MC1 IBUS : OA3
     * Positive Input   : DIM:029 - PIN26: OA3IN+/AD3AN1/AD4AN5/CVDAN21/CVDTX5/CMP3B/RP22/RB5
     * Negative Input   : DIM:031 - PIN25: OA3IN-/AD3ANN2/AD3AN2/CVDAN6/CMPDN/CMP3C/RP7/RA6
     * Amplifier Output : DIM:033 - PIN24: OA3OUT/AD3AN0/CVDAN5/CMP3A/RP6/INT0/RA5 */
    ANSELBbits.ANSELB5 = 1;
    TRISBbits.TRISB5 = 1;   
    ANSELAbits.ANSELA6 = 1;
    TRISAbits.TRISA6 = 1;   
    ANSELAbits.ANSELA5 = 1;
    TRISAbits.TRISA5 = 0;
    
#ifndef INTERNAL_OPAMP_CONFIG_MC1
    /* External Amplifier mode - Amplifier output pins are configured as
     analog input channels for MC1
     * MC1 IA   : DIM:019 - PIN21: OA1OUT/AD1AN0/CVDAN2/CMP1A/RP3/RA2   
     * MC1 IB   : DIM:027 - PIN31: OA2OUT/AD2AN0/CVDAN16/CVDTX0/CMP2A/RP17/RB0
     * MC1 IBUS : DIM:035 - PIN24: OA3OUT/AD3AN0/CVDAN5/CMP3A/RP6/INT0/RA5 */
    TRISAbits.TRISA2 = 1;   
    TRISBbits.TRISB0 = 1;  
    TRISAbits.TRISA5 = 1;  
#endif 
  
    /* MC1 PWM Signals
     * PWM1L : DIM:003 - PIN #83  TDI/RP52/PWM1L/IOMAD0/RD3
     * PWM1H : DIM:001 - PIN #82  TDO/RP51/PWM1H/IOMAD1/RD2
     * PWM2L : DIM:007 - PIN #79  TCK/RP50/PWM2L/IOMAD2/RD1
     * PWM2H : DIM:005 - PIN #78  RP49/PWM2H/IOMAD3/RD0
     * PWM3L : DIM:004 - PIN #67  RP37/PWM3L/IOMAD4/RC4
     * PWM3H : DIM:002 - PIN #66  RP36/PWM3H/IOMAD5/RC3      */
    TRISDbits.TRISD3 = 0 ;          
    TRISDbits.TRISD2 = 0 ;         
    TRISDbits.TRISD1 = 0 ;          
    TRISDbits.TRISD0 = 0 ;           
    TRISCbits.TRISC4 = 0 ;          
    TRISCbits.TRISC3 = 0 ;             
    
    /****** MC2 Current and PWM Signals on J11 XPRO1 connector *******/
    /* MC2 IA : J11-3 : DIM:082 - PIN #47 : AD4AN4/CVDAN26/CVDTX10/RP27/SCK2/IOMAF11/IOMBF11/RB10 */
    ANSELBbits.ANSELB10 = 1;
    TRISBbits.TRISB10 = 1;
    /* MC2 IB : J11-4 : DIM:084 - PIN #46 : AD4ANN1/AD4AN2/CVDAN23/CVDTX7/CMP4C/IBIAS1/ISRC1/RP24/IOMAF0/RB7 */
    ANSELBbits.ANSELB7 = 1;
    TRISBbits.TRISB7 = 1;
    /* MC2 PWM Signals */
    /* PWM1H : J11-5 : DIM:086 - PIN #76 : RP58/PWM6H/IOMBD3/IOMBF7/RD9
     * PWM1L : J11-7 : DIM:090 - PIN #77 : RP59/PWM6L/IOMBD2/IOMAF9/IOMBF9/RD10
     * PWM2H : PWM7H is Re-mapped to J11-9 : DIM:094 - PIN #13 : DACOUT2/AD5AN3/CVDAN8/IBIAS3/ISRC3/RP9/RA8
     * PWM2L : PWM7L is Re-mapped to J11-10 : DIM:096 - PIN #45 : AD4ANN2/AD4AN1/CVDAN22/CVDTX6/CMP/IBIAS0/ISRC0/RP23/RB6
     * PWM3H : J11-6 : DIM:088 - PIN #70 : RP45/PWM8H/IOMBD7/RC12
     * PWM3L : J11-8 : DIM:092 - PIN #71 : RP46/PWM8L/IOMBD6/RC13
     */
    TRISDbits.TRISD9 = 0;
    TRISDbits.TRISD10 = 0;
    _RP9R = 13;
    _RP23R = 14;
    TRISCbits.TRISC12 = 0;
    TRISCbits.TRISC13 = 0;
    
    /****** MC3 Current and PWM Signals on J11 XPRO1 connector *******/
    /* MC3 IA : J11-11 : DIM:098 - PIN #43 : PGD1/AD4AN3/CVDAN19/CVDTX3/CMP4A/RP20/SDA1/RB3 */
    ANSELBbits.ANSELB3 = 1;
    TRISBbits.TRISB3 = 1;
    /* MC3 IB : J11-12 : DIM:100 - PIN #44 : PGC1/AD4AN0/CVDAN20/CVDTX4/CMP4B/RP21/SCL1/RB4 */
    ANSELBbits.ANSELB4 = 1;
    TRISBbits.TRISB4 = 1;
    /* MC3 APWM Signals */
    /* APWM1H : Re-mapped to J11-13 : DIM:102 - PIN #68 : RP43/PWM7H/IOMBD5/IOMBF1/RC10
     * APWM1L : Re-mapped to J11-14 : DIM:104 - PIN #69 : RP44/PWM7L/IOMBD4/IOMBF0/RC11
     * APWM2H : Re-mapped to J11-15 : DIM:106 - PIN #20 : CVDAN14/CMP1D/RP15/RA14
     * APWM2L : Re-mapped to J11-16 : DIM:108 - PIN #17 : CVDAN15/RP16/RA15
     * APWM3H : Re-mapped to J11-17 : DIM:110 - PIN #16 : CVDTX23/RP69/RE4
     * APWM3L : Re-mapped to J11-18 : DIM:112 - PIN #11 : CVDAN13/RP14/RA13
     */
    _RP43R = 88;
    _RP44R = 89;
    _RP15R = 90;
    _RP16R = 91;
    _RP69R = 92;
    _RP14R = 93;
    
     /** Below signals are common for all motors - MC1, MC2 and MC3 **/
    /* Potentiometer  input (POT1) - used as Speed Reference for MC1, MC2 and MC3
     * DIM:028 - PIN #36: AD2AN5/CVDAN31/CVDTX15/RP32/RB15 */
    ANSELBbits.ANSELB15 = 1;
    TRISBbits.TRISB15 = 1;   
    
    /* DC Bus Voltage (VBUS) - common for MC1, MC2 and MC3 
     * DIM:039 -  PIN #37: AD3AN4/CVDTX29/RP81/RF0 */
    ANSELFbits.ANSELF0 = 1;
    TRISFbits.TRISF0 = 1;  
     
    /* Debug LEDs on MCLV-48V-300W Dev Board and MC DIM */
    /* LED1 : DIM:030 - PIN #10 : CVDTX21/RP67/RE2   */
    TRISEbits.TRISE2 = 0;
    /* LED2 : DIM:032 - PIN #06 : CVDTX22/RP68/RE3  */
    TRISEbits.TRISE3 = 0;
    /* DIM_LED : - PIN #38 : CVDTX30/RP82/RF1  */
    TRISFbits.TRISF1 = 0;

    /* Push button Switches on MCLV-48V-300W Dev Board*/
    /* SW1 : DIM:034 - PIN #03 : CVDAN12/RP13/RA12   */
    TRISAbits.TRISA12 = 1;            
    /* SW2 : DIM:036 - PIN #02 : CVDTX20/RP66/RE1  */
    TRISEbits.TRISE1 = 1;            
	
    /* Configuring FLTLAT_OC_OV (DIM:040) - Pin #01 : CVDTX19/RP65/RE0 as PCI8 
     Please note PCI8 is configured as PWM fault control for MC1, MC2 and MC3*/
	_PCI8R = 65;
    
	/** Diagnostic Interface in MCLV-48V-300W Board
        Re-map UART Channels to the device pins connected to the following 
        pins on the Motor Control Development Board.
        UART_RX : DIM:054 - PIN #74 : RP47/APWM3H/IOMBD9/RC14 (Input)
        UART_TX : DIM:052 - PIN #75 : RP48/APWM3L/IOMBD8/RC15(Output)   */
    _U1RXR = 47;
    _RP48R = 19;
     
}

/**
* <B> Function: OpampConfig() </B>
*
* @brief Function to configure and enable the Op-Amp Module
*        
* @param none.
* @return none.
* 
* @example
* <CODE> OpampConfig(); </CODE>
*
*/
void OpampConfig (void)
{
    /** AMP1CON1 :AMP1 Control Register 1*/
    AMP1CON1 = 0x0000;
    /** Bit 15 = AMPEN Op Amp Enable/On bit 
        1 Enables op amp module ; 
        0 Disables op amp module */
    AMP1CON1bits.AMPEN = 0;
    /** Bit 14 = HPEN High-Power Enable bit  
        1 Enables Op Amp High-Power (high bandwidth) mode ; 
        0 Disables Op Amp High-Power mode */    
    AMP1CON1bits.HPEN = 1;
    /** Bit 13 = UGE Unity Gain Buffer Enable bit 
        1 Enables Unity Gain mode ; 
        0 Disables Unity Gain mode */
    AMP1CON1bits.UGE = 0;
    /** Bits 12:11 = DIFFCON[1:0] Differential Input Mode Control bits
        11 Reserved, do not use; 
        10 Turn NMOS differential input pair off
        01 Turn PMOS differential input pair off 
        00 Use both NMOS and PMOS differential input pair */
    AMP1CON1bits.DIFFCON = 0;
    /** Bit 10 = REFEN Input Reference Enable bit
        1 Noninverting input connected to internal device voltage source
        0 Noninverting input of op amp connected to user pin */
    AMP1CON1bits.REFEN = 0;
    
    /** AMP1CON2 :AMP1 Control Register 2*/
    /** bits 28-24 POFFSETHP[4:0]
        Offset Correction for PMOS differential input pair (high power mode)*/
    AMP1CON2bits.POFFSETHP = 0;
    /** bits 20-16 = NOFFSETHP[4:0]
        Offset Correction for NMOS differential input pair (high power mode)*/
    AMP1CON2bits.NOFFSETHP = 0;
    /** bits 12-8 = POFFSETLP[4:0]
        Offset Correction for PMOS differential input pair (low power mode)*/
    AMP1CON2bits.POFFSETLP = 0;
   /** bits 4-0 =  NOFFSETLP[4:0]
       Offset Correction for NMOS differential input pair (low power mode)*/
    AMP1CON2bits.NOFFSETLP = 0;
    
    /** AMP2CON1 :AMP2 Control Register 1*/
    AMP2CON1 = 0x0000;
    /** Bit 15 = AMPEN Op Amp Enable/On bit 
        1 Enables op amp module ; 
        0 Disables op amp module */
    AMP2CON1bits.AMPEN = 0;
    /** Bit 14 = HPEN High-Power Enable bit  
        1 Enables Op Amp High-Power (high bandwidth) mode ; 
        0 Disables Op Amp High-Power mode */    
    AMP2CON1bits.HPEN = 1;
    /** Bit 13 = UGE Unity Gain Buffer Enable bit 
        1 Enables Unity Gain mode ; 
        0 Disables Unity Gain mode */
    AMP2CON1bits.UGE = 0;
    /** Bits 12:11 = DIFFCON[1:0] Differential Input Mode Control bits
        11 Reserved, do not use; 
        10 Turn NMOS differential input pair off
        01 Turn PMOS differential input pair off 
        00 Use both NMOS and PMOS differential input pair */
    AMP2CON1bits.DIFFCON = 0;
    /** Bit 10 = REFEN Input Reference Enable bit
        1 Noninverting input connected to internal device voltage source
        0 Noninverting input of op amp connected to user pin */
    AMP2CON1bits.REFEN = 0;
    
    /** AMP2CON2 :AMP2 Control Register 2*/
    /** bits 28-24 = POFFSETHP[4:0]
        Offset Correction for PMOS differential input pair (high power mode)*/
    AMP2CON2bits.POFFSETHP = 0;
    /** bits 20-16 = NOFFSETHP[4:0]
        Offset Correction for NMOS differential input pair (high power mode)*/
    AMP2CON2bits.NOFFSETHP = 0;
    /** bits 12-8 = POFFSETLP[4:0]
        Offset Correction for PMOS differential input pair (low power mode)*/
    AMP2CON2bits.POFFSETLP = 0;
   /** bits 4-0 = NOFFSETLP[4:0]
       Offset Correction for NMOS differential input pair (low power mode)*/
    AMP2CON2bits.NOFFSETLP = 0;
    
    /** AMP3CON1 :AMP3 Control Register 1*/
    AMP3CON1 = 0x0000;
    /** Bit 15 = AMPEN Op Amp Enable/On bit 
        1 Enables op amp module ; 
        0 Disables op amp module */
    AMP3CON1bits.AMPEN = 0;
    /** Bit 14 = HPEN High-Power Enable bit  
        1 Enables Op Amp High-Power (high bandwidth) mode ; 
        0 Disables Op Amp High-Power mode */    
    AMP3CON1bits.HPEN = 1;
    /** Bit 13 = UGE Unity Gain Buffer Enable bit 
        1 Enables Unity Gain mode ; 
        0 Disables Unity Gain mode */
    AMP3CON1bits.UGE = 0;
    /** Bits 12:11 = DIFFCON[1:0] Differential Input Mode Control bits
        11 Reserved, do not use; 
        10 Turn NMOS differential input pair off
        01 Turn PMOS differential input pair off 
        00 Use both NMOS and PMOS differential input pair */
    AMP3CON1bits.DIFFCON = 0;
    /** Bit 10 = REFEN Input Reference Enable bit
        1 Noninverting input connected to internal device voltage source
        0 Noninverting input of op amp connected to user pin */
    AMP3CON1bits.REFEN = 0;
    
    /** AMP3CON2 :AMP3 Control Register 2*/
    /** bits 28-24 = POFFSETHP[4:0]
        Offset Correction for PMOS differential input pair (high power mode)*/
    AMP3CON2bits.POFFSETHP = 0;
    /** bits 20-16 = NOFFSETHP[4:0]
        Offset Correction for NMOS differential input pair (high power mode)*/
    AMP3CON2bits.NOFFSETHP = 0;
    /** bits 12-8 = POFFSETLP[4:0]
        Offset Correction for PMOS differential input pair (low power mode)*/
    AMP3CON2bits.POFFSETLP = 0;
   /** bits 4-0 = NOFFSETLP[4:0]
       Offset Correction for NMOS differential input pair (low power mode)*/
    AMP3CON2bits.NOFFSETLP = 0;
    
	/* Enabling the amplifiers */
    /** Bit 15 = AMPEN Op Amp Enable/On bit 
        1 Enables op amp module ; 
        0 Disables op amp module */
    AMP1CON1bits.AMPEN = 1;
    /** Bit 15 = AMPEN Op Amp Enable/On bit 
        1 Enables op amp module ; 
        0 Disables op amp module */
    AMP2CON1bits.AMPEN = 1;
    /** Bit 15 = AMPEN Op Amp Enable/On bit 
        1 Enables op amp module ; 
        0 Disables op amp module */
    AMP3CON1bits.AMPEN = 1;
}

// </editor-fold>