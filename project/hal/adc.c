// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file adc.c
 *
 * @brief This module configures and enables the ADC Module 
 * 
 * Definitions in this file are for dsPIC33AK512MC510
 *
 * Component: ADC
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

#include "adc.h"

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">
/**
* <B> Function: InitializeADCs() </B>
*
* @brief Function initializes and enable the ADC Module
*        
* @param none.
* @return none.
* 
* @example
* <CODE> InitializeADCs(); </CODE>
*
*/
void InitializeADCs(void)
{
    /* Channel Configuration for MC1 - IA */
    /*ADC1 Channel 0 for IA : OA1OUT/AD1AN0/CVDAN2/CMP1A/RP3/RA2 */
    AD1CH0CON1bits.PINSEL = 0;       
    AD1CH0CON1bits.SAMC = 3;              
    AD1CH0CON1bits.FRAC = 0;
    AD1CH0CON1bits.DIFF = 0;

    /* Channel Configuration for MC1 - IB */
    /*ADC2 Channel 0 for IB : OA2OUT/AD2AN0/CVDAN16/CVDTX0/CMP2A/RP17/RB0 */    
    AD2CH0CON1bits.PINSEL = 0;       
    AD2CH0CON1bits.SAMC = 3;              
    AD2CH0CON1bits.FRAC = 0;
    AD2CH0CON1bits.DIFF = 0;
    
    /* Channel Configuration for MC1 - IBUS */
    /*ADC3 Channel 0 for IBUS : OA3OUT/AD3AN0/CVDAN5/CMP3A/RP6/INT0/RA5 */ 
    AD3CH0CON1bits.PINSEL = 0;      
    AD3CH0CON1bits.SAMC = 3;      
    AD3CH0CON1bits.FRAC = 0;
    AD3CH0CON1bits.DIFF = 0;
    
    /* Channel Configuration for POT */
    /*ADC2 Channel 1 for POT : AD2AN5/CVDAN31/CVDTX15/RP32/RB15 */     
    AD2CH1CON1bits.PINSEL = 5;      
    AD2CH1CON1bits.SAMC = 5;              
    AD2CH1CON1bits.FRAC = 0;
    AD2CH1CON1bits.DIFF = 0;
    
    /* Channel Configuration for VBUS */
    /*ADC1 Channel 4 for VBUS : AD3AN4/CVDTX29/RP81/RF0 */     
    AD3CH2CON1bits.PINSEL = 4;   
    AD3CH2CON1bits.SAMC = 5;         
    AD3CH2CON1bits.FRAC = 0;
    AD3CH2CON1bits.DIFF = 0;
    
    /* Channel Configuration for MC2 - IA */
    /* ADC4 Channel 0 for MC2 IA : DIM:082 - PIN #47 : AD4AN4/CVDAN26/CVDTX10/RP27/SCK2/IOMAF11/IOMBF11/RB10 */
    AD4CH0CON1bits.PINSEL = 4;       
    AD4CH0CON1bits.SAMC = 3;              
    AD4CH0CON1bits.FRAC = 0;
    AD4CH0CON1bits.DIFF = 0;
    
    /* Channel Configuration for MC2 - IB */
    /* ADC4 Channel 1 for MC2 IB : DIM:084 - PIN #46 : AD4ANN1/AD4AN2/CVDAN23/CVDTX7/CMP4C/IBIAS1/ISRC1/RP24/IOMAF0/RB7 */
    AD4CH1CON1bits.PINSEL = 2;       
    AD4CH1CON1bits.SAMC = 3;              
    AD4CH1CON1bits.FRAC = 0;
    AD4CH1CON1bits.DIFF = 0;
    
    /* Channel Configuration for MC3 - IA */
    /* ADC4 Channel 2 for MC3 IA : DIM:098 - PIN #43 : PGD1/AD4AN3/CVDAN19/CVDTX3/CMP4A/RP20/SDA1/RB3 */
    AD4CH2CON1bits.PINSEL = 3;       
    AD4CH2CON1bits.SAMC = 3;              
    AD4CH2CON1bits.FRAC = 0;
    AD4CH2CON1bits.DIFF = 0;
    
    /* Channel Configuration for MC3 - IB */
    /* ADC4 Channel 3 for MC3 IB : DIM:100 - PIN #44 : PGC1/AD4AN0/CVDAN20/CVDTX4/CMP4B/RP21/SCL1/RB4 */
    AD4CH3CON1bits.PINSEL = 0;       
    AD4CH3CON1bits.SAMC = 3;              
    AD4CH3CON1bits.FRAC = 0;
    AD4CH3CON1bits.DIFF = 0;
    
    /* Turn on the ADC Core 1 */   
    AD1CONbits.ON = 1;     
    /* Waiting till the ADC Core 1 is ready*/
    while(AD1CONbits.ADRDY == 0);  
    
    /* Turn on the ADC Core 2 */
    AD2CONbits.ON = 1;             
    /* Waiting till the ADC Core 2 is ready*/
    while(AD2CONbits.ADRDY == 0);   
    
    /* Turn on the ADC Core 3 */   
    AD3CONbits.ON = 1;     
    /* Waiting till the ADC Core 3 is ready*/
    while(AD3CONbits.ADRDY == 0);
    
    /* Turn on the ADC Core 4 */   
    AD4CONbits.ON = 1;     
    /* Waiting till the ADC Core 4 is ready*/
    while(AD4CONbits.ADRDY == 0); 
 
    /*AD2CH1 - POT used for ADC Interrupt for MC1*/
    /* Set ADC interrupt priority IPL 7  */ 
    _AD2CH1IP = 7;
    /* Clear ADC interrupt flag */
    _AD2CH1IF = 0;
    /* Disable the AD2CH1 interrupt  */
    _AD2CH1IE = 0;
    
    /*AD4CH1 - IB used for ADC Interrupt for MC2*/
    /* Set ADC interrupt priority IPL 7  */ 
    _AD4CH1IP = 7;
    /* Clear ADC interrupt flag */
    _AD4CH1IF = 0;
    /* Disable the AD4CH1 interrupt  */
    _AD4CH1IE = 0;
    
    /*AD4CH3 - IB used for ADC Interrupt for MC3*/
    /* Set ADC interrupt priority IPL 7  */ 
    _AD4CH3IP = 7;
    /* Clear ADC interrupt flag */
    _AD4CH3IF = 0;
    /* Disable the AD4CH3 interrupt  */
    _AD4CH3IE = 0;
  
    /*PWM1 ADC Trigger 1 for MC1 IA - AD1CH0*/
    AD1CH0CON1bits.TRG1SRC = 0b000100;      
    /*PWM1 ADC Trigger 1 for MC1 IB - AD2CH0*/
    AD2CH0CON1bits.TRG1SRC = 0b000100;    

    /*PWM1 ADC Trigger 1 for POT AD2CH1*/
    AD2CH1CON1bits.TRG1SRC = 0b000100;
    
    /*PWM1 ADC Trigger 1 for VBUS - AD3CH2*/
    AD3CH2CON1bits.TRG1SRC = 0b000100; 

    /*PWM6 ADC Trigger 1 MC2 - IA - AD4CH0 */
    AD4CH0CON1bits.TRG1SRC = 0b001110;
    /*PWM6 ADC Trigger 1 MC2 - IB - AD4CH1 */
    AD4CH1CON1bits.TRG1SRC = 0b001110;
    
    /*APWM1 ADC Trigger 1 for MC3 - IA - AD4CH2 */
    AD4CH2CON1bits.TRG1SRC = 0b010100;
    /*APWM1 ADC Trigger 1 for MC3 - IB - AD4CH3 */
    AD4CH3CON1bits.TRG1SRC = 0b010100; 

}

// </editor-fold>