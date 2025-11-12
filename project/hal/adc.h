// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file adc.h
 *
 * @brief This header file lists the functions and definitions - to configure 
 * and enable ADC Module and its features 
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

#ifndef _ADC_H
#define _ADC_H
        
// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">
        
#include <xc.h>
#include <stdint.h>
        
// </editor-fold>

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif
        
// <editor-fold defaultstate="expanded" desc="DEFINITIONS/CONSTANTS ">
/*Maximum count in 12-bit ADC*/
#define MAX_ADC_COUNT     4096.0f
#define HALF_ADC_COUNT    2048    
        
/*Converting 2^11 format to 2^15 format  */
#define MC1_ADCBUF_IA    (int16_t)(HALF_ADC_COUNT - AD1CH0DATA)<<4
#define MC1_ADCBUF_IB    (int16_t)(HALF_ADC_COUNT - AD2CH0DATA)<<4
#define MC1_ADCBUF_IBUS  (int16_t)((AD3CH0DATA- HALF_ADC_COUNT)<<4) 
        
#define MC1_ADCBUF_POT   (int16_t)AD2CH1DATA
#define MC_ADCBUF_VDC    (int16_t)AD3CH2DATA
        
#define MC2_ADCBUF_IA    (int16_t)(HALF_ADC_COUNT - AD4CH0DATA)<<4
#define MC2_ADCBUF_IB    (int16_t)(HALF_ADC_COUNT - AD4CH1DATA)<<4

#define MC3_ADCBUF_IA    (int16_t)(HALF_ADC_COUNT - AD4CH2DATA)<<4
#define MC3_ADCBUF_IB    (int16_t)(HALF_ADC_COUNT - AD4CH3DATA)<<4

/* POT (AD2CH1) is the ADC Interrupt source MC1 */
#define MC1_EnableADCInterrupt()            _AD2CH1IE = 1
#define MC1_DisableADCInterrupt()           _AD2CH1IE = 0
#define MC1_ADC_INTERRUPT                   _AD2CH1Interrupt  
#define MC1_ClearADCIF()           			_AD2CH1IF = 0  
#define MC1_ClearADCIF_ReadADCBUF()         MC1_ADCBUF_POT 

/* MC2 - IB (AD4CH1) is the ADC Interrupt source MC2 */
#define MC2_EnableADCInterrupt()            _AD4CH1IE = 1
#define MC2_DisableADCInterrupt()           _AD4CH1IE = 0
#define MC2_ADC_INTERRUPT                   _AD4CH1Interrupt  
#define MC2_ClearADCIF()           			_AD4CH1IF = 0  
#define MC2_ClearADCIF_ReadADCBUF()         MC2_ADCBUF_IB
        
/* MC3 - IB (AD4CH3) is the ADC Interrupt source MC2 */
#define MC3_EnableADCInterrupt()            _AD4CH3IE = 1
#define MC3_DisableADCInterrupt()           _AD4CH3IE = 0
#define MC3_ADC_INTERRUPT                   _AD4CH3Interrupt  
#define MC3_ClearADCIF()           			_AD4CH3IF = 0  
#define MC3_ClearADCIF_ReadADCBUF()         MC3_ADCBUF_IB
        
// </editor-fold>
        
// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">
        
void InitializeADCs(void);

// </editor-fold>

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
    
#endif      // end of __ADC_H

