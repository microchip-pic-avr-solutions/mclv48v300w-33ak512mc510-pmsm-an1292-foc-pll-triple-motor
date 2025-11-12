// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file pwm.h
 *
 * @brief This header file lists the functions and definitions to configure the 
 * PWM Module
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

#ifndef _PWM_H
#define _PWM_H

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <xc.h>
#include <stdint.h>
        
#include "clock.h"

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="DEFINITIONS/CONSTANTS ">
       
/* PWM Gen 1,2 & 3 for MC1 */
#define MC1_PWM_PDC1                PG1DCbits.DC
#define MC1_PWM_PDC2                PG2DCbits.DC
#define MC1_PWM_PDC3                PG3DCbits.DC
        
#define MC1_PWM_PHASE1              PG1PHASEbits.PHASE 
#define MC1_PWM_PHASE2              PG2PHASEbits.PHASE
#define MC1_PWM_PHASE3              PG3PHASEbits.PHASE
        
/* PWM Gen 6,7 & 8 for MC2 */
#define MC2_PWM_PDC1                PG6DCbits.DC
#define MC2_PWM_PDC2                PG7DCbits.DC
#define MC2_PWM_PDC3                PG8DCbits.DC
        
#define MC2_PWM_PHASE1              PG6PHASEbits.PHASE 
#define MC2_PWM_PHASE2              PG7PHASEbits.PHASE
#define MC2_PWM_PHASE3              PG8PHASEbits.PHASE
        
/* APWM Gen 1,2 & 3 for MC3 */
#define MC3_PWM_PDC1                APG1DCbits.DC
#define MC3_PWM_PDC2                APG2DCbits.DC
#define MC3_PWM_PDC3                APG3DCbits.DC
        
#define MC3_PWM_PHASE1              APG1PHASEbits.PHASE 
#define MC3_PWM_PHASE2              APG2PHASEbits.PHASE
#define MC3_PWM_PHASE3              APG3PHASEbits.PHASE
        
/* Define to enable PWM Fault PCI for MC1, MC2 and MC3*/        
#define ENABLE_PWM_FAULT_PCI_MC1
#define ENABLE_PWM_FAULT_PCI_MC2
#define ENABLE_PWM_FAULT_PCI_MC3
/* PWM 1 PCI Fault interrupt for MC1 */               
#define _PWM1Interrupt               _PWM1Interrupt
#define ClearPWM1IF()                _PWM1IF = 0 
#define EnablePWM1IF()               _PWM1IE = 1
#define DisablePWM1IF()              _PWM1IE = 0
/* PWM 6 PCI Fault interrupt for MC2 */           
#define _PWM6Interrupt               _PWM6Interrupt
#define ClearPWM6IF()                _PWM6IF = 0 
#define EnablePWM6IF()               _PWM6IE = 1
#define DisablePWM6IF()              _PWM6IE = 0
/* APWM 1 PCI Fault interrupt for MC3 */         
#define _APWM1Interrupt               _APWM1Interrupt
#define ClearAPWM1IF()                _APWM1IF = 0 
#define EnableAPWM1IF()               _APWM1IE = 1
#define DisableAPWM1IF()              _APWM1IE = 0

/*Specify PWM Module Clock in Mega Hertz*/
#define PWM_CLOCK_MHZ                           400 
        
/*Specify PWM Switching Frequency in Hertz*/
#define MC1_PWMFREQUENCY_HZ                     16000
#define MC2_PWMFREQUENCY_HZ                     16000
#define MC3_PWMFREQUENCY_HZ                     16000
  
/* Specify PWM module dead time in micro seconds*/
#define MC1_DEADTIME_MICROSEC                   0.75f
#define MC2_DEADTIME_MICROSEC                   0.75f
#define MC3_DEADTIME_MICROSEC                   0.75f
        
/*Specify ADC Triggering Point w.r.t PWM Output for sensing Analog Inputs*/ 
#define MC1_ADC_SAMPLING_POINT                  0
#define MC2_ADC_SAMPLING_POINT                  0
#define MC3_ADC_SAMPLING_POINT                  0
        
/* Loop time in seconds derived from PWM switching frequency*/
#define MC1_LOOPTIME_SEC                        (float)(1.0/MC1_PWMFREQUENCY_HZ)
#define MC2_LOOPTIME_SEC                        (float)(1.0/MC2_PWMFREQUENCY_HZ)
#define MC3_LOOPTIME_SEC                        (float)(1.0/MC3_PWMFREQUENCY_HZ)
/*Dead time in terms of PWM clock period for MC1 and MC2*/  
#define MC1_DEADTIME                            (uint32_t)(MC1_DEADTIME_MICROSEC*16*PWM_CLOCK_MHZ)
#define MC2_DEADTIME                            (uint32_t)(MC2_DEADTIME_MICROSEC*16*PWM_CLOCK_MHZ)
/*Dead time in terms of Aux PWM clock period for MC3*/  
#define MC3_DEADTIME                            (uint32_t)(MC3_DEADTIME_MICROSEC*4*PWM_CLOCK_MHZ)    
/* Loop Time in micro seconds*/
#define MC1_LOOPTIME_MICROSEC                   (MC1_LOOPTIME_SEC * 1000000.0f)
#define MC2_LOOPTIME_MICROSEC                   (MC2_LOOPTIME_SEC * 1000000.0f)
#define MC3_LOOPTIME_MICROSEC                   (MC3_LOOPTIME_SEC * 1000000.0f)
/*Loop time in terms of PWM clock period for MC1 and MC2*/
#define MC1_LOOPTIME_TCY                        (uint32_t)((MC1_LOOPTIME_MICROSEC*8*PWM_CLOCK_MHZ)-16)
#define MC2_LOOPTIME_TCY                        (uint32_t)((MC2_LOOPTIME_MICROSEC*8*PWM_CLOCK_MHZ)-16)
/*Loop time in terms of Aux PWM clock period for MC3*/
#define MC3_LOOPTIME_TCY                        (uint32_t)((MC3_LOOPTIME_MICROSEC*2*PWM_CLOCK_MHZ)-16)

        
/****Bootstrap Capacitor Charging Parameters*/
/*Specify bootstrap charging time in seconds (mention at least 0.01 seconds)*/
#define MC1_BOOTSTRAP_CHARGING_TIME_SECS        0.015
#define MC2_BOOTSTRAP_CHARGING_TIME_SECS        0.015
#define MC3_BOOTSTRAP_CHARGING_TIME_SECS        0.015
/*Specify bootstrap Capacitor Tickle Charge Time in Micro Seconds
 * if Bootstrap duty is less than MIN_DUTY, then MIN_DUTY will be applied */
#define MC1_TICKLE_CHARGE_TIME_MICROSEC         3.0
#define MC2_TICKLE_CHARGE_TIME_MICROSEC         3.0
#define MC3_TICKLE_CHARGE_TIME_MICROSEC         3.0
/*Calculate Bootstrap charging time in number of PWM Half Cycles*/
#define MC1_BOOTSTRAP_CHARGING_COUNTS           (uint32_t)(MC1_BOOTSTRAP_CHARGING_TIME_SECS/MC1_LOOPTIME_SEC)
#define MC2_BOOTSTRAP_CHARGING_COUNTS           (uint32_t)(MC2_BOOTSTRAP_CHARGING_TIME_SECS/MC2_LOOPTIME_SEC)
#define MC3_BOOTSTRAP_CHARGING_COUNTS           (uint32_t)(MC3_BOOTSTRAP_CHARGING_TIME_SECS/MC3_LOOPTIME_SEC)
/*Calculate Bootstrap Capacitor Tickle Charge duty in terms of PWM clock period for MC1 and MC2*/
#define MC1_TICKLE_CHARGE_DUTY                  (uint32_t)((MC1_TICKLE_CHARGE_TIME_MICROSEC*8*PWM_CLOCK_MHZ) + (MC1_DEADTIME/2))
#define MC2_TICKLE_CHARGE_DUTY                  (uint32_t)((MC2_TICKLE_CHARGE_TIME_MICROSEC*8*PWM_CLOCK_MHZ) + (MC2_DEADTIME/2)) 
/*Calculate Bootstrap Capacitor Tickle Charge duty in terms of Aux PWM clock period for MC3*/
#define MC3_TICKLE_CHARGE_DUTY                  (uint32_t)((MC3_TICKLE_CHARGE_TIME_MICROSEC*2*PWM_CLOCK_MHZ) + (MC3_DEADTIME/2))

/*Giving more room in the maximum duty for phase shunt to the measure current*/
#define MC1_MIN_DUTY                            (uint32_t)(MC1_DEADTIME + MC1_DEADTIME)
#define MC1_MAX_DUTY                            MC1_LOOPTIME_TCY - (uint32_t)(MC1_DEADTIME + MC1_DEADTIME)

#define MC2_MIN_DUTY                            (uint32_t)(MC2_DEADTIME + MC2_DEADTIME)
#define MC2_MAX_DUTY                            MC2_LOOPTIME_TCY - (uint32_t)(MC2_DEADTIME + MC2_DEADTIME)

#define MC3_MIN_DUTY                            (uint32_t)(MC3_DEADTIME + MC3_DEADTIME)
#define MC3_MAX_DUTY                            MC3_LOOPTIME_TCY - (uint32_t)(MC3_DEADTIME + MC3_DEADTIME)

// </editor-fold>      

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">
        
void InitPWMGenerators(void);  
void InitPWMGenerator1 (void);
void InitPWMGenerator2 (void);
void InitPWMGenerator3 (void);
void InitPWMGenerator5 (void);
void InitPWMGenerator6 (void);
void InitPWMGenerator7 (void);
void InitPWMGenerator8 (void);
void InitAuxPWMGenerator1 (void);
void InitAuxPWMGenerator2 (void);
void InitAuxPWMGenerator3 (void);
void InitDutyPWM123Generators(void);
void InitDutyPWM678Generators(void);
void InitDutyAPWM123Generators(void);
// </editor-fold>
        
#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of PWM_H


