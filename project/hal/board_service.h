// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file board_service.h
 *
 * @brief This header file lists variable types and interface functions for 
 * board services
 * 
 * Definitions in this file are for dsPIC33AK512MC510
 *
 * Component: BOARD SERVICE
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

#ifndef __BOARD_SERVICE_H
#define __BOARD_SERVICE_H

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include <stdbool.h>

#include <xc.h>

#include "clock.h"
#include "cmp.h"
#include "pwm.h"
#include "adc.h"
#include "port_config.h"
#include "timer1.h"
#include "measure.h"
#include "svm.h"
#include "delay.h"

// </editor-fold>

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif
        
// <editor-fold defaultstate="collapsed" desc="VARIABLE TYPE DEFINITIONS ">
        
/* Button Scanning State

  Description:
    This structure will host parameters required by Button scan routine.
 */
typedef enum tagBUTTON_STATE
{
    BUTTON_NOT_PRESSED = 0, /* wait for button to be pressed */
    BUTTON_PRESSED = 1,     /* button was pressed, check if short press or long press */
    BUTTON_DEBOUNCE = 2     /* button de-bounced after pressing */
} BUTTON_STATE;
    
// *****************************************************************************
/* Button data type

  Description:
    This structure will host parameters required by Button scan routine.
 */
typedef struct
{
   BUTTON_STATE state;
   uint16_t debounceCount;
   bool logicState;
   bool status;
} BUTTON_T;

/* Boot Strap charging routine - data types */
/**
 * State machine states for bootstrap charging routine
 */
typedef enum tagHAL_BOOTSTRAP_FSM_STATE
{
    /** Initialization state */
    BOOTSTRAP_INIT = 0,
    /** Waiting state */
    BOOTSTRAP_INIT_WAIT = 1,
    /** PHASE A bootstrap charging */
    BOOTSTRAP_PHASE_A_CHARGING = 2,
    /** PHASE B bootstrap charging */
    BOOTSTRAP_PHASE_B_CHARGING = 3,
    /** PHASE C bootstrap charging */
    BOOTSTRAP_PHASE_C_CHARGING = 4,
    /** Bootstrap charging is complete */
    BOOTSTRAP_COMPLETE = 5,
} HAL_BOOTSTRAP_FSM_STATE;

typedef struct tagHAL_BOOTSTRAP_STATE
{
    /** Bootstrap charging routine state */
    HAL_BOOTSTRAP_FSM_STATE state;
    /** Bootstrap charging duty cycles of Phase A, B, and C */
    uint32_t dutycycle;
    /** Count used to cause delay in several states */
    uint16_t delayCount;
} HAL_BOOTSTRAP_T;
// </editor-fold>

// <editor-fold defaultstate="expanded" desc="DEFINITIONS/CONSTANTS ">

/* Define INTERNAL_OPAMP_CONFIG_MC1 to use internal op-amp outputs(default) for MC1 current feedback, 
 * Undefine INTERNAL_OPAMP_CONFIG_MC1 to use external op-amp outputs from the 
   development board;Ensure the jumper resistors are modified on DIM  */
#define INTERNAL_OPAMP_CONFIG_MC1

/* Heart beat LED - specify in no.of milli second */
#define HEART_BEAT_LED_mSec                 250
/* Rate at which board service i executed**/
/* Specify the board service tick in milli second */
#define BOARD_SERVICE_TICK_mSec             1
/* Button De-bounce in milli Seconds */
#define BUTTON_DEBOUNCE_mSec                30
/* Wait delay before starting the bootstrap charging sequence in PWM Cycles */
#define BOOTSTRAP_INITIAL_DELAY_PWM_CYCLES  1

/* Heart beat LED count is incremented in every Timer 1 interrupt */
#define HEART_BEAT_LED_COUNT        (HEART_BEAT_LED_mSec*1000/TIMER1_PERIOD_uSec)
/* Board service tick count is incremented in every Timer 1 interrupt */
#define BOARD_SERVICE_TICK_COUNT    (BOARD_SERVICE_TICK_mSec*1000/TIMER1_PERIOD_uSec)
/* Button De-bounce count is incremented at every board service function call */
#define BUTTON_DEBOUNCE_COUNT       (BUTTON_DEBOUNCE_mSec/BOARD_SERVICE_TICK_mSec)

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

void BoardServiceInit(void);
void BoardServiceStepIsr(void);
void BoardService(void);
bool IsPressed_Button1(void);
bool IsPressed_Button2(void);
void HAL_InitPeripherals(void);
void HAL_MC1ResetPeripherals(void);
void HAL_MC2ResetPeripherals(void);
void HAL_MC3ResetPeripherals(void);
void HAL_MC1PWMDisableOutputs(void);
void HAL_MC2PWMDisableOutputs(void);
void HAL_MC3PWMDisableOutputs(void);
void HAL_MC1PWMEnableOutputs(void);
void HAL_MC2PWMEnableOutputs(void);
void HAL_MC3PWMEnableOutputs(void);
void HAL_PWMDutyCycleLimitCheck(MC_DUTYCYCLEOUT_T *,uint32_t, uint32_t);
void HAL_MC1PWMDutyCycleRegister_Set(MC_DUTYCYCLEOUT_T * );
void HAL_MC2PWMDutyCycleRegister_Set(MC_DUTYCYCLEOUT_T * );
void HAL_MC3PWMDutyCycleRegister_Set(MC_DUTYCYCLEOUT_T * );
void HAL_MC1MotorInputsRead(MCAPP_MEASURE_T *);
void HAL_MC2MotorInputsRead(MCAPP_MEASURE_T *);
void HAL_MC3MotorInputsRead(MCAPP_MEASURE_T *);

void HAL_MC1ClearPWMPCIFault(void);
void HAL_MC2ClearPWMPCIFault(void);
void HAL_MC3ClearPWMPCIFault(void);
void HAL_TrapHandler(void);

uint8_t HAL_MC1BootstrapChargeRoutine(MC_DUTYCYCLEOUT_T *,HAL_BOOTSTRAP_T *);
uint8_t HAL_MC2BootstrapChargeRoutine(MC_DUTYCYCLEOUT_T *,HAL_BOOTSTRAP_T *);
uint8_t HAL_MC3BootstrapChargeRoutine(MC_DUTYCYCLEOUT_T *,HAL_BOOTSTRAP_T *);
void HAL_MC1BootstrapChargeInit(HAL_BOOTSTRAP_T *);
void HAL_MC2BootstrapChargeInit(HAL_BOOTSTRAP_T *);
void HAL_MC3BootstrapChargeInit(HAL_BOOTSTRAP_T *);
// </editor-fold

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_SERVICE_H */
