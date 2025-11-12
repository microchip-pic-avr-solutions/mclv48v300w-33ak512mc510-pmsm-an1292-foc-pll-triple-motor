// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file mc3_init.h
 *
 * @brief This module initializes data structure holding motor control
 * parameters required to run motor 1 using field oriented control.
 * In this application to initialize variable required to run the application.
 *
 * Component: APPLICATION (Motor Control 1 - mc3)
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

#ifndef __MC3_INIT_H
#define __MC3_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include <stdbool.h>

#include "measure.h"
#include "motor_types.h"
#include "svm.h"
#include "foc.h"
#include "foc_types.h"
#include "fault.h"
#include "generic_load.h"
#include "board_service.h"
    
// </editor-fold>

    
// <editor-fold defaultstate="collapsed" desc="DEFINITIONS ">
    
#define MCAPP_CONTROL_SCHEME_T              MCAPP_FOC_T
    
// </editor-fold>
    
typedef enum
{
    MCAPP_INIT = 0,                     /* Initialize Run time parameters */
    MCAPP_CMD_WAIT = 1,                 /* Wait for Run command */
    MCAPP_OFFSET = 2,                   /* Measure current offsets */
    MCAPP_LOAD_START_READY_CHECK = 3,   /* Wait for load to be ready to start */
    MCAPP_RUN = 4,                      /* Run the motor */
    MCAPP_LOAD_STOP_READY_CHECK = 5,    /* Wait for load to be ready to stop */
    MCAPP_STOP = 6,                     /* Stop the motor */
    MCAPP_FAULT = 7,                    /* Motor is in Fault mode */

}MCAPP_STATE_T;
    
// <editor-fold defaultstate="collapsed" desc="VARIABLE TYPE DEFINITIONS ">

typedef struct
{
    uint8_t
        appState,                   /* Application State */
        runCmd,                     /* Run command for motor */
        runCmdBuffer,               /* Run command buffer for validation */
        directionCmd,               /* Direction Change command for motor */
        directionCmdBuffer;         /* Direction Change command buffer for validation */
    
    float        
        targetSpeed;                /* Target motor Speed */
    
    MCAPP_MEASURE_T
        motorInputs;
    
    MCAPP_MOTOR_T
        motor;
    
    MCAPP_CONTROL_SCHEME_T
        controlScheme;              /* Motor Control parameters */
    
    MCAPP_LOAD_T
        load;                       /* Load parameters */
    
    MC_DUTYCYCLEOUT_T
        PWMDuty;
    
    MCAPP_FAULT_T
        fault;
    
    HAL_BOOTSTRAP_T
        bootStrap;

    MCAPP_MEASURE_T *pMotorInputs;
    MCAPP_MOTOR_T *pMotor;
    MCAPP_CONTROL_SCHEME_T *pControlScheme;
    MCAPP_LOAD_T *pLoad;
    MC_DUTYCYCLEOUT_T *pPWMDuty;
    HAL_BOOTSTRAP_T *pBootStrap;
        
    /* Function pointers for motor inputs */    
    void (*MCAPP_InputsInit) (MCAPP_MEASURE_T *);
    void (*MCAPP_MeasureOffset) (MCAPP_MEASURE_T *);
    void (*MCAPP_GetProcessedInputs) (MCAPP_MEASURE_T *);
    int16_t (*MCAPP_IsOffsetMeasurementComplete) (MCAPP_MEASURE_T *);
    void (*HAL_MotorInputsRead) (MCAPP_MEASURE_T *);
    
    /* Function pointers for control scheme */
    void (*MCAPP_ControlSchemeInit) (MCAPP_CONTROL_SCHEME_T *);
    void (*MCAPP_ControlStateMachine) (MCAPP_CONTROL_SCHEME_T *);
    
    /* Function pointers for load */
    void (*MCAPP_LoadStateMachine) (MCAPP_LOAD_T *);
    void (*MCAPP_LoadInit) (MCAPP_LOAD_T *);
    
    void (*MCAPP_LoadStartTransition) (MCAPP_CONTROL_SCHEME_T *, 
                                        MCAPP_LOAD_T *);
    void (*MCAPP_LoadStopTransition) (MCAPP_CONTROL_SCHEME_T *, 
                                        MCAPP_LOAD_T *);
    
    uint8_t (*MCAPP_IsLoadReadyToStart) (MCAPP_LOAD_T *);
    uint8_t (*MCAPP_IsLoadReadyToStop) (MCAPP_LOAD_T *);
    
    /* Function pointers for motor outputs */
    void (*HAL_PWMSetDutyCycles)(MC_DUTYCYCLEOUT_T *);
    void (*HAL_PWMEnableOutputs) (void);
    void (*HAL_PWMDisableOutputs) (void);
    void (*MCAPP_HALSetVoltageVector) (int16_t);
    
    /* Function pointer for bootstrap capacitor charging */
    uint8_t (*HAL_BootstrapCharging)(MC_DUTYCYCLEOUT_T *,HAL_BOOTSTRAP_T *);
    void    (*HAL_BootstrapChargeInit)(HAL_BOOTSTRAP_T *);

}MC3APP_DATA_T;

// </editor-fold>
    
// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

void MCAPP_MC3ParamsInit(MC3APP_DATA_T *);

// </editor-fold>


#ifdef __cplusplus
}
#endif

#endif /* end of __MC3_INIT_H */
