// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file mc2_service.c
 *
 * @brief This module implements motor control.
 *
 * Component: MOTOR CONTROL APPLICATION
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

#include <stdint.h>
#include <stdbool.h>

#include <libq.h>
#include "diagnostics.h"
#include "board_service.h"
#include "mc2_init.h"
#include "foc_types.h"
#include "mc2_user_params.h"
#include "fault.h"
#include "generic_load_types.h"
#include "generic_load.h"
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Definitions ">


// </editor-fold>

// <editor-fold defaultstate="expanded" desc="VARIABLES ">

MC2APP_DATA_T mc2;
MC2APP_DATA_T *pMC2Data = &mc2;

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="STATIC FUNCTIONS ">

static void MC2APP_StateMachine(MC2APP_DATA_T *);
static void MCAPP_MC2ReceivedDataProcess(MC2APP_DATA_T *);

// </editor-fold>

/**
* <B> Function: void MC2APP_StateMachine (MC2APP_DATA_T *)  </B>
*
* @brief Application state machine.
*
* @param Pointer to the data structure containing Application parameters.
* @return none.
* 
* @example
* <CODE> MC2APP_StateMachine(&mc); </CODE>
*
*/
static void MC2APP_StateMachine(MC2APP_DATA_T *pMCData)
{
    MCAPP_MEASURE_T *pMotorInputs = pMCData->pMotorInputs;
    MCAPP_CONTROL_SCHEME_T *pControlScheme = pMCData->pControlScheme;
    MCAPP_LOAD_T *pLoad = pMCData->pLoad;
    HAL_BOOTSTRAP_T *pBootStrap = pMCData->pBootStrap;
 
    MCAPP_MC2ReceivedDataProcess(pMCData);

    switch(pMCData->appState)
    {
    case MCAPP_INIT:
        pMCData->HAL_PWMDisableOutputs();

        /* Stop the motor */
        pMCData->runCmd = 0;
        /* Reset the motor direction*/
        pMCData->directionCmd = 0;
                
        pMCData->MCAPP_ControlSchemeInit(pControlScheme);
        pMCData->MCAPP_InputsInit(pMotorInputs);
        pMCData->MCAPP_LoadInit(pLoad);
        pMCData->HAL_BootstrapChargeInit(pBootStrap);
        
        pMCData->appState = MCAPP_CMD_WAIT;
        
        break;
        
    case MCAPP_CMD_WAIT:
        if(pMCData->runCmd == 1)
        {
            if(pMCData->HAL_BootstrapCharging(pMC2Data->pPWMDuty,pBootStrap))
            {
                pMCData->appState = MCAPP_OFFSET;
            }
        }
       break;
       
    case MCAPP_OFFSET:
        /* Measure Initial Offsets */
        pMCData->MCAPP_MeasureOffset(pMotorInputs);

        if(pMCData->MCAPP_IsOffsetMeasurementComplete(pMotorInputs))
        {
            pMCData->appState = MCAPP_LOAD_START_READY_CHECK;
        }
       
        break;

    case MCAPP_LOAD_START_READY_CHECK:
        pMCData->MCAPP_GetProcessedInputs(pMotorInputs);
        pMCData->MCAPP_LoadStateMachine(pLoad);
        
        if(pMCData->MCAPP_IsLoadReadyToStart(pLoad))
        {
            /* Load is ready, start the motor */
            pMCData->HAL_PWMEnableOutputs();

            pMCData->MCAPP_LoadStartTransition(pControlScheme, pLoad); 

            pMCData->appState = MCAPP_RUN;
        }
        break;
            
    case MCAPP_RUN:
        /* Compensate motor current offsets */
        pMCData->MCAPP_GetProcessedInputs(pMotorInputs);
        /* Check for over current fault */
        if (MCAPP_OverCurrentFault_Detect(pMotorInputs, &pMCData->fault) == 1)
        {
            pMCData->appState = MCAPP_FAULT;
            break;
        } 

        pMCData->MCAPP_ControlStateMachine(pControlScheme);
        /* Check for control scheme faults */
        if(pControlScheme->faultStatus == 1) 
        {
            pMCData->appState = MCAPP_FAULT;
            break;
        }
        
        pMCData->MCAPP_LoadStateMachine(pLoad);

        if (pMCData->runCmd == 0)
        {
            /* Exit loop if motor not run */
            pMCData->appState = MCAPP_LOAD_STOP_READY_CHECK;
        }
        
        break;

    case MCAPP_LOAD_STOP_READY_CHECK:
        pMCData->MCAPP_LoadStateMachine(pLoad);
        
        /* Load is ready, stop the motor */
        pMCData->MCAPP_GetProcessedInputs(pMotorInputs);
        pMCData->MCAPP_ControlStateMachine(pControlScheme);
        
        if(pMCData->MCAPP_IsLoadReadyToStop(pLoad))
        {    
            pMCData->MCAPP_LoadStopTransition(pControlScheme, pLoad);
            pMCData->appState = MCAPP_STOP;
        }

        break;

    case MCAPP_STOP:
        pMCData->HAL_PWMDisableOutputs();
        pMCData->appState = MCAPP_INIT;
        
        break;
        
    case MCAPP_FAULT:
        pMCData->HAL_PWMDisableOutputs();
        break;
        
    default:
        pMCData->HAL_PWMDisableOutputs();
        break;     

    } /* end of switch-case */
    
    
    /* Fault Handler */
    if ((pControlScheme->faultStatus == 1)||(pMCData->appState == MCAPP_FAULT))
    {
        pMCData->HAL_PWMDisableOutputs();
    } 
}

/**
* <B> Function: MC2_ADC_INTERRUPT()  </B>
*
* @brief ADC interrupt vector ,and it performs following actions:
*        (1) Increments DiagnosticsStepIsr for X2C Scope 
*        (2) Reads motor 1 phase currents,bus current and phase voltage
*            feedbacks from ADC data buffers.
*        (3) Executes Field Oriented Control based on the current,voltage
*            feedbacks.
*        (4) Loads duty cycle values generated by FOC to the registers
*            of PWM Generators controlling motor 1.
* 
* @param none.
* @return none.
* 
* @example none
*
*/
void __attribute__((__interrupt__,no_auto_psv)) MC2_ADC_INTERRUPT()
{

    int16_t __attribute__((__unused__)) adcBuffer;
  
    pMC2Data->HAL_MotorInputsRead(pMC2Data->pMotorInputs);
    
    MC2APP_StateMachine(pMC2Data);

    /* Do not set duty cycle unless inverter 2 is connected*/
    pMC2Data->HAL_PWMSetDutyCycles(pMC2Data->pPWMDuty);
        
    adcBuffer = MC2_ClearADCIF_ReadADCBUF();
    
	MC2_ClearADCIF();

}
/**
* <B> Function: _PWM6Interrupt()     </B>
*
* @brief Function to handle PWM Fault Interrupt from Fault PCI of PWM Gen 6
*        
* @param none.
* @return none.
* 
* @example none
* 
*/
void __attribute__((__interrupt__,no_auto_psv)) _PWM6Interrupt()
{
    HAL_MC2ClearPWMPCIFault();
    mc2.appState = MCAPP_FAULT;
    mc2.fault.faultState = MCAPP_OVERCURRENT_FAULT_DCBUS;
    ClearPWM6IF(); 
}

/**
* <B> Function: void MCAPP_MC2ServiceInit (void)  </B>
*
* @brief Function to initialize the MC2 parameters
*
* @param Pointer to the data structure containing Application parameters.
* @return none.
* 
* @example
* <CODE> MCAPP_MC2ServiceInit(); </CODE>
*
*/
void MCAPP_MC2ServiceInit(void)
{
    MCAPP_MC2ParamsInit(pMC2Data);

    MC2_ClearADCIF();
    MC2_EnableADCInterrupt();
    
    pMC2Data->HAL_PWMDisableOutputs();
}

/**
* <B> Function: void MCAPP_MC2InputBufferSet (uint8_t, uint8_t)  </B>
*
* @brief Function store the run command and direction change command
*
* @param run command
* @return none.
* 
* @example
* <CODE> MCAPP_MC2InputBufferSet(runCmdMC2,directionCmdMC2); </CODE>
*
*/
void MCAPP_MC2InputBufferSet(uint8_t runCmd, uint8_t directionCmd )
{ 
    pMC2Data->runCmdBuffer = runCmd;
    pMC2Data->directionCmdBuffer = directionCmd;
}

/**
* <B> Function: void MCAPP_MC2ReceivedDataProcess (MC2APP_DATA_T *)  </B>
*
* @brief Function to process the received ADC and digital input data
*
* @param Pointer to the data structure containing Application parameters.
* @return none.
* 
* @example
* <CODE> MCAPP_MC2ReceivedDataProcess(&mc); </CODE>
*
*/
static void MCAPP_MC2ReceivedDataProcess(MC2APP_DATA_T *pMCData)
{
    MCAPP_CONTROL_SCHEME_T *pControlScheme = pMCData->pControlScheme;
    MCAPP_MOTOR_T *pMotor = pMCData->pMotor;
    MCAPP_MEASURE_T *pMotorInputs = pMCData->pMotorInputs;
    float potValueNormalized;
    
    pMCData->runCmd = pMCData->runCmdBuffer;
    pMCData->directionCmd = pMCData->directionCmdBuffer;

    potValueNormalized = ( (float)pMotorInputs->measurePot/MAX_ADC_COUNT); 
    
    pMCData->targetSpeed =  pMotor->MinSpeed + 
                ((pMotor->MaxSpeed - pMotor->MinSpeed) * potValueNormalized) ;
    
    if(pMCData->directionCmd)
    {
        pMCData->targetSpeed = -pMCData->targetSpeed;
    }
    if(pMCData->runCmd == 1)
    {
        pControlScheme->ctrlParam.targetSpeed = pMCData->targetSpeed;        
    }
    else
    {
        pControlScheme->ctrlParam.targetSpeed = 0.0;      
    }
    
    
    pMotorInputs->measureVdc.value = pMotorInputs->measureVdc.count * MC2_PEAK_VOLTAGE/MAX_ADC_COUNT;
}


