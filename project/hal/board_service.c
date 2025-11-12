// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file board_service.c
 *
 * @brief This module implements the board service routines 
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

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "board_service.h"
#include "mc1_calc_params.h"

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="VARIABLES">

BUTTON_T buttonStartStop;
BUTTON_T buttonDirectionChange;

uint16_t boardServiceISRCounter = 0;

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="STATIC FUNCTIONS ">

static void ButtonGroupInitialize(void);
static void ButtonScan(BUTTON_T * ,bool);

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">
/**
* <B> Function: IsPressed_Button1() </B>
*
* @brief Function monitors the actuation of push button
*        
* @param none.
* @return boolean for button actuation.
* 
* @example
* <CODE> IsPressed_Button1(); </CODE>
*
*/
bool IsPressed_Button1(void)
{
    if(buttonStartStop.status)
    {
        buttonStartStop.status = false;
        return true;
    }
    else
    {
        return false;
    }
}
/**
* <B> Function: IsPressed_Button2() </B>
*
* @brief Function monitors the actuation of push button
*        
* @param none.
* @return boolean for button actuation.
* 
* @example
* <CODE> IsPressed_Button2(); </CODE>
*
*/
bool IsPressed_Button2(void)
{
    if (buttonDirectionChange.status)
    {
        buttonDirectionChange.status = false;
        return true;
    }
    else
    {
        return false;
    }
}

/**
* <B> Function: BoardServiceStepIsr() </B>
*
* @brief Function to increment the board service routine counter
*        
* @param none.
* @return none.
* 
* @example
* <CODE> BoardServiceStepIsr(); </CODE>
*
*/
void BoardServiceStepIsr(void)
{
    if (boardServiceISRCounter <  BOARD_SERVICE_TICK_COUNT)
    {
        boardServiceISRCounter += 1;
    }
}

/**
* <B> Function: BoardService() </B>
*
* @brief Function to reset the board service routine counter 
*        
* @param none.
* @return none.
* 
* @example
* <CODE> BoardService(); </CODE>
*
*/
void BoardService(void)
{
    if (boardServiceISRCounter ==  BOARD_SERVICE_TICK_COUNT)
    {
        /* Button scanning loop for Button 1 to start Motor A */
        ButtonScan(&buttonStartStop,BUTTON_START_STOP);

        /* Button scanning loop for SW2 to enter into filed
            weakening mode */
        ButtonScan(&buttonDirectionChange,BUTTON_DIRECTION_CHANGE);

        boardServiceISRCounter = 0;
    }
}

/**
* <B> Function: BoardServiceInit() </B>
*
* @brief Function to initialize the board service routine
*        
* @param none.
* @return none.
* 
* @example
* <CODE> BoardServiceInit(); </CODE>
*
*/
void BoardServiceInit(void)
{
    ButtonGroupInitialize();
    boardServiceISRCounter = BOARD_SERVICE_TICK_COUNT;
}

/**
* <B> Function: ButtonScan() </B>
*
* @brief Function to monitor the push button actuation with de-bounce delay
*        
* @param none.
* @return none.
* 
* @example
* <CODE> ButtonScan(); </CODE>
*
*/
void ButtonScan(BUTTON_T *pButton,bool button) 
{
    if (button == true) 
    {
        if (pButton->debounceCount < BUTTON_DEBOUNCE_COUNT) 
        {
            pButton->debounceCount--;
            pButton->state = BUTTON_DEBOUNCE;
        }
    } 
    else 
    {
        if (pButton->debounceCount < BUTTON_DEBOUNCE_COUNT) 
        {
            pButton->state = BUTTON_NOT_PRESSED;
        } 
        else 
        {
            pButton->state = BUTTON_PRESSED;
            pButton->status = true;
        }
        pButton->debounceCount = 0;
    }
}

/**
* <B> Function: ButtonGroupInitialize() </B>
*
* @brief Function to initialize the push button states
*        
* @param none.
* @return none.
* 
* @example
* <CODE> ButtonGroupInitialize(); </CODE>
*
*/
void ButtonGroupInitialize(void)
{
    buttonStartStop.state = BUTTON_NOT_PRESSED;
    buttonStartStop.debounceCount = 0;
    buttonStartStop.state = false;

    buttonDirectionChange.state = BUTTON_NOT_PRESSED;
    buttonDirectionChange.debounceCount = 0;
    buttonDirectionChange.state = false;
}

/**
* <B> Function: HAL_InitPeripherals() </B>
*
* @brief Function to initialize the peripherals(Op-Amp, ADC, CMP, DAC and PWM)
*        
* @param none.
* @return none.
* 
* @example
* <CODE> HAL_InitPeripherals(); </CODE>
*
*/
void HAL_InitPeripherals(void)
{                
#ifdef INTERNAL_OPAMP_CONFIG_MC1
    /* MC1 current feedback can be amplified using op-amp internal to dsPIC */
    OpampConfig();
#endif
    
    InitializeADCs();
    
    /* CMP3D is connected to IBUS of MC1
       Comparator output can be used as a source to PWM Fault PCI */
    InitializeCMPs();  
    CMP3_ReferenceSet(MC1_CMP_REF_DCBUS_FAULT);
    CMP3_ModuleEnable(true);

    /*400ms POR delay for IBUS_EXT signal coming from MCP651S in Dev Board*/
    __delay_ms(400);
    
    /* Make sure ADC does not generate interrupt while initializing parameters*/
    MC1_DisableADCInterrupt();  
    MC2_DisableADCInterrupt();
    MC3_DisableADCInterrupt();
    
    InitPWMGenerators(); 
    
    /*Clearing and enabling PWM Interrupt to handle PCI Fault for MC1*/
    ClearPWM1IF();
    EnablePWM1IF();
    
    /*Clearing and enabling PWM Interrupt to handle PCI Fault for MC2*/
    ClearPWM6IF();
    EnablePWM6IF();
    
    /*Clearing and enabling PWM Interrupt to handle PCI Fault for MC3*/
    ClearAPWM1IF();
    EnableAPWM1IF();
    
    /*Timer 1 initialization*/
    TIMER1_Initialize();
    TIMER1_InputClockSet();
    TIMER1_PeriodSet(TIMER1_PERIOD_COUNT);
    TIMER1_InterruptPrioritySet(5);
    TIMER1_InterruptFlagClear();
    TIMER1_InterruptEnable(); 
    TIMER1_ModuleStart();

}

/**
* <B> Function: HAL_MC1ResetPeripherals() </B>
*
* @brief Function to reset the clear ADC interrupt and disable PWM outputs
*        
* @param none.
* @return none.
* 
* @example
* <CODE> HAL_MC1ResetPeripherals(); </CODE>
*
*/
void HAL_MC1ResetPeripherals(void)
{
    MC1_ClearADCIF();
    MC1_EnableADCInterrupt();
    HAL_MC1PWMDisableOutputs();
}

/**
* <B> Function: HAL_MC2ResetPeripherals() </B>
*
* @brief Function to reset the clear ADC interrupt and disable PWM outputs
*        
* @param none.
* @return none.
* 
* @example
* <CODE> HAL_MC2ResetPeripherals(); </CODE>
*
*/
void HAL_MC2ResetPeripherals(void)
{
    MC2_ClearADCIF();
    MC2_EnableADCInterrupt();
    HAL_MC2PWMDisableOutputs();
}

/**
* <B> Function: HAL_MC3ResetPeripherals() </B>
*
* @brief Function to reset the clear ADC interrupt and disable PWM outputs
*        
* @param none.
* @return none.
* 
* @example
* <CODE> HAL_MC3ResetPeripherals(); </CODE>
*
*/
void HAL_MC3ResetPeripherals(void)
{
    MC3_ClearADCIF();
    MC3_EnableADCInterrupt();
    HAL_MC3PWMDisableOutputs();
}

/**
* <B> Function: HAL_MC1PWMEnableOutputs() </B>
*
* @brief Function to enable the PWM outputs -  override is removed
* PWM generator controls the PWM outputs now
*        
* @param none.
* @return none.
* 
* @example
* <CODE> HAL_MC1PWMEnableOutputs(); </CODE>
*
*/
void HAL_MC1PWMEnableOutputs(void)
{
    /* Set PWM Duty Cycles */
    MC1_PWM_PDC3 = 0;
    MC1_PWM_PDC2 = 0;
    MC1_PWM_PDC1 = 0;
	
	/*  0 = PWM Generator provides data for the PWM3H pin */
    PG3IOCON2bits.OVRENH = 0; 
    /*  0 = PWM Generator provides data for the PWM3L pin */
    PG3IOCON2bits.OVRENL = 0; 
    
    /*  0 = PWM Generator provides data for the PWM2H pin */
    PG2IOCON2bits.OVRENH = 0;
    /*  0 = PWM Generator provides data for the PWM2L pin */
    PG2IOCON2bits.OVRENL = 0; 
    
    /*  0 = PWM Generator provides data for the PWM1H pin */
    PG1IOCON2bits.OVRENH = 0;  
    /*  0 = PWM Generator provides data for the PWM1L pin */
    PG1IOCON2bits.OVRENL = 0;     
}

/**
* <B> Function: HAL_MC2PWMEnableOutputs() </B>
*
* @brief Function to enable the PWM outputs -  override is removed
* PWM generator controls the PWM outputs now
*        
* @param none.
* @return none.
* 
* @example
* <CODE> HAL_MC2PWMEnableOutputs(); </CODE>
*
*/
void HAL_MC2PWMEnableOutputs(void)
{
    /* Set PWM Duty Cycles */
    MC2_PWM_PDC3 = 0;
    MC2_PWM_PDC2 = 0;
    MC2_PWM_PDC1 = 0;
	
	/*  0 = PWM Generator provides data for the PWM8H pin */
    PG8IOCON2bits.OVRENH = 0; 
    /*  0 = PWM Generator provides data for the PWM8L pin */
    PG8IOCON2bits.OVRENL = 0; 
    
    /*  0 = PWM Generator provides data for the PWM7H pin */
    PG7IOCON2bits.OVRENH = 0;
    /*  0 = PWM Generator provides data for the PWM7L pin */
    PG7IOCON2bits.OVRENL = 0; 
    
    /*  0 = PWM Generator provides data for the PWM6H pin */
    PG6IOCON2bits.OVRENH = 0;  
    /*  0 = PWM Generator provides data for the PWM6L pin */
    PG6IOCON2bits.OVRENL = 0;     
}

/**
* <B> Function: HAL_MC3PWMEnableOutputs() </B>
*
* @brief Function to enable the PWM outputs -  override is removed
* PWM generator controls the PWM outputs now
*        
* @param none.
* @return none.
* 
* @example
* <CODE> HAL_MC3PWMEnableOutputs(); </CODE>
*
*/
void HAL_MC3PWMEnableOutputs(void)
{
    /* Set PWM Duty Cycles */
    MC3_PWM_PDC3 = 0;
    MC3_PWM_PDC2 = 0;
    MC3_PWM_PDC1 = 0;
	
	/*  0 = PWM Generator provides data for the APWM3H pin */
    APG3IOCON2bits.OVRENH = 0; 
    /*  0 = PWM Generator provides data for the APWM3L pin */
    APG3IOCON2bits.OVRENL = 0; 
    
    /*  0 = PWM Generator provides data for the APWM2H pin */
    APG2IOCON2bits.OVRENH = 0;
    /*  0 = PWM Generator provides data for the APWM2L pin */
    APG2IOCON2bits.OVRENL = 0; 
    
    /*  0 = PWM Generator provides data for the APWM1H pin */
    APG1IOCON2bits.OVRENH = 0;  
    /*  0 = PWM Generator provides data for the APWM1L pin */
    APG1IOCON2bits.OVRENL = 0;     
}

/**
* <B> Function: HAL_MC1PWMDisableOutputs() </B>
*
* @brief Function to disable the PWM outputs -  override is activated
* OVRDAT<> register controls the PWM outputs now
*        
* @param none.
* @return none.
* 
* @example
* <CODE> HAL_MC1PWMDisableOutputs(); </CODE>
*
*/
void HAL_MC1PWMDisableOutputs(void)
{
    /* Set PWM Duty Cycles */
    MC1_PWM_PDC3 = 0;
    MC1_PWM_PDC2 = 0;
    MC1_PWM_PDC1 = 0; 
    
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
}

/**
* <B> Function: HAL_MC2PWMDisableOutputs() </B>
*
* @brief Function to disable the PWM outputs -  override is activated
* OVRDAT<> register controls the PWM outputs now
*        
* @param none.
* @return none.
* 
* @example
* <CODE> HAL_MC2PWMDisableOutputs(); </CODE>
*
*/
void HAL_MC2PWMDisableOutputs(void)
{
    /* Set PWM Duty Cycles */
    MC2_PWM_PDC3 = 0;
    MC2_PWM_PDC2 = 0;
    MC2_PWM_PDC1 = 0; 
    
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
}

/**
* <B> Function: HAL_MC3PWMDisableOutputs() </B>
*
* @brief Function to disable the PWM outputs -  override is activated
* OVRDAT<> register controls the PWM outputs now
*        
* @param none.
* @return none.
* 
* @example
* <CODE> HAL_MC3PWMDisableOutputs(); </CODE>
*
*/
void HAL_MC3PWMDisableOutputs(void)
{
    /* Set PWM Duty Cycles */
    MC3_PWM_PDC3 = 0;
    MC3_PWM_PDC2 = 0;
    MC3_PWM_PDC1 = 0; 
    
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
}
/**
* <B> Function: HAL_PWMDutyCycleLimitCheck(MC_DUTYCYCLEOUT_T *uint32_t, uint32_t) </B>
*
* @brief Function to limit the PWM duty cycles between Min and Max duty
*        
* @param Pointer to the data structure containing duty cycles
* @param Minimum Duty
* @param Maximum Duty
* @return none.
* 
* @example
* <CODE> HAL_PWMDutyCycleLimitCheck(pPdc,min,max); </CODE>
*
*/
void HAL_PWMDutyCycleLimitCheck(MC_DUTYCYCLEOUT_T *pdc,uint32_t min, uint32_t max)
{
    if(pdc->dutycycle3 < min)
    {
        pdc->dutycycle3 = min;
    }
    if(pdc->dutycycle2 < min)
    {
        pdc->dutycycle2 = min;
    }
    if(pdc->dutycycle1 < min)
    {
        pdc->dutycycle1 = min;
    }

    if(pdc->dutycycle3 > max)
    {
        pdc->dutycycle3 = max;
    }
    if(pdc->dutycycle2 > max)
    {
        pdc->dutycycle2 = max;
    }
    if(pdc->dutycycle1 > max)
    {
        pdc->dutycycle1 = max;
    }
}

/**
* <B> Function: HAL_MC1PWMDutyCycleRegister_Set(MC_DUTYCYCLEOUT_T *) </B>
*
* @brief Function to set the duty cycle values to the PDC registers
*        
* @param Pointer to the data structure containing duty cycles
* @return none.
* 
* @example
* <CODE> HAL_MC1PWMDutyCycleRegister_Set(pMC1Data->pPWMDuty); </CODE>
*
*/
void HAL_MC1PWMDutyCycleRegister_Set(MC_DUTYCYCLEOUT_T *pPdc)
{
    
    HAL_PWMDutyCycleLimitCheck(pPdc,MC1_MIN_DUTY,MC1_MAX_DUTY);
    MC1_PWM_PDC3 = (uint32_t)(pPdc->dutycycle3);
    MC1_PWM_PDC2 = (uint32_t)(pPdc->dutycycle2);
    MC1_PWM_PDC1 = (uint32_t)(pPdc->dutycycle1);
}

/**
* <B> Function: HAL_MC2PWMDutyCycleRegister_Set(MC_DUTYCYCLEOUT_T *) </B>
*
* @brief Function to set the duty cycle values to the PDC registers
*        
* @param Pointer to the data structure containing duty cycles
* @return none.
* 
* @example
* <CODE> HAL_MC2PWMDutyCycleRegister_Set(pMC1Data->pPWMDuty); </CODE>
*
*/
void HAL_MC2PWMDutyCycleRegister_Set(MC_DUTYCYCLEOUT_T *pPdc)
{
    
    HAL_PWMDutyCycleLimitCheck(pPdc,MC2_MIN_DUTY,MC2_MAX_DUTY);
    MC2_PWM_PDC3 = (uint32_t)(pPdc->dutycycle3);
    MC2_PWM_PDC2 = (uint32_t)(pPdc->dutycycle2);
    MC2_PWM_PDC1 = (uint32_t)(pPdc->dutycycle1);
}

/**
* <B> Function: HAL_MC3PWMDutyCycleRegister_Set(MC_DUTYCYCLEOUT_T *) </B>
*
* @brief Function to set the duty cycle values to the PDC registers
*        
* @param Pointer to the data structure containing duty cycles
* @return none.
* 
* @example
* <CODE> HAL_MC3PWMDutyCycleRegister_Set(pMC1Data->pPWMDuty); </CODE>
*
*/
void HAL_MC3PWMDutyCycleRegister_Set(MC_DUTYCYCLEOUT_T *pPdc)
{
    
    HAL_PWMDutyCycleLimitCheck(pPdc,MC3_MIN_DUTY,MC3_MAX_DUTY);
    MC3_PWM_PDC3 = (uint32_t)(pPdc->dutycycle3);
    MC3_PWM_PDC2 = (uint32_t)(pPdc->dutycycle2);
    MC3_PWM_PDC1 = (uint32_t)(pPdc->dutycycle1);
}

/**
* <B> Function: HAL_TrapHandler(void) </B>
*
* @brief Function handle the traps
*        
* @param none.
* @return none.
* 
* @example
* <CODE> HAL_TrapHandler(); </CODE>
*
*/
void HAL_TrapHandler(void)
{
    HAL_MC1PWMDisableOutputs();
    while (1)
    {
        Nop();
        Nop();
        Nop();
    }
}

/**
* <B> Function: HAL_MC1MotorInputsRead(MCAPP_MEASURE_T *)  </B>
*
* @brief Function to assign the variables with respective ADC buffers
*        
* @param Pointer to the data structure containing measured currents.
* @return none.
* 
* @example
* <CODE> HAL_MC1MotorInputsRead(pMotorInputs); </CODE>
*
*/
void HAL_MC1MotorInputsRead(MCAPP_MEASURE_T *pMotorInputs)
{
    pMotorInputs->measureCurrent.Ia = MC1_ADCBUF_IA;
    pMotorInputs->measureCurrent.Ib = MC1_ADCBUF_IB;
    pMotorInputs->measureCurrent.Ibus = MC1_ADCBUF_IBUS;
    pMotorInputs->measurePot = MC1_ADCBUF_POT;
    pMotorInputs->measureVdc.count = MC_ADCBUF_VDC;
}

/**
* <B> Function: HAL_MC2MotorInputsRead(MCAPP_MEASURE_T *)  </B>
*
* @brief Function to assign the variables with respective ADC buffers
*        
* @param Pointer to the data structure containing measured currents.
* @return none.
* 
* @example
* <CODE> HAL_MC2MotorInputsRead(pMotorInputs); </CODE>
*
*/
void HAL_MC2MotorInputsRead(MCAPP_MEASURE_T *pMotorInputs)
{
    pMotorInputs->measureCurrent.Ia = MC2_ADCBUF_IA;
    pMotorInputs->measureCurrent.Ib = MC2_ADCBUF_IB;
    pMotorInputs->measureCurrent.Ibus = MC1_ADCBUF_IBUS;
    pMotorInputs->measurePot = MC1_ADCBUF_POT;
    pMotorInputs->measureVdc.count = MC_ADCBUF_VDC;
}

/**
* <B> Function: HAL_MC3MotorInputsRead(MCAPP_MEASURE_T *)  </B>
*
* @brief Function to assign the variables with respective ADC buffers
*        
* @param Pointer to the data structure containing measured currents.
* @return none.
* 
* @example
* <CODE> HAL_MC3MotorInputsRead(pMotorInputs); </CODE>
*
*/
void HAL_MC3MotorInputsRead(MCAPP_MEASURE_T *pMotorInputs)
{
    pMotorInputs->measureCurrent.Ia = MC3_ADCBUF_IA;
    pMotorInputs->measureCurrent.Ib = MC3_ADCBUF_IB;
    pMotorInputs->measureCurrent.Ibus = MC1_ADCBUF_IBUS;
    pMotorInputs->measurePot = MC1_ADCBUF_POT;
    pMotorInputs->measureVdc.count = MC_ADCBUF_VDC;
}

/**
* <B> Function: HAL_MC1ClearPWMPCIFault(void)  </B>
*
* @brief Function to terminate the PWM Fault PWM event through software
*        
* @param none.
* @return none.
* 
* @example
* <CODE> HAL_MC1ClearPWMPCIFault(); </CODE>
*
*/
void HAL_MC1ClearPWMPCIFault(void)
{
    /* write of '1' to SWTERM bit will produce a PCI Fault termination event */
    PG1F1PCI1bits.SWTERM = 1;
    PG2F1PCI1bits.SWTERM = 1;
    PG3F1PCI1bits.SWTERM = 1;  
}

/**
* <B> Function: HAL_MC2ClearPWMPCIFault(void)  </B>
*
* @brief Function to terminate the PWM Fault PWM event through software
*        
* @param none.
* @return none.
* 
* @example
* <CODE> HAL_MC2ClearPWMPCIFault(); </CODE>
*
*/
void HAL_MC2ClearPWMPCIFault(void)
{
    /* write of '1' to SWTERM bit will produce a PCI Fault termination event */
    PG6F1PCI1bits.SWTERM = 1;
    PG7F1PCI1bits.SWTERM = 1;
    PG8F1PCI1bits.SWTERM = 1;  
}

/**
* <B> Function: HAL_MC3ClearPWMPCIFault(void)  </B>
*
* @brief Function to terminate the PWM Fault PWM event through software
*        
* @param none.
* @return none.
* 
* @example
* <CODE> HAL_MC3ClearPWMPCIFault(); </CODE>
*
*/
void HAL_MC3ClearPWMPCIFault(void)
{
    /* write of '1' to SWTERM bit will produce a PCI Fault termination event */
    APG1F1PCI1bits.SWTERM = 1;
    APG2F1PCI1bits.SWTERM = 1;
    APG3F1PCI1bits.SWTERM = 1;  
}

/**
* <B> Function: HAL_MC1BootstrapChargeRoutine(MC_DUTYCYCLEOUT_T *, HAL_BOOTSTRAP_T *) </B>
*
* @brief Function to execute bootstrap charging routine for MC1
*        
* @param Pointer to the data structure containing duty cycles
* @param Pointer to the data structure containing bootstrap variables
* @return bootstrap charging completion status
* 
* @example
* <CODE> HAL_MC1BootstrapChargeRoutine(pPdc,pBootstrap); </CODE>
*
*/
uint8_t HAL_MC1BootstrapChargeRoutine(MC_DUTYCYCLEOUT_T *pDuty,HAL_BOOTSTRAP_T *pBootstrap)
{
    uint8_t returnState = 0;

    /* Bootstrap charging state machine */
    switch(pBootstrap->state)
    {
        case BOOTSTRAP_INIT:
            /* Boot Strap routine initialization */
            HAL_MC1PWMDisableOutputs();
            pBootstrap->delayCount = BOOTSTRAP_INITIAL_DELAY_PWM_CYCLES;
            pBootstrap->state = BOOTSTRAP_INIT_WAIT;
            break;
            
        case BOOTSTRAP_INIT_WAIT:
            /* Bootstrap duty */
            /* if Bootstrap duty is less than MIN_DUTY, then MIN_DUTY will be applied */
            pDuty->dutycycle1 = pBootstrap->dutycycle;
            pDuty->dutycycle2 = pBootstrap->dutycycle;
            pDuty->dutycycle3 = pBootstrap->dutycycle;
            pBootstrap->delayCount = MC1_BOOTSTRAP_CHARGING_COUNTS;
            pBootstrap->state = BOOTSTRAP_PHASE_A_CHARGING;
            break;
            
        case BOOTSTRAP_PHASE_A_CHARGING:
            /*  Override Disable ; 0 = PWM Generator provides data for the PWM1L pin */
            PG1IOCON2bits.OVRENL = 0;
            /* Wait for a preset duration of time to charge the Phase-A bootstrap */
            if (pBootstrap->delayCount == 0)
            {
                pBootstrap->delayCount = MC1_BOOTSTRAP_CHARGING_COUNTS;
                pBootstrap->state = BOOTSTRAP_PHASE_B_CHARGING;
            }
            break;

        case BOOTSTRAP_PHASE_B_CHARGING:
            /* Override Enable ; 1 = OVRDAT<0> provides data for output on PWM1L pin */
            PG1IOCON2bits.OVRENL = 1;
            /*  Override Disable ; 0 = PWM Generator provides data for the PWM2L pin */
            PG2IOCON2bits.OVRENL = 0;
            /* Wait for a preset duration of time to charge the Phase-B bootstrap */
            if (pBootstrap->delayCount == 0)
            {
                pBootstrap->delayCount = MC1_BOOTSTRAP_CHARGING_COUNTS;
 
                pBootstrap->state = BOOTSTRAP_PHASE_C_CHARGING;
            }
            break;
            
        case BOOTSTRAP_PHASE_C_CHARGING:
            /* Override Enable ; 1 = OVRDAT<0> provides data for output on PWM2L pin */
            PG2IOCON2bits.OVRENL = 1;
            /*  Override Disable ; 0 = PWM Generator provides data for the PWM3L pin */
            PG3IOCON2bits.OVRENL = 0;
            /* Wait for a preset duration of time to charge the Phase-C bootstrap */
            if (pBootstrap->delayCount == 0)
            {
                pBootstrap->delayCount = MC1_BOOTSTRAP_CHARGING_COUNTS;
                pBootstrap->state = BOOTSTRAP_COMPLETE;
            }
            break;
            
        case BOOTSTRAP_COMPLETE:
            /* Bootstrap sequence is complete, wait in this state */
            returnState = 1;
            HAL_MC1PWMDisableOutputs();
            break;  
    }

    /* Decrement counter for delay */
    if (pBootstrap->delayCount > 0)
    {
        pBootstrap->delayCount--;
    }
    
    return returnState;
}
/**
* <B> Function: HAL_MC1BootstrapChargeInit(HAL_BOOTSTRAP_T *) </B>
*
* @brief Function to execute initialize the bootstrap charging variables
*        
* @param Pointer to the data structure containing bootstrap variables
* @return none
* 
* @example
* <CODE> HAL_MC1BootstrapChargeInit(pBootstrap); </CODE>
*
*/
void HAL_MC1BootstrapChargeInit(HAL_BOOTSTRAP_T *pBootStrap)
{
    /* Initialize the bootstrap parameters */
    pBootStrap->state = 0;
}

/**
* <B> Function: HAL_MC2BootstrapChargeRoutine(MC_DUTYCYCLEOUT_T *, HAL_BOOTSTRAP_T *) </B>
*
* @brief Function to execute bootstrap charging routine for MC2
*        
* @param Pointer to the data structure containing duty cycles
* @param Pointer to the data structure containing bootstrap variables
* @return bootstrap charging completion status
* 
* @example
* <CODE> HAL_MC2BootstrapChargeRoutine(pPdc,pBootstrap); </CODE>
*
*/
uint8_t HAL_MC2BootstrapChargeRoutine(MC_DUTYCYCLEOUT_T *pDuty,HAL_BOOTSTRAP_T *pBootstrap)
{
    uint8_t returnState = 0;

    /* Bootstrap charging state machine */
    switch(pBootstrap->state)
    {
        case BOOTSTRAP_INIT:
            /* Boot Strap routine initialization */
            HAL_MC2PWMDisableOutputs();
            pBootstrap->delayCount = BOOTSTRAP_INITIAL_DELAY_PWM_CYCLES;
            pBootstrap->state = BOOTSTRAP_INIT_WAIT;
            break;
            
        case BOOTSTRAP_INIT_WAIT:
            /* Bootstrap duty */
            /* if Bootstrap duty is less than MIN_DUTY, then MIN_DUTY will be applied */
            pDuty->dutycycle1 = pBootstrap->dutycycle;
            pDuty->dutycycle2 = pBootstrap->dutycycle;
            pDuty->dutycycle3 = pBootstrap->dutycycle;
            pBootstrap->delayCount = MC2_BOOTSTRAP_CHARGING_COUNTS;
            pBootstrap->state = BOOTSTRAP_PHASE_A_CHARGING;
            break;
            
        case BOOTSTRAP_PHASE_A_CHARGING:
            /*  Override Disable ; 0 = PWM Generator provides data for the PWM6L pin */
            PG6IOCON2bits.OVRENL = 0;
            /* Wait for a preset duration of time to charge the Phase-A bootstrap */
            if (pBootstrap->delayCount == 0)
            {
                pBootstrap->delayCount = MC2_BOOTSTRAP_CHARGING_COUNTS;
                pBootstrap->state = BOOTSTRAP_PHASE_B_CHARGING;
            }
            break;

        case BOOTSTRAP_PHASE_B_CHARGING:
            /* Override Enable ; 1 = OVRDAT<0> provides data for output on PWM6L pin */
            PG6IOCON2bits.OVRENL = 1;
            /*  Override Disable ; 0 = PWM Generator provides data for the PWM7L pin */
            PG7IOCON2bits.OVRENL = 0;
            /* Wait for a preset duration of time to charge the Phase-B bootstrap */
            if (pBootstrap->delayCount == 0)
            {
                pBootstrap->delayCount = MC2_BOOTSTRAP_CHARGING_COUNTS;
 
                pBootstrap->state = BOOTSTRAP_PHASE_C_CHARGING;
            }
            break;
            
        case BOOTSTRAP_PHASE_C_CHARGING:
            /* Override Enable ; 1 = OVRDAT<0> provides data for output on PWM7L pin */
            PG7IOCON2bits.OVRENL = 1;
            /*  Override Disable ; 0 = PWM Generator provides data for the PWM8L pin */
            PG8IOCON2bits.OVRENL = 0;
            /* Wait for a preset duration of time to charge the Phase-C bootstrap */
            if (pBootstrap->delayCount == 0)
            {
                pBootstrap->delayCount = MC2_BOOTSTRAP_CHARGING_COUNTS;
                pBootstrap->state = BOOTSTRAP_COMPLETE;
            }
            break;
            
        case BOOTSTRAP_COMPLETE:
            /* Bootstrap sequence is complete, wait in this state */
            returnState = 1;
            HAL_MC2PWMDisableOutputs();
            break;  
    }

    /* Decrement counter for delay */
    if (pBootstrap->delayCount > 0)
    {
        pBootstrap->delayCount--;
    }
    
    return returnState;
}
/**
* <B> Function: HAL_MC2BootstrapChargeInit(HAL_BOOTSTRAP_T *) </B>
*
* @brief Function to execute initialize the bootstrap charging variables
*        
* @param Pointer to the data structure containing bootstrap variables
* @return none
* 
* @example
* <CODE> HAL_MC2BootstrapChargeInit(pBootstrap); </CODE>
*
*/
void HAL_MC2BootstrapChargeInit(HAL_BOOTSTRAP_T *pBootStrap)
{
    /* Initialize the bootstrap parameters */
    pBootStrap->state = 0;
}

/**
* <B> Function: HAL_MC3BootstrapChargeRoutine(MC_DUTYCYCLEOUT_T *, HAL_BOOTSTRAP_T *) </B>
*
* @brief Function to execute bootstrap charging routine for MC3
*        
* @param Pointer to the data structure containing duty cycles
* @param Pointer to the data structure containing bootstrap variables
* @return bootstrap charging completion status
* 
* @example
* <CODE> HAL_MC3BootstrapChargeRoutine(pPdc,pBootstrap); </CODE>
*
*/
uint8_t HAL_MC3BootstrapChargeRoutine(MC_DUTYCYCLEOUT_T *pDuty,HAL_BOOTSTRAP_T *pBootstrap)
{
    uint8_t returnState = 0;

    /* Bootstrap charging state machine */
    switch(pBootstrap->state)
    {
        case BOOTSTRAP_INIT:
            /* Boot Strap routine initialization */
            HAL_MC3PWMDisableOutputs();
            pBootstrap->delayCount = BOOTSTRAP_INITIAL_DELAY_PWM_CYCLES;
            pBootstrap->state = BOOTSTRAP_INIT_WAIT;
            break;
            
        case BOOTSTRAP_INIT_WAIT:
            /* Bootstrap duty */
            /* if Bootstrap duty is less than MIN_DUTY, then MIN_DUTY will be applied */
            pDuty->dutycycle1 = pBootstrap->dutycycle;
            pDuty->dutycycle2 = pBootstrap->dutycycle;
            pDuty->dutycycle3 = pBootstrap->dutycycle;
            pBootstrap->delayCount = MC3_BOOTSTRAP_CHARGING_COUNTS;
            pBootstrap->state = BOOTSTRAP_PHASE_A_CHARGING;
            break;
            
        case BOOTSTRAP_PHASE_A_CHARGING:
            /*  Override Disable ; 0 = PWM Generator provides data for the APWM1L pin */
            APG1IOCON2bits.OVRENL = 0;
            /* Wait for a preset duration of time to charge the Phase-A bootstrap */
            if (pBootstrap->delayCount == 0)
            {
                pBootstrap->delayCount = MC3_BOOTSTRAP_CHARGING_COUNTS;
                pBootstrap->state = BOOTSTRAP_PHASE_B_CHARGING;
            }
            break;

        case BOOTSTRAP_PHASE_B_CHARGING:
            /* Override Enable ; 1 = OVRDAT<0> provides data for output on APWM1L pin */
            APG1IOCON2bits.OVRENL = 1;
            /*  Override Disable ; 0 = PWM Generator provides data for the APWM2L pin */
            APG2IOCON2bits.OVRENL = 0;
            /* Wait for a preset duration of time to charge the Phase-B bootstrap */
            if (pBootstrap->delayCount == 0)
            {
                pBootstrap->delayCount = MC3_BOOTSTRAP_CHARGING_COUNTS;
 
                pBootstrap->state = BOOTSTRAP_PHASE_C_CHARGING;
            }
            break;
            
        case BOOTSTRAP_PHASE_C_CHARGING:
            /* Override Enable ; 1 = OVRDAT<0> provides data for output on APWM2L pin */
            APG2IOCON2bits.OVRENL = 1;
            /*  Override Disable ; 0 = PWM Generator provides data for the APWM3L pin */
            APG3IOCON2bits.OVRENL = 0;
            /* Wait for a preset duration of time to charge the Phase-C bootstrap */
            if (pBootstrap->delayCount == 0)
            {
                pBootstrap->delayCount = MC3_BOOTSTRAP_CHARGING_COUNTS;
                pBootstrap->state = BOOTSTRAP_COMPLETE;
            }
            break;
            
        case BOOTSTRAP_COMPLETE:
            /* Bootstrap sequence is complete, wait in this state */
            returnState = 1;
            HAL_MC3PWMDisableOutputs();
            break;  
    }

    /* Decrement counter for delay */
    if (pBootstrap->delayCount > 0)
    {
        pBootstrap->delayCount--;
    }
    
    return returnState;
}
/**
* <B> Function: HAL_MC3BootstrapChargeInit(HAL_BOOTSTRAP_T *) </B>
*
* @brief Function to execute initialize the bootstrap charging variables
*        
* @param Pointer to the data structure containing bootstrap variables
* @return none
* 
* @example
* <CODE> HAL_MC3BootstrapChargeInit(pBootstrap); </CODE>
*
*/
void HAL_MC3BootstrapChargeInit(HAL_BOOTSTRAP_T *pBootStrap)
{
    /* Initialize the bootstrap parameters */
    pBootStrap->state = 0;
}