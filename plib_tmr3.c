/*******************************************************************************
  TMR Peripheral Library Interface Source File

  Company
    Microchip Technology Inc.

  File Name
    plib_tmr3.c

  Summary
    TMR3 peripheral library source file.

  Description
    This file implements the interface to the TMR peripheral library.  This
    library provides access to and control of the associated peripheral
    instance.

*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "device.h"
#include "plib_tmr3.h"
#include "definitions.h"                // SYS function prototypes
#include "AT45DB161.h"

static TMR_TIMER_OBJECT tmr3Obj;

static unsigned char TM3Rflag = true;
static unsigned char MemoryTimeoutFlag = false;
static unsigned char TMR3MillisecondFlag = false;
static long Delay100MicroSeconds = 0;
static unsigned long MemoryDelayCounter = 0;
static unsigned long TMR3MilliSecondCounter = 0;
static unsigned long TMR3_X10Counter = 0;
static unsigned long AtmelTimeout = DEFAULT_ATMEL_TIMEOUT * 10; // 

void IntializeAtmelTimeout(unsigned long AtmelTimeoutMilliseconds)
{
    AtmelTimeout = AtmelTimeoutMilliseconds * 10;
}

void SetMilliSecondFlag(unsigned char BoolValue)
{
    TMR3MillisecondFlag = BoolValue;
}

void SetMilliSecondTimeout(unsigned short TimeoutMilliSeconds)
{
    if (TimeoutMilliSeconds) 
    {
        TMR3MilliSecondCounter = TimeoutMilliSeconds;    
        TMR3MillisecondFlag = false;
        TMR3_X10Counter = 0;
    }
    else TMR3MillisecondFlag = true;
}

unsigned char GetMilliSecondTimeout()
{
    return (TMR3MillisecondFlag);
}

unsigned char GetMemoryTimeout()
{
    return MemoryTimeoutFlag;
}

void SetI2CTimeout()
{
    MemoryDelayCounter = 10;
    MemoryTimeoutFlag = false;
}

void SetAtmelTimeout()
{
    MemoryDelayCounter = 100000;
    MemoryTimeoutFlag = false;
}

void SetDelay(long Delay)
{
    Delay100MicroSeconds = Delay;
}

long GetDelay()
{
    return Delay100MicroSeconds;
}

unsigned char getT3Flag()
{
    return (TM3Rflag);
}

void setT3Flag()
{
    TM3Rflag = true;
}

void ClearT3Flag()
{
    TM3Rflag = false;
}


void TMR3_Initialize(void)
{
    /* Disable Timer */
    T3CONCLR = _T3CON_ON_MASK;

    /*
    SIDL = 0
    TCKPS =0
    T32   = 0
    TCS = 0
    */
    T3CONSET = 0x0;

    /* Clear counter */
    TMR3 = 0x0;

    /*Set period */
    PR3 = 3999U;

    /* Enable TMR Interrupt */
    IEC0SET = _IEC0_T3IE_MASK;

}


void TMR3_Start(void)
{
    T3CONSET = _T3CON_ON_MASK;
}


void TMR3_Stop (void)
{
    T3CONCLR = _T3CON_ON_MASK;
}

void TMR3_PeriodSet(uint16_t period)
{
    PR3  = period;
}

uint16_t TMR3_PeriodGet(void)
{
    return (uint16_t)PR3;
}

uint16_t TMR3_CounterGet(void)
{
    return (uint16_t)(TMR3);
}


uint32_t TMR3_FrequencyGet(void)
{
    return (40000000);
}

void TIMER_3_InterruptHandler (void)
{
    uint32_t status;
    status = IFS0bits.T3IF;   
    
    static unsigned short T3Counter = 0;
    static unsigned char StartFlag = TRUE; 
    
    SPARE1_Toggle();
    
    if (StartFlag) 
    {
        StartFlag = FALSE;
        SetMilliSecondFlag(FALSE);
    }

    IFS0CLR = _IFS0_T3IF_MASK;       
            
    if (Delay100MicroSeconds) Delay100MicroSeconds--;

    TMR3_X10Counter++;
    if (TMR3_X10Counter >= 10)
    {    
        TMR3_X10Counter = 0;
        if (TMR3MilliSecondCounter)
        {
            TMR3MilliSecondCounter--;
            if (!TMR3MilliSecondCounter)
                TMR3MillisecondFlag = true;
        }
    }
        
    if (T3Counter) T3Counter--;
    if (!T3Counter)
    {
        T3Counter = 40;     
        setT3Flag();
    }
    
    if (MemoryDelayCounter)
    {
        MemoryDelayCounter--;
        if (!MemoryDelayCounter)
            MemoryTimeoutFlag = true;
    }

    
    if((tmr3Obj.callback_fn != NULL))
    {
        tmr3Obj.callback_fn(status, tmr3Obj.context);
    }
    
}

void TMR3_InterruptEnable(void)
{
    IEC0SET = _IEC0_T3IE_MASK;
}


void TMR3_InterruptDisable(void)
{
    IEC0CLR = _IEC0_T3IE_MASK;
}


void TMR3_CallbackRegister( TMR_CALLBACK callback_fn, uintptr_t context )
{
    /* Save callback_fn and context in local memory */
    tmr3Obj.callback_fn = callback_fn;
    tmr3Obj.context = context;
}
