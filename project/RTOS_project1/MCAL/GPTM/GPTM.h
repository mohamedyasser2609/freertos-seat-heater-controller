/******************************************************************************
 *
 * Module: GPTM
 *
 * File Name: GPTM.h
 *
 * Description: Header file for the TM4C123GH6PM GPTM driver
 *
 * Author: mohamed yasser
 *
 *******************************************************************************/

#ifndef GPTM_H_
#define GPTM_H_

#include "Common/std_types.h"

/*******************************************************************************
 *                              Module Configurations                            *
 *******************************************************************************/

/* Timer Types */
typedef enum {
    GPTM_TIMER0,
    GPTM_TIMER1,
    GPTM_TIMER2,
    GPTM_TIMER3,
    GPTM_TIMER4,
    GPTM_TIMER5,
    GPTM_WTIMER0,
    GPTM_WTIMER1,
    GPTM_WTIMER2,
    GPTM_WTIMER3,
    GPTM_WTIMER4,
    GPTM_WTIMER5
} GPTM_TimerType;

/* Timer Modes */
typedef enum {
    GPTM_ONE_SHOT,
    GPTM_PERIODIC,
    GPTM_CAPTURE
} GPTM_ModeType;

/* Timer Count Direction */
typedef enum {
    GPTM_COUNT_DOWN,
    GPTM_COUNT_UP
} GPTM_CountDirType;

/* Timer Configuration */
typedef struct {
    GPTM_TimerType timerSelect;    /* Select which timer to use */
    GPTM_ModeType mode;            /* Timer mode */
    GPTM_CountDirType countDir;    /* Count direction */
    uint32 tickTime;               /* Desired tick time in microseconds */
    uint32 preloadValue;           /* Initial value to load */
    boolean enableInterrupt;        /* Enable/Disable timer interrupt */
} GPTM_ConfigType;

/*******************************************************************************
 *                              Functions Prototypes                             *
 *******************************************************************************/

/* Initialize the specified timer with given configuration */
void GPTM_Init(const GPTM_ConfigType* config);

/* Start the specified timer */
void GPTM_StartTimer(GPTM_TimerType timer);

/* Stop the specified timer */
void GPTM_StopTimer(GPTM_TimerType timer);

/* Read current value of the specified timer */
uint32 GPTM_ReadTimer(GPTM_TimerType timer);

/* Set callback function for timer interrupt */
void GPTM_SetCallBack(GPTM_TimerType timer, void (*callBack)(void));

/* Clear timer interrupt flag */
void GPTM_ClearInterruptFlag(GPTM_TimerType timer);

#endif /* GPTM_H_ */

