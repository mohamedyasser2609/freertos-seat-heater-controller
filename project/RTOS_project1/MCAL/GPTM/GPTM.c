/******************************************************************************
 *
 * Module: GPTM
 *
 * File Name: GPTM.c
 *
 * Description: Source file for the TM4C123GH6PM GPTM driver
 *
 * Author: mohamed yasser
 *
 *******************************************************************************/

#include "GPTM.h"
#include "tm4c123gh6pm_registers.h"

/* Array of function pointers to store callback functions for each timer */
static void (*g_callBackPtr[12])(void) = {NULL_PTR};

/* Helper function to calculate prescaler and load values */
static void GPTM_CalculateTimerValues(uint32 tickTime, uint32 *prescaler, uint32 *loadValue)
{
    uint32 systemClock = 16000000; /* 16 MHz system clock */
    uint32 maxPrescaler = 256;     /* Maximum prescaler value */
    uint32 desiredTicks = (systemClock / 1000000) * tickTime; /* Convert microseconds to ticks */
    
    /* Calculate prescaler and load value */
    if (desiredTicks < 65536) {
        *prescaler = 0;
        *loadValue = desiredTicks;
    } else {
        *prescaler = (desiredTicks / 65536) + 1;
        if (*prescaler > maxPrescaler) *prescaler = maxPrescaler;
        *loadValue = desiredTicks / *prescaler;
    }
}

void GPTM_Init(const GPTM_ConfigType* config)
{
    uint32 prescalerValue, loadValue;
    
    if (!config) return;
    
    /* Calculate prescaler and load values based on desired tick time */
    GPTM_CalculateTimerValues(config->tickTime, &prescalerValue, &loadValue);
    
    /* Enable clock for the selected timer */
    if (config->timerSelect <= GPTM_TIMER5) {
        SYSCTL_RCGCTIMER_REG |= (1 << config->timerSelect);
        while(!(SYSCTL_PRTIMER_REG & (1 << config->timerSelect))); /* Wait for clock to be ready */
    } else {
        SYSCTL_RCGCWTIMER_REG |= (1 << (config->timerSelect - GPTM_WTIMER0));
        while(!(SYSCTL_PRWTIMER_REG & (1 << (config->timerSelect - GPTM_WTIMER0)))); /* Wait for clock to be ready */
    }
    
    /* Configure the timer based on selection */
    switch(config->timerSelect) {
        case GPTM_TIMER0:
            TIMER0_CTL_REG = 0;              /* Disable timer */
            TIMER0_CFG_REG = 0x04;           /* 16-bit timer configuration */
            TIMER0_TAMR_REG = config->mode;  /* Set timer mode */
            if(config->countDir == GPTM_COUNT_UP) {
                TIMER0_TAMR_REG |= (1<<4);   /* Set count direction up */
            }
            TIMER0_TAPR_REG = prescalerValue;/* Set prescaler value */
            TIMER0_TAILR_REG = loadValue;    /* Set load value */
            if(config->enableInterrupt) {
                TIMER0_IMR_REG |= 0x01;      /* Enable timer timeout interrupt */
                NVIC_EN0_REG |= (1<<19);     /* Enable IRQ19 for Timer0A */
            }
            break;
            
        case GPTM_WTIMER0:
            WTIMER0_CTL_REG = 0;             /* Disable timer */
            WTIMER0_CFG_REG = 0x04;          /* 32-bit timer configuration */
            WTIMER0_TAMR_REG = config->mode; /* Set timer mode */
            if(config->countDir == GPTM_COUNT_UP) {
                WTIMER0_TAMR_REG |= (1<<4);  /* Set count direction up */
            }
            WTIMER0_TAPR_REG = prescalerValue;/* Set prescaler value */
            WTIMER0_TAILR_REG = loadValue;   /* Set load value */
            if(config->enableInterrupt) {
                WTIMER0_IMR_REG |= 0x01;     /* Enable timer timeout interrupt */
                NVIC_EN2_REG |= (1<<30);     /* Enable IRQ94 for WTimer0A */
            }
            break;
            
        /* Add cases for other timers as needed */
        default:
            break;
    }
}

void GPTM_StartTimer(GPTM_TimerType timer)
{
    switch(timer) {
        case GPTM_TIMER0:
            TIMER0_CTL_REG |= 0x01;  /* Enable Timer0A */
            break;
        case GPTM_WTIMER0:
            WTIMER0_CTL_REG |= 0x01; /* Enable WTimer0A */
            break;
        /* Add cases for other timers as needed */
        default:
            break;
    }
}

void GPTM_StopTimer(GPTM_TimerType timer)
{
    switch(timer) {
        case GPTM_TIMER0:
            TIMER0_CTL_REG &= ~0x01;  /* Disable Timer0A */
            break;
        case GPTM_WTIMER0:
            WTIMER0_CTL_REG &= ~0x01; /* Disable WTimer0A */
            break;
        /* Add cases for other timers as needed */
        default:
            break;
    }
}

uint32 GPTM_ReadTimer(GPTM_TimerType timer)
{
    uint32 value = 0;
    
    switch(timer) {
        case GPTM_TIMER0:
            value = TIMER0_TAR_REG;
            break;
        case GPTM_WTIMER0:
            value = WTIMER0_TAR_REG;
            break;
        /* Add cases for other timers as needed */
        default:
            break;
    }
    
    return value;
}

void GPTM_SetCallBack(GPTM_TimerType timer, void (*callBack)(void))
{
    if (timer < 12 && callBack != NULL_PTR) {
        g_callBackPtr[timer] = callBack;
    }
}

void GPTM_ClearInterruptFlag(GPTM_TimerType timer)
{
    switch(timer) {
        case GPTM_TIMER0:
            TIMER0_ICR_REG = 0x01;  /* Clear Timer0A timeout interrupt flag */
            break;
        case GPTM_WTIMER0:
            WTIMER0_ICR_REG = 0x01; /* Clear WTimer0A timeout interrupt flag */
            break;
        /* Add cases for other timers as needed */
        default:
            break;
    }
}

void GPTM_Delay1ms(void)
{
    /* Configure Timer0A for 1ms delay */
    SYSCTL_RCGCTIMER_REG |= (1 << 0);  /* Enable Timer0 clock */
    while(!(SYSCTL_PRTIMER_REG & (1 << 0))); /* Wait for clock to be ready */
    
    TIMER0_CTL_REG &= ~(1 << 0);  /* Disable Timer0A */
    TIMER0_CFG_REG = 0x00000000;  /* 32-bit mode */
    TIMER0_TAMR_REG = 0x00000002; /* Periodic mode */
    TIMER0_TAILR_REG = 16000 - 1; /* 1ms at 16MHz */
    TIMER0_ICR_REG = 0x00000001;  /* Clear Timer0A timeout flag */
    TIMER0_CTL_REG |= (1 << 0);   /* Enable Timer0A */
    
    /* Wait for timeout */
    while(!(TIMER0_RIS_REG & 0x00000001));
    
    /* Disable Timer0A */
    TIMER0_CTL_REG &= ~(1 << 0);
}

/*******************************************************************************
 *                       Interrupt Service Routines                              *
 *******************************************************************************/

void TIMER0A_Handler(void)
{
    if (g_callBackPtr[GPTM_TIMER0] != NULL_PTR)
    {
        g_callBackPtr[GPTM_TIMER0]();
    }
    GPTM_ClearInterruptFlag(GPTM_TIMER0);
}

void WTimer0A_Handler(void)
{
    GPTM_ClearInterruptFlag(GPTM_WTIMER0);
    if(g_callBackPtr[GPTM_WTIMER0] != NULL_PTR)
    {
        (*g_callBackPtr[GPTM_WTIMER0])();
    }
} 

