/******************************************************************************
 *
 * Module: ADC
 *
 * File Name: ADC.h
 *
 * Description: Header file for the TM4C123GH6PM ADC driver
 *
 * Author: mohamed yasser
 *
 *******************************************************************************/

#ifndef ADC_H_
#define ADC_H_

#include "Common/std_types.h"

/*******************************************************************************
 *                              Module Configurations                            *
 *******************************************************************************/

/* ADC Module Selection */
typedef enum {
    ADC0,
    ADC1
} ADC_ModuleType;

/* ADC Sequencer Selection */
typedef enum {
    ADC_SEQUENCER0,  /* 8 samples */
    ADC_SEQUENCER1,  /* 4 samples */
    ADC_SEQUENCER2,  /* 4 samples */
    ADC_SEQUENCER3   /* 1 sample */
} ADC_SequencerType;

/* ADC Channel Selection */
typedef enum {
    ADC_CHANNEL0,    /* AIN0 - PE3 */
    ADC_CHANNEL1,    /* AIN1 - PE2 */
    ADC_CHANNEL2,    /* AIN2 - PE1 */
    ADC_CHANNEL3,    /* AIN3 - PE0 */
    ADC_CHANNEL4,    /* AIN4 - PD3 */
    ADC_CHANNEL5,    /* AIN5 - PD2 */
    ADC_CHANNEL6,    /* AIN6 - PD1 */
    ADC_CHANNEL7,    /* AIN7 - PD0 */
    ADC_CHANNEL8,    /* AIN8 - PE5 */
    ADC_CHANNEL9,    /* AIN9 - PE4 */
    ADC_CHANNEL10,   /* AIN10 - PB4 */
    ADC_CHANNEL11    /* AIN11 - PB5 */
} ADC_ChannelType;

/* ADC Trigger Source */
typedef enum {
    ADC_TRIGGER_PROCESSOR,  /* Processor trigger */
    ADC_TRIGGER_COMP0,     /* Analog comparator 0 */
    ADC_TRIGGER_COMP1,     /* Analog comparator 1 */
    ADC_TRIGGER_EXTERNAL,  /* External trigger */
    ADC_TRIGGER_TIMER,     /* Timer trigger */
    ADC_TRIGGER_PWM0,      /* PWM0 trigger */
    ADC_TRIGGER_PWM1,      /* PWM1 trigger */
    ADC_TRIGGER_PWM2,      /* PWM2 trigger */
    ADC_TRIGGER_PWM3,      /* PWM3 trigger */
    ADC_TRIGGER_ALWAYS     /* Continuous sampling */
} ADC_TriggerType;

/* ADC Sample Speed */
typedef enum {
    ADC_SPEED_125K,    /* 125K samples/second */
    ADC_SPEED_250K,    /* 250K samples/second */
    ADC_SPEED_500K,    /* 500K samples/second */
    ADC_SPEED_1M       /* 1M samples/second */
} ADC_SpeedType;

/* ADC Configuration Structure */
typedef struct {
    ADC_ModuleType module;           /* ADC module selection */
    ADC_SequencerType sequencer;     /* Sequencer selection */
    ADC_ChannelType channel;         /* Channel selection */
    ADC_TriggerType trigger;         /* Trigger source */
    ADC_SpeedType speed;             /* Sampling speed */
    boolean enableInterrupt;          /* Enable/Disable interrupt */
} ADC_ConfigType;

/*******************************************************************************
 *                              Functions Prototypes                             *
 *******************************************************************************/

/* Initialize ADC module with the given configuration */
void ADC_Init(const ADC_ConfigType* config);

/* Start ADC conversion */
void ADC_StartConversion(ADC_ModuleType module, ADC_SequencerType sequencer);

/* Read ADC conversion result */
uint16 ADC_ReadResult(ADC_ModuleType module, ADC_SequencerType sequencer);

/* Set callback function for ADC conversion complete interrupt */
void ADC_SetCallBack(ADC_ModuleType module, ADC_SequencerType sequencer, void (*callBack)(void));

/* Check if ADC conversion is complete */
boolean ADC_IsConversionComplete(ADC_ModuleType module, ADC_SequencerType sequencer);

/* Clear ADC interrupt flag */
void ADC_ClearInterruptFlag(ADC_ModuleType module, ADC_SequencerType sequencer);

#endif /* ADC_H_ */
