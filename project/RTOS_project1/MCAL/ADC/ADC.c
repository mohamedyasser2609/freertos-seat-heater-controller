/******************************************************************************
 *
 * Module: ADC
 *
 * File Name: ADC.c
 *
 * Description: Source file for the TM4C123GH6PM ADC driver
 *
 * Author: mohamed yasser
 *
 *******************************************************************************/

#include "ADC.h"
#include "tm4c123gh6pm_registers.h"

/* Array of function pointers to store callback functions for each ADC sequencer */
static void (*g_ADC0CallBackPtr[4])(void) = {NULL_PTR};
static void (*g_ADC1CallBackPtr[4])(void) = {NULL_PTR};

/* Helper function to enable ADC clock */
static void ADC_EnableClock(ADC_ModuleType module)
{
    SYSCTL_RCGCADC_REG |= (1 << module);
    while(!(SYSCTL_PRADC_REG & (1 << module))); /* Wait for ADC clock to be ready */
}

/* Helper function to configure ADC GPIO pins */
static void ADC_ConfigureGPIO(ADC_ChannelType channel)
{
    switch(channel) {
        case ADC_CHANNEL0: /* PE3 */
        case ADC_CHANNEL1: /* PE2 */
        case ADC_CHANNEL2: /* PE1 */
        case ADC_CHANNEL3: /* PE0 */
            SYSCTL_RCGCGPIO_REG |= (1<<4);  /* Enable PORTE clock */
            while(!(SYSCTL_PRGPIO_REG & (1<<4))); /* Wait for clock to be ready */
            GPIO_PORTE_AFSEL_REG |= (1 << (channel % 4)); /* Enable alternate function */
            GPIO_PORTE_DEN_REG &= ~(1 << (channel % 4));  /* Disable digital function */
            GPIO_PORTE_AMSEL_REG |= (1 << (channel % 4)); /* Enable analog function */
            break;
            
        case ADC_CHANNEL4: /* PD3 */
        case ADC_CHANNEL5: /* PD2 */
        case ADC_CHANNEL6: /* PD1 */
        case ADC_CHANNEL7: /* PD0 */
            SYSCTL_RCGCGPIO_REG |= (1<<3);  /* Enable PORTD clock */
            while(!(SYSCTL_PRGPIO_REG & (1<<3))); /* Wait for clock to be ready */
            GPIO_PORTD_AFSEL_REG |= (1 << (channel % 4)); /* Enable alternate function */
            GPIO_PORTD_DEN_REG &= ~(1 << (channel % 4));  /* Disable digital function */
            GPIO_PORTD_AMSEL_REG |= (1 << (channel % 4)); /* Enable analog function */
            break;
            
        case ADC_CHANNEL8: /* PE5 */
        case ADC_CHANNEL9: /* PE4 */
            SYSCTL_RCGCGPIO_REG |= (1<<4);  /* Enable PORTE clock */
            while(!(SYSCTL_PRGPIO_REG & (1<<4))); /* Wait for clock to be ready */
            GPIO_PORTE_AFSEL_REG |= (1 << (4 + (channel % 2))); /* Enable alternate function */
            GPIO_PORTE_DEN_REG &= ~(1 << (4 + (channel % 2)));  /* Disable digital function */
            GPIO_PORTE_AMSEL_REG |= (1 << (4 + (channel % 2))); /* Enable analog function */
            break;
            
        case ADC_CHANNEL10: /* PB4 */
        case ADC_CHANNEL11: /* PB5 */
            SYSCTL_RCGCGPIO_REG |= (1<<1);  /* Enable PORTB clock */
            while(!(SYSCTL_PRGPIO_REG & (1<<1))); /* Wait for clock to be ready */
            GPIO_PORTB_AFSEL_REG |= (1 << (4 + (channel % 2))); /* Enable alternate function */
            GPIO_PORTB_DEN_REG &= ~(1 << (4 + (channel % 2)));  /* Disable digital function */
            GPIO_PORTB_AMSEL_REG |= (1 << (4 + (channel % 2))); /* Enable analog function */
            break;
            
        default:
            break;
    }
}

void ADC_Init(const ADC_ConfigType* config)
{
    if (!config) return;
    
    /* Enable ADC clock */
    ADC_EnableClock(config->module);
    
    /* Configure GPIO pins */
    ADC_ConfigureGPIO(config->channel);
    
    /* Configure ADC based on module selection */
    if (config->module == ADC0) {
        /* Disable sequencer during configuration */
        ADC0_ACTSS_REG &= ~(1 << config->sequencer);
        
        /* Configure trigger source */
        switch(config->sequencer) {
            case ADC_SEQUENCER0:
                ADC0_EMUX_REG = (ADC0_EMUX_REG & ~0xF) | config->trigger;
                break;
            case ADC_SEQUENCER1:
                ADC0_EMUX_REG = (ADC0_EMUX_REG & ~0xF0) | (config->trigger << 4);
                break;
            case ADC_SEQUENCER2:
                ADC0_EMUX_REG = (ADC0_EMUX_REG & ~0xF00) | (config->trigger << 8);
                break;
            case ADC_SEQUENCER3:
                ADC0_EMUX_REG = (ADC0_EMUX_REG & ~0xF000) | (config->trigger << 12);
                break;
        }
        
        /* Configure sample sequence */
        switch(config->sequencer) {
            case ADC_SEQUENCER0:
                ADC0_SSMUX0_REG = config->channel;
                ADC0_SSCTL0_REG = 0x6;  /* Enable interrupt and mark as end of sequence */
                break;
            case ADC_SEQUENCER1:
                ADC0_SSMUX1_REG = config->channel;
                ADC0_SSCTL1_REG = 0x6;
                break;
            case ADC_SEQUENCER2:
                ADC0_SSMUX2_REG = config->channel;
                ADC0_SSCTL2_REG = 0x6;
                break;
            case ADC_SEQUENCER3:
                ADC0_SSMUX3_REG = config->channel;
                ADC0_SSCTL3_REG = 0x6;
                break;
        }
        
        /* Configure sample rate */
        switch(config->speed) {
            case ADC_SPEED_125K:
                ADC0_PC_REG = 0x0;  /* 125K samples/second */
                break;
            case ADC_SPEED_250K:
                ADC0_PC_REG = 0x1;  /* 250K samples/second */
                break;
            case ADC_SPEED_500K:
                ADC0_PC_REG = 0x2;  /* 500K samples/second */
                break;
            case ADC_SPEED_1M:
                ADC0_PC_REG = 0x3;  /* 1M samples/second */
                break;
        }
        
        /* Configure interrupt */
        if(config->enableInterrupt) {
            ADC0_IM_REG |= (1 << config->sequencer);
            switch(config->sequencer) {
                case ADC_SEQUENCER0:
                    NVIC_EN0_REG |= (1<<14);  /* Enable IRQ14 for ADC0 Sequence 0 */
                    break;
                case ADC_SEQUENCER1:
                    NVIC_EN0_REG |= (1<<15);  /* Enable IRQ15 for ADC0 Sequence 1 */
                    break;
                case ADC_SEQUENCER2:
                    NVIC_EN0_REG |= (1<<16);  /* Enable IRQ16 for ADC0 Sequence 2 */
                    break;
                case ADC_SEQUENCER3:
                    NVIC_EN0_REG |= (1<<17);  /* Enable IRQ17 for ADC0 Sequence 3 */
                    break;
            }
        }
        
        /* Enable sequencer */
        ADC0_ACTSS_REG |= (1 << config->sequencer);
    }
    else if (config->module == ADC1) {
        /* Disable sequencer during configuration */
        ADC1_ACTSS_REG &= ~(1 << config->sequencer);
        
        /* Configure trigger source */
        switch(config->sequencer) {
            case ADC_SEQUENCER0:
                ADC1_EMUX_REG = (ADC1_EMUX_REG & ~0xF) | config->trigger;
                break;
            case ADC_SEQUENCER1:
                ADC1_EMUX_REG = (ADC1_EMUX_REG & ~0xF0) | (config->trigger << 4);
                break;
            case ADC_SEQUENCER2:
                ADC1_EMUX_REG = (ADC1_EMUX_REG & ~0xF00) | (config->trigger << 8);
                break;
            case ADC_SEQUENCER3:
                ADC1_EMUX_REG = (ADC1_EMUX_REG & ~0xF000) | (config->trigger << 12);
                break;
        }
        
        /* Configure sample sequence */
        switch(config->sequencer) {
            case ADC_SEQUENCER0:
                ADC1_SSMUX0_REG = config->channel;
                ADC1_SSCTL0_REG = 0x6;  /* Enable interrupt and mark as end of sequence */
                break;
            case ADC_SEQUENCER1:
                ADC1_SSMUX1_REG = config->channel;
                ADC1_SSCTL1_REG = 0x6;
                break;
            case ADC_SEQUENCER2:
                ADC1_SSMUX2_REG = config->channel;
                ADC1_SSCTL2_REG = 0x6;
                break;
            case ADC_SEQUENCER3:
                ADC1_SSMUX3_REG = config->channel;
                ADC1_SSCTL3_REG = 0x6;
                break;
        }
        
        /* Configure sample rate */
        switch(config->speed) {
            case ADC_SPEED_125K:
                ADC1_PC_REG = 0x0;  /* 125K samples/second */
                break;
            case ADC_SPEED_250K:
                ADC1_PC_REG = 0x1;  /* 250K samples/second */
                break;
            case ADC_SPEED_500K:
                ADC1_PC_REG = 0x2;  /* 500K samples/second */
                break;
            case ADC_SPEED_1M:
                ADC1_PC_REG = 0x3;  /* 1M samples/second */
                break;
        }
        
        /* Configure interrupt */
        if(config->enableInterrupt) {
            ADC1_IM_REG |= (1 << config->sequencer);
            switch(config->sequencer) {
                case ADC_SEQUENCER0:
                    NVIC_EN1_REG |= (1<<(48-32));  /* Enable IRQ48 for ADC1 Sequence 0 */
                    break;
                case ADC_SEQUENCER1:
                    NVIC_EN1_REG |= (1<<(49-32));  /* Enable IRQ49 for ADC1 Sequence 1 */
                    break;
                case ADC_SEQUENCER2:
                    NVIC_EN1_REG |= (1<<(50-32));  /* Enable IRQ50 for ADC1 Sequence 2 */
                    break;
                case ADC_SEQUENCER3:
                    NVIC_EN1_REG |= (1<<(51-32));  /* Enable IRQ51 for ADC1 Sequence 3 */
                    break;
            }
        }
        
        /* Enable sequencer */
        ADC1_ACTSS_REG |= (1 << config->sequencer);
    }
}

void ADC_StartConversion(ADC_ModuleType module, ADC_SequencerType sequencer)
{
    if (module == ADC0) {
        ADC0_PSSI_REG |= (1 << sequencer);
    }
    else if (module == ADC1) {
        ADC1_PSSI_REG |= (1 << sequencer);
    }
}

uint16 ADC_ReadResult(ADC_ModuleType module, ADC_SequencerType sequencer)
{
    uint16 result = 0;
    
    if (module == ADC0) {
        switch(sequencer) {
            case ADC_SEQUENCER0:
                result = (uint16)ADC0_SSFIFO0_REG;
                break;
            case ADC_SEQUENCER1:
                result = (uint16)ADC0_SSFIFO1_REG;
                break;
            case ADC_SEQUENCER2:
                result = (uint16)ADC0_SSFIFO2_REG;
                break;
            case ADC_SEQUENCER3:
                result = (uint16)ADC0_SSFIFO3_REG;
                break;
        }
    }
    else if (module == ADC1) {
        switch(sequencer) {
            case ADC_SEQUENCER0:
                result = (uint16)ADC1_SSFIFO0_REG;
                break;
            case ADC_SEQUENCER1:
                result = (uint16)ADC1_SSFIFO1_REG;
                break;
            case ADC_SEQUENCER2:
                result = (uint16)ADC1_SSFIFO2_REG;
                break;
            case ADC_SEQUENCER3:
                result = (uint16)ADC1_SSFIFO3_REG;
                break;
        }
    }
    
    return result;
}

void ADC_SetCallBack(ADC_ModuleType module, ADC_SequencerType sequencer, void (*callBack)(void))
{
    if (callBack != NULL_PTR) {
        if (module == ADC0) {
            g_ADC0CallBackPtr[sequencer] = callBack;
        }
        else if (module == ADC1) {
            g_ADC1CallBackPtr[sequencer] = callBack;
        }
    }
}

boolean ADC_IsConversionComplete(ADC_ModuleType module, ADC_SequencerType sequencer)
{
    boolean status = FALSE;
    
    if (module == ADC0) {
        status = (ADC0_RIS_REG & (1 << sequencer)) ? TRUE : FALSE;
    }
    else if (module == ADC1) {
        status = (ADC1_RIS_REG & (1 << sequencer)) ? TRUE : FALSE;
    }
    
    return status;
}

void ADC_ClearInterruptFlag(ADC_ModuleType module, ADC_SequencerType sequencer)
{
    if (module == ADC0) {
        ADC0_ISC_REG = (1 << sequencer);
    }
    else if (module == ADC1) {
        ADC1_ISC_REG = (1 << sequencer);
    }
}

/*******************************************************************************
 *                       Interrupt Service Routines                              *
 *******************************************************************************/

void ADC0Seq0_Handler(void)
{
    /* Clear the ADC interrupt flag first */
    ADC0_ISC_REG = 0x1;  /* Clear Sequencer 0 interrupt flag */
    
    /* Read the result if needed */
    volatile uint32 result = ADC0_SSFIFO0_REG;
    
    /* Call the callback function if registered */
    if(g_ADC0CallBackPtr[ADC_SEQUENCER0] != NULL_PTR)
    {
        (*g_ADC0CallBackPtr[ADC_SEQUENCER0])();
    }
    
    /* Start next conversion */
    ADC0_PSSI_REG |= 0x1;  /* Start Sequencer 0 conversion */
}

void ADC0Seq1_Handler(void)
{
    ADC_ClearInterruptFlag(ADC0, ADC_SEQUENCER1);
    if(g_ADC0CallBackPtr[ADC_SEQUENCER1] != NULL_PTR)
    {
        (*g_ADC0CallBackPtr[ADC_SEQUENCER1])();
    }
}

void ADC0Seq2_Handler(void)
{
    ADC_ClearInterruptFlag(ADC0, ADC_SEQUENCER2);
    if(g_ADC0CallBackPtr[ADC_SEQUENCER2] != NULL_PTR)
    {
        (*g_ADC0CallBackPtr[ADC_SEQUENCER2])();
    }
}

void ADC0Seq3_Handler(void)
{
    /* Clear the ADC interrupt flag first */
    ADC0_ISC_REG = 0x8;  /* Clear Sequencer 3 interrupt flag */
    
    /* Read the result */
    volatile uint32 result = ADC0_SSFIFO3_REG;
    
    /* Call the callback function if registered */
    if(g_ADC0CallBackPtr[ADC_SEQUENCER3] != NULL_PTR)
    {
        (*g_ADC0CallBackPtr[ADC_SEQUENCER3])();
    }
}

/* ADC1 Interrupt Handlers */
void ADC1Seq0_Handler(void)
{
    ADC1_ISC_REG |= (1<<0); /* Clear the interrupt flag */
    if(g_ADC1CallBackPtr[ADC_SEQUENCER0] != NULL_PTR)
    {
        (*g_ADC1CallBackPtr[ADC_SEQUENCER0])();
    }
}

void ADC1Seq1_Handler(void)
{
    ADC1_ISC_REG |= (1<<1);
    if(g_ADC1CallBackPtr[ADC_SEQUENCER1] != NULL_PTR)
    {
        (*g_ADC1CallBackPtr[ADC_SEQUENCER1])();
    }
}

void ADC1Seq2_Handler(void)
{
    ADC1_ISC_REG |= (1<<2);
    if(g_ADC1CallBackPtr[ADC_SEQUENCER2] != NULL_PTR)
    {
        (*g_ADC1CallBackPtr[ADC_SEQUENCER2])();
    }
}

void ADC1Seq3_Handler(void)
{
    ADC1_ISC_REG |= (1<<3);
    if(g_ADC1CallBackPtr[ADC_SEQUENCER3] != NULL_PTR)
    {
        (*g_ADC1CallBackPtr[ADC_SEQUENCER3])();
    }
}


