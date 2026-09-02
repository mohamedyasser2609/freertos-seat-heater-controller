/******************************************************************************
 *
 * Module: GPIO
 *
 * File Name: GPIO.c
 *
 * Description: Source file for GPIO Driver
 *
 * Author: mohamed yasser
 *
 *******************************************************************************/

#include "GPIO.h"
#include "tm4c123gh6pm_registers.h"

/* Helper Function: Validate Pin Number */
static inline uint8 GPIO_IsValidPin(uint8 pin) {
    return (pin < 8);
}

/* Initialize GPIO Pin */
void GPIO_Init(const GPIO_ConfigType *config) {
    if (!config || !GPIO_IsValidPin(config->pin)) return;
    
    uint32 pinMask = (1UL << config->pin);
    
    /* Select the appropriate GPIO port based on the configuration */
    switch(config->port) {
        case GPIO_PORT_A:
            /* Enable Clock for the GPIO Port */
            SYSCTL_RCGCGPIO_REG |= (1UL << 0);
            while(!(SYSCTL_PRGPIO_REG & (1UL << 0)));
            
            /* Unlock the GPIO Port (if needed) */
            GPIO_PORTA_LOCK_REG = GPIO_LOCK_KEY;
            GPIO_PORTA_CR_REG |= pinMask;
            
            /* Configure Direction */
            if (config->mode == GPIO_MODE_INPUT) {
                GPIO_PORTA_DIR_REG &= ~pinMask;
            } else {
                GPIO_PORTA_DIR_REG |= pinMask;
            }
            
            /* Configure Pull-Up/Pull-Down */
            if (config->pull == GPIO_PULL_UP) {
                GPIO_PORTA_PUR_REG |= pinMask;
            } else if (config->pull == GPIO_PULL_DOWN) {
                GPIO_PORTA_PDR_REG |= pinMask;
            }
            
            /* Configure Alternate Function if needed */
            if (config->mode == GPIO_MODE_AF) {
                uint32 pctlShift = config->pin * 4;
                GPIO_PORTA_PCTL_REG &= ~(0xFUL << pctlShift);
                GPIO_PORTA_PCTL_REG |= (config->afConfig.pctl & 0xF) << pctlShift;
            }
            
            /* Configure Analog Mode if needed */
            if (config->mode == GPIO_MODE_ANALOG) {
                GPIO_PORTA_AMSEL_REG |= pinMask;
            } else {
                /* Enable Digital Function */
                GPIO_PORTA_DEN_REG |= pinMask;
            }
            
            /* Configure Interrupt if enabled */
            if (config->intConfig.enable == GPIO_INT_ENABLE) {
                /* Configure Interrupt Type */
                if (config->intConfig.type == GPIO_INT_EDGE) {
                    GPIO_PORTA_IS_REG &= ~pinMask;
                } else {
                    GPIO_PORTA_IS_REG |= pinMask;
                }
                
                /* Configure Interrupt Trigger */
                if (config->intConfig.trigger == GPIO_INT_BOTH) {
                    GPIO_PORTA_IBE_REG |= pinMask;
                } else {
                    GPIO_PORTA_IBE_REG &= ~pinMask;
                    if (config->intConfig.trigger == GPIO_INT_RISING) {
                        GPIO_PORTA_IEV_REG |= pinMask;
                    } else {
                        GPIO_PORTA_IEV_REG &= ~pinMask;
                    }
                }
                
                /* Enable Interrupt */
                GPIO_PORTA_IM_REG |= pinMask;
            }
            
            /* Set Initial Output Value (if output mode) */
            if (config->mode == GPIO_MODE_OUTPUT) {
                if (config->outputValue) {
                    GPIO_PORTA_DATA_REG |= pinMask;
                } else {
                    GPIO_PORTA_DATA_REG &= ~pinMask;
                }
            }
            break;
            
        case GPIO_PORT_B:
            /* Enable Clock for the GPIO Port */
            SYSCTL_RCGCGPIO_REG |= (1UL << 1);
            while(!(SYSCTL_PRGPIO_REG & (1UL << 1)));
            
            /* Unlock the GPIO Port (if needed) */
            GPIO_PORTB_LOCK_REG = GPIO_LOCK_KEY;
            GPIO_PORTB_CR_REG |= pinMask;
            
            /* Configure Direction */
            if (config->mode == GPIO_MODE_INPUT) {
                GPIO_PORTB_DIR_REG &= ~pinMask;
            } else {
                GPIO_PORTB_DIR_REG |= pinMask;
            }
            
            /* Configure Pull-Up/Pull-Down */
            if (config->pull == GPIO_PULL_UP) {
                GPIO_PORTB_PUR_REG |= pinMask;
            } else if (config->pull == GPIO_PULL_DOWN) {
                GPIO_PORTB_PDR_REG |= pinMask;
            }
            
            /* Configure Alternate Function if needed */
            if (config->mode == GPIO_MODE_AF) {
                uint32 pctlShift = config->pin * 4;
                GPIO_PORTB_PCTL_REG &= ~(0xFUL << pctlShift);
                GPIO_PORTB_PCTL_REG |= (config->afConfig.pctl & 0xF) << pctlShift;
            }
            
            /* Configure Analog Mode if needed */
            if (config->mode == GPIO_MODE_ANALOG) {
                GPIO_PORTB_AMSEL_REG |= pinMask;
            } else {
                /* Enable Digital Function */
                GPIO_PORTB_DEN_REG |= pinMask;
            }
            
            /* Configure Interrupt if enabled */
            if (config->intConfig.enable == GPIO_INT_ENABLE) {
                /* Configure Interrupt Type */
                if (config->intConfig.type == GPIO_INT_EDGE) {
                    GPIO_PORTB_IS_REG &= ~pinMask;
                } else {
                    GPIO_PORTB_IS_REG |= pinMask;
                }
                
                /* Configure Interrupt Trigger */
                if (config->intConfig.trigger == GPIO_INT_BOTH) {
                    GPIO_PORTB_IBE_REG |= pinMask;
                } else {
                    GPIO_PORTB_IBE_REG &= ~pinMask;
                    if (config->intConfig.trigger == GPIO_INT_RISING) {
                        GPIO_PORTB_IEV_REG |= pinMask;
                    } else {
                        GPIO_PORTB_IEV_REG &= ~pinMask;
                    }
                }
                
                /* Enable Interrupt */
                GPIO_PORTB_IM_REG |= pinMask;
            }
            
            /* Set Initial Output Value (if output mode) */
            if (config->mode == GPIO_MODE_OUTPUT) {
                if (config->outputValue) {
                    GPIO_PORTB_DATA_REG |= pinMask;
                } else {
                    GPIO_PORTB_DATA_REG &= ~pinMask;
                }
            }
            break;
            
        case GPIO_PORT_C:
            /* Enable Clock for the GPIO Port */
            SYSCTL_RCGCGPIO_REG |= (1UL << 2);
            while(!(SYSCTL_PRGPIO_REG & (1UL << 2)));
            
            /* Unlock the GPIO Port (if needed) */
            GPIO_PORTC_LOCK_REG = GPIO_LOCK_KEY;
            GPIO_PORTC_CR_REG |= pinMask;
            
            /* Configure Direction */
            if (config->mode == GPIO_MODE_INPUT) {
                GPIO_PORTC_DIR_REG &= ~pinMask;
            } else {
                GPIO_PORTC_DIR_REG |= pinMask;
            }
            
            /* Configure Pull-Up/Pull-Down */
            if (config->pull == GPIO_PULL_UP) {
                GPIO_PORTC_PUR_REG |= pinMask;
            } else if (config->pull == GPIO_PULL_DOWN) {
                GPIO_PORTC_PDR_REG |= pinMask;
            }
            
            /* Configure Alternate Function if needed */
            if (config->mode == GPIO_MODE_AF) {
                uint32 pctlShift = config->pin * 4;
                GPIO_PORTC_PCTL_REG &= ~(0xFUL << pctlShift);
                GPIO_PORTC_PCTL_REG |= (config->afConfig.pctl & 0xF) << pctlShift;
            }
            
            /* Configure Analog Mode if needed */
            if (config->mode == GPIO_MODE_ANALOG) {
                GPIO_PORTC_AMSEL_REG |= pinMask;
            } else {
                /* Enable Digital Function */
                GPIO_PORTC_DEN_REG |= pinMask;
            }
            
            /* Configure Interrupt if enabled */
            if (config->intConfig.enable == GPIO_INT_ENABLE) {
                /* Configure Interrupt Type */
                if (config->intConfig.type == GPIO_INT_EDGE) {
                    GPIO_PORTC_IS_REG &= ~pinMask;
                } else {
                    GPIO_PORTC_IS_REG |= pinMask;
                }
                
                /* Configure Interrupt Trigger */
                if (config->intConfig.trigger == GPIO_INT_BOTH) {
                    GPIO_PORTC_IBE_REG |= pinMask;
                } else {
                    GPIO_PORTC_IBE_REG &= ~pinMask;
                    if (config->intConfig.trigger == GPIO_INT_RISING) {
                        GPIO_PORTC_IEV_REG |= pinMask;
                    } else {
                        GPIO_PORTC_IEV_REG &= ~pinMask;
                    }
                }
                
                /* Enable Interrupt */
                GPIO_PORTC_IM_REG |= pinMask;
            }
            
            /* Set Initial Output Value (if output mode) */
            if (config->mode == GPIO_MODE_OUTPUT) {
                if (config->outputValue) {
                    GPIO_PORTC_DATA_REG |= pinMask;
                } else {
                    GPIO_PORTC_DATA_REG &= ~pinMask;
                }
            }
            break;
            
        case GPIO_PORT_D:
            /* Enable Clock for the GPIO Port */
            SYSCTL_RCGCGPIO_REG |= (1UL << 3);
            while(!(SYSCTL_PRGPIO_REG & (1UL << 3)));
            
            /* Unlock the GPIO Port (if needed) */
            GPIO_PORTD_LOCK_REG = GPIO_LOCK_KEY;
            GPIO_PORTD_CR_REG |= pinMask;
            
            /* Configure Direction */
            if (config->mode == GPIO_MODE_INPUT) {
                GPIO_PORTD_DIR_REG &= ~pinMask;
            } else {
                GPIO_PORTD_DIR_REG |= pinMask;
            }
            
            /* Configure Pull-Up/Pull-Down */
            if (config->pull == GPIO_PULL_UP) {
                GPIO_PORTD_PUR_REG |= pinMask;
            } else if (config->pull == GPIO_PULL_DOWN) {
                GPIO_PORTD_PDR_REG |= pinMask;
            }
            
            /* Configure Alternate Function if needed */
            if (config->mode == GPIO_MODE_AF) {
                uint32 pctlShift = config->pin * 4;
                GPIO_PORTD_PCTL_REG &= ~(0xFUL << pctlShift);
                GPIO_PORTD_PCTL_REG |= (config->afConfig.pctl & 0xF) << pctlShift;
            }
            
            /* Configure Analog Mode if needed */
            if (config->mode == GPIO_MODE_ANALOG) {
                GPIO_PORTD_AMSEL_REG |= pinMask;
            } else {
                /* Enable Digital Function */
                GPIO_PORTD_DEN_REG |= pinMask;
            }
            
            /* Configure Interrupt if enabled */
            if (config->intConfig.enable == GPIO_INT_ENABLE) {
                /* Configure Interrupt Type */
                if (config->intConfig.type == GPIO_INT_EDGE) {
                    GPIO_PORTD_IS_REG &= ~pinMask;
                } else {
                    GPIO_PORTD_IS_REG |= pinMask;
                }
                
                /* Configure Interrupt Trigger */
                if (config->intConfig.trigger == GPIO_INT_BOTH) {
                    GPIO_PORTD_IBE_REG |= pinMask;
                } else {
                    GPIO_PORTD_IBE_REG &= ~pinMask;
                    if (config->intConfig.trigger == GPIO_INT_RISING) {
                        GPIO_PORTD_IEV_REG |= pinMask;
                    } else {
                        GPIO_PORTD_IEV_REG &= ~pinMask;
                    }
                }
                
                /* Enable Interrupt */
                GPIO_PORTD_IM_REG |= pinMask;
            }
            
            /* Set Initial Output Value (if output mode) */
            if (config->mode == GPIO_MODE_OUTPUT) {
                if (config->outputValue) {
                    GPIO_PORTD_DATA_REG |= pinMask;
                } else {
                    GPIO_PORTD_DATA_REG &= ~pinMask;
                }
            }
            break;
            
        case GPIO_PORT_E:
            /* Enable Clock for the GPIO Port */
            SYSCTL_RCGCGPIO_REG |= (1UL << 4);
            while(!(SYSCTL_PRGPIO_REG & (1UL << 4)));
            
            /* Unlock the GPIO Port (if needed) */
            GPIO_PORTE_LOCK_REG = GPIO_LOCK_KEY;
            GPIO_PORTE_CR_REG |= pinMask;
            
            /* Configure Direction */
            if (config->mode == GPIO_MODE_INPUT) {
                GPIO_PORTE_DIR_REG &= ~pinMask;
            } else {
                GPIO_PORTE_DIR_REG |= pinMask;
            }
            
            /* Configure Pull-Up/Pull-Down */
            if (config->pull == GPIO_PULL_UP) {
                GPIO_PORTE_PUR_REG |= pinMask;
            } else if (config->pull == GPIO_PULL_DOWN) {
                GPIO_PORTE_PDR_REG |= pinMask;
            }
            
            /* Configure Alternate Function if needed */
            if (config->mode == GPIO_MODE_AF) {
                uint32 pctlShift = config->pin * 4;
                GPIO_PORTE_PCTL_REG &= ~(0xFUL << pctlShift);
                GPIO_PORTE_PCTL_REG |= (config->afConfig.pctl & 0xF) << pctlShift;
            }
            
            /* Configure Analog Mode if needed */
            if (config->mode == GPIO_MODE_ANALOG) {
                GPIO_PORTE_AMSEL_REG |= pinMask;
            } else {
                /* Enable Digital Function */
                GPIO_PORTE_DEN_REG |= pinMask;
            }
            
            /* Configure Interrupt if enabled */
            if (config->intConfig.enable == GPIO_INT_ENABLE) {
                /* Configure Interrupt Type */
                if (config->intConfig.type == GPIO_INT_EDGE) {
                    GPIO_PORTE_IS_REG &= ~pinMask;
                } else {
                    GPIO_PORTE_IS_REG |= pinMask;
                }
                
                /* Configure Interrupt Trigger */
                if (config->intConfig.trigger == GPIO_INT_BOTH) {
                    GPIO_PORTE_IBE_REG |= pinMask;
                } else {
                    GPIO_PORTE_IBE_REG &= ~pinMask;
                    if (config->intConfig.trigger == GPIO_INT_RISING) {
                        GPIO_PORTE_IEV_REG |= pinMask;
                    } else {
                        GPIO_PORTE_IEV_REG &= ~pinMask;
                    }
                }
                
                /* Enable Interrupt */
                GPIO_PORTE_IM_REG |= pinMask;
            }
            
            /* Set Initial Output Value (if output mode) */
            if (config->mode == GPIO_MODE_OUTPUT) {
                if (config->outputValue) {
                    GPIO_PORTE_DATA_REG |= pinMask;
                } else {
                    GPIO_PORTE_DATA_REG &= ~pinMask;
                }
            }
            break;
            
        case GPIO_PORT_F:
            /* Enable Clock for the GPIO Port */
            SYSCTL_RCGCGPIO_REG |= (1UL << 5);
            while(!(SYSCTL_PRGPIO_REG & (1UL << 5)));
            
            /* Unlock the GPIO Port (if needed) */
            GPIO_PORTF_LOCK_REG = GPIO_LOCK_KEY;
            GPIO_PORTF_CR_REG |= pinMask;
            
            /* Configure Direction */
            if (config->mode == GPIO_MODE_INPUT) {
                GPIO_PORTF_DIR_REG &= ~pinMask;
            } else {
                GPIO_PORTF_DIR_REG |= pinMask;
            }
            
            /* Configure Pull-Up/Pull-Down */
            if (config->pull == GPIO_PULL_UP) {
                GPIO_PORTF_PUR_REG |= pinMask;
            } else if (config->pull == GPIO_PULL_DOWN) {
                GPIO_PORTF_PDR_REG |= pinMask;
            }
            
            /* Configure Alternate Function if needed */
            if (config->mode == GPIO_MODE_AF) {
                uint32 pctlShift = config->pin * 4;
                GPIO_PORTF_PCTL_REG &= ~(0xFUL << pctlShift);
                GPIO_PORTF_PCTL_REG |= (config->afConfig.pctl & 0xF) << pctlShift;
            }
            
            /* Configure Analog Mode if needed */
            if (config->mode == GPIO_MODE_ANALOG) {
                GPIO_PORTF_AMSEL_REG |= pinMask;
            } else {
                /* Enable Digital Function */
                GPIO_PORTF_DEN_REG |= pinMask;
            }
            
            /* Configure Interrupt if enabled */
            if (config->intConfig.enable == GPIO_INT_ENABLE) {
                /* Configure Interrupt Type */
                if (config->intConfig.type == GPIO_INT_EDGE) {
                    GPIO_PORTF_IS_REG &= ~pinMask;
                } else {
                    GPIO_PORTF_IS_REG |= pinMask;
                }
                
                /* Configure Interrupt Trigger */
                if (config->intConfig.trigger == GPIO_INT_BOTH) {
                    GPIO_PORTF_IBE_REG |= pinMask;
                } else {
                    GPIO_PORTF_IBE_REG &= ~pinMask;
                    if (config->intConfig.trigger == GPIO_INT_RISING) {
                        GPIO_PORTF_IEV_REG |= pinMask;
                    } else {
                        GPIO_PORTF_IEV_REG &= ~pinMask;
                    }
                }
                
                /* Enable Interrupt */
                GPIO_PORTF_IM_REG |= pinMask;
            }
            
            /* Set Initial Output Value (if output mode) */
            if (config->mode == GPIO_MODE_OUTPUT) {
                if (config->outputValue) {
                    GPIO_PORTF_DATA_REG |= pinMask;
                } else {
                    GPIO_PORTF_DATA_REG &= ~pinMask;
                }
            }
            break;
            
        default:
            break;
    }
}

/* Write to GPIO Pin */
void GPIO_WritePin(GPIO_PortType port, uint8 pin, uint8 value) {
    if (!GPIO_IsValidPin(pin)) return;
    
    uint32 pinMask = (1UL << pin);
    
    switch(port) {
        case GPIO_PORT_A:
            if (value) {
                GPIO_PORTA_DATA_REG |= pinMask;
            } else {
                GPIO_PORTA_DATA_REG &= ~pinMask;
            }
            break;
            
        case GPIO_PORT_B:
            if (value) {
                GPIO_PORTB_DATA_REG |= pinMask;
            } else {
                GPIO_PORTB_DATA_REG &= ~pinMask;
            }
            break;
            
        case GPIO_PORT_C:
            if (value) {
                GPIO_PORTC_DATA_REG |= pinMask;
            } else {
                GPIO_PORTC_DATA_REG &= ~pinMask;
            }
            break;
            
        case GPIO_PORT_D:
            if (value) {
                GPIO_PORTD_DATA_REG |= pinMask;
            } else {
                GPIO_PORTD_DATA_REG &= ~pinMask;
            }
            break;
            
        case GPIO_PORT_E:
            if (value) {
                GPIO_PORTE_DATA_REG |= pinMask;
            } else {
                GPIO_PORTE_DATA_REG &= ~pinMask;
            }
            break;
            
        case GPIO_PORT_F:
            if (value) {
                GPIO_PORTF_DATA_REG |= pinMask;
            } else {
                GPIO_PORTF_DATA_REG &= ~pinMask;
            }
            break;
            
        default:
            break;
    }
}

/* Read from GPIO Pin */
uint8 GPIO_ReadPin(GPIO_PortType port, uint8 pin) {
    if (!GPIO_IsValidPin(pin)) return 0;
    
    uint32 pinMask = (1UL << pin);
    uint8 value = 0;
    
    switch(port) {
        case GPIO_PORT_A:
            value = (GPIO_PORTA_DATA_REG & pinMask) ? 1 : 0;
            break;
            
        case GPIO_PORT_B:
            value = (GPIO_PORTB_DATA_REG & pinMask) ? 1 : 0;
            break;
            
        case GPIO_PORT_C:
            value = (GPIO_PORTC_DATA_REG & pinMask) ? 1 : 0;
            break;
            
        case GPIO_PORT_D:
            value = (GPIO_PORTD_DATA_REG & pinMask) ? 1 : 0;
            break;
            
        case GPIO_PORT_E:
            value = (GPIO_PORTE_DATA_REG & pinMask) ? 1 : 0;
            break;
            
        case GPIO_PORT_F:
            value = (GPIO_PORTF_DATA_REG & pinMask) ? 1 : 0;
            break;
            
        default:
            break;
    }
    
    return value;
}

/* Write to entire GPIO Port */
void GPIO_WritePort(GPIO_PortType port, uint8 value) {
    switch(port) {
        case GPIO_PORT_A:
            GPIO_PORTA_DATA_REG = value;
            break;
            
        case GPIO_PORT_B:
            GPIO_PORTB_DATA_REG = value;
            break;
            
        case GPIO_PORT_C:
            GPIO_PORTC_DATA_REG = value;
            break;
            
        case GPIO_PORT_D:
            GPIO_PORTD_DATA_REG = value;
            break;
            
        case GPIO_PORT_E:
            GPIO_PORTE_DATA_REG = value;
            break;
            
        case GPIO_PORT_F:
            GPIO_PORTF_DATA_REG = value;
            break;
            
        default:
            break;
    }
}

/* Read from entire GPIO Port */
uint8 GPIO_ReadPort(GPIO_PortType port) {
    uint8 value = 0;
    
    switch(port) {
        case GPIO_PORT_A:
            value = (uint8)GPIO_PORTA_DATA_REG;
            break;
            
        case GPIO_PORT_B:
            value = (uint8)GPIO_PORTB_DATA_REG;
            break;
            
        case GPIO_PORT_C:
            value = (uint8)GPIO_PORTC_DATA_REG;
            break;
            
        case GPIO_PORT_D:
            value = (uint8)GPIO_PORTD_DATA_REG;
            break;
            
        case GPIO_PORT_E:
            value = (uint8)GPIO_PORTE_DATA_REG;
            break;
            
        case GPIO_PORT_F:
            value = (uint8)GPIO_PORTF_DATA_REG;
            break;
            
        default:
            break;
    }
    
    return value;
}

/* Toggle GPIO Pin */
void GPIO_TogglePin(GPIO_PortType port, uint8 pin) {
    if (!GPIO_IsValidPin(pin)) return;
    
    uint32 pinMask = (1UL << pin);
    
    switch(port) {
        case GPIO_PORT_A:
            GPIO_PORTA_DATA_REG ^= pinMask;
            break;
            
        case GPIO_PORT_B:
            GPIO_PORTB_DATA_REG ^= pinMask;
            break;
            
        case GPIO_PORT_C:
            GPIO_PORTC_DATA_REG ^= pinMask;
            break;
            
        case GPIO_PORT_D:
            GPIO_PORTD_DATA_REG ^= pinMask;
            break;
            
        case GPIO_PORT_E:
            GPIO_PORTE_DATA_REG ^= pinMask;
            break;
            
        case GPIO_PORT_F:
            GPIO_PORTF_DATA_REG ^= pinMask;
            break;
            
        default:
            break;
    }
}

/* Enable GPIO Interrupt */
void GPIO_EnableInterrupt(GPIO_PortType port, uint8 pin) {
    if (!GPIO_IsValidPin(pin)) return;
    
    uint32 pinMask = (1UL << pin);
    
    switch(port) {
        case GPIO_PORT_A:
            GPIO_PORTA_IM_REG |= pinMask;
            break;
            
        case GPIO_PORT_B:
            GPIO_PORTB_IM_REG |= pinMask;
            break;
            
        case GPIO_PORT_C:
            GPIO_PORTC_IM_REG |= pinMask;
            break;
            
        case GPIO_PORT_D:
            GPIO_PORTD_IM_REG |= pinMask;
            break;
            
        case GPIO_PORT_E:
            GPIO_PORTE_IM_REG |= pinMask;
            break;
            
        case GPIO_PORT_F:
            GPIO_PORTF_IM_REG |= pinMask;
            break;
            
        default:
            break;
    }
}

/* Disable GPIO Interrupt */
void GPIO_DisableInterrupt(GPIO_PortType port, uint8 pin) {
    if (!GPIO_IsValidPin(pin)) return;
    
    uint32 pinMask = (1UL << pin);
    
    switch(port) {
        case GPIO_PORT_A:
            GPIO_PORTA_IM_REG &= ~pinMask;
            break;
            
        case GPIO_PORT_B:
            GPIO_PORTB_IM_REG &= ~pinMask;
            break;
            
        case GPIO_PORT_C:
            GPIO_PORTC_IM_REG &= ~pinMask;
            break;
            
        case GPIO_PORT_D:
            GPIO_PORTD_IM_REG &= ~pinMask;
            break;
            
        case GPIO_PORT_E:
            GPIO_PORTE_IM_REG &= ~pinMask;
            break;
            
        case GPIO_PORT_F:
            GPIO_PORTF_IM_REG &= ~pinMask;
            break;
            
        default:
            break;
    }
}

/* Clear GPIO Interrupt */
void GPIO_ClearInterrupt(GPIO_PortType port, uint8 pin) {
    if (!GPIO_IsValidPin(pin)) return;
    
    uint32 pinMask = (1UL << pin);
    
    switch(port) {
        case GPIO_PORT_A:
            GPIO_PORTA_ICR_REG = pinMask;
            break;
            
        case GPIO_PORT_B:
            GPIO_PORTB_ICR_REG = pinMask;
            break;
            
        case GPIO_PORT_C:
            GPIO_PORTC_ICR_REG = pinMask;
            break;
            
        case GPIO_PORT_D:
            GPIO_PORTD_ICR_REG = pinMask;
            break;
            
        case GPIO_PORT_E:
            GPIO_PORTE_ICR_REG = pinMask;
            break;
            
        case GPIO_PORT_F:
            GPIO_PORTF_ICR_REG = pinMask;
            break;
            
        default:
            break;
    }
}

/* Get GPIO Interrupt Status */
uint8 GPIO_GetInterruptStatus(GPIO_PortType port, uint8 pin) {
    if (!GPIO_IsValidPin(pin)) return 0;
    
    uint32 pinMask = (1UL << pin);
    uint8 status = 0;
    
    switch(port) {
        case GPIO_PORT_A:
            status = (GPIO_PORTA_RIS_REG & pinMask) ? 1 : 0;
            break;
            
        case GPIO_PORT_B:
            status = (GPIO_PORTB_RIS_REG & pinMask) ? 1 : 0;
            break;
            
        case GPIO_PORT_C:
            status = (GPIO_PORTC_RIS_REG & pinMask) ? 1 : 0;
            break;
            
        case GPIO_PORT_D:
            status = (GPIO_PORTD_RIS_REG & pinMask) ? 1 : 0;
            break;
            
        case GPIO_PORT_E:
            status = (GPIO_PORTE_RIS_REG & pinMask) ? 1 : 0;
            break;
            
        case GPIO_PORT_F:
            status = (GPIO_PORTF_RIS_REG & pinMask) ? 1 : 0;
            break;
            
        default:
            break;
    }
    
    return status;
}

/* Initialize built-in buttons and LEDs */
void GPIO_BuiltinButtonsLedsInit(void)
{
    /* Enable clock for PORTF */
    SYSCTL_RCGCGPIO_REG |= 0x20;
    while(!(SYSCTL_PRGPIO_REG & 0x20));

    /* Unlock PORTF */
    GPIO_PORTF_LOCK_REG = 0x4C4F434B;
    GPIO_PORTF_CR_REG = 0x1F;

    /* Configure PF1-PF3 as outputs (LEDs) */
    GPIO_PORTF_DIR_REG |= 0x0E;       /* PF1, PF2, PF3 as outputs */
    GPIO_PORTF_DEN_REG |= 0x1F;       /* Enable digital I/O on PF0-PF4 */
    GPIO_PORTF_PUR_REG |= 0x11;       /* Enable pull-up on PF0 and PF4 (switches) */
}

/* Toggle red LED (PF1) */
void GPIO_RedLedToggle(void)
{
    GPIO_PORTF_DATA_REG ^= 0x02;
}

/* Toggle green LED (PF3) */
void GPIO_GreenLedToggle(void)
{
    GPIO_PORTF_DATA_REG ^= 0x08;
}

/* Toggle blue LED (PF2) */
void GPIO_BlueLedToggle(void)
{
    GPIO_PORTF_DATA_REG ^= 0x04;
}
