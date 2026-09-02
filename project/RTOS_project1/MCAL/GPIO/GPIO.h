/**********************************************************************************************
 *
 * Module: GPIO
 *
 * File Name: GPIO.h
 *
 * Description: Header file for the TM4C123GH6PM GPIO driver for TivaC Built-in Buttons and LEDs
 *
 * Author: mohamed yasser
 *
 ***********************************************************************************************/
#ifndef GPIO_H
#define GPIO_H

#include "Common/std_types.h"
#include "tm4c123gh6pm_registers.h"

/* GPIO Port Definitions */
typedef enum {
    GPIO_PORT_A = 0,
    GPIO_PORT_B,
    GPIO_PORT_C,
    GPIO_PORT_D,
    GPIO_PORT_E,
    GPIO_PORT_F,
    GPIO_NUM_PORTS
} GPIO_PortType;

/* GPIO Pin Modes */
typedef enum {
    GPIO_MODE_INPUT = 0,
    GPIO_MODE_OUTPUT,
    GPIO_MODE_AF,  // Alternate Function
    GPIO_MODE_ANALOG
} GPIO_ModeType;

/* GPIO Pull-Up/Pull-Down Configuration */
typedef enum {
    GPIO_PULL_NONE = 0,
    GPIO_PULL_UP,
    GPIO_PULL_DOWN
} GPIO_PullType;

/* GPIO Interrupt Configuration */
typedef enum {
    GPIO_INT_DISABLE = 0,
    GPIO_INT_ENABLE
} GPIO_IntEnableType;

typedef enum {
    GPIO_INT_EDGE = 0,
    GPIO_INT_LEVEL
} GPIO_IntType;

typedef enum {
    GPIO_INT_FALLING = 0,
    GPIO_INT_RISING,
    GPIO_INT_BOTH
} GPIO_IntTriggerType;

/* GPIO Alternate Function Configuration */
typedef struct {
    uint8 afsel;    // Alternate Function Select (0-15)
    uint8 pctl;     // Port Control (0-15)
} GPIO_AFConfigType;

/* GPIO Interrupt Configuration Structure */
typedef struct {
    GPIO_IntEnableType enable;
    GPIO_IntType type;
    GPIO_IntTriggerType trigger;
} GPIO_IntConfigType;

/* GPIO Initialization Structure */
typedef struct {
    GPIO_PortType port;
    uint8 pin;            // Pin number (0-7)
    GPIO_ModeType mode;
    GPIO_PullType pull;
    uint8 outputValue;    // Initial output value (for output mode)
    GPIO_AFConfigType afConfig;  // Alternate function configuration (if mode is GPIO_MODE_AF)
    GPIO_IntConfigType intConfig; // Interrupt configuration
} GPIO_ConfigType;

/* GPIO Lock Key */
#define GPIO_LOCK_KEY            0x4C4F434B

/* Public Functions */
void GPIO_Init(const GPIO_ConfigType *config);
void GPIO_WritePin(GPIO_PortType port, uint8 pin, uint8 value);
uint8 GPIO_ReadPin(GPIO_PortType port, uint8 pin);
void GPIO_WritePort(GPIO_PortType port, uint8 value);
uint8 GPIO_ReadPort(GPIO_PortType port);
void GPIO_TogglePin(GPIO_PortType port, uint8 pin);

/* Interrupt Functions */
void GPIO_EnableInterrupt(GPIO_PortType port, uint8 pin);
void GPIO_DisableInterrupt(GPIO_PortType port, uint8 pin);
void GPIO_ClearInterrupt(GPIO_PortType port, uint8 pin);
uint8 GPIO_GetInterruptStatus(GPIO_PortType port, uint8 pin);

/* Function Declarations */
void GPIO_BuiltinButtonsLedsInit(void);
void GPIO_RedLedToggle(void);
void GPIO_GreenLedToggle(void);
void GPIO_BlueLedToggle(void);

#endif /* GPIO_H */
