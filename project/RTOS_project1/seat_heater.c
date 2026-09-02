#include "seat_heater.h"
#include "gpio.h"
#include "adc.h"
#include "uart.h"
#include "gptm.h"
#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "Common/app_config.h"
#include "task.h"
#include "MCAL/tm4c123gh6pm_registers.h"

/* Queue Handles */
QueueHandle_t xDriverSeatQueue = NULL;
QueueHandle_t xPassengerSeatQueue = NULL;
QueueHandle_t xDisplayQueue = NULL;
QueueHandle_t xDiagnosticQueue = NULL;
QueueHandle_t xStatisticsQueue = NULL;

/* Task Handles */
TaskHandle_t xDriverSeatTaskHandle = NULL;
TaskHandle_t xPassengerSeatTaskHandle = NULL;
TaskHandle_t xDisplayTaskHandle = NULL;
TaskHandle_t xDiagnosticTaskHandle = NULL;
TaskHandle_t xStatisticsTaskHandle = NULL;
TaskHandle_t xSafetyTaskHandle = NULL;
TaskHandle_t xPowerManagementTaskHandle = NULL;

/* Event Group for Button Presses */
EventGroupHandle_t xSeatHeaterEventGroup = NULL;

/* Static queue storage */
static uint8_t ucDriverSeatQueueStorage[2 * sizeof(SeatStateType)];
static StaticQueue_t xDriverSeatQueueBuffer;
static uint8_t ucPassengerSeatQueueStorage[2 * sizeof(SeatStateType)];
static StaticQueue_t xPassengerSeatQueueBuffer;
static uint8_t ucDisplayQueueStorage[2 * sizeof(SeatStateMessageType)];
static StaticQueue_t xDisplayQueueBuffer;
static uint8_t ucDiagnosticQueueStorage[2 * sizeof(DiagnosticLogEntryType)];
static StaticQueue_t xDiagnosticQueueBuffer;
static uint8_t ucStatisticsQueueStorage[2 * sizeof(SeatStateType)];
static StaticQueue_t xStatisticsQueueBuffer;

/* Mutex for Shared Resources */
SemaphoreHandle_t xDisplayMutex = NULL;
SemaphoreHandle_t xDiagnosticMutex = NULL;
SemaphoreHandle_t xStatisticsMutex = NULL;

/* Diagnostic Log */
DiagnosticLogEntryType diagnosticLog[25];  /* Match size with header */
uint16_t diagnosticLogIndex = 0;

/* Seat States */
SeatStateType driverSeatState = {
    .seat = DRIVER_SEAT,
    .currentLevel = HEAT_OFF,
    .currentTemperature = 0,  /* Fixed-point value (0.0 * 10) */
    .heaterIntensity = HEATER_OFF,
    .sensorError = 0,
    .lastLevelChangeTime = 0,
    .lastErrorTime = 0,
    .totalHeatingTime = 0,
    .lastDiagnosticTime = 0
};

SeatStateType passengerSeatState = {
    .seat = PASSENGER_SEAT,
    .currentLevel = HEAT_OFF,
    .currentTemperature = 0,  /* Fixed-point value (0.0 * 10) */
    .heaterIntensity = HEATER_OFF,
    .sensorError = 0,
    .lastLevelChangeTime = 0,
    .lastErrorTime = 0,
    .totalHeatingTime = 0,
    .lastDiagnosticTime = 0
};

/* GPIO Pin Definitions */
#define DRIVER_BUTTON_PIN_1    0    /* PF0 - Onboard SW2 for driver */
#define PASSENGER_BUTTON_PIN   4    /* PF4 - Onboard SW1 for passenger */
#define DRIVER_BUTTON_PIN_2    2    /* PA2 - External button for driver */
#define DRIVER_HEATER_PIN      1    /* PF1 - Blue LED */
#define PASSENGER_HEATER_PIN   3    /* PF3 - Green LED */
#define ERROR_LED_PIN         2    /* PF2 - Red LED for both errors */

/* ADC Channel Definitions */
#define DRIVER_TEMP_CHANNEL    0  /* ADC channel for driver seat temperature */
#define PASSENGER_TEMP_CHANNEL 1  /* ADC channel for passenger seat temperature */

/* Event Group Bits */
#define DRIVER_BUTTON_PRESSED_BIT    (1 << 0)
#define PASSENGER_BUTTON_PRESSED_BIT (1 << 1)
#define STEERING_BUTTON_PRESSED_BIT  (1 << 2)

/* Temperature Constants */
#define TEMP_LOW    25.0
#define TEMP_MEDIUM 30.0
#define TEMP_HIGH   35.0
#define TEMP_TOLERANCE 2.0

/* Error Codes */
#define ERROR_SENSOR_FAILURE 0x01
#define ERROR_OVER_TEMPERATURE 0x02
#define ERROR_POWER_FAILURE 0x03

/* Current seat being monitored */
static uint8_t xCurrentSeat = DRIVER_SEAT;

/* Temperature variables */
static uint32_t xDriverTemp = 0;
static uint32_t xPassengerTemp = 0;

/* Optimized display data structure */
static struct {
    uint8_t driverLevel;
    uint8_t driverHeater;
    uint8_t passengerLevel;
    uint8_t passengerHeater;
    uint16_t driverTemp;
    uint16_t passengerTemp;
    uint8_t driverError;
    uint8_t passengerError;
} displayData;

/* Error LED blinking patterns */
#define ERROR_BLINK_PERIOD    100   /* 100ms per blink state */
#define DRIVER_ERROR_PATTERN   0x5   /* 0b0101 - Single blink */
#define PASSENGER_ERROR_PATTERN 0xF  /* 0b1111 - Double blink */

/* Add error LED state tracking */
static struct {
    bool isBlinking;
    uint8_t pattern;
    uint8_t patternIndex;
    TickType_t lastToggle;
} errorLedState = {0};

/* Function declarations */
static uint16_t prvConvertADCToTemperature(uint16 adcValue);
static void prvInitGPIO(void);
static void prvInitADC(void);
static void prvInitUART(void);
static void prvInitGPTM(void);
static HeaterIntensityType prvCalculateHeaterIntensity(uint16_t currentTemp, uint16_t desiredTemp);
static boolean prvIsTemperatureValid(uint16_t temperature);
static void prvUpdateHeaterState(SeatStateType *seatState);
static void prvControlHeaterLEDs(SeatType seat, HeaterIntensityType intensity);
static void prvControlErrorLEDs(SeatType seat, boolean error);
static void prvADCCallback(void);
static void prvUpdateErrorLED(void);
static void prvGPIODriverButton1Callback(void);
static void prvGPIODriverButton2Callback(void);
static void prvGPIOPassengerButtonCallback(void);

/* Add missing register definition */
#define GPIO_PORTA_MIS_REG       (*((volatile uint32 *)0x40004418))

/* ADC callback function */
static void prvADCCallback(void)
{
    uint32_t ulADCValue;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    /* Enter critical section */
    UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
    
    /* Clear ADC interrupt flag */
    ADC0_ISC_REG |= (1UL << 3);
    
    /* Read ADC value */
    ulADCValue = ADC0_SSFIFO3_REG & 0xFFFUL;
    
    /* Ensure ADC value is valid (12-bit ADC) */
    if (ulADCValue > 0xFFF) {
        ulADCValue = 0xFFF;
    }
    
    /* Convert ADC value to temperature and update current seat */
    if (xCurrentSeat == DRIVER_SEAT) {
        xDriverTemp = (uint32_t)prvConvertADCToTemperature((uint16_t)ulADCValue);
        driverSeatState.currentTemperature = (uint16_t)xDriverTemp;
        xCurrentSeat = PASSENGER_SEAT;
        ADC0_SSMUX3_REG = PASSENGER_TEMP_CHANNEL;
    } else {
        xPassengerTemp = (uint32_t)prvConvertADCToTemperature((uint16_t)ulADCValue);
        passengerSeatState.currentTemperature = (uint16_t)xPassengerTemp;
        xCurrentSeat = DRIVER_SEAT;
        ADC0_SSMUX3_REG = DRIVER_TEMP_CHANNEL;
    }
    
    /* Start next conversion */
    ADC0_PSSI_REG |= (1UL << 3);
    
    /* Exit critical section */
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
    
    /* Notify task if needed */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* Function to initialize GPIO pins */
static void prvInitGPIO(void)
{
    /* Configure onboard buttons (PF0 and PF4) */
    GPIO_ConfigType buttonConfig = {
        .port = GPIO_PORT_F,
        .pin = DRIVER_BUTTON_PIN_1,
        .mode = GPIO_MODE_INPUT,
        .pull = GPIO_PULL_UP,
        .intConfig = {
            .enable = GPIO_INT_ENABLE,
            .type = GPIO_INT_EDGE,
            .trigger = GPIO_INT_FALLING
        }
    };
    GPIO_Init(&buttonConfig);

    buttonConfig.pin = PASSENGER_BUTTON_PIN;
    GPIO_Init(&buttonConfig);

    /* Configure external driver button (PA2) */
    buttonConfig.port = GPIO_PORT_A;
    buttonConfig.pin = DRIVER_BUTTON_PIN_2;
    GPIO_Init(&buttonConfig);

    /* Configure LED pins */
    GPIO_ConfigType ledConfig = {
        .port = GPIO_PORT_F,
        .mode = GPIO_MODE_OUTPUT,
        .pull = GPIO_PULL_NONE,
        .intConfig = {
            .enable = GPIO_INT_DISABLE
        }
    };

    /* Configure onboard LEDs */
    ledConfig.pin = DRIVER_HEATER_PIN;    /* PF1 - Blue LED */
    GPIO_Init(&ledConfig);

    ledConfig.pin = ERROR_LED_PIN;        /* PF2 - Red LED */
    GPIO_Init(&ledConfig);

    ledConfig.pin = PASSENGER_HEATER_PIN;  /* PF3 - Green LED */
    GPIO_Init(&ledConfig);

    /* Turn off all LEDs initially */
    GPIO_WritePin(GPIO_PORT_F, DRIVER_HEATER_PIN, 0);
    GPIO_WritePin(GPIO_PORT_F, PASSENGER_HEATER_PIN, 0);
    GPIO_WritePin(GPIO_PORT_F, ERROR_LED_PIN, 0);

    /* Configure interrupts for Port F buttons */
    GPIO_PORTF_IS_REG &= ~((1<<DRIVER_BUTTON_PIN_1) | (1<<PASSENGER_BUTTON_PIN));  /* Edge sensitive */
    GPIO_PORTF_IBE_REG &= ~((1<<DRIVER_BUTTON_PIN_1) | (1<<PASSENGER_BUTTON_PIN)); /* Single edge */
    GPIO_PORTF_IEV_REG &= ~((1<<DRIVER_BUTTON_PIN_1) | (1<<PASSENGER_BUTTON_PIN)); /* Falling edge */
    GPIO_PORTF_ICR_REG = ((1<<DRIVER_BUTTON_PIN_1) | (1<<PASSENGER_BUTTON_PIN));   /* Clear any prior interrupts */
    GPIO_PORTF_IM_REG |= ((1<<DRIVER_BUTTON_PIN_1) | (1<<PASSENGER_BUTTON_PIN));   /* Enable interrupts */

    /* Configure interrupts for Port A button */
    GPIO_PORTA_IS_REG &= ~(1<<DRIVER_BUTTON_PIN_2);  /* Edge sensitive */
    GPIO_PORTA_IBE_REG &= ~(1<<DRIVER_BUTTON_PIN_2); /* Single edge */
    GPIO_PORTA_IEV_REG &= ~(1<<DRIVER_BUTTON_PIN_2); /* Falling edge */
    GPIO_PORTA_ICR_REG = (1<<DRIVER_BUTTON_PIN_2);   /* Clear any prior interrupts */
    GPIO_PORTA_IM_REG |= (1<<DRIVER_BUTTON_PIN_2);   /* Enable interrupts */

    /* Set interrupt priorities */
    NVIC_PRI7_REG = (NVIC_PRI7_REG & 0xFF1FFFFF) | (6 << 21); /* Priority 6 for Port F */
    NVIC_PRI0_REG = (NVIC_PRI0_REG & 0xFFFFFF1F) | (6 << 5);  /* Priority 6 for Port A */

    /* Enable interrupts in NVIC */
    NVIC_EN0_REG |= (1 << 30); /* Enable Port F interrupt */
    NVIC_EN0_REG |= (1 << 0);  /* Enable Port A interrupt */
}

/* GPIO Port F interrupt handler */
void GPIOF_Handler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t status = GPIO_PORTF_MIS_REG;

    /* Clear the interrupts */
    GPIO_PORTF_ICR_REG = status;

    /* Handle button presses */
    if (status & (1 << DRIVER_BUTTON_PIN_1)) {
        xEventGroupSetBitsFromISR(xSeatHeaterEventGroup, 
                                DRIVER_BUTTON_PRESSED_BIT, 
                                &xHigherPriorityTaskWoken);
    }

    if (status & (1 << PASSENGER_BUTTON_PIN)) {
        xEventGroupSetBitsFromISR(xSeatHeaterEventGroup, 
                                PASSENGER_BUTTON_PRESSED_BIT, 
                                &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* GPIO Port A interrupt handler */
void GPIOA_Handler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t status = GPIO_PORTA_MIS_REG;

    /* Clear the interrupts */
    GPIO_PORTA_ICR_REG = status;

    /* Handle button press */
    if (status & (1 << DRIVER_BUTTON_PIN_2)) {
        xEventGroupSetBitsFromISR(xSeatHeaterEventGroup, 
                                DRIVER_BUTTON_PRESSED_BIT, 
                                &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* Function to initialize ADC */
static void prvInitADC(void)
{
    /* Configure ADC for driver seat temperature */
    ADC_ConfigType adcConfig = {
        .module = ADC0,
        .sequencer = ADC_SEQUENCER3,
        .channel = (ADC_ChannelType)DRIVER_TEMP_CHANNEL,
        .trigger = ADC_TRIGGER_PROCESSOR,
        .speed = ADC_SPEED_125K,
        .enableInterrupt = TRUE
    };
    
    /* Initialize ADC for driver seat */
    ADC_Init(&adcConfig);
    
    /* Configure ADC for passenger seat temperature */
    adcConfig.channel = (ADC_ChannelType)PASSENGER_TEMP_CHANNEL;
    ADC_Init(&adcConfig);
    
    /* Set ADC Sequencer 3 interrupt priority (higher than kernel) */
    NVIC_PRI4_REG = (NVIC_PRI4_REG & 0xFFFF1FFF) | (6 << 13);  /* Priority 6 - lower than configMAX_SYSCALL_INTERRUPT_PRIORITY */
    
    /* Clear any pending interrupts */
    ADC0_ISC_REG = 0x8;  /* Clear Sequencer 3 interrupt flag */
    
    /* Enable ADC interrupt in NVIC */
    NVIC_EN0_REG |= (1 << 17);  /* Enable IRQ17 for ADC0 Sequence 3 */
    
    /* Register callback function */
    ADC_SetCallBack(ADC0, ADC_SEQUENCER3, prvADCCallback);
    
    /* Start first conversion */
    ADC_StartConversion(ADC0, ADC_SEQUENCER3);
}

/* Function to initialize UART for display */
static void prvInitUART(void)
{
    UART_ConfigType uartConfig = {
        .port = UART_PORT_0,
        .baudRate = 115200,
        .dataLength = UART_DATA_8BITS,
        .stopBits = UART_STOP_1BIT,
        .parity = UART_PARITY_NONE,
        .fifoEnable = UART_FIFO_ENABLE
    };
    
    UART_Init(&uartConfig);
}

/* Function to initialize GPTM for timing */
static void prvInitGPTM(void)
{
    /* Enable Timer0 clock */
    SYSCTL_RCGCTIMER_REG |= 0x01;  /* Enable Timer0 clock */
    while(!(SYSCTL_PRTIMER_REG & 0x01)); /* Wait for Timer0 clock to be ready */
    
    /* Disable Timer0 before configuration */
    TIMER0_CTL_REG &= ~(1 << 0);  /* Clear bit 0 to disable Timer A */
    
    /* Configure Timer0 as periodic timer */
    TIMER0_CFG_REG = 0x00;  /* 32-bit timer configuration */
    TIMER0_TAMR_REG = 0x2;  /* Periodic mode and count down */
    TIMER0_TAILR_REG = (uint32_t)15999U;  /* 1ms at 16MHz system clock (16000 - 1) */
    
    /* Set Timer0A interrupt priority (higher than kernel) */
    NVIC_PRI4_REG = (NVIC_PRI4_REG & 0x1FFFFFFF) | (6 << 29);  /* Priority 6 - lower than configMAX_SYSCALL_INTERRUPT_PRIORITY */
    
    /* Clear any pending interrupt */
    TIMER0_ICR_REG = 0x1;  /* Clear Timer A timeout interrupt flag */
    
    /* Enable Timer0A interrupt */
    TIMER0_IMR_REG |= 0x1;  /* Enable Timer A timeout interrupt */
    
    /* Enable Timer0A interrupt in NVIC */
    NVIC_EN0_REG |= (1 << 19);  /* Enable IRQ19 for Timer0A */
    
    /* Enable Timer0 */
    TIMER0_CTL_REG |= (1 << 0);  /* Set bit 0 to enable Timer A */
}

/* Function to calculate heater intensity based on temperature difference */
static HeaterIntensityType prvCalculateHeaterIntensity(uint16_t currentTemp, uint16_t desiredTemp)
{
    int16_t diff = desiredTemp - currentTemp;
    
    if (diff > 50)  /* 5.0°C difference in fixed-point */
        return HEATER_HIGH;
    else if (diff > 20)  /* 2.0°C difference in fixed-point */
        return HEATER_MEDIUM;
    else if (diff > 0)
        return HEATER_LOW;
    else
        return HEATER_OFF;
}

/* Function to convert ADC reading to temperature */
static uint16_t prvConvertADCToTemperature(uint16 adcValue)
{
    /* Convert ADC value to temperature in fixed-point (temperature * 10) */
    /* Assuming 10mV/°C and 3.3V reference */
    float voltage = (adcValue * 3.3f) / 4096.0f;  /* 12-bit ADC */
    float temperature = voltage * 100.0f;  /* Convert to fixed-point (temperature * 10) */
    
    /* Ensure temperature is within valid range (0-50°C) */
    if (temperature > 500.0f) {  /* 50.0°C in fixed-point */
        temperature = 500.0f;
    } else if (temperature < 0.0f) {
        temperature = 0.0f;
    }
    
    /* Add some filtering to prevent rapid changes */
    static uint16_t lastTemp[2] = {0, 0};
    uint16_t newTemp = (uint16_t)temperature;
    uint8_t seatIndex = (xCurrentSeat == DRIVER_SEAT) ? 0 : 1;
    
    /* Simple moving average filter */
    newTemp = (newTemp + lastTemp[seatIndex]) / 2;
    lastTemp[seatIndex] = newTemp;
    
    return newTemp;
}

/* Function to check if temperature is within valid range */
static boolean prvIsTemperatureValid(uint16_t temperature)
{
    /* Check if temperature is within valid range (5-50°C) */
    return (temperature >= (uint16_t)(5.0f * 10.0f) && 
            temperature <= (uint16_t)(50.0f * 10.0f));  /* Temperature range in fixed-point */
}

/* Function to update heater state */
static void prvUpdateHeaterState(SeatStateType *seatState)
{
    uint16_t desiredTemp;
    
    /* If there's a sensor error or heating is off, disable heater */
    if (seatState->sensorError || seatState->currentLevel == HEAT_OFF) {
        seatState->heaterIntensity = HEATER_OFF;
        return;
    }
    
    /* Set desired temperature based on heating level */
    switch (seatState->currentLevel) {
        case HEAT_LOW:
            desiredTemp = (uint16_t)(TEMP_LOW * 10.0f);
            break;
        case HEAT_MEDIUM:
            desiredTemp = (uint16_t)(TEMP_MEDIUM * 10.0f);
            break;
        case HEAT_HIGH:
            desiredTemp = (uint16_t)(TEMP_HIGH * 10.0f);
            break;
        default:
            desiredTemp = 0;
            seatState->heaterIntensity = HEATER_OFF;
            return;
    }
    
    seatState->heaterIntensity = prvCalculateHeaterIntensity(seatState->currentTemperature, desiredTemp);
}

/* Function to control heater LEDs */
static void prvControlHeaterLEDs(SeatType seat, HeaterIntensityType intensity)
{
    uint8_t pin = (seat == DRIVER_SEAT) ? DRIVER_HEATER_PIN : PASSENGER_HEATER_PIN;
    GPIO_PortType port = (seat == DRIVER_SEAT) ? GPIO_PORT_F : GPIO_PORT_F;
    
    GPIO_WritePin(port, pin, (intensity != HEATER_OFF) ? 1 : 0);
}

/* Function to control error LEDs */
static void prvControlErrorLEDs(SeatType seat, boolean error)
{
    if (error) {
        errorLedState.isBlinking = true;
        errorLedState.pattern = (seat == DRIVER_SEAT) ? DRIVER_ERROR_PATTERN : PASSENGER_ERROR_PATTERN;
        errorLedState.patternIndex = 0;
        errorLedState.lastToggle = xTaskGetTickCount();
    } else {
        /* Only turn off if both seats are error-free */
        if ((seat == DRIVER_SEAT && !passengerSeatState.sensorError) ||
            (seat == PASSENGER_SEAT && !driverSeatState.sensorError)) {
            errorLedState.isBlinking = false;
        }
    }
}

/* Function to update error LED pattern */
static void prvUpdateErrorLED(void)
{
    if (!errorLedState.isBlinking) {
        GPIO_WritePin(GPIO_PORT_F, ERROR_LED_PIN, 0);
        return;
    }

    TickType_t currentTime = xTaskGetTickCount();
    
    if ((currentTime - errorLedState.lastToggle) >= pdMS_TO_TICKS(ERROR_BLINK_PERIOD)) {
        /* Update pattern index */
        errorLedState.patternIndex = (errorLedState.patternIndex + 1) % 4;
        
        /* Set LED based on current pattern bit */
        uint8_t ledState = (errorLedState.pattern >> errorLedState.patternIndex) & 0x1;
        GPIO_WritePin(GPIO_PORT_F, ERROR_LED_PIN, ledState);
        
        errorLedState.lastToggle = currentTime;
    }
}

/* Task for driver seat control */
static void prvDriverSeatTask(void *pvParameters)
{
    SeatStateType *seatState = &driverSeatState;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100UL); /* 100ms period */
    SeatStateMessageType message;
    
    for (;;) {
        /* Wait for the next cycle */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        /* Check for sensor error */
        seatState->sensorError = !prvIsTemperatureValid(seatState->currentTemperature);
        prvControlErrorLEDs(DRIVER_SEAT, seatState->sensorError);
        
        /* Update heater state */
        prvUpdateHeaterState(seatState);
        prvControlHeaterLEDs(DRIVER_SEAT, seatState->heaterIntensity);
        
        /* Update heating time */
        if (seatState->currentLevel != HEAT_OFF) {
            seatState->totalHeatingTime += xFrequency;
            /* Prevent overflow */
            if (seatState->totalHeatingTime > UINT32_MAX - xFrequency) {
                seatState->totalHeatingTime = 0;
            }
        }
        
        /* Prepare message for display queue */
        message.seat = DRIVER_SEAT;
        message.currentLevel = seatState->currentLevel;
        message.currentTemperature = seatState->currentTemperature;
        message.heaterIntensity = seatState->heaterIntensity;
        message.sensorError = seatState->sensorError;
        message.totalHeatingTime = seatState->totalHeatingTime;
        
        /* Send state to display queue */
        xQueueSend(xDisplayQueue, &message, 0);
        
        /* Send to diagnostic queue if needed */
        if (seatState->sensorError) {
            SeatHeater_LogDiagnostic(DRIVER_SEAT, ERROR_SENSOR_FAILURE);
        }
    }
}

/* Task for passenger seat control */
static void prvPassengerSeatTask(void *pvParameters)
{
    SeatStateType *seatState = &passengerSeatState;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100UL); /* 100ms period */
    SeatStateMessageType message;
    
    for (;;) {
        /* Wait for the next cycle */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        /* Check for sensor error */
        seatState->sensorError = !prvIsTemperatureValid(seatState->currentTemperature);
        prvControlErrorLEDs(PASSENGER_SEAT, seatState->sensorError);
        
        /* Update heater state */
        prvUpdateHeaterState(seatState);
        prvControlHeaterLEDs(PASSENGER_SEAT, seatState->heaterIntensity);
        
        /* Update heating time */
        if (seatState->currentLevel != HEAT_OFF) {
            seatState->totalHeatingTime += xFrequency;
            /* Prevent overflow */
            if (seatState->totalHeatingTime > UINT32_MAX - xFrequency) {
                seatState->totalHeatingTime = 0;
            }
        }
        
        /* Prepare message for display queue */
        message.seat = PASSENGER_SEAT;
        message.currentLevel = seatState->currentLevel;
        message.currentTemperature = seatState->currentTemperature;
        message.heaterIntensity = seatState->heaterIntensity;
        message.sensorError = seatState->sensorError;
        message.totalHeatingTime = seatState->totalHeatingTime;
        
        /* Send state to display queue */
        xQueueSend(xDisplayQueue, &message, 0);
        
        /* Send to diagnostic queue if needed */
        if (seatState->sensorError) {
            SeatHeater_LogDiagnostic(PASSENGER_SEAT, ERROR_SENSOR_FAILURE);
        }
    }
}

/* Task for display */
static void prvDisplayRefreshTask(void *pvParameters)
{
    /* Minimize string constants by combining common patterns */
    static const char NL[] = "\r\n";
    static const char HEADER[] = "=== Seat Heater ===\r\n\r\n";
    static const char DRIVER[] = "D: ";
    static const char PASSENGER[] = "P: ";
    static const char *const level_str[] = {"OFF", "LOW", "MED", "HIGH"};
    static const char *const status_str[] = {"OK", "ERR"};
    
    TickType_t xLastWakeTime;
    uint16_t temp;
    uint8_t level, error;
    char digit;
    
    xLastWakeTime = xTaskGetTickCount();
    
    for (;;) {
        /* Process any pending messages */
        SeatStateMessageType message;
        while (xQueueReceive(xDisplayQueue, &message, 0) == pdPASS) {
            /* Just drain the queue */
        }
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2000));
        
        if (xSemaphoreTake(xDisplayMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
            continue;
        }
        
        /* Header */
        UART_SendString(UART_PORT_0, (uint8 *)HEADER);
        
        /* Driver Seat */
        UART_SendString(UART_PORT_0, (uint8 *)DRIVER);
        
        /* Temperature */
        temp = driverSeatState.currentTemperature;
        digit = '0' + (uint8_t)(temp / 100);
        if (digit != '0') {
            UART_SendByte(UART_PORT_0, digit);
        }
        UART_SendByte(UART_PORT_0, '0' + ((temp / 10) % 10));
        UART_SendByte(UART_PORT_0, '.');
        UART_SendByte(UART_PORT_0, '0' + (temp % 10));
        UART_SendByte(UART_PORT_0, ' ');
        
        /* Level and Status */
        level = (uint8_t)driverSeatState.currentLevel;
        error = driverSeatState.sensorError;
        UART_SendString(UART_PORT_0, (uint8 *)level_str[level]);
        UART_SendByte(UART_PORT_0, ' ');
        UART_SendString(UART_PORT_0, (uint8 *)status_str[error]);
        UART_SendString(UART_PORT_0, (uint8 *)NL);
        
        /* Passenger Seat */
        UART_SendString(UART_PORT_0, (uint8 *)PASSENGER);
        
        /* Temperature */
        temp = passengerSeatState.currentTemperature;
        digit = '0' + (uint8_t)(temp / 100);
        if (digit != '0') {
            UART_SendByte(UART_PORT_0, digit);
        }
        UART_SendByte(UART_PORT_0, '0' + ((temp / 10) % 10));
        UART_SendByte(UART_PORT_0, '.');
        UART_SendByte(UART_PORT_0, '0' + (temp % 10));
        UART_SendByte(UART_PORT_0, ' ');
        
        /* Level and Status */
        level = (uint8_t)passengerSeatState.currentLevel;
        error = passengerSeatState.sensorError;
        UART_SendString(UART_PORT_0, (uint8 *)level_str[level]);
        UART_SendByte(UART_PORT_0, ' ');
        UART_SendString(UART_PORT_0, (uint8 *)status_str[error]);
        UART_SendString(UART_PORT_0, (uint8 *)NL);
        UART_SendString(UART_PORT_0, (uint8 *)NL);
        
        xSemaphoreGive(xDisplayMutex);
    }
}

/* Task for diagnostics */
static void prvDiagnosticTask(void *pvParameters)
{
    EventBits_t eventBits;
    DiagnosticLogEntryType logEntry;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); /* 100ms period */
    
    for (;;) {
        /* Wait for button events with timeout */
        eventBits = xEventGroupWaitBits(xSeatHeaterEventGroup,
                                      DRIVER_BUTTON_PRESSED_BIT | PASSENGER_BUTTON_PRESSED_BIT | STEERING_BUTTON_PRESSED_BIT,
                                      pdTRUE,  /* Clear bits on exit */
                                      pdFALSE, /* Don't wait for all bits */
                                      xFrequency);
        
        if (eventBits & DRIVER_BUTTON_PRESSED_BIT) {
            /* Debounce delay */
            vTaskDelay(pdMS_TO_TICKS(50));
            SeatHeater_HandleButtonPress(DRIVER_SEAT);
        }
        if (eventBits & PASSENGER_BUTTON_PRESSED_BIT) {
            /* Debounce delay */
            vTaskDelay(pdMS_TO_TICKS(50));
            SeatHeater_HandleButtonPress(PASSENGER_SEAT);
        }
        if (eventBits & STEERING_BUTTON_PRESSED_BIT) {
            /* Debounce delay */
            vTaskDelay(pdMS_TO_TICKS(50));
            SeatHeater_HandleButtonPress(DRIVER_SEAT);
        }
        
        /* Process diagnostic queue */
        if (xQueueReceive(xDiagnosticQueue, &logEntry, 0) == pdPASS) {
            if (xSemaphoreTake(xDiagnosticMutex, portMAX_DELAY) == pdTRUE) {
                diagnosticLog[diagnosticLogIndex] = logEntry;
                diagnosticLogIndex = (diagnosticLogIndex + 1) % 25;  /* Match array size */
                xSemaphoreGive(xDiagnosticMutex);
            }
        }
        
        /* Update last wake time */
        xLastWakeTime = xTaskGetTickCount();
    }
}

/* Task for statistics */
static void prvStatisticsTask(void *pvParameters)
{
    SeatStateType seatState;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000); /* 1s period */
    
    for (;;) {
        /* Wait for the next cycle */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        /* Process driver seat statistics */
        if (xQueueReceive(xDriverSeatQueue, &seatState, 0) == pdPASS) {
            if (xSemaphoreTake(xStatisticsMutex, portMAX_DELAY) == pdTRUE) {
                /* Update statistics */
                SeatHeater_UpdateStatistics(DRIVER_SEAT);
                xSemaphoreGive(xStatisticsMutex);
            }
        }
        
        /* Process passenger seat statistics */
        if (xQueueReceive(xPassengerSeatQueue, &seatState, 0) == pdPASS) {
            if (xSemaphoreTake(xStatisticsMutex, portMAX_DELAY) == pdTRUE) {
                /* Update statistics */
                SeatHeater_UpdateStatistics(PASSENGER_SEAT);
                xSemaphoreGive(xStatisticsMutex);
            }
        }
    }
}

/* Task for power management */
static void prvPowerManagementTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000); /* 1s period */
    
    for (;;) {
        /* Wait for the next cycle */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        /* Check if both seats are off for more than 5 minutes */
        if ((driverSeatState.currentLevel == HEAT_OFF) && 
            (passengerSeatState.currentLevel == HEAT_OFF)) {
            /* Implement power saving measures */
        }
    }
}

/* Task for safety monitoring */
static void prvSafetyTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); /* 50ms period for smoother LED updates */
    const uint16_t OVER_TEMP_THRESHOLD = (uint16_t)(40.0f * 10.0f); /* 40.0°C in fixed-point */
    
    for (;;) {
        /* Wait for the next cycle */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        /* Update error LED pattern */
        prvUpdateErrorLED();
        
        /* Check for over-temperature conditions */
        if (driverSeatState.currentTemperature > OVER_TEMP_THRESHOLD) {
            if (driverSeatState.currentLevel != HEAT_OFF) {
                driverSeatState.currentLevel = HEAT_OFF;
                driverSeatState.heaterIntensity = HEATER_OFF;
                SeatHeater_LogDiagnostic(DRIVER_SEAT, ERROR_OVER_TEMPERATURE);
                prvControlHeaterLEDs(DRIVER_SEAT, HEATER_OFF);
            }
        }
        
        if (passengerSeatState.currentTemperature > OVER_TEMP_THRESHOLD) {
            if (passengerSeatState.currentLevel != HEAT_OFF) {
                passengerSeatState.currentLevel = HEAT_OFF;
                passengerSeatState.heaterIntensity = HEATER_OFF;
                SeatHeater_LogDiagnostic(PASSENGER_SEAT, ERROR_OVER_TEMPERATURE);
                prvControlHeaterLEDs(PASSENGER_SEAT, HEATER_OFF);
            }
        }
        
        /* Check for sensor failures */
        if (driverSeatState.sensorError) {
            if (driverSeatState.currentLevel != HEAT_OFF) {
                driverSeatState.currentLevel = HEAT_OFF;
                driverSeatState.heaterIntensity = HEATER_OFF;
                SeatHeater_LogDiagnostic(DRIVER_SEAT, ERROR_SENSOR_FAILURE);
                prvControlHeaterLEDs(DRIVER_SEAT, HEATER_OFF);
            }
        }
        
        if (passengerSeatState.sensorError) {
            if (passengerSeatState.currentLevel != HEAT_OFF) {
                passengerSeatState.currentLevel = HEAT_OFF;
                passengerSeatState.heaterIntensity = HEATER_OFF;
                SeatHeater_LogDiagnostic(PASSENGER_SEAT, ERROR_SENSOR_FAILURE);
                prvControlHeaterLEDs(PASSENGER_SEAT, HEATER_OFF);
            }
        }
    }
}

/* Timer0A interrupt handler */
void Timer0A_Handler(void)
{
    /* Use volatile to prevent optimization */
    volatile BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    volatile uint32_t status;
    
    /* Immediately clear the interrupt to prevent re-entry */
    status = TIMER0_MIS_REG;  /* Read masked interrupt status */
    TIMER0_ICR_REG = 0x1;     /* Clear the timeout interrupt flag */
    
    /* Only process if this is a timeout interrupt */
    if (status & 0x1) {
        /* Notify tasks if needed */
        if (xHigherPriorityTaskWoken == pdTRUE) {
            /* Yield from ISR only if we actually need to */
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

/* Static task stacks */
static StackType_t uxDriverSeatTaskStack[TASK_STACK_SIZE_SEAT_CONTROL];
static StaticTask_t xDriverSeatTaskBuffer;
static StackType_t uxPassengerSeatTaskStack[TASK_STACK_SIZE_SEAT_CONTROL];
static StaticTask_t xPassengerSeatTaskBuffer;
static StackType_t uxDisplayTaskStack[TASK_STACK_SIZE_DISPLAY];
static StaticTask_t xDisplayTaskBuffer;
static StackType_t uxDiagnosticTaskStack[TASK_STACK_SIZE_DIAGNOSTIC];
static StaticTask_t xDiagnosticTaskBuffer;
static StackType_t uxStatisticsTaskStack[TASK_STACK_SIZE_STATISTICS];
static StaticTask_t xStatisticsTaskBuffer;
static StackType_t uxSafetyTaskStack[TASK_STACK_SIZE_SAFETY];
static StaticTask_t xSafetyTaskBuffer;
static StackType_t uxPowerManagementTaskStack[TASK_STACK_SIZE_POWER_MGMT];
static StaticTask_t xPowerManagementTaskBuffer;

/* Public Functions */
void SeatHeater_Init(void)
{
    /* Initialize hardware */
    prvInitGPIO();
    prvInitADC();
    prvInitUART();
    prvInitGPTM();
    
    /* Create event group */
    xSeatHeaterEventGroup = xEventGroupCreate();
    
    /* Create queues with smaller sizes */
    xDriverSeatQueue = xQueueCreateStatic(2, sizeof(SeatStateType), ucDriverSeatQueueStorage, &xDriverSeatQueueBuffer);
    xPassengerSeatQueue = xQueueCreateStatic(2, sizeof(SeatStateType), ucPassengerSeatQueueStorage, &xPassengerSeatQueueBuffer);
    xDisplayQueue = xQueueCreateStatic(2, sizeof(SeatStateMessageType), ucDisplayQueueStorage, &xDisplayQueueBuffer);
    xDiagnosticQueue = xQueueCreateStatic(2, sizeof(DiagnosticLogEntryType), ucDiagnosticQueueStorage, &xDiagnosticQueueBuffer);
    xStatisticsQueue = xQueueCreateStatic(2, sizeof(SeatStateType), ucStatisticsQueueStorage, &xStatisticsQueueBuffer);
    
    /* Create mutexes */
    xDisplayMutex = xSemaphoreCreateMutex();
    xDiagnosticMutex = xSemaphoreCreateMutex();
    xStatisticsMutex = xSemaphoreCreateMutex();
    
    /* Initialize display data */
    memset(&displayData, 0, sizeof(displayData));
    
    /* Create tasks */
    xDriverSeatTaskHandle = xTaskCreateStatic(prvDriverSeatTask, "DriverSeat", TASK_STACK_SIZE_SEAT_CONTROL, NULL, 
                TASK_PRIORITY_SEAT_CONTROL, uxDriverSeatTaskStack, &xDriverSeatTaskBuffer);
                
    xPassengerSeatTaskHandle = xTaskCreateStatic(prvPassengerSeatTask, "PassengerSeat", TASK_STACK_SIZE_SEAT_CONTROL, NULL, 
                TASK_PRIORITY_SEAT_CONTROL, uxPassengerSeatTaskStack, &xPassengerSeatTaskBuffer);
                
    xDisplayTaskHandle = xTaskCreateStatic(prvDisplayRefreshTask, "Display", TASK_STACK_SIZE_DISPLAY, NULL, 
                TASK_PRIORITY_DISPLAY, uxDisplayTaskStack, &xDisplayTaskBuffer);
                
    xDiagnosticTaskHandle = xTaskCreateStatic(prvDiagnosticTask, "Diagnostic", TASK_STACK_SIZE_DIAGNOSTIC, NULL, 
                TASK_PRIORITY_DIAGNOSTIC, uxDiagnosticTaskStack, &xDiagnosticTaskBuffer);
                
    xStatisticsTaskHandle = xTaskCreateStatic(prvStatisticsTask, "Statistics", TASK_STACK_SIZE_STATISTICS, NULL, 
                TASK_PRIORITY_STATISTICS, uxStatisticsTaskStack, &xStatisticsTaskBuffer);
                
    xSafetyTaskHandle = xTaskCreateStatic(prvSafetyTask, "Safety", TASK_STACK_SIZE_SAFETY, NULL, 
                TASK_PRIORITY_SAFETY, uxSafetyTaskStack, &xSafetyTaskBuffer);
                
    xPowerManagementTaskHandle = xTaskCreateStatic(prvPowerManagementTask, "Power", TASK_STACK_SIZE_POWER_MGMT, NULL, 
                TASK_PRIORITY_POWER_MGMT, uxPowerManagementTaskStack, &xPowerManagementTaskBuffer);
}

void SeatHeater_UpdateTemperature(SeatType seat, uint16_t temperature)
{
    if (prvIsTemperatureValid(temperature))
    {
        if (seat == DRIVER_SEAT)
        {
            driverSeatState.currentTemperature = temperature;
        }
        else
        {
            passengerSeatState.currentTemperature = temperature;
        }
    }
    else
    {
        /* Set error flag for invalid temperature */
        if (seat == DRIVER_SEAT)
        {
            driverSeatState.sensorError = 1;
        }
        else
        {
            passengerSeatState.sensorError = 1;
        }
    }
}

void SeatHeater_SetLevel(SeatType seat, HeatingLevelType level)
{
    SeatStateType *seatState = (seat == DRIVER_SEAT) ? &driverSeatState : &passengerSeatState;
    seatState->currentLevel = level;
    seatState->lastLevelChangeTime = GPTM_ReadTimer(GPTM_TIMER0);
}

void SeatHeater_LogDiagnostic(SeatType seat, uint8 errorCode)
{
    DiagnosticLogEntryType logEntry;
    SeatStateType *seatState = (seat == DRIVER_SEAT) ? &driverSeatState : &passengerSeatState;
    
    logEntry.timestamp = GPTM_ReadTimer(GPTM_TIMER0);
    logEntry.seat = seat;
    logEntry.errorCode = errorCode;
    logEntry.temperature = seatState->currentTemperature;
    
    if (xQueueSend(xDiagnosticQueue, &logEntry, 0) != pdPASS) {
        /* Handle queue full error */
    }
}

void SeatHeater_UpdateStatistics(SeatType seat)
{
    SeatStateType *seatState = (seat == DRIVER_SEAT) ? &driverSeatState : &passengerSeatState;
    
    /* Update last diagnostic time */
    seatState->lastDiagnosticTime = GPTM_ReadTimer(GPTM_TIMER0);
    
    /* Send to statistics queue */
    if (xQueueSend(xStatisticsQueue, seatState, 0) != pdPASS) {
        /* Handle queue full error */
    }
}

void SeatHeater_HandleButtonPress(SeatType seat)
{
    SeatStateType *seatState = (seat == DRIVER_SEAT) ? &driverSeatState : &passengerSeatState;
    
    /* Cycle through heating levels */
    switch (seatState->currentLevel) {
        case HEAT_OFF:
            seatState->currentLevel = HEAT_LOW;
            break;
        case HEAT_LOW:
            seatState->currentLevel = HEAT_MEDIUM;
            break;
        case HEAT_MEDIUM:
            seatState->currentLevel = HEAT_HIGH;
            break;
        case HEAT_HIGH:
            seatState->currentLevel = HEAT_OFF;
            break;
    }
    
    seatState->lastLevelChangeTime = GPTM_ReadTimer(GPTM_TIMER0);
} 

