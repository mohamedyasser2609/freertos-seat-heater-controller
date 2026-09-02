/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include <stdio.h>  /* For snprintf */

/* MCAL includes. */
#include "gpio.h"
#include "adc.h"
#include "uart.h"
#include "gptm.h"
#include "tm4c123gh6pm_registers.h"

/* Application includes */
#include "seat_heater.h"
#include "Common/app_config.h"

/* Stack overflow and malloc failed hooks */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    /* Log the stack overflow */
    UART_SendString(UART_PORT_0, (uint8 *)"STACK OVERFLOW in task: ");
    UART_SendString(UART_PORT_0, (uint8 *)pcTaskName);
    UART_SendString(UART_PORT_0, (uint8 *)"\r\n");
    
    /* Disable interrupts and enter infinite loop for debugging */
    __asm(" CPSID I");
    for(;;);
}

void vApplicationMallocFailedHook(void)
{
    /* Log the malloc failure */
    UART_SendString(UART_PORT_0, (uint8 *)"Malloc Failed!\r\n");
    
    /* Disable interrupts and enter infinite loop for debugging */
    __asm(" CPSID I");
    for(;;);
}

/* GPIO Pin Definitions */
#define DRIVER_HEATER_PIN      3  /* PF3 - Driver seat heater control */
#define PASSENGER_HEATER_PIN   4  /* PF4 - Passenger seat heater control */
#define DRIVER_ERROR_LED_PIN   5  /* PF5 - Driver seat error LED */
#define PASSENGER_ERROR_LED_PIN 6  /* PF6 - Passenger seat error LED */

/* Static memory allocation for tasks */
static StaticTask_t xIdleTaskTCB;
static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

static StaticTask_t xTimerTaskTCB;
static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

static StaticTask_t xSafetyTaskTCB;
static StackType_t uxSafetyTaskStack[TASK_STACK_SIZE_SAFETY];

static StaticTask_t xPowerManagementTaskTCB;
static StackType_t uxPowerManagementTaskStack[TASK_STACK_SIZE_POWER_MGMT];

static StaticTask_t xDisplayTaskTCB;
static StackType_t uxDisplayTaskStack[TASK_STACK_SIZE_DISPLAY];

/* Static memory allocation for queues */
static StaticQueue_t xDriverSeatQueueBuffer;
static uint8_t ucDriverSeatQueueStorage[4 * sizeof(uint8_t)];

static StaticQueue_t xPassengerSeatQueueBuffer;
static uint8_t ucPassengerSeatQueueStorage[4 * sizeof(uint8_t)];

static StaticQueue_t xDisplayQueueBuffer;
static uint8_t ucDisplayQueueStorage[2 * sizeof(uint8_t)];

static StaticQueue_t xDiagnosticQueueBuffer;
static uint8_t ucDiagnosticQueueStorage[2 * sizeof(uint8_t)];

static StaticQueue_t xStatisticsQueueBuffer;
static uint8_t ucStatisticsQueueStorage[2 * sizeof(uint8_t)];

/* Static memory allocation for semaphores */
static StaticSemaphore_t xDisplayMutexBuffer;
static StaticSemaphore_t xDiagnosticMutexBuffer;
static StaticSemaphore_t xStatisticsMutexBuffer;

/* Task Handles - declared as extern since they are defined in seat_heater.c */
extern TaskHandle_t xDisplayTaskHandle;
extern TaskHandle_t xSafetyTaskHandle;
extern TaskHandle_t xPowerManagementTaskHandle;

/* Function to initialize hardware */
static void prvSetupHardware(void)
{
    /* Initialize system clock */

    
    /* Initialize basic hardware */
    GPIO_BuiltinButtonsLedsInit();
    
    /* Enable interrupts */
    __asm(" CPSIE I");
}

/* Display Refresh Task */
static void prvDisplayRefreshTask(void *pvParameters)
{
    /* Static strings to reduce stack usage */
    static const char * const OFF_STR = "OFF\r\n";
    static const char * const LOW_STR = "LOW\r\n";
    static const char * const MED_STR = "MED\r\n";
    static const char * const HIGH_STR = "HIGH\r\n";
    static const char * const OK_STR = "OK\r\n";
    static const char * const ERR_STR = "ERROR\r\n";
    
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000UL);
    static char displayBuffer[24];  /* Reduced buffer size */
    uint16_t temp;
    
    /* Initialize the xLastWakeTime variable with the current time */
    xLastWakeTime = xTaskGetTickCount();
    
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        /* Header */
        UART_SendString(UART_PORT_0, (uint8 *)"=== Seat Heater ===\r\n\r\n");
        
        /* Driver seat info */
        UART_SendString(UART_PORT_0, (uint8 *)"Driver:\r\nLevel: ");
        UART_SendString(UART_PORT_0, (uint8 *)(
            driverSeatState.currentLevel == HEAT_OFF ? OFF_STR :
            driverSeatState.currentLevel == HEAT_LOW ? LOW_STR :
            driverSeatState.currentLevel == HEAT_MEDIUM ? MED_STR : HIGH_STR));
        
        /* Temperature */
        temp = driverSeatState.currentTemperature;
        snprintf(displayBuffer, sizeof(displayBuffer), "Temp: %d.%d C\r\n", 
                temp / 10, temp % 10);
        UART_SendString(UART_PORT_0, (uint8 *)displayBuffer);
        
        /* Heater */
        UART_SendString(UART_PORT_0, (uint8 *)"Heat: ");
        UART_SendString(UART_PORT_0, (uint8 *)(
            driverSeatState.heaterIntensity == HEATER_OFF ? OFF_STR :
            driverSeatState.heaterIntensity == HEATER_LOW ? LOW_STR :
            driverSeatState.heaterIntensity == HEATER_MEDIUM ? MED_STR : HIGH_STR));
        
        /* Status */
        UART_SendString(UART_PORT_0, (uint8 *)"Status: ");
        UART_SendString(UART_PORT_0, (uint8 *)(
            driverSeatState.sensorError ? ERR_STR : OK_STR));
        UART_SendString(UART_PORT_0, (uint8 *)"\r\n");
        
        /* Passenger seat info */
        UART_SendString(UART_PORT_0, (uint8 *)"Passenger:\r\nLevel: ");
        UART_SendString(UART_PORT_0, (uint8 *)(
            passengerSeatState.currentLevel == HEAT_OFF ? OFF_STR :
            passengerSeatState.currentLevel == HEAT_LOW ? LOW_STR :
            passengerSeatState.currentLevel == HEAT_MEDIUM ? MED_STR : HIGH_STR));
        
        /* Temperature */
        temp = passengerSeatState.currentTemperature;
        snprintf(displayBuffer, sizeof(displayBuffer), "Temp: %d.%d C\r\n",
                temp / 10, temp % 10);
        UART_SendString(UART_PORT_0, (uint8 *)displayBuffer);
        
        /* Heater */
        UART_SendString(UART_PORT_0, (uint8 *)"Heat: ");
        UART_SendString(UART_PORT_0, (uint8 *)(
            passengerSeatState.heaterIntensity == HEATER_OFF ? OFF_STR :
            passengerSeatState.heaterIntensity == HEATER_LOW ? LOW_STR :
            passengerSeatState.heaterIntensity == HEATER_MEDIUM ? MED_STR : HIGH_STR));
        
        /* Status */
        UART_SendString(UART_PORT_0, (uint8 *)"Status: ");
        UART_SendString(UART_PORT_0, (uint8 *)(
            passengerSeatState.sensorError ? ERR_STR : OK_STR));
        UART_SendString(UART_PORT_0, (uint8 *)"\r\n\r\n");
        
        /* Stack usage - minimal format */
        snprintf(displayBuffer, sizeof(displayBuffer), "S:%lu P:%lu D:%lu\r\n",
                uxTaskGetStackHighWaterMark(xSafetyTaskHandle),
                uxTaskGetStackHighWaterMark(xPowerManagementTaskHandle),
                uxTaskGetStackHighWaterMark(NULL));
        UART_SendString(UART_PORT_0, (uint8 *)displayBuffer);
        
        /* Small delay to prevent UART flooding */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* Safety Task */
static void prvSafetyTask(void *pvParameters);

/* Power Management Task */
static void prvPowerManagementTask(void *pvParameters);

/* Power Management Functions */
static BaseType_t prvCheckPowerSavingConditions(void)
{
    /* Check if both seats are off for more than 5 minutes */
    return ((driverSeatState.currentLevel == HEAT_OFF) && 
            (passengerSeatState.currentLevel == HEAT_OFF));
}

/* Required by FreeRTOS when configSUPPORT_STATIC_ALLOCATION is 1 */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                 StackType_t **ppxIdleTaskStackBuffer,
                                 uint32_t *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/* Required by FreeRTOS when configSUPPORT_STATIC_ALLOCATION is 1 */
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                  StackType_t **ppxTimerTaskStackBuffer,
                                  uint32_t *pulTimerTaskStackSize)
{
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

int main(void)
{
    /* Initialize hardware */
    prvSetupHardware();
    
    /* Initialize seat heater system */
    SeatHeater_Init();

    /* Create queues using static allocation */
    xDriverSeatQueue = xQueueCreateStatic(4, sizeof(uint8_t), 
                                        ucDriverSeatQueueStorage, 
                                        &xDriverSeatQueueBuffer);
    configASSERT(xDriverSeatQueue != NULL);
    
    xPassengerSeatQueue = xQueueCreateStatic(4, sizeof(uint8_t), 
                                           ucPassengerSeatQueueStorage, 
                                           &xPassengerSeatQueueBuffer);
    configASSERT(xPassengerSeatQueue != NULL);
    
    xDisplayQueue = xQueueCreateStatic(2, sizeof(uint8_t), 
                                     ucDisplayQueueStorage, 
                                     &xDisplayQueueBuffer);
    configASSERT(xDisplayQueue != NULL);
    
    xDiagnosticQueue = xQueueCreateStatic(2, sizeof(uint8_t), 
                                        ucDiagnosticQueueStorage, 
                                        &xDiagnosticQueueBuffer);
    configASSERT(xDiagnosticQueue != NULL);
    
    xStatisticsQueue = xQueueCreateStatic(2, sizeof(uint8_t), 
                                        ucStatisticsQueueStorage, 
                                        &xStatisticsQueueBuffer);
    configASSERT(xStatisticsQueue != NULL);

    /* Create mutexes using static allocation */
    xDisplayMutex = xSemaphoreCreateMutexStatic(&xDisplayMutexBuffer);
    configASSERT(xDisplayMutex != NULL);
    
    xDiagnosticMutex = xSemaphoreCreateMutexStatic(&xDiagnosticMutexBuffer);
    configASSERT(xDiagnosticMutex != NULL);
    
    xStatisticsMutex = xSemaphoreCreateMutexStatic(&xStatisticsMutexBuffer);
    configASSERT(xStatisticsMutex != NULL);

    /* Create Safety Task - Highest Priority */
    xSafetyTaskHandle = xTaskCreateStatic(prvSafetyTask, 
                                        "Safety", 
                                        TASK_STACK_SIZE_SAFETY, 
                                        NULL, 
                                        TASK_PRIORITY_SAFETY, 
                                        uxSafetyTaskStack, 
                                        &xSafetyTaskTCB);
    configASSERT(xSafetyTaskHandle != NULL);

    /* Create Power Management Task */
    xPowerManagementTaskHandle = xTaskCreateStatic(prvPowerManagementTask, 
                                                 "PowerMgmt", 
                                                 TASK_STACK_SIZE_POWER_MGMT, 
                                                 NULL, 
                                                 TASK_PRIORITY_POWER_MGMT, 
                                                 uxPowerManagementTaskStack, 
                                                 &xPowerManagementTaskTCB);
    configASSERT(xPowerManagementTaskHandle != NULL);
    
    /* Create Display Refresh Task */
    xDisplayTaskHandle = xTaskCreateStatic(prvDisplayRefreshTask, 
                                         "DisplayRefresh", 
                                         TASK_STACK_SIZE_DISPLAY, 
                                         NULL, 
                                         TASK_PRIORITY_DISPLAY, 
                                         uxDisplayTaskStack, 
                                         &xDisplayTaskTCB);
    configASSERT(xDisplayTaskHandle != NULL);

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();
    
    /* If we get here, the scheduler failed to start */
    for(;;);
}

/* Safety Task Implementation */
static void prvSafetyTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(100UL); /* 100ms period */
    
    /* Initialize the xLastWakeTime variable with the current time */
    xLastWakeTime = xTaskGetTickCount();
    
    for (;;)
    {
        /* Wait for the next cycle */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        /* Check for critical safety conditions */
        if (driverSeatState.sensorError || passengerSeatState.sensorError)
        {
            /* Emergency shutdown of heaters */
            SeatHeater_SetLevel(DRIVER_SEAT, HEAT_OFF);
            SeatHeater_SetLevel(PASSENGER_SEAT, HEAT_OFF);
            
            /* Log the safety event */
            SeatHeater_LogDiagnostic(DRIVER_SEAT, 0x01);  /* Sensor error code */
            SeatHeater_LogDiagnostic(PASSENGER_SEAT, 0x01);
        }
        
        /* Check for temperature limits */
        if (driverSeatState.currentTemperature > 400 ||  /* 40.0°C in fixed-point */
            passengerSeatState.currentTemperature > 400)  /* 40.0°C in fixed-point */
        {
            /* Emergency shutdown of heaters */
            SeatHeater_SetLevel(DRIVER_SEAT, HEAT_OFF);
            SeatHeater_SetLevel(PASSENGER_SEAT, HEAT_OFF);
            
            /* Log the safety event */
            SeatHeater_LogDiagnostic(DRIVER_SEAT, 0x02);  /* Over-temperature error code */
            SeatHeater_LogDiagnostic(PASSENGER_SEAT, 0x02);
        }
    }
}

/* Power Management Task Implementation */
static void prvPowerManagementTask(void *pvParameters)
{
    const TickType_t xPowerSaveDelay = pdMS_TO_TICKS(5000UL);
    
    /* Unused parameter */
    (void)pvParameters;
    
    for (;;)
    {
        /* Wait for 5 seconds */
        vTaskDelay(xPowerSaveDelay);
        
        /* Check if we need to enter power saving mode */
        if (prvCheckPowerSavingConditions())
        {
            /* Turn off all LEDs */
            GPIO_WritePin(GPIO_PORT_F, (uint8_t)DRIVER_HEATER_PIN, 0);
            GPIO_WritePin(GPIO_PORT_F, (uint8_t)PASSENGER_HEATER_PIN, 0);
            GPIO_WritePin(GPIO_PORT_F, (uint8_t)DRIVER_ERROR_LED_PIN, 0);
            GPIO_WritePin(GPIO_PORT_F, (uint8_t)PASSENGER_ERROR_LED_PIN, 0);
        }
    }
}
