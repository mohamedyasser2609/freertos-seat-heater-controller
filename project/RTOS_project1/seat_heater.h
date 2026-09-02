#ifndef SEAT_HEATER_H
#define SEAT_HEATER_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include <stdint.h>
#include <stdbool.h>
#include "Common/std_types.h"
#include "Common/app_config.h"

/* Type Definitions */
// Remove duplicate type definitions since they're in std_types.h

/* Seat Types */
typedef enum {
    DRIVER_SEAT,
    PASSENGER_SEAT
} SeatType;

/* Heating Levels */
typedef enum {
    HEAT_OFF,
    HEAT_LOW,    /* 25°C */
    HEAT_MEDIUM, /* 30°C */
    HEAT_HIGH    /* 35°C */
} HeatingLevelType;

/* Heater Intensity */
typedef enum {
    HEATER_OFF,
    HEATER_LOW,    /* Green LED */
    HEATER_MEDIUM, /* Blue LED */
    HEATER_HIGH    /* Cyan LED */
} HeaterIntensityType;

/* Diagnostic Log Entry */
typedef struct {
    uint32 timestamp;
    uint8_t seat : 1;      /* Using bit field for seat type (0 or 1) */
    uint8_t errorCode : 4; /* Using bit field for error code (4 bits) */
    uint16_t temperature;  /* Using fixed-point (temperature * 10) instead of float */
} DiagnosticLogEntryType;

/* Seat State Structure */
typedef struct {
    uint8_t seat : 1;           /* Using bit field for seat type */
    uint8_t currentLevel : 2;   /* Using bit field for heating level (4 states) */
    uint16_t currentTemperature; /* Using fixed-point (temperature * 10) */
    uint8_t heaterIntensity : 2; /* Using bit field for heater intensity (4 states) */
    uint8_t sensorError : 1;    /* Using bit field for error flag */
    uint32_t lastLevelChangeTime;
    uint32_t lastErrorTime;
    uint32_t totalHeatingTime;
    uint32_t lastDiagnosticTime;
} SeatStateType;

/* Message type for queue communication */
typedef struct {
    uint8_t seat : 1;           /* Using bit field for seat type */
    uint8_t currentLevel : 2;   /* Using bit field for heating level */
    uint16_t currentTemperature; /* Fixed-point temperature */
    uint8_t heaterIntensity : 2; /* Using bit field for heater intensity */
    uint8_t sensorError : 1;    /* Using bit field for error flag */
    uint32_t totalHeatingTime;
} SeatStateMessageType;

/* Function Prototypes */
void SeatHeater_Init(void);
void SeatHeater_UpdateTemperature(SeatType seat, uint16_t temperature);
void SeatHeater_SetLevel(SeatType seat, HeatingLevelType level);
void SeatHeater_DisplayState(SeatType seat);
void SeatHeater_HandleButtonPress(SeatType seat);
void SeatHeater_LogDiagnostic(SeatType seat, uint8 errorCode);
void SeatHeater_UpdateStatistics(SeatType seat);

/* Task Handles */
extern TaskHandle_t xDriverSeatTaskHandle;
extern TaskHandle_t xPassengerSeatTaskHandle;
extern TaskHandle_t xDisplayTaskHandle;
extern TaskHandle_t xDiagnosticTaskHandle;
extern TaskHandle_t xStatisticsTaskHandle;
extern TaskHandle_t xSafetyTaskHandle;
extern TaskHandle_t xPowerManagementTaskHandle;

/* Event Group for Button Presses */
extern EventGroupHandle_t xSeatHeaterEventGroup;

/* Queues for Communication - Reduced queue sizes */
extern QueueHandle_t xDriverSeatQueue;     /* Size: 4 */
extern QueueHandle_t xPassengerSeatQueue;  /* Size: 4 */
extern QueueHandle_t xDisplayQueue;        /* Size: 2 */
extern QueueHandle_t xDiagnosticQueue;     /* Size: 2 */
extern QueueHandle_t xStatisticsQueue;     /* Size: 2 */

/* Mutex for Shared Resources - Using binary semaphores instead of mutexes where possible */
extern SemaphoreHandle_t xDisplayMutex;
extern SemaphoreHandle_t xDiagnosticMutex;
extern SemaphoreHandle_t xStatisticsMutex;

/* Diagnostic Log - Reduced size and using more efficient storage */
extern DiagnosticLogEntryType diagnosticLog[25];  /* Further reduced from 50 to 25 entries */
extern uint16_t diagnosticLogIndex;  /* Using uint16_t instead of uint32_t */

/* Seat States */
extern SeatStateType driverSeatState;
extern SeatStateType passengerSeatState;

/* Queue Handles */
extern QueueHandle_t xDriverSeatQueue;
extern QueueHandle_t xPassengerSeatQueue;
extern QueueHandle_t xDisplayQueue;
extern QueueHandle_t xDiagnosticQueue;
extern QueueHandle_t xStatisticsQueue;

#endif /* SEAT_HEATER_H */ 

