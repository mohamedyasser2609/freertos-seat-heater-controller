/**********************************************************************************************
 *
 * Module: UART
 *
 * File Name: UART.h
 *
 * Description: Header file for the TM4C123GH6PM UART driver
 *
 * Author: mohamed yasser
 *
 ***********************************************************************************************/
#ifndef UART_H
#define UART_H

#include "Common/std_types.h"
#include "tm4c123gh6pm_registers.h"

/* UART Port Definitions */
typedef enum {
    UART_PORT_0 = 0,
    UART_NUM_PORTS
} UART_PortType;

/* UART Channel Type (for backward compatibility) */
typedef UART_PortType UART_ChannelType;

/* UART Data Length */
typedef enum {
    UART_DATA_5BITS = 0,
    UART_DATA_6BITS,
    UART_DATA_7BITS,
    UART_DATA_8BITS
} UART_DataLengthType;

/* UART Stop Bits */
typedef enum {
    UART_STOP_1BIT = 0,
    UART_STOP_2BITS
} UART_StopBitsType;

/* UART Parity */
typedef enum {
    UART_PARITY_NONE = 0,
    UART_PARITY_ODD,
    UART_PARITY_EVEN
} UART_ParityType;

/* UART FIFO Enable */
typedef enum {
    UART_FIFO_DISABLE = 0,
    UART_FIFO_ENABLE
} UART_FIFOEnableType;

/* UART Configuration Structure */
typedef struct {
    UART_PortType port;
    uint32 baudRate;
    UART_DataLengthType dataLength;
    UART_StopBitsType stopBits;
    UART_ParityType parity;
    UART_FIFOEnableType fifoEnable;
} UART_ConfigType;

/* UART Control Masks */
#define UART_CTL_UARTEN_MASK    0x00000001  /* UART Enable */
#define UART_CTL_TXE_MASK       0x00000100  /* UART Transmit Enable */
#define UART_CTL_RXE_MASK       0x00000200  /* UART Receive Enable */

/* UART Flag Register Masks */
#define UART_FR_TXFE_MASK       0x00000080  /* UART Transmit FIFO Empty */
#define UART_FR_RXFE_MASK       0x00000010  /* UART Receive FIFO Empty */

/* Public Functions */
void UART_Init(const UART_ConfigType *config);
void UART_SendByte(UART_PortType port, uint8 data);
uint8 UART_ReceiveByte(UART_PortType port);
void UART_SendString(UART_PortType port, const uint8 *pData);
void UART_SendInteger(UART_PortType port, sint64 sNumber);

/* Additional Functions for FIFO Control */
void UART_EnableFIFO(UART_ChannelType channel);
void UART_DisableFIFO(UART_ChannelType channel);
void UART_SetFIFOThreshold(UART_ChannelType channel, uint8 rxThreshold, uint8 txThreshold);
uint8 UART_IsFIFOFull(UART_ChannelType channel);
uint8 UART_IsFIFOEmpty(UART_ChannelType channel);

/* Additional Functions for Interrupt Control */
void UART_EnableInterrupt(UART_ChannelType channel);
void UART_DisableInterrupt(UART_ChannelType channel);
void UART_ClearInterrupt(UART_ChannelType channel);
uint8 UART_GetInterruptStatus(UART_ChannelType channel);

/* Additional Functions for Error Handling */
uint8 UART_GetErrorStatus(UART_ChannelType channel);
void UART_ClearError(UART_ChannelType channel);

/* Additional Functions for Status */
uint8 UART_IsBusy(UART_ChannelType channel);
uint8 UART_IsTransmitEmpty(UART_ChannelType channel);
uint8 UART_IsReceiveFull(UART_ChannelType channel);

#endif /* UART_H */
