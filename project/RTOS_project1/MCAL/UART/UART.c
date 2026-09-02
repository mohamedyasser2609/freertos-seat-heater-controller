/*
 * UART.c
 *
 *  Created on: Mar 22, 2025
 *      Author: Lenovo
 */

#include "UART.h"

/* UART Register Offsets */
#define UART_DR_OFFSET           0x00
#define UART_RSR_OFFSET          0x04
#define UART_ECR_OFFSET          0x04
#define UART_FR_OFFSET           0x18
#define UART_ILPR_OFFSET         0x20
#define UART_IBRD_OFFSET         0x24
#define UART_FBRD_OFFSET         0x28
#define UART_LCRH_OFFSET         0x2C
#define UART_CTL_OFFSET          0x30
#define UART_IFLS_OFFSET         0x34
#define UART_IM_OFFSET           0x38
#define UART_RIS_OFFSET          0x3C
#define UART_MIS_OFFSET          0x40
#define UART_ICR_OFFSET          0x44
#define UART_DMACTL_OFFSET       0x48

/* Helper Function: Setup GPIO pins for UART0 */
static void UART_SetupGPIOPins(void)
{
    /* Enable clock for GPIO PORTA */
    SYSCTL_RCGCGPIO_REG |= 0x01;
    while(!(SYSCTL_PRGPIO_REG & 0x01));
    
    /* Disable Analog on PA0 & PA1 */
    GPIO_PORTA_AMSEL_REG &= 0xFC;
    
    /* Configure PA0 as input pin and PA1 as output pin */
    GPIO_PORTA_DIR_REG &= 0xFE;
    GPIO_PORTA_DIR_REG |= 0x02;
    
    /* Enable alternative function on PA0 & PA1 */
    GPIO_PORTA_AFSEL_REG |= 0x03;
    
    /* Set PMCx bits for PA0 & PA1 with value 1 to use PA0 as UART0 RX pin and PA1 as UART0 Tx pin */
    GPIO_PORTA_PCTL_REG = (GPIO_PORTA_PCTL_REG & 0xFFFFFF00) | 0x00000011;
    
    /* Enable Digital I/O on PA0 & PA1 */
    GPIO_PORTA_DEN_REG |= 0x03;
}

/* Helper Function: Calculate UART Baud Rate Divisors */
static void UART_CalculateBaudRateDivisors(uint32 baudRate, uint32 *integerDivisor, uint32 *fractionalDivisor)
{
    uint32 systemClock = 16000000; /* System clock is 16 MHz */
    uint32 divisor = (systemClock * 64) / (baudRate * 16);
    
    *integerDivisor = divisor / 64;
    *fractionalDivisor = divisor % 64;
}

/* Initialize UART0 */
void UART_Init(const UART_ConfigType *config)
{
    if (!config) return;
    
    /* Setup GPIO pins for UART0 */
    UART_SetupGPIOPins();
    
    /* Enable clock for UART0 */
    SYSCTL_RCGCUART_REG |= 0x01;
    while(!(SYSCTL_PRUART_REG & 0x01));
    
    /* Disable UART0 at the beginning */
    UART0_CTL_REG = 0;
    UART0_CC_REG = 0; /* Use System Clock */
    
    /* Calculate baud rate divisors */
    uint32 integerDivisor, fractionalDivisor;
    UART_CalculateBaudRateDivisors(config->baudRate, &integerDivisor, &fractionalDivisor);
    
    /* Configure baud rate */
    UART0_IBRD_REG = integerDivisor;
    UART0_FBRD_REG = fractionalDivisor;
    
    /* Configure UART Line Control Register */
    uint32 lcrhValue = 0;
    
    /* Set data length */
    lcrhValue |= (config->dataLength << 5);
    
    /* Set stop bits */
    if (config->stopBits == UART_STOP_2BITS) {
        lcrhValue |= (1 << 3);
    }
    
    /* Set parity */
    if (config->parity != UART_PARITY_NONE) {
        lcrhValue |= (1 << 1); /* Enable parity */
        if (config->parity == UART_PARITY_EVEN) {
            lcrhValue |= (1 << 2); /* Even parity */
        }
    }
    
    /* Set FIFO enable */
    if (config->fifoEnable == UART_FIFO_ENABLE) {
        lcrhValue |= (1 << 4);
    }
    
    /* Apply Line Control Register settings */
    UART0_LCRH_REG = lcrhValue;
    
    /* Enable UART0 */
    UART0_CTL_REG = UART_CTL_UARTEN_MASK | UART_CTL_TXE_MASK | UART_CTL_RXE_MASK;
}

/* Send a byte over UART0 */
void UART_SendByte(UART_PortType port, uint8 data)
{
    /* Wait until the transmit FIFO is empty */
    while(!(UART0_FR_REG & UART_FR_TXFE_MASK));
    
    /* Send the byte */
    UART0_DR_REG = data;
}

/* Receive a byte from UART0 */
uint8 UART_ReceiveByte(UART_PortType port)
{
    uint8 data = 0;
    
    /* Wait until the receive FIFO is not empty */
    while(UART0_FR_REG & UART_FR_RXFE_MASK);
    
    /* Read the byte */
    data = UART0_DR_REG;
    
    return data;
}

/* Send a string over UART0 */
void UART_SendString(UART_PortType port, const uint8 *pData)
{
    uint32 uCounter = 0;
    
    /* Transmit the whole string */
    while(pData[uCounter] != '\0')
    {
        UART_SendByte(port, pData[uCounter]); /* Send the byte */
        uCounter++; /* increment the counter to the next byte */
    }
}

/* Send an integer over UART0 */
void UART_SendInteger(UART_PortType port, sint64 sNumber)
{
    uint8 uDigits[20];
    sint8 uCounter = 0;

    /* Send the negative sign in case of negative numbers */
    if (sNumber < 0)
    {
        UART_SendByte(port, '-');
        sNumber *= -1;
    }

    /* Convert the number to an array of characters */
    do
    {
        uDigits[uCounter++] = sNumber % 10 + '0'; /* Convert each digit to its corresponding ASCI character */
        sNumber /= 10; /* Remove the already converted digit */
    }
    while (sNumber != 0);

    /* Send the array of characters in a reverse order as the digits were converted from right to left */
    for( uCounter--; uCounter>= 0; uCounter--)
    {
        UART_SendByte(port, uDigits[uCounter]);
    }
}

/* FIFO Control Functions */
void UART_EnableFIFO(UART_ChannelType channel)
{
    UART0_LCRH_REG |= 0x10; /* Set FEN bit */
}

void UART_DisableFIFO(UART_ChannelType channel)
{
    UART0_LCRH_REG &= ~0x10; /* Clear FEN bit */
}

void UART_SetFIFOThreshold(UART_ChannelType channel, uint8 rxThreshold, uint8 txThreshold)
{
    UART0_IFLS_REG = (txThreshold << 3) | rxThreshold;
}

uint8 UART_IsFIFOFull(UART_ChannelType channel)
{
    return (UART0_FR_REG & (1 << 5)) ? 1 : 0;
}

uint8 UART_IsFIFOEmpty(UART_ChannelType channel)
{
    return (UART0_FR_REG & (1 << 4)) ? 1 : 0;
}

/* Interrupt Control Functions */
void UART_EnableInterrupt(UART_ChannelType channel)
{
    UART0_IM_REG |= 0x10; /* Enable RX interrupt */
}

void UART_DisableInterrupt(UART_ChannelType channel)
{
    UART0_IM_REG &= ~0x10; /* Disable RX interrupt */
}

void UART_ClearInterrupt(UART_ChannelType channel)
{
    UART0_ICR_REG = 0x10; /* Clear RX interrupt */
}

uint8 UART_GetInterruptStatus(UART_ChannelType channel)
{
    return (UART0_MIS_REG & 0x10) ? 1 : 0;
}

/* Error Handling Functions */
uint8 UART_GetErrorStatus(UART_ChannelType channel)
{
    return (uint8)(UART0_RSR_REG & 0x0F);
}

void UART_ClearError(UART_ChannelType channel)
{
    UART0_ECR_REG = 0xFF; /* Clear all error bits */
}

/* Status Functions */
uint8 UART_IsBusy(UART_ChannelType channel)
{
    return (UART0_FR_REG & (1 << 3)) ? 1 : 0;
}

uint8 UART_IsTransmitEmpty(UART_ChannelType channel)
{
    return (UART0_FR_REG & (1 << 7)) ? 1 : 0;
}

uint8 UART_IsReceiveFull(UART_ChannelType channel)
{
    return (UART0_FR_REG & (1 << 6)) ? 1 : 0;
}
