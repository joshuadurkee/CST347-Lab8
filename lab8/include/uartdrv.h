/**************************************************************************
 File:          uartdrv.h
 Author:        Jeremy Greenwood
 Course:        CST 347
 Assignment:    Lab 6
 Instructor:    Jay Bockelman
 Description:   UART driver header file.
**************************************************************************/

#ifndef UARTDRV_H
#define	UARTDRV_H


#include <plib.h>
#include <stdint.h>
#include <plib.h>
#include <FreeRTOS.h>
#include "task.h"

#include <myTask.h>


// prototypes
void initUart( UART_MODULE umPortNum, uint32_t ui32WantedBaud );
void enableUartISR( UART_MODULE umPortNum );
void disableUartISR( UART_MODULE umPortNum );
void UARTPutStr( char *tx_data );
BYTE UARTGetChar( void );


#endif	/* UARTDRV_H */

