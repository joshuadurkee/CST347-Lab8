/**************************************************************************
 File:          myCommands.h
 Authors:       Jeremy Greenwood
                Joshua Durkee
 Course:        CST 347
 Assignment:    end-of-term mini-project
 Instructor:    Jay Bockelman
 Description:   Command header file.
**************************************************************************/


#ifndef MYCOMMAND_H
#define	MYCOMMAND_H


#include <xc.h>

/* Standard includes. */
#include <stdint.h>
#include <stdbool.h>
#include <plib.h>

/* FreeRTOS includes. */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include <FreeRTOS_CLI.h>


// prototypes
portBASE_TYPE prvChangeMaximumSpeedCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE prvChangeAccelerationCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE prvTaskStatsCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE prvRunTimeStatsCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
void register_commands( void );


#endif	/* MYCOMMAND_H */