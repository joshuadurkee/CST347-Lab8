/**************************************************************************
 File:          myCommands.c
 Authors:       Jeremy Greenwood
                Joshua Durkee
 Course:        CST 347
 Assignment:    end-of-term mini-project
 Instructor:    Jay Bockelman
 Description:   Commands source file, perfom desired mini-project functionality
                on a PIC32 Starter Kit including intertask communication.
**************************************************************************/


#include <myCommands.h>

#include "myTask.h"


extern elevator_movement_t elevator;


/*----------------------------------------------------------------------
    task-stats command
----------------------------------------------------------------------*/
const char taskListHdr[] = "Name\t\tStat\tPri\tS/Space\tTCB";

const xCommandLineInput xTaskStatsCommand =
{
    "task-stats",
    "task-stats: Displays a table of task state information\r\n",
    prvTaskStatsCommand,
    0
};

portBASE_TYPE prvTaskStatsCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    sprintf( pcWriteBuffer, taskListHdr );
    vTaskList( pcWriteBuffer + strlen( taskListHdr ) );

    return pdFALSE;
}

/*----------------------------------------------------------------------
    S command (change max speed)
----------------------------------------------------------------------*/
const xCommandLineInput xChangeMaximumSpeedCommand =
{
    "S",
    "Change Maximum Speed: changes the maximum speed of the elevator\r\n",
    prvChangeMaximumSpeedCommand,
    1
};

portBASE_TYPE prvChangeMaximumSpeedCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    portBASE_TYPE xParameter1StringLength;

    /* Obtain the LED number , and the length of its name, from the command string. */
    const int8_t *pcParameter1 = FreeRTOS_CLIGetParameter (
                          /* The command string itself. */
                          pcCommandString,
                          /* Return the first parameter. */
                          1,
                          /* Store the parameter string length. */
                          &xParameter1StringLength );

    elevator.new_max_speed = atoi( pcParameter1 );
    
    sprintf( pcWriteBuffer, "Maximum speed changed to %d feet/sec.\r\n", elevator.new_max_speed );

    return pdFALSE;
}

/*----------------------------------------------------------------------
    AP command (change acceleration)
----------------------------------------------------------------------*/
const xCommandLineInput xChangeAccelerationCommand =
{
    "AP",
    "Change Acceleration: changes the acceleration of the elevator\r\n",
    prvChangeAccelerationCommand,
    1
};

portBASE_TYPE prvChangeAccelerationCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    portBASE_TYPE xParameter1StringLength;

    /* Obtain the LED number , and the length of its name, from the command string. */
    const int8_t *pcParameter1 = FreeRTOS_CLIGetParameter (
                          /* The command string itself. */
                          pcCommandString,
                          /* Return the first parameter. */
                          1,
                          /* Store the parameter string length. */
                          &xParameter1StringLength );

    elevator.new_accel = atoi( pcParameter1 );

    sprintf( pcWriteBuffer, "Acceleration changed to %d feet/sec/sec.\r\n", elevator.new_accel );

    return pdFALSE;
}

/*----------------------------------------------------------------------
    ES command (emergency stop)
----------------------------------------------------------------------*/
const xCommandLineInput xEmergencyStopCommand =
{
    "ES",
    "Emergency Stop: sets the emergency stop flag, moving the elevator to the ground floor and holding the doors open till the flag is cleared\r\n",
    prvEmergencyStopCommand,
    0
};

portBASE_TYPE prvEmergencyStopCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    set_estop();
    sprintf( pcWriteBuffer, "Emergency stop triggered!\r\n");

    return pdFALSE;
}

/*----------------------------------------------------------------------
    ER command (emergency clear)
----------------------------------------------------------------------*/
const xCommandLineInput xEmergencyClearCommand =
{
    "ER",
    "Emergancy Clear: clears the emergency stop flag, allowing the elevator to resume normal operations\r\n",
    prvEmergencyClearCommand,
    0
};

portBASE_TYPE prvEmergencyClearCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    clear_estop();
    sprintf( pcWriteBuffer, "Emergency clear triggered!\r\n");

    return pdFALSE;
}

/*----------------------------------------------------------------------
    run-time-stats command
----------------------------------------------------------------------*/
const char runTimeListHdr[] = "Name\t\tAbs Time\t% Time\r\n";

const xCommandLineInput xRunTimeStatsCommand =
{
    "run-time-stats",
    "run-time-stats: Displays a table of run-time information\r\n",
    prvRunTimeStatsCommand,
    0
};

portBASE_TYPE prvRunTimeStatsCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    sprintf( pcWriteBuffer, runTimeListHdr );
    //vTaskGetRunTimeStats( pcWriteBuffer + strlen( runTimeListHdr ) );

    return pdFALSE;
}


// registers all CLI commands
void register_commands( void )
{
    if( FreeRTOS_CLIRegisterCommand( &xTaskStatsCommand ) == pdFAIL
     || FreeRTOS_CLIRegisterCommand( &xChangeMaximumSpeedCommand ) == pdFAIL
     || FreeRTOS_CLIRegisterCommand( &xChangeAccelerationCommand ) == pdFAIL
     || FreeRTOS_CLIRegisterCommand( &xEmergencyStopCommand ) == pdFAIL
     || FreeRTOS_CLIRegisterCommand( &xEmergencyClearCommand ) == pdFAIL
     || FreeRTOS_CLIRegisterCommand( &xRunTimeStatsCommand ) == pdFAIL )
    {
        // error registering commands
        while( 1 );
    }
}