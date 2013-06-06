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
    extern int max_speed_fps;

    /* Obtain the LED number , and the length of its name, from the command string. */
    const int8_t *pcParameter1 = FreeRTOS_CLIGetParameter (
                          /* The command string itself. */
                          pcCommandString,
                          /* Return the first parameter. */
                          1,
                          /* Store the parameter string length. */
                          &xParameter1StringLength );

    max_speed_fps = atoi( pcParameter1 );
    
    sprintf( pcWriteBuffer, "\0" );

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
    extern int accel_fpss;

    /* Obtain the LED number , and the length of its name, from the command string. */
    const int8_t *pcParameter1 = FreeRTOS_CLIGetParameter (
                          /* The command string itself. */
                          pcCommandString,
                          /* Return the first parameter. */
                          1,
                          /* Store the parameter string length. */
                          &xParameter1StringLength );

    accel_fpss = atoi( pcParameter1 );

    sprintf( pcWriteBuffer, "\0" );

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
     || FreeRTOS_CLIRegisterCommand( &xRunTimeStatsCommand ) == pdFAIL
     || 0 )
    {
        // error registering commands
        while( 1 );
    }
}