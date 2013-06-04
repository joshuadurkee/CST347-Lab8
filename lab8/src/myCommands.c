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


// registers all CLI commands
void register_commands( void )
{
    if( FreeRTOS_CLIRegisterCommand( &xTaskStatsCommand ) == pdFAIL
     || 1 )
    {
        // error registering commands
        while( 1 );
    }
}