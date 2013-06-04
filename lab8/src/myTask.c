/**************************************************************************
 File:          myTask.c
 Authors:       Jeremy Greenwood
                Joshua Durkee
 Course:        CST 347
 Assignment:    end-of-term mini-project
 Instructor:    Jay Bockelman
 Description:   Task source file, perfom desired mini-project functionality
                on a PIC32 Starter Kit including intertask communication.
**************************************************************************/


#include <myTask.h>


// variables
extern xTaskHandle irq_button_task_handle;
extern xTaskHandle poll_button_task_handle;
extern xTaskHandle led_task_handle[ NUM_LEDS ];
extern xTaskHandle tx_task_handle;
extern xTaskHandle rx_task_handle;

extern xQueueHandle led_queue_handle[ NUM_LEDS ];
extern xQueueHandle tx_queue_handle;

extern xSemaphoreHandle buttonPress;
extern xSemaphoreHandle ledNAction[ NUM_LEDS ];
extern xSemaphoreHandle inputByteBuffer;

task_parameter_t task_parameter;


void irqButtonControlTask( void *params )
{
    int     i;
    int     switch_port[ NUM_SWITCHES ] = { P2_BUTTON_BIT, P1_BUTTON_BIT, GD_BUTTON_BIT };
    int     button_pressed[ NUM_SWITCHES ] = { 0, 0, 0 };
    int     button_pressed_old[ NUM_SWITCHES ];

    while( 1 )
    {        
        // take the button semaphore
        xSemaphoreTake( buttonPress, portMAX_DELAY );

        // debounce buttons
        msDelay( 10 );

        for( i = 0; i < NUM_SWITCHES; i++ )
        {
            button_pressed_old[ i ] = button_pressed[ i ];
            button_pressed[ i ] = mPORTDReadBits( switch_port[ i ] );

            // check if button is released to give the semaphore (this causes the LED to toggle)
            if( !button_pressed[ i ] && button_pressed_old[ i ] )
                xSemaphoreGive( ledNAction[ i ] );
        }

        enable_change_notification_irq();
    }
}


// poll switches every 100ms and perform appropriate action
void pollButtonControlTask( void *params )
{
    int         button_pressed_old = -1;
    int         button_pressed = -1;

    while( 1 )
    {
        button_pressed_old = button_pressed;
        button_pressed = poll_buttons();

        // debounce buttons
        msDelay( 100 );

        // check if button is pressed and is not stale
        if( button_pressed >= 0 && button_pressed != button_pressed_old )
        {
            switch( button_pressed )
            {
                // open button pressed, toggle LED 2
                case OPEN_BUTTON_BIT:
                    mPORTDToggleBits( DOOR_LED_2_BIT );
                    break;

                // close button pressed, toggle LED 0
                case CLOSE_BUTTON_BIT:
                    mPORTDToggleBits( DOOR_LED_0_BIT );
                    break;
            }
        }
    }
}


void ledControlTask( void *params )
{
    int     task_led;

    task_led = *( (int *)params );

    while( 1 )
    {
        xSemaphoreTake( ledNAction[ task_led ], portMAX_DELAY );

        // toggle LED
        mPORTDToggleBits( 1 << task_led );
    }
}


void txControlTask( void *params )
{
    char tx_data[ MSG_SIZE ];

    while( 1 )
    {
        xQueueReceive( tx_queue_handle, tx_data, portMAX_DELAY );
        UARTPutStr( tx_data );
    }
}


void rxControlTask( void *params )
{
    char        rx_char;
    char        rx_echo[ MSG_SIZE ];
    char        command_string[ MSG_SIZE ];
    static int  cmd_str_idx = 0;
    static char pcOutputString[ configCOMMAND_INT_MAX_OUTPUT_SIZE ];
    char       *tx_ptr;
    char        tx_buf[ MSG_SIZE ];

    while( 1 )
    {
        xSemaphoreTake( inputByteBuffer, portMAX_DELAY );

        rx_char = UARTGetChar();

        // add the character to the command string, completing on carriage return
        if( rx_char == '\r' )
        {
            // send characters to Tx queue to echo back end of the line entered by user
            strcpy( rx_echo, "\r\n" );
            xQueueSendToBack(
                                tx_queue_handle,
                                (void *) rx_echo,
                                QUEUE_WAIT_MS
                            );

            command_string[ cmd_str_idx ] = '\0';
            cmd_str_idx = 0;
            // TODO call FreeRTOS_CLIProcessCommand() repeatedly until it returns pdFALSE
            FreeRTOS_CLIProcessCommand( command_string, pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );

            tx_ptr = pcOutputString;

            // transmit all but the last chunk
            while( strlen( tx_ptr ) > MSG_SIZE )
            {
                strncpy( tx_buf, tx_ptr, MSG_SIZE - 1 );
                // null terminate string
                tx_buf[ MSG_SIZE - 1 ] = '\0';
                xQueueSendToBack(
                                    tx_queue_handle,
                                    (void *) tx_buf,
                                    QUEUE_WAIT_MS
                                );

                tx_ptr += (char)(MSG_SIZE - 1);
            }

            // transmit the remainder of the output string
            strcpy( tx_buf, tx_ptr );
            xQueueSendToBack(
                                tx_queue_handle,
                                (void *) tx_buf,
                                QUEUE_WAIT_MS
                            );
        }
        else
        {
            // send character to Tx queue to echo back the character
            sprintf( rx_echo, "%c", rx_char );

            xQueueSendToBack(
                                tx_queue_handle,
                                (void *) rx_echo,
                                QUEUE_WAIT_MS
                            );

            command_string[ cmd_str_idx++ ] = rx_char;
        }

        vTaskSuspend( rx_task_handle );
    }
}


// millisecond delay
void msDelay( int ms )
{
    vTaskDelay( ms / portTICK_RATE_MS );
}


// get switch bit that was pushed, otherwise returns -1
int poll_buttons( void )
{
    if( mPORTCReadBits( OPEN_BUTTON_BIT ) == 0 )
        return OPEN_BUTTON_BIT;

    if( mPORTCReadBits( CLOSE_BUTTON_BIT ) == 0 )
        return CLOSE_BUTTON_BIT;

    msDelay( 10 );

    if( mPORTCReadBits( OPEN_BUTTON_BIT ) == 0 )
        return OPEN_BUTTON_BIT;

    if( mPORTCReadBits( CLOSE_BUTTON_BIT ) == 0 )
        return CLOSE_BUTTON_BIT;

    return -1;
}