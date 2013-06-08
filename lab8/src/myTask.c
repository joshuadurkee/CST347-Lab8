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

static bool door_interference = false;
bool emergency_stop_flag = false;

static task_parameter_t task_parameter;


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
        ms_delay( 10 );

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
        ms_delay( 100 );

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
    long        ret;

    while( 1 )
    {
        xSemaphoreTake( inputByteBuffer, portMAX_DELAY );

        rx_char = UARTGetChar();

        // echo back the character
        sprintf( rx_echo, "%c", rx_char );
        transmit_string( rx_echo );

        switch( rx_char )
        {
            case GD_CALL_CHAR:

                break;
            case P1_CALL_DN_CHAR:

                break;
            case P1_CALL_UP_CHAR:

                break;
            case P2_CALL_CHAR:

                break;
            case EM_STOP_CHAR:
                emergency_stop_flag = true;
                break;
            case EM_CLR_CHAR_:
                emergency_stop_flag = false;
                break;
            case DOOR_INTF_CHAR:
                door_interference = true;
                break;
            case '\r':              /* complete command string on carriage return */
                // echo back end of the line entered by user
                transmit_string( "\n" );

                command_string[ cmd_str_idx ] = '\0';
                cmd_str_idx = 0;

                do
                {
                    ret = FreeRTOS_CLIProcessCommand( command_string, pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );
                    transmit_string( pcOutputString );
                } while( ret == pdTRUE );
                break;
            default:                /* add the character to the command string */
                command_string[ cmd_str_idx++ ] = rx_char;
        }

        vTaskSuspend( rx_task_handle );
    }
}


// millisecond delay
void ms_delay( int ms )
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

    ms_delay( 10 );

    if( mPORTCReadBits( OPEN_BUTTON_BIT ) == 0 )
        return OPEN_BUTTON_BIT;

    if( mPORTCReadBits( CLOSE_BUTTON_BIT ) == 0 )
        return CLOSE_BUTTON_BIT;

    return -1;
}


void transmit_string( char *tx_ptr )
{
    char tx_buf[ MSG_SIZE ];

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


void send_elevator_status( floor_t destination_floor, bool is_moving )
{
    char movement_str[ 16 ];
    char floor_str[ 8 ];
    char msg[ MSG_SIZE ];

    if( is_moving )
        strcpy( movement_str, "Moving to" );
    else
        strcpy( movement_str, "Stopped at" );

    switch( destination_floor )
    {
        case GD:
            strcpy( floor_str, "GD" );
            break;
        case P1:
            strcpy( floor_str, "P1" );
            break;
        case P2:
            strcpy( floor_str, "P2" );
            break;
    }

    // create message
    sprintf( msg, "%s floor %s\r\n", movement_str, floor_str );
    transmit_string( msg );
}


// transmit elevator distance and speed status (prints integer values)
void send_movement_status( float distance_f, float speed_fps )
{
    char distance_str[ 8 ];
    char speed_str[ 8 ];
    char msg[ MSG_SIZE ];

    itoa( distance_str, (int)distance_f, 10 );
    itoa( speed_str, (int)speed_fps, 10 );

    // create message
    sprintf( msg, "%s feet :: %s ft/s\r\n", distance_str, speed_str );
    transmit_string( msg );
}


void set_elevator_up_down_leds( elevator_movement_t led_state )
{
    switch( led_state )
    {
        case UP:
            // up LED off, down LED on (set/clear inverted for port B?)
            SET_BITS( UP_LED );
            CLEAR_BITS( DOWN_LED );
            break;
        case DOWN:
            // up LED on, down LED off
            CLEAR_BITS( UP_LED );
            SET_BITS( DOWN_LED );
            break;
        case STOP:
            // LEDs off
            SET_BITS( UP_LED );
            SET_BITS( DOWN_LED );
            break;
    }
}


void set_door_leds( door_movement_t state )
{
    switch( state )
    {
        case CLOSED:
            SET_BITS( DOOR_LED_0 );
            SET_BITS( DOOR_LED_1 );
            SET_BITS( DOOR_LED_2 );
            break;
        case MOSTLY_CLOSED:
            SET_BITS( DOOR_LED_0 );
            SET_BITS( DOOR_LED_1 );
            CLEAR_BITS( DOOR_LED_2 );
            break;
        case MOSTLY_OPEN:
            SET_BITS( DOOR_LED_0 );
            CLEAR_BITS( DOOR_LED_1 );
            CLEAR_BITS( DOOR_LED_2 );
            break;
        case OPEN:
            CLEAR_BITS( DOOR_LED_0 );
            CLEAR_BITS( DOOR_LED_1 );
            CLEAR_BITS( DOOR_LED_2 );
            break;
    }
}


void open_door( void )
{
    set_door_leds( CLOSED );
    ms_delay( DOOR_STATE_DURATION_MS );

    set_door_leds( MOSTLY_CLOSED );
    ms_delay( DOOR_STATE_DURATION_MS );

    set_door_leds( MOSTLY_OPEN );
    ms_delay( DOOR_STATE_DURATION_MS );

    set_door_leds( OPEN );
    ms_delay( DOOR_STATE_DURATION_MS );
}


// return true if door closed successfully, otherwise return false
bool close_door( void )
{
    set_door_leds( OPEN );
    ms_delay( DOOR_STATE_DURATION_MS );

    set_door_leds( MOSTLY_OPEN );
    ms_delay( DOOR_STATE_DURATION_MS );

    set_door_leds( MOSTLY_CLOSED );
    ms_delay( DOOR_STATE_DURATION_MS );

    // check for door interference
    if( door_interference )
    {
        door_interference = false;

        transmit_string( "Door interference closing door!\r\n" );

        set_door_leds( MOSTLY_OPEN );
        ms_delay( DOOR_STATE_DURATION_MS );

        set_door_leds( OPEN );
        ms_delay( DOOR_STATE_DURATION_MS );

        return false;
    }

    set_door_leds( CLOSED );
    ms_delay( DOOR_STATE_DURATION_MS );

    return true;
}


// open and close door (until close door succeeds)
void operate_door( void )
{
    open_door();
    ms_delay( DOOR_OPEN_DURATION_MS );

    while( !close_door() )
        ms_delay( DOOR_OPEN_DURATION_MS );
}


void elevatorMoveTask ( void )
{
    extern xQueueHandle elevator_move_queue_handle;

    //To show that this task is running:
    transmit_string( "Elevator move task running!\r\n" );

    while(1)
    {
    }
}

void motorControlTask( void )
{
    //To show that this task is running:
    transmit_string( "Motor control task running!\r\n" );

    static motor_led_state_t state = LED3;
    extern float current_speed_fps;
    float motor_speed;

    while (1)
    {
        set_motor_leds(state);
        state = (state + MOTOR_CONTROL_LED_INCREMENT) % NUM_MOTOR_LED_STATES;
        if (current_speed_fps != 0)
        {
            //protect against negative speeds
            if (current_speed_fps > 0) { motor_speed = current_speed_fps; }
            else                       { motor_speed = current_speed_fps * -1; }

            //shave off one's digit to round down to factor of 10
            motor_speed = motor_speed - ( ( int ) motor_speed % MOTOR_CONTROL_DELAY_FACTOR );

            ms_delay( MOTOR_CONTROL_BASE_DELAY / motor_speed );
        }
        else
        {
            vTaskSuspend(NULL);
        }
    }
}

void set_motor_leds( motor_led_state_t state )
{
    switch( state )
    {
        case LED1:
            SET_BITS  ( DOOR_LED_0 );
            CLEAR_BITS( DOOR_LED_1 );
            CLEAR_BITS( DOOR_LED_2 );
            break;
        case LED2:
            CLEAR_BITS( DOOR_LED_0 );
            SET_BITS  ( DOOR_LED_1 );
            CLEAR_BITS( DOOR_LED_2 );
            break;
        case LED3:
            CLEAR_BITS( DOOR_LED_0 );
            CLEAR_BITS( DOOR_LED_1 );
            SET_BITS  ( DOOR_LED_2 );
            break;
    }
}