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
extern xTaskHandle tx_task_handle;
extern xTaskHandle rx_task_handle;
extern xTaskHandle elevator_move_task_handle;
extern xTaskHandle motor_control_task_handle;

extern xQueueHandle tx_queue_handle;
extern xQueueHandle elevator_move_queue_handle;

extern xSemaphoreHandle buttonPress;
extern xSemaphoreHandle inputByteBuffer;

elevator_movement_t elevator =
{
    ACCEL_STATE,
    STOP,
    (float)GD_FLOOR_POS,
    (float)GD_FLOOR_POS,
    (float)GD_FLOOR_POS,
    (float)GD_FLOOR_POS,
    0.0f,
    MAX_SPEED_FPS_DFLT,
    MAX_SPEED_FPS_DFLT,
    ACCEL_FPSS_DFLT,
    ACCEL_FPSS_DFLT,
};

static bool estop_decel_finished_flag = false;
static bool door_interference_flag = false;
static bool emergency_stop_flag    = false;
static bool door_open_flag         = true;
static bool force_door_closed_flag = false;
static bool operate_door_flag      = false;


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
            {
                switch( i )
                {
                    case 0:
                        queue_elevator_movement( P2_FLOOR_POS );
                        break;
                    case 1:
                        queue_elevator_movement( P1_FLOOR_POS );
                        break;
                    case 2:
                        queue_elevator_movement( GD_FLOOR_POS );
                        break;
                }
            }
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
                case OPEN_BUTTON_BIT:               // open button pressed
                    operate_door_flag = true;
                    break;                
                case CLOSE_BUTTON_BIT:              // close button pressed
                    force_door_closed_flag = true;
                    close_door();
                    break;
            }
        }
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
                queue_elevator_movement( GD_FLOOR_POS );
                transmit_string( "\r\n" );
                break;
            case P1_CALL_DN_CHAR:
            case P1_CALL_UP_CHAR:
                queue_elevator_movement( P1_FLOOR_POS );
                transmit_string( "\r\n" );
                break;
            case P2_CALL_CHAR:
                queue_elevator_movement( P2_FLOOR_POS );
                transmit_string( "\r\n" );
                break;
            case EM_STOP_CHAR:
                set_estop();
                transmit_string( "\r\nEmergency stop triggered!\r\n" );
                break;
            case EM_CLR_CHAR_:
                clear_estop();
                transmit_string( "\r\nEmergency clear triggered!\r\n" );
                break;
            case DOOR_INTF_CHAR:
                door_interference_flag = true;
                transmit_string( "\r\n" );
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


void send_elevator_status( int destination_floor, bool is_moving )
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
        case GD_FLOOR_POS:
            strcpy( floor_str, "GD" );
            break;
        case P1_FLOOR_POS:
            strcpy( floor_str, "P1" );
            break;
        case P2_FLOOR_POS:
            strcpy( floor_str, "P2" );
            break;
    }

    // create message
    sprintf( msg, "%s floor %s\r\n", movement_str, floor_str );
    transmit_string( msg );
}


// transmit elevator distance and speed status (prints integer values)
void send_movement_status( float position_f, float speed_fps )
{
    char distance_str[ 8 ];
    char speed_str[ 8 ];
    char msg[ MSG_SIZE ];

    itoa( distance_str, (int)position_f, 10 );
    itoa( speed_str, (int)speed_fps, 10 );

    // create message
    sprintf( msg, "%s feet :: %s ft/s\r\n", distance_str, speed_str );
    transmit_string( msg );
}


void set_elevator_up_down_leds( elevator_direction_t led_state )
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
    if( elevator.dir == STOP && door_open_flag == false )
    {
        door_open_flag = true;

        set_door_leds( CLOSED );
        ms_delay( DOOR_STATE_DURATION_MS );

        set_door_leds( MOSTLY_CLOSED );
        ms_delay( DOOR_STATE_DURATION_MS );

        set_door_leds( MOSTLY_OPEN );
        ms_delay( DOOR_STATE_DURATION_MS );

        set_door_leds( OPEN );
        ms_delay( DOOR_STATE_DURATION_MS );
    }
}


// return true if door closed successfully, otherwise return false
bool close_door( void )
{
    if( door_open_flag == true )
    {
        set_door_leds( OPEN );
        ms_delay( DOOR_STATE_DURATION_MS );

        set_door_leds( MOSTLY_OPEN );
        ms_delay( DOOR_STATE_DURATION_MS );

        set_door_leds( MOSTLY_CLOSED );
        ms_delay( DOOR_STATE_DURATION_MS );

        // check for door interference
        if( door_interference_flag )
        {
            door_interference_flag = false;

            transmit_string( "Door interference closing door!\r\n" );

            set_door_leds( MOSTLY_OPEN );
            ms_delay( DOOR_STATE_DURATION_MS );

            set_door_leds( OPEN );
            ms_delay( DOOR_STATE_DURATION_MS );

            return false;
        }

        set_door_leds( CLOSED );
        ms_delay( DOOR_STATE_DURATION_MS );
        door_open_flag = false;
    }

    return true;
}


// open and close door (until closing door succeeds)
void operate_door( void )
{
    int wait_ms = 10;
    int i;

    // clear any stale close door button presses
    force_door_closed_flag = false;

    open_door();

    do
    {
        for( i = 0; i < DOOR_OPEN_DURATION_MS / wait_ms; i++ )
        {
            ms_delay( wait_ms );
            if( force_door_closed_flag == true )
            {
                force_door_closed_flag = false;
                break;
            }
        }
    } while( !close_door() && door_open_flag == true );
}


void elevatorDoorTask( void )
{
    close_door();

    while( 1 )
    {        
        // check operate_door_flag
        if( operate_door_flag )
        {
            operate_door();
            operate_door_flag = false;
        }
    }
}


void motorControlTask( void )
{
    static motor_led_state_t state = LED3;
    float motor_speed;

    while( 1 )
    {
        set_motor_leds( state );
        
        if( elevator.speed >= MOTOR_CONTROL_DELAY_FACTOR )
        {
            // protect against negative velocities
            motor_speed = ABS( elevator.speed );

            // shave off one's digit to round down to factor of 10
            motor_speed = motor_speed - ( (int)motor_speed % MOTOR_CONTROL_DELAY_FACTOR );

            ms_delay( MOTOR_CONTROL_BASE_DELAY / motor_speed );
        }

        if( elevator.dir == STOP )
        {
            vTaskSuspend( NULL );
            state = LED3;
        }
        
        // decrement motor led state
        state--;
        state += NUM_MOTOR_LED_STATES;
        state %= NUM_MOTOR_LED_STATES;
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


void set_estop( void )
{
    int floor = GD_FLOOR_POS;

    emergency_stop_flag = true;

    // send move to ground floor command in case it is needed to jump start elevatorMoveTask,
    // ok because elevator_move_queue_handle is cleared after an estop
    queue_elevator_movement_high_priority( floor );
}


void clear_estop( void )
{
    emergency_stop_flag = false;
}


void queue_elevator_movement( int floor )
{
    xQueueSendToBack(
                        elevator_move_queue_handle,
                        (void *) &floor,
                        QUEUE_WAIT_MS
                    );
}


void queue_elevator_movement_high_priority( int floor )
{
    xQueueSendToFront(
                        elevator_move_queue_handle,
                        (void *) &floor,
                        QUEUE_WAIT_MS
                     );
}


void elevatorMoveTask( void )
{
    int     next_floor_pos;
    int     iter = 0;
    
    while( 1 )
    {
        // receive the next floor position to move to
        xQueueReceive( elevator_move_queue_handle, &next_floor_pos, portMAX_DELAY );

        elevator.dest_pos = (float)next_floor_pos;
        elevator.dir = get_dir_to_dest_flr( elevator );

        /* set static acceleration and max speed for each move of the elevator,
         * this is necessary as using a dynamic acceleration could result in an
         * elevator not being able to decelerate quickly enough to prevent a crash */
        elevator.max_speed = elevator.new_max_speed;
        elevator.accel = elevator.new_accel;

        // determine position where acceleration should stop in order to not exceed max speed
        // and to meet requirements to start decelerating in time to stop at destination floor
        elevator.stop_accel_pos = get_stop_accel_pos( elevator );

        // determine position where deceleration should start in order to stop at destination floor
        elevator.decel_pos = get_decel_pos( elevator );

        elevator.move_state = ACCEL_STATE;

        vTaskResume( motor_control_task_handle );

        send_elevator_status( elevator.dest_pos, 1 );

        // goto the next destination
        while( elevator.cur_pos != elevator.dest_pos )
        {
            ms_delay( ELEVATOR_PROCESS_INTERVAL_MS );
            iter++;

            // check if deceleration is needed (just got an estop)
            if( emergency_stop_flag == true && estop_decel_finished_flag == false )
            {
                // deccelerate immediately
                elevator.move_state = DECEL_STATE;
                if( elevator.speed == 0.0f )
                {
                     estop_decel_finished_flag = true;
                     break;
                }
            }
            
            // calculate updated position for a time delta of 500ms
            elevator.cur_pos = calc_pos( elevator );
            
            if( iter == ELEVATOR_UPDATE_INTERVAL_CNT )
            {
                iter = 0;

                // display direction LEDs
                set_elevator_up_down_leds( elevator.dir );

                // display distance and speed status
                send_movement_status( elevator.cur_pos, elevator.speed );
            }

            // determine if elevator has reached the destination or overshot it
            if( elevator.cur_pos == elevator.dest_pos
             || get_dir_to_dest_flr( elevator ) != elevator.dir )
            {
                elevator.speed = 0;
                elevator.dir = STOP;
                elevator.cur_pos = elevator.dest_pos;
                send_elevator_status( elevator.dest_pos, (bool)elevator.speed );
                set_elevator_up_down_leds( STOP );
                estop_decel_finished_flag = false;
            }
        }

        vTaskSuspend( motor_control_task_handle );

        if( estop_decel_finished_flag == false )
        {
            if( emergency_stop_flag )
            {
                open_door();

                // wait for emergency clear to clear the estop flag
                while( emergency_stop_flag )
                    ms_delay( EM_CLR_WAIT_MS );

                close_door();
            }
            else
            {
                operate_door();
            }
        }
    }
}


// get direction to destination floor
elevator_direction_t get_dir_to_dest_flr( elevator_movement_t elev )
{
    if( elev.cur_pos - elev.dest_pos > 0 )
        return DOWN;
    else if( elev.cur_pos - elev.dest_pos < 0 )
        return UP;
    else
        return STOP;
}


// get the stop acceleration position (assumes static max speed and acceleration), which
// is the position where acceleration must stop in order to not exceed max speed
// and to meet requirements to start decelerating in time to stop at destination floor
int get_stop_accel_pos( elevator_movement_t elev )
{
    float   halfway_pos;
    float   halfway_dist;
    float   stop_accel_pos_for_max_speed;
    float   stop_accel_dist_for_max_speed;
    float   stop_accel_pos;

    // determine position half way between current position and destination floor
    halfway_pos = AVG( elev.cur_pos, elev.dest_pos );
    halfway_dist = ABS( elev.cur_pos - halfway_pos );

    // determine the acceleration distance needed when moving from zero to maximum speed,
    // this is a kinmatic equation
    stop_accel_dist_for_max_speed = SQRT( (float)elev.max_speed ) / ( 2.0f * (float)elev.accel );

    // calculate position relative from current (start) position
    stop_accel_pos_for_max_speed = elev.cur_pos + ( stop_accel_dist_for_max_speed * elev.dir );

    if( halfway_dist < ABS( stop_accel_dist_for_max_speed ) )
        stop_accel_pos = halfway_pos;
    else
        stop_accel_pos = stop_accel_pos_for_max_speed;

    return stop_accel_pos;
}


// get the deceleration position (assumes static max speed and acceleration), which
// is the position where deceleration must start in order to successfully stop at
// destination floor
int get_decel_pos( elevator_movement_t elev )
{
    float   halfway_pos;
    float   halfway_dist;
    float   decel_pos_for_max_speed;
    float   decel_dist_for_max_speed;
    float   decel_pos;

    // determine position half way between current position and destination floor
    halfway_pos = AVG( elev.cur_pos, elev.dest_pos );
    halfway_dist = ABS( elev.dest_pos - halfway_pos );

    // determine the deceleration distance needed when moving from maximum speed to zero,
    // this is a kinmatic equation
    decel_dist_for_max_speed = - SQRT( (float)elev.max_speed ) / ( 2.0f * (float)elev.accel );

    // calculate position relative from destination position
    decel_pos_for_max_speed = elev.dest_pos + ( decel_dist_for_max_speed * elev.dir );

    // set deceleration position to halfway point if closer to destination floor
    if( halfway_dist < ABS( decel_dist_for_max_speed ) )
        decel_pos = halfway_pos;
    else
        decel_pos = decel_pos_for_max_speed;

    return decel_pos;
}


float calc_pos_with_accel( elevator_movement_t elev )
{
    float calc_dis;
    float calc_pos;

    calc_dis = ( elev.speed * ELEVATOR_PROCESS_INTERVAL_S ) + ( elev.accel * SQRT( ELEVATOR_PROCESS_INTERVAL_S ) ) / 2;
    calc_dis *= elev.dir;

    calc_pos = elev.cur_pos + calc_dis;

    return calc_pos;
}


float calc_pos_with_const_speed( elevator_movement_t elev )
{
    float calc_dis;
    float calc_pos;

    calc_dis = elev.speed * ELEVATOR_PROCESS_INTERVAL_S;
    calc_dis *= elev.dir;

    calc_pos = elev.cur_pos + calc_dis;
    
    return calc_pos;
}


// TODO verify correctness (not sure of minus at -elev.accel)
float calc_pos_with_decel( elevator_movement_t elev )
{
    float calc_dis;
    float calc_pos;

    calc_dis = ( elev.speed * ELEVATOR_PROCESS_INTERVAL_S ) + ( -elev.accel * SQRT( ELEVATOR_PROCESS_INTERVAL_S ) ) / 2;
    calc_dis *= elev.dir;

    calc_pos = elev.cur_pos + calc_dis;

    return calc_pos;
}


bool beyond_stop_accel_pos( elevator_movement_t elev )
{
    switch( elev.dir )
    {
        case UP:
            if( elev.cur_pos > elev.stop_accel_pos )
                return true;
            else
                return false;
        case DOWN:
            if( elev.cur_pos < elev.stop_accel_pos )
                return true;
            else
                return false;
    }
    return false;
}


bool beyond_decel_pos( elevator_movement_t elev )
{
    switch( elev.dir )
    {
        case UP:
            if( elev.cur_pos > elev.decel_pos )
                return true;
            else
                return false;
        case DOWN:
            if( elev.cur_pos < elev.decel_pos )
                return true;
            else
                return false;
    }
    return false;
}


float calc_pos( elevator_movement_t elev )
{
    float calc_pos;

    // calculate position/speed
    switch( elev.move_state )
    {
        case ACCEL_STATE:       // calculate new position and speed for case where stopped or accelerating
            calc_pos = calc_pos_with_accel( elevator );

            // calculate speed
            if (elevator.speed < elevator.max_speed)
            {
                elevator.speed += elevator.accel * ELEVATOR_PROCESS_INTERVAL_S;
                if (elevator.speed > elevator.max_speed)
                    elevator.speed = elevator.max_speed;
            }

            // check if beyond stop_accel_pos or decel_pos
            if( beyond_decel_pos( elevator ) )
            {
                elevator.move_state = DECEL_STATE;

                // constrain position
                calc_pos = elev.decel_pos;
            }
            else if( beyond_stop_accel_pos( elevator ) )
            {
                elevator.move_state = CONST_STATE;
                
                // constrain position
                calc_pos = elev.stop_accel_pos;
            }
            break;
        case CONST_STATE:
            calc_pos = calc_pos_with_const_speed( elevator );

            // Do nothing, speed is not changing

            // check if beyond decel_pos
            if( beyond_decel_pos( elevator ) )
            {
                elevator.move_state = DECEL_STATE;

                // constrain position
                calc_pos = elev.decel_pos;
            }
            break;
        case DECEL_STATE:
            calc_pos = calc_pos_with_decel( elevator );

            // this is here in case it is deemed necessary to uncomment later
            // ensure small errors don't cause elevator to not reach destination floor,
            // if elevator is more than 95% of DECEL_STATE, add an extra 0.5 ft
            if( ABS( calc_pos - elev.dest_pos ) < 0.05f * ABS( elev.dest_pos - elev.decel_pos ) )
                calc_pos += 0.1f * elev.dir;
            
            if (elevator.speed > 0)
            {
                elevator.speed -= elevator.accel * ELEVATOR_PROCESS_INTERVAL_S;
                if (elevator.speed < 0)
                    elevator.speed = 0;
            }

            break;
    }

    return calc_pos;
}
