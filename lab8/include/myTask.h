/**************************************************************************
 File:          myTask.h
 Authors:       Jeremy Greenwood
                Joshua Durkee
 Course:        CST 347
 Assignment:    end-of-term mini-project
 Instructor:    Jay Bockelman
 Description:   Task header file.
**************************************************************************/


#ifndef MYTASK_H
#define	MYTASK_H


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

#include <myCommands.h>


//
// constants
//

#define CHAR_TO_INT_OFFSET  48

#define TARGET_UART         UART1
#define TARGET_BAUD         9600

#define MSG_SIZE            50

#define LED_QUEUE_DEPTH             5
#define TX_QUEUE_DEPTH              20
#define ELEVATOR_MOVE_QUEUE_DEPTH   20
#define LED_QUEUE_SIZE              sizeof( int )
#define TX_QUEUE_SIZE               ( MSG_SIZE * sizeof( char ) )
#define ELEVATOR_MOVE_QUEUE_SIZE    sizeof( int )
#define QUEUE_WAIT_MS               ( 10 / portTICK_RATE_MS )

#define NUM_SWITCHES        3
#define NUM_LEDS            3

// inputs
#define GD_BUTTON_BIT       BIT_13
#define GD_BUTTON_PORT      D
#define P1_BUTTON_BIT       BIT_7
#define P1_BUTTON_PORT      D
#define P2_BUTTON_BIT       BIT_6
#define P2_BUTTON_PORT      D
#define OPEN_BUTTON_BIT     BIT_1
#define OPEN_BUTTON_PORT    C
#define CLOSE_BUTTON_BIT    BIT_2
#define CLOSE_BUTTON_PORT   C

// keyboard inputs
#define GD_CALL_CHAR        'z'
#define P1_CALL_DN_CHAR     'x'
#define P1_CALL_UP_CHAR     'c'
#define P2_CALL_CHAR        'v'
#define EM_STOP_CHAR        'b'
#define EM_CLR_CHAR_        'n'
#define DOOR_INTF_CHAR      'm'

// outputs
#define DOOR_LED_0_BIT      BIT_0
#define DOOR_LED_0_PORT     D
#define DOOR_LED_1_BIT      BIT_1
#define DOOR_LED_1_PORT     D
#define DOOR_LED_2_BIT      BIT_2
#define DOOR_LED_2_PORT     D
#define UP_LED_BIT          BIT_5
#define UP_LED_PORT         B
#define DOWN_LED_BIT        BIT_4
#define DOWN_LED_PORT       B

// elevator movement
#define ACCEL_FPSS_DFLT                 2
#define MAX_SPEED_FPS_DFLT              40
#define ELEVATOR_PROCESS_INTERVAL_MS    10
#define ELEVATOR_UPDATE_INTERVAL_MS     500
#define ELEVATOR_PROCESS_INTERVAL_S     ( (float)ELEVATOR_PROCESS_INTERVAL_MS / 1000.0f )
#define EM_CLR_WAIT_MS                  20
#define ELEVATOR_UPDATE_INTERVAL_CNT    ( ELEVATOR_UPDATE_INTERVAL_MS / ELEVATOR_PROCESS_INTERVAL_MS )

// elevator positions
#define GD_FLOOR_POS                    0
#define P1_FLOOR_POS                    500
#define P2_FLOOR_POS                    510

// door movement
#define DOOR_SEQUENCE_DURATION_MS       1000
#define DOOR_STATE_DURATION_MS          ( DOOR_SEQUENCE_DURATION_MS / NUM_DOOR_STATES )
#define DOOR_OPEN_DURATION_MS           5000

// motor control
#define MOTOR_CONTROL_BASE_DELAY        10000
#define MOTOR_CONTROL_DELAY_FACTOR      10


//
// macros
//

#define ABS(x)              (  ( x ) < 0 ? -( x ) : ( x )  )
#define AVG(x,y)            (  ( x + y ) / 2  )
#define MAX(x,y)            (  x > y ? x : y  )
#define MIN(x,y)            (  x < y ? x : y  )
#define SQRT(x)             (  ( x ) * ( x )  )

/* helper macros */
#define PASTE(a, b)         a##b
#define GLUE(a, b)          PASTE( a, b )

#define GET_PIN_NAME(x)     GLUE( x, _BIT )
#define GET_PORT_NAME(x)    GLUE( IOPORT_, GLUE( x, _PORT ) )

/* port macros */
#define SET_DIGITAL_OUT(x)  PORTSetPinsDigitalOut( GET_PORT_NAME(x), GET_PIN_NAME(x) )
#define SET_DIGITAL_IN(x)   PORTSetPinsDigitalIn( GET_PORT_NAME(x), GET_PIN_NAME(x) )

#define CLEAR_BITS(x)       PORTClearBits( GET_PORT_NAME(x), GET_PIN_NAME(x) )
#define SET_BITS(x)         PORTSetBits( GET_PORT_NAME(x), GET_PIN_NAME(x) )
#define TOGGLE_BITS(x)      PORTToggleBits( GET_PORT_NAME(x), GET_PIN_NAME(x) )
#define READ_BITS(x)        PORTReadBits( GET_PORT_NAME(x), GET_PIN_NAME(x) )

//
// enumerated types
//

typedef enum
{
    UP = 1,                     /* elevator moving up                         */
    DOWN = -1,                  /* elevator moving down                       */
    STOP = 0                    /* elevator is stopped                        */
} elevator_direction_t;


typedef enum
{
    CLOSED,                     /* door is closed                             */
    MOSTLY_CLOSED,              /* door is mostly closed                      */
    MOSTLY_OPEN,                /* door is mostly open                        */
    OPEN,                       /* door is open                               */

    NUM_DOOR_STATES
} door_movement_t;


typedef enum
{
    LED1,                     /* LED 1 (RD0) is lit                           */
    LED2,                     /* LED 2 (RD1) is lit                           */
    LED3,                     /* LED 3 (RD2) is lit                           */

    NUM_MOTOR_LED_STATES
} motor_led_state_t;


typedef enum                        // pos vs time     ****************
{                                   //                *                *
    ACCEL_STATE,                    //               *                  *
    CONST_STATE,                    //              *                    *
    DECEL_STATE                     //             *                      *
} elev_move_state_t;                //            *                        *
                                    //           *                          *
                                    //          *                            *
                                    //         *                              *
                                    // states:   ACCEL |    CONST     |  DECEL
//
// stuctures
//

typedef struct
{
    elev_move_state_t
                move_state;     /* state machine state for elevator movement  */
    elevator_direction_t
                dir;            /* direction elevator is moving               */
    float       cur_pos;        /* current position of elevator in feet       */
    float       stop_accel_pos; /* calculated stop acceleration position      */
    float       decel_pos;      /* calculated start deceleration position     */
    float       dest_pos;       /* destination position of elevator in feet   */
    float       speed;          /* current speed of eleveator in feet per second */
    int         max_speed;      /* maximum speed of elevator in feet per second */
    int         new_max_speed;  /* future value of max_speed                  */
    int         accel;          /* acceleration of elevator in feet per second squared */
    int         new_accel;      /* future value of acceleration               */
} elevator_movement_t;


//
// prototypes
//

// tasks
void irqButtonControlTask( void *params );
void pollButtonControlTask( void *params );
void ledControlTask( void *params );
void txControlTask( void *params );
void rxControlTask( void *params );
void elevatorMoveTask( void );
void elevatorDoorTask( void );
void motorControlTask( void );

// helper functions
void ms_delay( int ms );
int poll_buttons( void );
void transmit_string( char *tx_ptr );
void send_elevator_status( int destination_floor, bool is_moving );
void send_movement_status( float position_f, float speed_fps );
void set_elevator_up_down_leds( elevator_direction_t led_state );
void set_door_leds( door_movement_t state );
void open_door( void );
bool door_is_closed( void );
bool close_door( void );
void operate_door( void );
void set_motor_leds( motor_led_state_t state );
void set_estop( void );
void clear_estop( void );
void queue_elevator_movement_high_priority( int floor );
void queue_elevator_movement( int floor );
elevator_direction_t get_dir_to_dest_flr( elevator_movement_t elev );
int get_decel_pos( elevator_movement_t elev );
float calc_pos_with_accel( elevator_movement_t elev );
float calc_pos_with_const_speed( elevator_movement_t elev );
float calc_pos_with_decel( elevator_movement_t elev );
bool beyond_stop_accel_pos( elevator_movement_t elev );
bool beyond_decel_pos( elevator_movement_t elev );
float calc_pos( elevator_movement_t elev );


#endif	/* MYTASK_H */

