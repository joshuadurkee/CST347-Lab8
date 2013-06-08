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

#define LED_QUEUE_DEPTH     5
#define TX_QUEUE_DEPTH      20
#define LED_QUEUE_SIZE      sizeof( int )
#define TX_QUEUE_SIZE       ( MSG_SIZE * sizeof( char ) )
#define QUEUE_WAIT_MS       ( 10 / portTICK_RATE_MS )

#define SEM_WAIT_MS         ( 10 / portTICK_RATE_MS )

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
#define ACCEL_FPSS_DFLT     2
#define MAX_SPEED_FPS_DFLT  40

// elevator positions
#define GD_FLOOR_POS        0
#define P1_FLOOR_POS        500
#define P2_FLOOR_POS        510

// door movement
#define DOOR_SEQUENCE_DURATION_MS \
                            ( 1000 / portTICK_RATE_MS )
#define DOOR_STATE_DURATION_MS \
                            ( DOOR_SEQUENCE_DURATION_MS / NUM_DOOR_STATES )
#define DOOR_OPEN_DURATION_MS \
                            ( 5000 / portTICK_RATE_MS )


//
// macros
//

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


// stuctures
typedef struct
{
    int num;                    /* The number of the LED to toggle, from 0-2. */
    int rate;                   /* The rate at which the LED should be toggle, in milliseconds. */
} task_parameter_t;


// enumerated types
typedef enum
{
    GD,                         /* ground floor                               */
    P1,                         /* penthouse 1                                */
    P2                          /* penthouse 2                                */
} floor_t;


typedef enum
{
    UP,                         /* elevator moving up                         */
    DOWN,                       /* elevator moving down                       */
    STOP                        /* elevator is stopped                        */
} elevator_movement_t;


typedef enum
{
    CLOSED,                     /* door is closed                             */
    MOSTLY_CLOSED,              /* door is mostly closed                      */
    MOSTLY_OPEN,                /* door is mostly open                        */
    OPEN,                       /* door is open                               */

    NUM_DOOR_STATES
} door_movement_t;


//
// prototypes
//

// tasks
void irqButtonControlTask( void *params );
void pollButtonControlTask( void *params );
void ledControlTask( void *params );
void txControlTask( void *params );
void rxControlTask( void *params );

// helper functions
void ms_delay( int ms );
int poll_buttons( void );
void transmit_string( char *tx_ptr );
void send_elevator_status( floor_t destination_floor, bool is_moving );
void send_movement_status( float distance_f, float speed_fps );
void set_elevator_up_down_leds( elevator_movement_t led_state );
void set_door_leds( door_movement_t state );
void open_door( void );
bool close_door( void );
void operate_door( void );


#endif	/* MYTASK_H */

