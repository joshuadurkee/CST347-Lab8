/**************************************************************************
 File:          main.c
 Authors:       Jeremy Greenwood
                Joshua Durkee
 Course:        CST 347
 Assignment:    end-of-term mini-project
 Instructor:    Jay Bockelman
 Description:   Main source file which sets up and launches FreeRTOS tasks.
**************************************************************************/

/*
    FreeRTOS V7.1.1 - Copyright (C) 2012 Real Time Engineers Ltd.


 ***************************************************************************
 *                                                                       *
 *    FreeRTOS tutorial books are available in pdf and paperback.        *
 *    Complete, revised, and edited pdf reference manuals are also       *
 *    available.                                                         *
 *                                                                       *
 *    Purchasing FreeRTOS documentation will not only help you, by       *
 *    ensuring you get running as quickly as possible and with an        *
 *    in-depth knowledge of how to use FreeRTOS, it will also help       *
 *    the FreeRTOS project to continue with its mission of providing     *
 *    professional grade, cross platform, de facto standard solutions    *
 *    for microcontrollers - completely free of charge!                  *
 *                                                                       *
 *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
 *                                                                       *
 *    Thank you for using FreeRTOS, and thank you for your support!      *
 *                                                                       *
 ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    >>>NOTE<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!
    
 ***************************************************************************
 *                                                                       *
 *    Having a problem?  Start by reading the FAQ "My application does   *
 *    not run, what could be wrong?                                      *
 *                                                                       *
 *    http://www.FreeRTOS.org/FAQHelp.html                               *
 *                                                                       *
 ***************************************************************************

    
    http://www.FreeRTOS.org - Documentation, training, latest information, 
    license and contact details.
    
    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool.

    Real Time Engineers ltd license FreeRTOS to High Integrity Systems, who sell 
    the code with commercial support, indemnification, and middleware, under 
    the OpenRTOS brand: http://www.OpenRTOS.com.  High Integrity Systems also
    provide a safety engineered and independently SIL3 certified version under 
    the SafeRTOS brand: http://www.SafeRTOS.com.
 */


/* Hardware include. */
#include <xc.h>

/* Standard includes. */
#include <stdint.h>
#include <plib.h>

/* FreeRTOS includes. */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <myTask.h>


/* Hardware configuration. */
#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1, FWDTEN = OFF
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_2, CP = OFF, BWP = OFF
#pragma config PWP = OFF /*, UPLLEN = OFF, FSRSSEL = PRIORITY_7 */

/* Time is measured in "ticks".  The tick rate is set by the configTICK_RATE_HZ
configuration parameter (defined in FreeRTOSConfig.h).  If configTICK_RATE_HZ
is equal to or less than 1000 (1KHz) then portTICK_RATE_MS can be used to 
convert a time in milliseconds into a time in ticks. */
#define mainTOGGLE_PERIOD ( 200UL / portTICK_RATE_MS )


// task priorities
#define IRQ_BUTTON_TASK_PRIORITY    1
#define POLL_BUTTON_TASK_PRIORITY   1
#define LED_TASK_PRIORITY           1
#define TX_TASK_PRIORITY            1
#define RX_TASK_PRIORITY            1
#define ELEVATOR_MOVE_TASK_PRIORITY 1
#define MOTOR_CONTROL_TASK_PRIORITY 1

// variables
xTaskHandle irq_button_task_handle;
xTaskHandle poll_button_task_handle;
xTaskHandle led_task_handle[ NUM_LEDS ];
xTaskHandle tx_task_handle;
xTaskHandle rx_task_handle;
xTaskHandle elevator_move_task_handle;
xTaskHandle motor_control_task_handle;

xQueueHandle led_queue_handle[ NUM_LEDS ];
xQueueHandle tx_queue_handle;
xQueueHandle elevator_move_queue_handle;

xSemaphoreHandle buttonPress;
xSemaphoreHandle ledNAction[ NUM_LEDS ];
xSemaphoreHandle inputByteBuffer;
xSemaphoreHandle outputStringBuffer;

int accel_fpss    = ACCEL_FPSS_DFLT;
int max_speed_fps = MAX_SPEED_FPS_DFLT;
float current_speed_fps = 0;

// prototypes
static void prv_setup_hardware(void);

/*-----------------------------------------------------------*/
int main(void)
{
    int     i;
    int     task_parameter[ NUM_LEDS ] = { 0, 1, 2 };

    /* Perform any hardware initialisation that may be necessary. */
    prv_setup_hardware();

    // create button control task
    xTaskCreate(
                    irqButtonControlTask,
                    "interrupt button task controller",
                    configMINIMAL_STACK_SIZE,
                    NULL,
                    IRQ_BUTTON_TASK_PRIORITY,
                    &irq_button_task_handle
               );

    // create button control task
    xTaskCreate(
                    pollButtonControlTask,
                    "polled button task controller",
                    configMINIMAL_STACK_SIZE,
                    NULL,
                    POLL_BUTTON_TASK_PRIORITY,
                    &poll_button_task_handle
               );

    for( i = 0; i < NUM_LEDS; i++ )
    {
        // create led control tasks
        xTaskCreate(
                        ledControlTask,
                        "LED task controller",
                        configMINIMAL_STACK_SIZE,
                        (void *) &task_parameter[ i ],
                        LED_TASK_PRIORITY,
                        &led_task_handle[ i ]
                   );
    }

    // create Tx control task
    xTaskCreate(
                    txControlTask,
                    "Tx task",
                    configMINIMAL_STACK_SIZE,
                    NULL,
                    TX_TASK_PRIORITY,
                    &tx_task_handle
               );

    // create Rx control task
    xTaskCreate(
                    rxControlTask,
                    "Rx task",
                    configMINIMAL_STACK_SIZE,
                    NULL,
                    RX_TASK_PRIORITY,
                    &rx_task_handle
               );

    // create Elevator Move task
    xTaskCreate(
                    (pdTASK_CODE)elevatorMoveTask,
                    "Elevator Move task",
                    configMINIMAL_STACK_SIZE,
                    NULL,
                    ELEVATOR_MOVE_TASK_PRIORITY,
                    &elevator_move_task_handle
               );

    // create Motor Control task
    xTaskCreate(
                    (pdTASK_CODE)motorControlTask,
                    "Motor Control task",
                    configMINIMAL_STACK_SIZE,
                    NULL,
                    MOTOR_CONTROL_TASK_PRIORITY,
                    &motor_control_task_handle
               );

    // create binary semaphores which are safe to call with xSemaphoreGiveFromISR()
    vSemaphoreCreateBinary( buttonPress );
    vSemaphoreCreateBinary( ledNAction[ 0 ] );
    vSemaphoreCreateBinary( ledNAction[ 1 ] );
    vSemaphoreCreateBinary( ledNAction[ 2 ] );
    vSemaphoreCreateBinary( inputByteBuffer );
    vSemaphoreCreateBinary( outputStringBuffer );

    // set semaphores as "taken"
    xSemaphoreTake( ledNAction[ 0 ], portMAX_DELAY );
    xSemaphoreTake( ledNAction[ 1 ], portMAX_DELAY );
    xSemaphoreTake( ledNAction[ 2 ], portMAX_DELAY );

    vTaskSuspend( rx_task_handle );
    vTaskSuspend( motor_control_task_handle );

    // initialize Tx queue
    tx_queue_handle = xQueueCreate( TX_QUEUE_DEPTH, TX_QUEUE_SIZE );

    // initialize elevator move queue
    elevator_move_queue_handle = xQueueCreate( ELEVATOR_MOVE_QUEUE_DEPTH, ELEVATOR_MOVE_QUEUE_SIZE );

    // register commands
    register_commands();

    // enable the UART ISR after initializing semaphores and before starting the scheduler
    enableUartISR( UART1 );

    /* Start the scheduler so the tasks start executing.  This function should not return. */
    vTaskStartScheduler();
}
/*-----------------------------------------------------------*/
static void prv_setup_hardware( void )
{
    /* Setup the CPU clocks, and configure the interrupt controller. */
    SYSTEMConfigPerformance( configCPU_CLOCK_HZ );
    mOSCSetPBDIV( OSC_PB_DIV_2 );
    INTEnableSystemMultiVectoredInt();

    // LEDs off
    CLEAR_BITS( DOOR_LED_0 );
    CLEAR_BITS( DOOR_LED_1 );
    CLEAR_BITS( DOOR_LED_2 );
    // LEDs off (set/clear inverted for port B?)
    SET_BITS( UP_LED );
    SET_BITS( DOWN_LED );

    // LEDs are outputs
    SET_DIGITAL_OUT( DOOR_LED_0 );
    SET_DIGITAL_OUT( DOOR_LED_1 );
    SET_DIGITAL_OUT( DOOR_LED_2 );
    SET_DIGITAL_OUT( UP_LED );
    SET_DIGITAL_OUT( DOWN_LED );

    // switches are inputs with pullups
    SET_DIGITAL_IN( GD_BUTTON );
    SET_DIGITAL_IN( P1_BUTTON );
    SET_DIGITAL_IN( P2_BUTTON );
    SET_DIGITAL_IN( OPEN_BUTTON );
    SET_DIGITAL_IN( CLOSE_BUTTON );

    initCN();

    // initialize the UART
    initUart( TARGET_UART, TARGET_BAUD );
}
