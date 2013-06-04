/**************************************************************************
 File:          uartdrv.c
 Author:        Jeremy Greenwood
 Course:        CST 347
 Assignment:    Lab 6
 Instructor:    Jay Bockelman
 Description:   UART driver source file which handles UART initialization,
                Rx, and Tx functionality.
**************************************************************************/


#include <uartdrv.h>


// variables
extern xTaskHandle      rx_task_handle;
extern xSemaphoreHandle inputByteBuffer;
extern xSemaphoreHandle outputStringBuffer;

static char tx_buffer[ MSG_SIZE ];
static char rx_buffer;


void __attribute__( ( interrupt(ipl0 ), vector( _UART1_VECTOR ) ) ) vUART1_ISR_Wrapper( void );


void initUart( UART_MODULE umPortNum, uint32_t ui32WantedBaud )
{
    /* UART Configuration */
    UARTConfigure( umPortNum, UART_ENABLE_PINS_TX_RX_ONLY );

    /* UART FIFO Mode */
    UARTSetFifoMode( umPortNum, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY );

    /* UART Line Control */
    UARTSetLineControl( umPortNum, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1 );

    /* Set the Baud Rate of the UART */
    UARTSetDataRate( umPortNum, (uint32_t)configPERIPHERAL_CLOCK_HZ, ui32WantedBaud );

    /* Enable the UART for Transmit Only*/
    UARTEnable( umPortNum, UART_ENABLE_FLAGS( UART_PERIPHERAL | UART_TX | UART_RX ) );

    /* Set UART INterrupt Vector Priority*/
    INTSetVectorPriority( INT_VECTOR_UART( UART1 ), INT_PRIORITY_LEVEL_2 );

    disableUartISR( UART1 );
}


void enableUartISR( UART_MODULE umPortNum )
{
    /* Enable Interrupts */
    INTEnable( INT_SOURCE_UART_RX( umPortNum ), INT_ENABLED );
    INTEnable( INT_SOURCE_UART_TX( umPortNum ), INT_ENABLED );
}


void disableUartISR( UART_MODULE umPortNum )
{
    /* Disable Interrupts */
    INTEnable( INT_SOURCE_UART_RX( umPortNum ), INT_DISABLED );
    INTEnable( INT_SOURCE_UART_TX( umPortNum ), INT_DISABLED );
}


void vUART1_ISR( void )
{
    static portBASE_TYPE
                xHigherPriorityTaskWoken;
    UART_DATA   rx_data;
    static int  i = 0;

    if( INTGetFlag( INT_U1RX ) )
    {
        INTClearFlag( INT_U1RX );
        
        /*----------------------------------------------------------------------
            Rx interrupt functionality
        ----------------------------------------------------------------------*/
        rx_data = UARTGetData( TARGET_UART );
        rx_buffer = rx_data.__data;
        
        xHigherPriorityTaskWoken = xTaskResumeFromISR( rx_task_handle );

        xSemaphoreGiveFromISR( inputByteBuffer, &xHigherPriorityTaskWoken );
        
        // set Rx task to resume
        portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
    }
    else
    {
        if( INTGetFlag( INT_U1TX ) )
        {
            INTClearFlag( INT_U1TX );

            /*------------------------------------------------------------------
                Tx interrupt functionality
            ------------------------------------------------------------------*/
            // transmit tx_buffer
            if( tx_buffer[ i ] != '\0' )
                UARTSendDataByte( TARGET_UART, tx_buffer[ i++ ] );
            else
            {
                i = 0;
                
                xHigherPriorityTaskWoken = pdFALSE;
                xSemaphoreGiveFromISR( outputStringBuffer, &xHigherPriorityTaskWoken );
            }
        }
    }
}


void UARTPutStr( char *tx_data )
{
    // take the output semaphore
    xSemaphoreTake( outputStringBuffer, portMAX_DELAY );

    // format string for Tx ISR
    sprintf( tx_buffer, "%s", tx_data );

    // set Tx interrupt
    INTSetFlag( INT_U1TX );
}


BYTE UARTGetChar( void )
{
    return rx_buffer;
}



