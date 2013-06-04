#include <buttondrv.h>


// variables
extern xTaskHandle irq_button_task_handle;

extern xSemaphoreHandle buttonPress;


void __attribute__((interrupt(ipl0), vector(_CHANGE_NOTICE_VECTOR))) vCN_ISR_Wrapper(void);


void initCN(void)
{
    // The following sequence initializes the RD6 (SW1), RD7 (SW2) and RD13 (SW3) pins to be used
    // with the change notification module. This will allow an interrupt to notify when one of
    // these pins change from HIGH-to-LOW (pressed) or LOW-to-HIGH (released). Pullups need to
    // be enabled. The mCNOpen() eliminates the need for any ConfigCNPullups() calls.

    // CNCON<15>
    // Set CNPUE<x> for each pin used - Done by main config
    // set CNEN<x> for each pin used
    mCNOpen( CN_ON, CN15_ENABLE | CN16_ENABLE | CN19_ENABLE, CN15_PULLUP_ENABLE | CN16_PULLUP_ENABLE | CN19_PULLUP_ENABLE );

    // Read PORTx to clear any *mismatch* for each PORT having a used pin
    mPORTDRead();

    enable_change_notification_irq();
}


void vCN_ISR(void)
{
    /* Variables */
    static portBASE_TYPE xHigherPriorityTaskWoken;

    disable_change_notification_irq();
    mCNClearIntFlag();

    xHigherPriorityTaskWoken = xTaskResumeFromISR( irq_button_task_handle );

    xSemaphoreGiveFromISR( buttonPress, &xHigherPriorityTaskWoken );

    /* If sending or receiving necessitates a context switch, then switch now. */
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}


void enable_change_notification_irq( void )
{
    // Set IPC6<20-18> and <17-16> for interrupt priority and subpriority
    // Clear IFS1<0> to clear interrupt status
    // Set IEC1<0> to unmask/enable interrupt
    ConfigIntCN( CHANGE_INT_PRI_3 | CHANGE_INT_ON );
}


void disable_change_notification_irq( void )
{
    ConfigIntCN( CHANGE_INT_OFF );
}

