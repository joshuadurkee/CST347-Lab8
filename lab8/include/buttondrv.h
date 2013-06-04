#ifndef BUTTONDRV_H
#define	BUTTONDRV_H

#include <FreeRTOS.h>
#include <plib.h>
#include <myTask.h>


// Prototypes
void initCN(void);
void enable_change_notification_irq( void );
void disable_change_notification_irq( void );


#endif	/* BUTTONDRV_H */

