#include "ISR_Support.h"

    .set	nomips16
	.set 	noreorder

    .extern vCN_ISR
	.extern xISRStackTop
    .global vCN_ISR_Wrapper
    .set noreorder
    .set noat
    .ent vCN_ISR_Wrapper

vCN_ISR_Wrapper:
    portSAVE_CONTEXT

    jal vCN_ISR
    nop

    portRESTORE_CONTEXT

    .end vCN_ISR_Wrapper