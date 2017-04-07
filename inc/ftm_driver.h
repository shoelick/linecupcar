
#ifndef FTM_DRIVER_H
#define FTM_DRIVER_H

#include "MK64F12.h"

#define FTM_ERR_NONE 0
#define FTM_ERR_INIT 1

typedef struct ftm_driver {
    FTM_Type *regs;
    IRQn_Type irqn; 
} ftm_driver;

int ftm_init(ftm_driver *drv, int num);
void ftm_enable_int(ftm_driver *drv);

#endif
