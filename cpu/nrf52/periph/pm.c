#include "periph_cpu.h"
#include "cpu.h"



void pm_set(unsigned mode)
{
    switch (mode) {
        case 0:
            //Implementar o light sleep
            cortexm_sleep(0);
        break;
        case 1:
            //Implementar o deep sleep
            cortexm_sleep(1);
        break;
        case 2:
            //Implementar o Wait fo event sleep
            NRF_POWER->TASKS_CONSTLAT = 0;
            NRF_POWER->TASKS_LOWPWR = 1;
            __DSB();
            /* Clear Event Register */
            __SEV();
            /* Wait for event */
            __WFE();
            /* Wait for event */
            __WFE();
        break;
    }
}

void pm_off(void);

void pm_set_lowest(void);