#include "periph/pm.h"
#include "periph_cpu.h"
#include "pm_layered.h"
#include "cpu_conf.h"


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
            nrf52_sleep();
        break;
    }
}

void pm_off(void);

void pm_set_lowest(void);