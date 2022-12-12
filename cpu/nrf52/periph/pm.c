#include "periph_cpu.h"
#include "cpu.h"
#include "pm_layered.h"

#include "clk.h"
#include "board.h"
#include "periph_conf.h"
#include "timex.h"
#include "ztimer.h"
#include "nrf_clock.h"
#include "ztimer/periph_rtc.h"

void nrf52_sys_on(void) {
    /* Clear Event Register */
    __SEV();
    /* Wait for event */
    __WFE();
    /* Wait for event */
    __WFE();

}

void pm_set(unsigned mode)
{
    switch (mode) {
        case 0:
            //RTC wake up source sleep
            
            puts("Entering sleep");
            nrf52_sys_on();
            
            pm_set(1);
        break;
        case 1:
            //Ztimer active mode
            
            puts("Entering active");
            __WFE();
            ztimer_sleep(ZTIMER_SEC, 10);

            pm_set(0);

        break;
        case 2:
            //No sleep
            
        break;
    }
}

void pm_off(void);

void pm_set_lowest(void);