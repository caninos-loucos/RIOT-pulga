#ifndef MOBURB_LED_H
#define MOBURB_LED_H

#include <stdio.h>
#include "thread.h"
#include "timex.h"
#include "ztimer.h"
#include "periph_conf.h"

#include "shell.h"

#include "log.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BLINK_LED_PRIO        (THREAD_PRIORITY_MAIN - 1)

void run_blink_led(kernel_pid_t *led_pid);

//extern const shell_command_t shell_commands[];

#ifdef __cplusplus
}
#endif

#endif
