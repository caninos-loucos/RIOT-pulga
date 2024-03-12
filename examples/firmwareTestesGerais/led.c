#include "led.h"

static char led_stack[THREAD_STACKSIZE_MAIN];
int blinking;


static void *blink_led_cb(void *arg)
{
	int *blinking = arg;
	*blinking = 1;

	while(1)
	{
		if (*blinking)
		{
			ztimer_sleep(ZTIMER_USEC, 50 * US_PER_MS);
			LED0_TOGGLE;

			ztimer_sleep(ZTIMER_USEC, 50 * US_PER_MS);
			LED0_TOGGLE;

			ztimer_sleep(ZTIMER_USEC, 50 * US_PER_MS);
			LED1_TOGGLE;

			ztimer_sleep(ZTIMER_USEC, 50 * US_PER_MS);
			LED1_TOGGLE;
		}

		ztimer_sleep(ZTIMER_USEC, 4 * US_PER_SEC);
	}

	return NULL;
}

void run_blink_led(kernel_pid_t *led_pid)
{
	void * blinking_pointer = &blinking;

	*led_pid = thread_create(led_stack, sizeof(led_stack), BLINK_LED_PRIO,
			0, blink_led_cb, blinking_pointer, "blink_led_cb");
}

int _disable_heartbeat(int argc, char **argv)
{
	(void)argc;
	(void)argv;

	blinking = 0;
	LOG_INFO("[led] heartbeat disabled\n");

	return 1;
}
SHELL_COMMAND(disable_heartbeat, "Disable heartbeat", _disable_heartbeat);

int _enable_heartbeat(int argc, char** argv)
{
	(void)argc;
	(void)argv;

	blinking = 1;
	LOG_INFO("[led] heartbeat enabled\n");

	return 1;
}
SHELL_COMMAND(enable_heartbeat, "Enable heartbeat", _enable_heartbeat);

//const shell_command_t shell_commands[] = {
//    { "ehrtb", "Enables heartbeat frase teste", _enable_heartbeat },
//    { "dhrtb", "Disables heartbeat", _disable_heartbeat },
//    { NULL, NULL, NULL }
//};

