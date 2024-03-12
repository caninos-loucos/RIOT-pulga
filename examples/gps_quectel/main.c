#include <stdio.h>
#include <string.h>

#include "thread.h"
#include "shell.h"
#include "xtimer.h"

#include "ringbuffer.h"
#include "periph/uart.h"
#include "minmea.h"

#define PMTK_SET_NMEA_OUTPUT_RMC    "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"
#define PMTK_SET_UPDATE_F_2HZ       "$PMTK300,500,0,0,0,0*28\r\n"

#define GPS_HANDLER_PRIO        (THREAD_PRIORITY_MAIN - 1)
static kernel_pid_t gps_handler_pid;
static char gps_handler_stack[THREAD_STACKSIZE_MAIN];

#ifndef UART_BUFSIZE
#define UART_BUFSIZE        (128U)
#endif

typedef struct {
    char rx_mem[UART_BUFSIZE];
    ringbuffer_t rx_buf;
} uart_ctx_t;
static uart_ctx_t ctx;

void rx_cb(void *arg, uint8_t data)
{
    uart_t dev = (uart_t)(uintptr_t)arg;

    ringbuffer_add_one(&ctx.rx_buf, data);

    if (data == '\n') {
        msg_t msg;
        msg.content.value = (uint32_t)dev;
        msg_send(&msg, gps_handler_pid);
    }
}

static void *gps_handler(void *arg)
{
    (void)arg;
    msg_t msg;
    msg_t msg_queue[8];
    msg_init_queue(msg_queue, 8);

    int pos = 0;
    char line[MINMEA_MAX_LENGTH];


    
    while (1) {

        msg_receive(&msg);
        char c;

        do {
            c = (char)ringbuffer_get_one(&(ctx.rx_buf));
            if (c == '\n') {
                line[pos++] = c;
		pos = 0;
                //printf("Will parse line of sentence id %d: %s\n", minmea_sentence_id(line, false), line);
                switch (minmea_sentence_id(line, false)) {
                    case MINMEA_SENTENCE_RMC: {
                        struct minmea_sentence_rmc frame;
                        if (minmea_parse_rmc(&frame, line)) {
                            printf("$RMC floating point degree coordinates and speed: (%f,%f) %f\n",
                                    minmea_tocoord(&frame.latitude),
                                    minmea_tocoord(&frame.longitude),
                                    minmea_tofloat(&frame.speed));
                        } else {
                            puts("Could not parse $RMC message. Possibly incomplete");
                        }
                    } break;

                    case MINMEA_SENTENCE_GGA: {
                        struct minmea_sentence_gga frame;
                        if (minmea_parse_gga(&frame, line)) {
                            printf("$GGA: fix quality: %d\n", frame.fix_quality);
                        }
                    } break;

                    case MINMEA_SENTENCE_GSV: {
                        struct minmea_sentence_gsv frame;
                        if (minmea_parse_gsv(&frame, line)) {
                            printf("$GSV: message %d of %d\n", frame.msg_nr, frame.total_msgs);
                            printf("$GSV: sattelites in view: %d\n", frame.total_sats);
                            for (int i = 0; i < 4; i++)
                                printf("$GSV: sat nr %d, elevation: %d, azimuth: %d, snr: %d dbm\n",
                                    frame.sats[i].nr,
                                    frame.sats[i].elevation,
                                    frame.sats[i].azimuth,
                                    frame.sats[i].snr);
                        }
                    } break;
                    default: break;
                }
            }
            else {
                line[pos++] = c;
            }
        } while (c != '\n');
    }

    /* this should never be reached */
    return NULL;
}

int init_gps(void)
{
    /* initialize UART */
    int dev = 1;
    uint32_t baud = 9600;

    int res = uart_init(UART_DEV(dev), baud, rx_cb, (void *)dev);
    if (res != UART_OK) {
        puts("Error: Unable to initialize UART device");
        return 1;
    }
    printf("Success: Initialized UART_DEV(%i) at BAUD %"PRIu32"\n", dev, baud);

    /* tell gps chip to wake up */
    uart_write(UART_DEV(dev), (uint8_t *)PMTK_SET_NMEA_OUTPUT_RMC, strlen(PMTK_SET_NMEA_OUTPUT_RMC));
    uart_write(UART_DEV(dev), (uint8_t *)PMTK_SET_UPDATE_F_2HZ, strlen(PMTK_SET_UPDATE_F_2HZ));
    puts("GPS Started.");
    return 0;
}

int main(void)
{
    (void) puts("Welcome to RIOT!");

    init_gps();

    /* initialize ringbuffer */
    ringbuffer_init(&(ctx.rx_buf), ctx.rx_mem, UART_BUFSIZE);

    /* start the gps_handler thread */
    gps_handler_pid = thread_create(gps_handler_stack, sizeof(gps_handler_stack),
                                GPS_HANDLER_PRIO, 0, gps_handler, NULL, "gps_handler");

    //char line_buf[SHELL_DEFAULT_BUFSIZE];
    //shell_run(NULL, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
