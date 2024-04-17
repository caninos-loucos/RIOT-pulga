/*
 * Copyright (C) 2024 Universade de São Paulo
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       A LoraWAN example with GPS
 *
 * @author      Kauê Rodrigues Barbosa <kaue.rodrigueskrb@usp.br>
 *
 * @}
 */

#include <limits.h>
#include <float.h>

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "assert.h"
#include "event/timeout.h"

#include "msg.h"
#include "thread.h"
#include "fmt.h"

#include "net/loramac.h"
#include "semtech_loramac.h"

#include "sx127x.h"
#include "sx127x_netdev.h"
#include "sx127x_params.h"

#include "periph/gpio.h"
#include "xtimer.h"

#include <inttypes.h>

#include "board.h"
#include "periph/adc.h"

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

/** 
*  Size of lorawan buffer. 
*  It stores latitude, longitude and timestamp (Epoch). 
*/
#ifndef LORAWAN_BUFSIZE
#define LORAWAN_BUFSIZE        (132) 
#endif

/* Messages are sent every 300s (5 minutes) to respect the duty cycle on each channel */
#define PERIOD_S            (20U)

#define SENDER_PRIO         (THREAD_PRIORITY_MAIN - 1)
static kernel_pid_t sender_pid;
static char sender_stack[THREAD_STACKSIZE_MAIN / 2];
static void *sender(void *arg);

typedef struct {
    char rx_mem[LORAWAN_BUFSIZE];
    ringbuffer_t rx_buf;
} lora_ctx_t;
static lora_ctx_t ctx_lora;

semtech_loramac_t loramac;
static sx127x_t sx127x;

static ztimer_t timer;

char message[100];

static uint8_t deveui[LORAMAC_DEVEUI_LEN];
static uint8_t appeui[LORAMAC_APPEUI_LEN];

static uint8_t appskey[LORAMAC_APPSKEY_LEN];
static uint8_t nwkskey[LORAMAC_NWKSKEY_LEN];
static uint8_t devaddr[LORAMAC_DEVADDR_LEN];

static void _alarm_cb(void *arg) {

   (void) arg;
   msg_t msg;
   msg_send(&msg, sender_pid);
}

static void _prepare_next_alarm(void) {

   timer.callback = _alarm_cb;
   ztimer_set(ZTIMER_MSEC, &timer, PERIOD_S * MS_PER_SEC);
}


static void _send_message(void) {

    char *destination = (char*)malloc(33*sizeof(char));
    ringbuffer_get(&(ctx_lora.rx_buf), destination, 33);
    printf("Destination: %s\n", destination);

    /* Try to send the message */
    uint8_t ret = semtech_loramac_send(&loramac,(uint8_t*)(destination), 33);
    if (ret != SEMTECH_LORAMAC_TX_DONE)  {
        printf("Cannot send message '%s', ret code: %d\n\n", message, ret);
        return;
    }

   free(destination);
}


static void *sender(void *arg) {

   (void)arg;

   msg_t msg;
   msg_t msg_queue[8];
   msg_init_queue(msg_queue, 8);

   while (1) {
       msg_receive(&msg);

       /* Trigger the message send */
       _send_message();

       /* Schedule the next wake-up alarm */
       _prepare_next_alarm();
   }

   /* this should never be reached */
   return NULL;
}

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

    /* Char arrays that gets GPS desired data and sends with lora */
    char *gps_readings = (char*)malloc(33*sizeof(char));   
    char *gps_sender = (char*)malloc(33*sizeof(char));

    while (1) {
        msg_receive(&msg);
        char c;

        do {
            c = (char)ringbuffer_get_one(&(ctx.rx_buf));
            if (c == '\n') {
                line[pos++] = c;
		        pos = 0;
                switch (minmea_sentence_id(line, false)) {
                    case MINMEA_SENTENCE_RMC: {
                        struct minmea_sentence_rmc frame;
                        struct timespec time; 
                        if (minmea_parse_rmc(&frame, line)) {    
                            printf("$RMC floating point degree coordinates and speed: (%f,%f) %f\n",
                                    minmea_tocoord(&frame.latitude),
                                    minmea_tocoord(&frame.longitude),
                                    minmea_tofloat(&frame.speed));

                            /* Gets gps readings */
                            float lat = minmea_tocoord(&frame.latitude);
                            float lon = minmea_tocoord(&frame.longitude);
                            minmea_gettime(&time, &(frame.date), &(frame.time));                                        

                            if(isnan(lat) && isnan(lon)) {
                                lat = -90;
                                lon = -90;
                            }    

                            /* 33 is for all the algarisms, ponctuations (-,;+ etc.) and the null char */
                            int ret = snprintf(gps_readings,33,"%.6f;%.6f;%ld", lat, lon, (long)time.tv_sec);
                            if (ret < 0) {
                                puts("Cannot get gps data");
                                return 0;
                            }

                            printf("GPS READINGS: %s\n", gps_readings);

                            /* strcpy is necessary to add the null character in the end of array */
                            strcpy(gps_sender, gps_readings);

                            uint16_t i;
                            for(i = 0; i <= strlen(gps_sender); i++) {
                                ringbuffer_add_one(&ctx_lora.rx_buf, gps_sender[i]);
                            }

                            free(gps_readings);
                            free(gps_sender);
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

    /* This should never be reached */
    return NULL;
}

int init_gps(void)
{
    /* Initialize UART */
    int dev = 1;
    uint32_t baud = 9600;

    int res = uart_init(UART_DEV(dev), baud, rx_cb, (void *)dev);
    if (res != UART_OK) {
        puts("Error: Unable to initialize UART device");
        return 1;
    }
    printf("Success: Initialized UART_DEV(%i) at BAUD %"PRIu32"\n", dev, baud);

    /* Tell gps chip to wake up */
    uart_write(UART_DEV(dev), (uint8_t *)PMTK_SET_NMEA_OUTPUT_RMC, strlen(PMTK_SET_NMEA_OUTPUT_RMC));
    uart_write(UART_DEV(dev), (uint8_t *)PMTK_SET_UPDATE_F_2HZ, strlen(PMTK_SET_UPDATE_F_2HZ));
    puts("GPS Started.");
    return 0;
}


int main(void)
{

    puts("\nRIOT ADC peripheral driver test\n");
    puts("This test will sample all available ADC lines once every 100ms with\n"
         "a 10-bit resolution and print the sampled results to STDIO\n\n");
    
    
    /* Initialize gps ringbuffer */ 
    ringbuffer_init(&(ctx.rx_buf), ctx.rx_mem, UART_BUFSIZE);

    /* Initialize lora ringbuffer */
    ringbuffer_init(&(ctx_lora.rx_buf), ctx_lora.rx_mem, LORAWAN_BUFSIZE);

    init_gps();
    
    /* Start the gps_handler thread */
    gps_handler_pid = thread_create(gps_handler_stack, sizeof(gps_handler_stack), GPS_HANDLER_PRIO, 0, gps_handler, NULL, "gps_handler");


    puts("LoRaWAN Class A low-power application");
    puts("=====================================");

    /* Convert identifiers and application key */
    fmt_hex_bytes(deveui, CONFIG_LORAMAC_DEV_EUI_DEFAULT);
    fmt_hex_bytes(appeui, CONFIG_LORAMAC_APP_EUI_DEFAULT);

    fmt_hex_bytes(devaddr, CONFIG_LORAMAC_DEV_ADDR_DEFAULT);
    fmt_hex_bytes(nwkskey, CONFIG_LORAMAC_NWK_SKEY_DEFAULT);
    fmt_hex_bytes(appskey, CONFIG_LORAMAC_APP_SKEY_DEFAULT);

    /* Initialize the radio driver */
    sx127x_setup(&sx127x, &sx127x_params[0], 0);
    loramac.netdev = &sx127x.netdev;
    loramac.netdev->driver = &sx127x_driver;

    /* Initialize the loramac stack */
    semtech_loramac_init(&loramac);
    semtech_loramac_set_deveui(&loramac, deveui);
    semtech_loramac_set_appeui(&loramac, appeui);
    semtech_loramac_set_devaddr(&loramac, devaddr);
    semtech_loramac_set_appskey(&loramac, appskey);
    semtech_loramac_set_nwkskey(&loramac, nwkskey);

    /* Set a channels mask that makes it use only the first 8 channels */
    uint16_t channel_mask[LORAMAC_CHANNELS_MASK_LEN] = { 0 };
    channel_mask[0] = 0x00FF;
    semtech_loramac_set_channels_mask(&loramac, channel_mask);

    /* Use a fast datarate */
    semtech_loramac_set_dr(&loramac, LORAMAC_DR_5);

    /* Start the ABP procedure */
    puts("Starting join procedure");
    if (semtech_loramac_join(&loramac, LORAMAC_JOIN_ABP) != SEMTECH_LORAMAC_JOIN_SUCCEEDED) {
        puts("Join procedure failed");
        return 1;
    }
    puts("Join procedure succeeded");

    sender_pid = thread_create(sender_stack, sizeof(sender_stack),
                               SENDER_PRIO, 0, sender, NULL, "sender");

    msg_t msg_s;
    msg_send(&msg_s, sender_pid); 

    return 0;
}