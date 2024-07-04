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

#include "bq2429x.h"

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

#include "ztimer.h"
#include "shell.h"

#define PMTK_SET_NMEA_OUTPUT_RMC    "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"
#define PMTK_SET_UPDATE_F_2HZ       "$PMTK300,500,0,0,0,0*28\r\n"

//Constantes

#define DATE_TIME_LEN 6U
#define LATITUDE_LEN 4U
#define LONGITUDE_LEN 4U
#define SPEED_LEN 3U
#define LORA_MSG_LEN 26U 

// LORA_MSM_LEN eh o total de digitos mais o \n



//const size_t DATE_TIME_LEN = 6;
//const size_t LATITUDE_LEN = 4;
//const size_t LONGITUDE_LEN = 4;
//const size_t SPEED_LEN = 3;

char HDR = 0x10;
char DEVID[LORAMAC_DEVADDR_LEN] = {0x00,0x00,0x00,0x00};
char DEFAULT = 0x00;
char DATETIME[DATE_TIME_LEN] = {0x18,0x02,0x09,0x13,0x25,0x02};
char LATITUDE[LATITUDE_LEN] = {0x16,0x94,0x95,0x75}; 
char LONGITUDE[LONGITUDE_LEN] = {0x2b,0x19,0x18,0x00};
char SPEED[SPEED_LEN] = {0x00,0x00,0x00};

//char p_datetime[8];
//p_datetime = (char *)&DATETIME;




// Era pra ser importado do pkg. Ver isso depois
#ifndef MINMEA_MAX_SENTENCE_LENGTH
#define MINMEA_MAX_SENTENCE_LENGTH 80
#endif

/**
*   GPS thread priority.
*/
#define GPS_HANDLER_PRIO        (THREAD_PRIORITY_MAIN - 1) 
static kernel_pid_t gps_handler_pid;
static char gps_handler_stack[THREAD_STACKSIZE_MAIN];

#ifndef UART_BUFSIZE
#define UART_BUFSIZE        (128U)
#endif

/** 
*   Size of lorawan ringbuffer. 
*   It stores latitude, longitude and timestamp (Epoch) 4 times. 
*/
#ifndef LORAWAN_BUFSIZE
#define LORAWAN_BUFSIZE        (53U) 
#endif

/** 
*   Messages are sent every 300s (5 minutes). 20s is te minimum to respect the duty cycle on each channel 
*/
#define PERIOD_S            (300U)

/** 
*   Priority of the Lora thread. Must be lower than the GPS 
*/
#define SENDER_PRIO         (THREAD_PRIORITY_IDLE - 1) 
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

    char *destination = (char*)malloc(LORA_MSG_LEN*sizeof(char));
    ringbuffer_get(&(ctx_lora.rx_buf), destination, LORA_MSG_LEN);

    printf("Destination: ");
    for(size_t i=0; i<LORA_MSG_LEN; i++)
        printf("%d", destination[i]);

    /* Try to send the message */
    uint8_t ret = semtech_loramac_send(&loramac,(uint8_t*)(destination), strlen(destination));
    if (ret != SEMTECH_LORAMAC_TX_DONE)  {
       printf("Cannot send message '%s', ret code: %d\n\n", message, ret);
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

/* Fixes the minmea received sentence, excluding possible trash data */
static void fix_minmea_sentence(char *line) {
    
    // Line must not be null
    if (line == NULL) 
        return;

    // Runs the string until a terminator char is found or it reaches its end '\n'
    while (*line != '\0') {
        if (*line == '\n') {
            // Substitues all values after '\n' for '\0'
            line++;
            while (*line != '\0') {
                *line = '\0';
                line++;
            }
            return;
        }
        line++;
    }
}

//struct tm *tm;

/**
*   Writes date, time, speed and additonal data into desired ringbuffer
*/
static void store_data(struct minmea_sentence_rmc *frame, lora_ctx_t *ring) { //struct minmea_sentence_rmc *frame, struct tm *time, lora_ctx_t *ring
    
    
    
    //int j;
    //char p_datetime[8];
    //p_datetime = (char*)DATETIME;
    

    /* Char arrays used to store desired GPS data */
    char *temp = (char*)malloc(LORA_MSG_LEN*sizeof(char));   


    //char *package = (char*)malloc(53*sizeof(char));

    /* Gets latitude, longitude, date, time and speed */
    /* Latitude and longitude in absolute values */

    float latitude =  minmea_tocoord(&(*frame).latitude); 
    float longitude = minmea_tocoord(&(*frame).longitude);
    float speed = minmea_tofloat(&(*frame).speed);

    /* Guarantees a nan value is not writen */
    if(isnan(latitude) && isnan(longitude)) {
        latitude = 90; // -90 before
        longitude = 90; 
    }   

    if (latitude < 0) latitude = -latitude;
    if (longitude < 0) longitude = -longitude;

    //float latitude =  abs(minmea_tocoord(&(*frame).latitude)); 
    //float longitude = abs(minmea_tocoord(&(*frame).longitude));
    //float speed = minmea_tofloat(&(*frame).speed);
    //minmea_getdatetime(tm, &(frame->date), &(frame->time));  

    

     

    /* Conveting data into bytes and desired format */
    for(size_t i = 0; i < LORAMAC_DEVADDR_LEN; i++)
        DEVID[i] = devaddr[i];

    // Printing DEVID
    printf("DEVID: ");
    for(size_t i = 0; i < LORAMAC_DEVADDR_LEN; i++)
        printf("%x", DEVID[i]);

    puts("");

    // Printing DEVID from source
    printf("DEVID(source): ");
    for(size_t i = 0; i < LORAMAC_DEVADDR_LEN; i++)
        printf("%x", devaddr[i]);

    puts("");

    /* Date and time */
    DATETIME[0] = frame->date.day;
    DATETIME[1] = frame->date.month;
    DATETIME[2] = frame->date.year;
    DATETIME[3] = frame->time.hours - 3; // UTC -3
    DATETIME[4] = frame->time.minutes;
    DATETIME[5] = frame->time.seconds; 

    //printf("DATETIME(foolproof): %d;%d;%d;%d;%d;%d\n\r", tm->tm_mday, tm->tm_mon, ((tm->tm_year + 1900) % 100), tm->tm_hour, tm->tm_min, tm->tm_sec);

    /* Latitude */
    printf("latitude1: %f\r\n", latitude);
    int decimal = (int) floor(latitude);
    LATITUDE[0] = decimal;
    //snprintf(&LATITUDE[0], 3, "%X", (char) decimal);

    printf("latitude2: %f\r\n", latitude);
    latitude = latitude - decimal;
    printf("lat3: %f\r\n", latitude);
    int lat = (int) floor(latitude * 1000000);
    printf("lat4: %d\r\n", lat);

    LATITUDE[3] =  lat % 100;
    printf("guardado: %d \r\n", ((int)lat % 100));
    printf("lat5: %d\r\n", lat);
    lat = lat / 100;
    printf("lat6: %d\r\n", lat);
    LATITUDE[2] =  ((int)lat % 100);
    printf("guardado: %d \r\n", ((int)lat % 100));
    printf("lat7: %d\r\n", lat);
    lat = lat / 100;
    printf("lat8: %d\r\n", lat);
    LATITUDE[1] =  ((int)lat % 100);
    printf("guardado: %d \r\n", ((int)lat % 100));

    /* Longitude */
    decimal = (int) floor(longitude);
    LONGITUDE[0] = decimal;
    //snprintf(&LONGITUDE[0], 3, "%X", (char) decimal);

    longitude = longitude - decimal;
    longitude = longitude * 1000000;
    int lon = (int) floor(longitude);

    LONGITUDE[3] =  lon % 100;
    lon = lon / 100;
    LONGITUDE[2] =  lon % 100;
    lon = lon / 100;
    LONGITUDE[1] =  lon % 100;

    /* Speed */
    decimal = (int) floor(speed);
    SPEED[0] = decimal;

    speed = speed - decimal;
    speed = speed * 10000;
    int speed_int = (int) floor(speed);

    SPEED[2] =  speed_int % 100;
    speed_int = speed_int / 100;
    SPEED[1] =  speed_int % 100;


    printf("DEFAULT: %02X\r\n", DEFAULT);


    //printf("HDR: %x \r\n", HDR);
    //printf("DEVID: %010lx \r\n", DEVID);
    //printf("DEFAULT: %02x \r\n", DEFAULT);
    //printf("DATETIME: %12lx \r\n", DATETIME);

    printf("DATETIME: ");
    
    for (uint8_t j=0; j<DATE_TIME_LEN; j++)
        printf("%02d;", DATETIME[j]);

    puts("");



    printf("LATITIDE: ");
    printf("%02X;", LATITUDE[0]);
    for (uint8_t j=1; j<LATITUDE_LEN; j++)
        printf("%02d;", LATITUDE[j]);

    puts("");

        

    printf("LONGITUDE: ");
    printf("%02X;", LONGITUDE[0]);
    for (uint8_t j=1; j<LONGITUDE_LEN; j++)
        printf("%02d;", LONGITUDE[j]);

    puts("");

    printf("SPEED: ");
    for (uint8_t j=0; j<SPEED_LEN; j++)
        printf("%02d;", SPEED[j]);

    puts("");

    temp[0] = DEFAULT; 
    temp[1] = DEVID[0]; temp[2] = DEVID[1]; temp[3] = DEVID[2]; temp[4] = DEVID[3]; 
    temp[5] = DEFAULT; temp[6] = DEFAULT; temp[7] = DEFAULT;
    temp[8] = (int) DATETIME[0]; temp[9] = (int) DATETIME[1]; temp[10] = (int) DATETIME[2]; temp[11] = (int) DATETIME[3]; 
    temp[12] = (int) DATETIME[4]; temp[13] = (int) DATETIME[5]; 
    temp[14] = LATITUDE[0]; temp[15] = (int) LATITUDE[1]; temp[16] = (int) LATITUDE[2]; temp[17] = (int) LATITUDE[3]; 
    temp[18] = LONGITUDE[0]; temp[19] = (int) LONGITUDE[1]; temp[20] = (int) LONGITUDE[2]; temp[21] = (int) LONGITUDE[3]; 
    temp[22] = (int) SPEED[0]; temp[23] = (int) SPEED[1]; temp[24] = (int) SPEED[2]; 
    temp[25] = '\n';

    //snprintf(temp, LORA_MSG_LEN, "%X%X%X%X%X%X%X%X%d%d%d%d%d%d%X%d%d%d%X%d%d%d%d%d%d", DEFAULT, 
    //        DEVID[0],DEVID[1], DEVID[2], DEVID[3], 
    //        DEFAULT, DEFAULT, DEFAULT, 
    //        (int) DATETIME[0], (int) DATETIME[1], (int) DATETIME[2], (int) DATETIME[3], (int) DATETIME[4], (int) DATETIME[5],
    //        LATITUDE[0], (int) LATITUDE[1], (int) LATITUDE[2], (int) LATITUDE[3],
    //        LONGITUDE[0], (int) LONGITUDE[1], (int) LONGITUDE[2], (int) LONGITUDE[3],
    //        (int) SPEED[0], (int) SPEED[1], (int) SPEED[2]);

    
    printf("String enviada: ");
    for (size_t j=0; j<LORA_MSG_LEN; j++)
        printf("%02x;", temp[j]);

    puts("");
        
        
    //printf("LATITUDE: %08lx \r\n", LATITUDE);
    //printf("LONGITUDE: %08lx \r\n", LONGITUDE);
    //printf("SPEED: %06lx \r\n", SPEED);

    /* 53 is for all the algarisms, ponctuations (-,;+ ...) and the null character */

    //size_t TESTE_SIZE = 7;
    //char *teste = (char*)malloc(TESTE_SIZE*sizeof(char));

    //int ret; 
    //snprintf(temp, 53, "%02x", HDR);
    //for (size_t i=0; i<4; i++)
    //    snprintf(temp, 53, "%02x", DEVID[i]);
    //snprintf(temp, 7, "%02x%02x%02x", DEFAULT, DEFAULT, DEFAULT);

    //size_t index = 0;
    //teste[index] = HDR; index++;
    //for (size_t j=0; j<sizeof(DATETIME); j++)
    //    teste[j] = DATETIME[j];


    //for (size_t i=0; i<3; i++)
    //    snprintf(temp, 53, "%02x", LATITUDE[i]);
    //for (size_t i=0; i<3; i++)
    //    snprintf(temp, 53, "%02x", LONGITUDE[i]);
    //for (size_t i=0; i<2; i++)
    //    snprintf(temp, 53, "%02x", SPEED[i]);

    //int ret = snprintf(temp,53,"%02x%10s%02x%02x%02x%12s%8s%8s%6s", HDR, DEVID, DEFAULT, DEFAULT, DEFAULT, DATETIME, LATITUDE,
    //                    LONGITUDE, SPEED); 

    //if (ret < 0) {
    //    puts("Cannot get gps data");
    //    return;
    //}

    //printf("SIZE TESTE: %d\r\n", sizeof(teste));

    //printf("String final: ");
    //for (size_t i=0; i<TESTE_SIZE; i++)
    //    printf("%02x", teste[i]);

    



    /* Strcpy is necessary to add the null character in the end of array */
    //strcpy(package, temp);

    /* It has to be done in a for loop to overwrite old data if ringbuffer is full */
    uint16_t i;
    for(i = 0; i <= strlen(temp); i++) {
        ringbuffer_add_one(&(*ring).rx_buf, temp[i]);
    }
    
    //free(teste);
    
    free(temp);
    //free(package);  

    puts("Data stored =)\r\n");                       
}

static void *gps_handler(void *arg)
{
    (void)arg;
    msg_t msg;
    msg_t msg_queue[8];
    msg_init_queue(msg_queue, 8);

    int pos = 0;
    char line[MINMEA_MAX_SENTENCE_LENGTH];  // char line[MINMEA_MAX_SENTENCE_LENGTH];

    while (1) {

        msg_receive(&msg);

        char c;

        do {
            c = (char)ringbuffer_get_one(&(ctx.rx_buf));

            if (c == '\n') {

                line[pos++] = c;
		        pos = 0;
                
                printf("LINE ANTES: %s\r\n", line);

                fix_minmea_sentence(line); 

                printf("LINE DEPOIS: %s\r\n", line);           

                switch (minmea_sentence_id(line, false)) {

                    case MINMEA_SENTENCE_RMC: {

                        struct minmea_sentence_rmc frame;

                        if (minmea_parse_rmc(&frame, line)) {    
                            printf("$RMC floating point degree coordinates and speed: (%f,%f) %f\r\n",
                                    minmea_tocoord(&frame.latitude),
                                    minmea_tocoord(&frame.longitude),
                                    minmea_tofloat(&frame.speed));

                            printf("DATETIME(gps_handler): %d;%d;%d;%d;%d;%d", frame.date.day, frame.date.month, frame.date.year, frame.time.hours, frame.time.minutes,
                                frame.time.seconds);

                            store_data(&frame, &ctx_lora); 

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
                    }break;                    
                    
                    default : {
                        printf("Could not parse any message!\r\n");
                    } break;
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


int main(void) {

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

    /* Start the Lora thread */
    sender_pid = thread_create(sender_stack, sizeof(sender_stack), SENDER_PRIO, 0, sender, NULL, "sender");

    /* Wakes the Lora thread up */
    msg_t msg;
    msg_send(&msg, sender_pid);

    return 0;
}