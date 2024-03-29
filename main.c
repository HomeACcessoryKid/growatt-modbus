/*  (c) 2020-2021 HomeAccessoryKid
 *  This is a growatt modbus master that is an http server to report the current values
 *  it reads from U0RXD = GPIO1
 *  and it writes to U1TXD = GPIO2
 *  it uses UDPlogger to report debugging state
 */

#include <stdio.h>
#include <espressif/esp_wifi.h>
#include <espressif/esp_sta.h>
#include <espressif/esp_system.h> //for timestamp report only
#include <esp/uart.h>
#include <esp8266.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <timers.h>
#include <string.h>
#include "lwip/api.h"
#include <lwip/sockets.h>
#include <http-parser/http_parser.h>
#include <udplogger.h>
#include <espressif/esp8266/eagle_soc.h>

#ifndef VERSION
 #error You must set VERSION=x.y.z to match github version tag x.y.z
#endif

#define WIFI_CONFIG_SERVER_PORT 80

uint32_t calccrc(uint32_t * data, int len) {   
    uint32_t crc = 0xFFFF;

    for (int pos = 0; pos < len; pos++) {
        crc ^= (int)data[pos];         // XOR byte into least sig. byte of crc
        for (int i = 8; i != 0; i--) {  // Loop over each bit
            if ((crc & 0x0001) != 0) {  // If the LSB is set
                crc >>= 1;              // Shift right and XOR 0xA001
                crc ^= 0xA001;
            } else  crc >>= 1;          // Else LSB is not set so Just shift right
        }
    }
    return crc; // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
}

//macros to compact formatstrings
#define LD(label)   #label ":%d,"
#define LF(label)   #label ":%.1f,"
#define LS          "%s"
#define LEnd        "\"X\":0}"
//macros to compact values syntax
#define D2(addr)                                     ((data[addr+0]<<8)+data[addr+1])
#define D4(addr) ((data[addr]<<24)+(data[addr+1]<<16)+(data[addr+2]<<8)+data[addr+3])
#define F2(addr) ((( float)D2(addr))/10.0) /* for T & V field */
#define F4(addr) (((double)D4(addr))/10.0) /* for P & E field */
#define T4(addr) (((double)D4(addr))/ 2.0) /* for timeT field */
#define MSGSIZE 128

typedef enum {
    ENDPOINT_INDEX=0,
    ENDPOINT_VALUE_1,
    ENDPOINT_VALUE_2,
    ENDPOINT_VALUE_3,
    ENDPOINT_VALUE_4,
    ENDPOINT_VALUE_5,
    ENDPOINT_VALUE_6,
    ENDPOINT_VALUE_7,
    ENDPOINT_VALUE_8,
    ENDPOINT_VALUE_9,
    ENDPOINT_VALUE_10,
    ENDPOINT_VALUE_11,
    ENDPOINT_VALUE_12,
    ENDPOINT_VALUE_13,
    ENDPOINT_VALUE_14,
    ENDPOINT_VALUE_15,
    ENDPOINT_VALUE_16,
    ENDPOINT_VALUE_17,
    ENDPOINT_VALUE_18,
    ENDPOINT_VALUE_19,
    ENDPOINT_VALUE_20,
    ENDPOINT_UNKNOWN,
} endpoint_t;

uint32_t buffer[MSGSIZE],idx=0;
char json[ENDPOINT_UNKNOWN][256],json2[128]=LEnd;

uint32_t   EacT,EacT_old=0,Pac,Efine,Eplus=0; //Efine in 0.1Wh, others in 0.1kWh 
TickType_t timeT,timeT_old=0; //time in ticks of 10ms 
int        query_type=0; // 0 will mean register 0x2d-0x39 and 1 will mean register 0x00 till 0x2c

TimerHandle_t xTimerP, xTimerS;
void serial_sender( TimerHandle_t xTimer ) {
    // collect 58 (0x3a) words for a single request does NOT work since it provokes an error 01 84 01 82 c0 (not supported by server)
    if (query_type) { //0x00 - 0x2c
        // 01040000002d3017
        uart_putc(1,1);uart_putc(1,4);uart_putc(1,0);uart_putc(1,0);uart_putc(1,0);uart_putc(1,0x2d);uart_putc(1,0x30);uart_putc(1,0x17);
        uart_flush_txfifo(1);
        query_type=0;
    } else { //0x2d-0x39
        // 0104002d000da1c6
        uart_putc(1,1);uart_putc(1,4);uart_putc(1,0);uart_putc(1,0x2d);uart_putc(1,0);uart_putc(1,0x0d);uart_putc(1,0xa1);uart_putc(1,0xc6);
        uart_flush_txfifo(1);
        query_type=1;
    }
}


void serial_parser( TimerHandle_t xTimer ) {
    uint32_t *data=NULL,message[MSGSIZE],msg_crc,datacrc;
    int i,msg_len=idx;
    int address,command,datalen;
    for (i=0; i<msg_len; i++) message[i]=buffer[i]; idx=0; //capture buffer and start over
    //process message
    for (i=0; i<msg_len; i++) printf("%02x",message[i]); printf("\n");
    if (msg_len>8) { // only reponses
        datacrc=message[msg_len-1]*256+message[msg_len-2]; //swap bytes
        msg_crc=calccrc(message,msg_len-2); //calc CRC
        if (msg_crc==datacrc) { // compare and process good CRC
            address=message[0];
            command=message[1];
            datalen=message[2];
            data=message+3;
            if (command==4) { //Ignore the Holding registers
                if (data[0]==0) { //registers 00-2c
                    for (i=ENDPOINT_UNKNOWN-1;i>0;i--) strcpy(json[i],json[i-1]);
                    if (D2(0)==1) { //Status is Active
                        timeT=xTaskGetTickCount(); Pac=D4(22); EacT=D4(56);
                        if (timeT_old) { //we need to initialze timeT_old first
                            if (EacT>EacT_old) {Eplus=0; EacT_old=EacT;}
                            Eplus+=Pac*(timeT-timeT_old);
                            if (Eplus>360000000) {printf("Eplus=%d!\n",Eplus); Eplus=360000000;}
                        }
                        timeT_old=timeT;
                        Efine=EacT*1000+Eplus/360000;
                        sprintf(json[0],"{" \
                        LD("Status")LF("Ppv")LF("Vpv1")LF("Ppv1")LF("Vpv2")LF("Ppv2")LF("Pac")LF("Vac")LF("EacT")LF("timeT")LF("Temp")LF("Vint")LD("Efine")  LS, \
                               D2(0),   F4(2),    F2(6),   F4(10),   F2(14),   F4(18),  F4(22),  F2(28),   F4(56),    T4(60),   F2(64),   F2(84),   Efine  ,     \
                        json2);
                    } else { //Status is Inactive or Fault
                        sprintf(json[0],"{" LD("Status")LF("EacT")LF("Vac") LS, \
                                                D2(0),   F4(56) ,   F2(28),     \
                        json2);
                    }
                    printf("addr=%x,cmd=%x,len=%d,json=%s\n",address,command,datalen,json[0]);
                } else { //    else registers 2d-59
                    sprintf(json2, \
                    LF("Epv1t")LF("Epv2t")LF("EpvT")LEnd, \
                          F4(6),    F4(14),   F4(22)      \
                    );
                }
            }
        } else { // bad CRC
            printf("calccrc=%04x not equal to datacrc=%04x\n",msg_crc,datacrc);
        }
    }
}
/* one example from version 0.3.8 with errors for EacT, tT, Temp and Vint
01045a4e20000000010000001f0001796d00000020000187010003006e000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000004934
                                                               28   30  32  34  36  38  40  42  44  46  48  50  52  54   56       60       64   66  68  70  72  74  76  78  80  82   84
01045a000100005c4e0668004800002e32064f004900002e1c00005a771387 09ae 005f00005a77000000000000000000000000000000000000003e 0002f8d7 0be7ef67 0220 000000000000000000000000000000000000 0ede 00000000b3c3
addr=1,cmd=4,len=90,json={"Status":1,"Ppv":2363.0,"Vpv1":164.0,"Ppv1":1182.6,"Vpv2":161.5,"Ppv2":1180.4,"Pac":2315.9,"Vac":247.8,"EacT":0.0,"tT":31.0,"Temp":0.2,"Vint":0.0,"Epv1t":3.1,"Epv2t":3.2,"EpvT":19671.8,"X":0}
*/

/* example of over-voltage results in status=3
01041a4e2000000000000000240001f9590000002500020aea00040443dd9e
01045a0003000000000830000000000000082a000000000000000000001385 09c2 00010000000000000000000000000000000000000000000000480003fa6f0fe3f62301d4000000000000000009de00000000001e0000082d0000000035e0
addr=1,cmd=4,len=90,json={"Status":3,"EacT":26071.9,"Epv1t":3.6,"Epv2t":3.7,"EpvT":26323.5,"X":0}
should report 249.8 Vac
*/

void capture_task(void *arg) {
    while(1) {
        int c=getchar(); buffer[idx]=c; //this cannot be collapsed, else first byte missing ?!?!?!?!?
        if (idx<MSGSIZE-1) idx++;
        xTimerReset(xTimerP, 0); //as long as new input keeps arriving within 30ms, do not parse yet
    }
}


typedef struct _client {
    int fd;
    http_parser parser;
    endpoint_t endpoint;
    uint8_t *body;
    size_t body_length;
} client_t;

static client_t *client_new() {
    client_t *client = malloc(sizeof(client_t));
    memset(client, 0, sizeof(client_t));

    http_parser_init(&client->parser, HTTP_REQUEST);
    client->parser.data = client;

    return client;
}

static void client_free(client_t *client) {
    if (client->body)
        free(client->body);

    free(client);
}

static void client_send(client_t *client, const char *payload, size_t payload_size) {
    lwip_write(client->fd, payload, payload_size);
}

static void client_send_chunk(client_t *client, const char *payload) {
    int len = strlen(payload);
    char buffer[10];
    int buffer_len = snprintf(buffer, sizeof(buffer), "%x\r\n", len);
    client_send(client, buffer, buffer_len);
    client_send(client, payload, len);
    client_send(client, "\r\n", 2);
}

static void client_send_redirect(client_t *client, int code, const char *redirect_url) {
    printf("Redirecting to %s\n", redirect_url);
    char buffer[128];
    size_t len = snprintf(buffer, sizeof(buffer), "HTTP/1.1 %d \r\nLocation: %s\r\nContent-Length: 0\r\nConnection: close\r\n\r\n", code, redirect_url);
    client_send(client, buffer, len);
}

static void my_server_on_value(client_t *client, int endpoint) {
    static const char http_prologue[] =
        "HTTP/1.1 200 \r\n"
        "Content-Type: text/html; charset=utf-8\r\n"
        "Cache-Control: no-store\r\n"
        "Transfer-Encoding: chunked\r\n"
        "Connection: close\r\n"
        "\r\n";

    client_send(client, http_prologue, sizeof(http_prologue)-1);
    client_send_chunk(client, json[endpoint]);
    client_send_chunk(client, "");
}

static int my_server_on_url(http_parser *parser, const char *data, size_t length) {
    client_t *client = (client_t*) parser->data;
    char endpoint[3];

    client->endpoint = ENDPOINT_UNKNOWN;
    if (parser->method == HTTP_GET) {
        if (!strncmp(data, "/", length)) {
            client->endpoint = ENDPOINT_INDEX;
        }
        for (int i=0;i<ENDPOINT_UNKNOWN;i++) {
            sprintf(endpoint, "/%d",i);
            if (!strncmp(data, endpoint , length)) {
                client->endpoint = i;
            }
        }
    }

    if (client->endpoint == ENDPOINT_UNKNOWN) {
        char *url = strndup(data, length);
        printf("Unknown endpoint: %s %s\n", http_method_str(parser->method), url);
        free(url);
    }

    return 0;
}

static int my_server_on_message_complete(http_parser *parser) {
    client_t *client = parser->data;

    if (client->endpoint != ENDPOINT_UNKNOWN) {
        my_server_on_value(client, client->endpoint);
    } else {
        printf("Unknown endpoint\n");
        client_send_redirect(client, 302, "http://192.168.178.115/"); //TODO: make this dynamic
    }

    if (client->body) {
        free(client->body);
        client->body = NULL;
        client->body_length = 0;
    }

    return 0;
}

static http_parser_settings my_http_parser_settings = {
    .on_url = my_server_on_url,
    .on_message_complete = my_server_on_message_complete,
};

static void http_task(void *arg) {
    printf("Starting HTTP server\n");

    struct sockaddr_in serv_addr;
    int listenfd = socket(AF_INET, SOCK_STREAM, 0);
    memset(&serv_addr, '0', sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(WIFI_CONFIG_SERVER_PORT);
    int flags;
    if ((flags = lwip_fcntl(listenfd, F_GETFL, 0)) < 0) {
        printf("Failed to get HTTP socket flags\n");
        lwip_close(listenfd);
        vTaskDelete(NULL);
        return;
    };
    if (lwip_fcntl(listenfd, F_SETFL, flags | O_NONBLOCK) < 0) {
        printf("Failed to set HTTP socket flags\n");
        lwip_close(listenfd);
        vTaskDelete(NULL);
        return;
    }
    bind(listenfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
    listen(listenfd, 2);

    char data[128];

    while (1) {
        int fd = accept(listenfd, (struct sockaddr *)NULL, (socklen_t *)NULL);
        if (fd < 0) {
            vTaskDelay(500 / portTICK_PERIOD_MS);
            continue;
        }

        const struct timeval timeout = { 2, 0 }; /* 2 second timeout */
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        client_t *client = client_new();
        client->fd = fd;

        for (;;) {
            int data_len = lwip_read(client->fd, data, sizeof(data));
            //printf("lwip_read: %d\n", data_len);

            if (data_len > 0) {
                http_parser_execute(
                    &client->parser, &my_http_parser_settings,
                    data, data_len
                );
            } else {
                break;
            }
        }

        //printf("Client disconnected\n");

        lwip_close(client->fd);
        client_free(client);
    }
}


void user_init(void) {
    uart_set_baud(0, 9600);
    uart_set_baud(1, 9600);
    strcpy(json[0],"{\"Status\":3}");
    gpio_set_iomux_function(2, IOMUX_GPIO2_FUNC_UART1_TXD); //Activate UART for GPIO2
    udplog_init(3);
    UDPLUS("\n\n\nGrowatt-ModBus " VERSION "\n");
    xTaskCreate(capture_task,"capture", 512, NULL, 1, NULL);
    xTimerP=xTimerCreate( "parser",   30/portTICK_PERIOD_MS, pdFALSE, NULL, serial_parser);
    xTimerS=xTimerCreate( "sender",  500/portTICK_PERIOD_MS, pdTRUE,  NULL, serial_sender);
    xTimerStart(xTimerS,0);
    xTaskCreate(http_task, "HTTP", 512, NULL, 1, NULL);
}
