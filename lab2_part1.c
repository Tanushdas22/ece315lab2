/*
 * Lab 2, Part 2 - Adding UART Interface to Lab 1 Part 3
 *
 * ECE-315 - Computer Interfacing
 * Extends Lab 1 Part 3 with UART commands to control:
 *   - RGB LED brightness and color
 *   - Seven-segment display
 *
 * Commands (simple format):
 *   B5   = brightness 5 (0-20)
 *   C2   = color 2 (0=RED, 1=GREEN, 2=BLUE, 3=MAGENTA)
 *   D42  = display "42" on SSD (left=4, right=2)
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "xparameters.h"
#include "xgpio.h"
#include "xuartps.h"
#include "xstatus.h"
#include "pmodkypd.h"
#include "rgb_led.h"

#include <string.h>
#include <stdio.h>

// Device IDs (from xparameters - same as Lab 1)
#define KYPD_DEVICE_ID        XPAR_GPIO_KYPD_BASEADDR
#define SSD_DEVICE_ID         XPAR_GPIO_SSD_BASEADDR
#define RGB_LED_DEVICE_ID     XPAR_GPIO_LEDS_BASEADDR
#define PUSH_BUTTON_DEVICE_ID XPAR_GPIO_INPUTS_BASEADDR
#define UART_BASEADDR         XPAR_UART1_BASEADDR

#define POLL_DELAY_MS     50
#define RX_QUEUE_LEN      256
#define TX_QUEUE_LEN      256
#define CMD_BUF_LEN       32

// Command types
typedef enum { CMD_RGB_BRIGHT, CMD_RGB_COLOR, CMD_SSD_DISPLAY } uart_cmd_type_t;

typedef struct {
    uart_cmd_type_t type;
    u32 param1;
    u32 param2;
} uart_cmd_t;

// Lab 1 types
typedef struct {
    u8 current_key;
    u8 previous_key;
} keyValues_t;

// Devices
PmodKYPD KYPDInst;
XGpio SSDInst, RGB_LEDInst, PUSH_BUTTONInst;
static XUartPs UartPs;

// Queues
static QueueHandle_t q_rx_byte = NULL;
static QueueHandle_t q_tx = NULL;
static QueueHandle_t xKeypadDisplayQueue = NULL;
static QueueHandle_t xButtonsRGBQueue = NULL;
static QueueHandle_t q_uart_to_rgb = NULL;
static QueueHandle_t q_uart_to_ssd = NULL;

// Prototypes
static void vKeypadTask(void *pvParameters);
static void vRGBTask(void *pvParameters);
static void vButtonsTask(void *pvParameters);
static void vDisplayTask(void *pvParameters);
static void vUART_RX_Task(void *pvParameters);
static void vUART_TX_Task(void *pvParameters);
static void vUART_ParseTask(void *pvParameters);
void InitializeKeypad(void);
void InitializeSSD(void);
void InitializeRGB_LED(void);
void InitializePush_Button(void);
static void uart_init(void);
static int uart_poll_rx(uint8_t *b);
static int uart_tx_byte_nonblocking(uint8_t b);
void receive_string(char *buf, size_t buf_len);
void print_string(const char *str);
u32 SSD_decode(u8 key_value, u8 cathode);
static void parse_and_send(const char *buf);

// Xilinx UART FIFO offset (from xuartps_hw.h)
#ifndef XUARTPS_FIFO_OFFSET
#define XUARTPS_FIFO_OFFSET  0x30
#endif

int main(void)
{
    InitializeKeypad();
    InitializeSSD();
    InitializeRGB_LED();
    InitializePush_Button();
    uart_init();

    xKeypadDisplayQueue = xQueueCreate(1, sizeof(keyValues_t));
    xButtonsRGBQueue    = xQueueCreate(1, sizeof(u32));
    q_rx_byte           = xQueueCreate(RX_QUEUE_LEN, sizeof(uint8_t));
    q_tx                = xQueueCreate(TX_QUEUE_LEN, sizeof(char));
    q_uart_to_rgb       = xQueueCreate(4, sizeof(uart_cmd_t));
    q_uart_to_ssd       = xQueueCreate(4, sizeof(uart_cmd_t));

    xTaskCreate(vUART_RX_Task,   "UART_RX",   512, NULL, 3, NULL);
    xTaskCreate(vUART_TX_Task,   "UART_TX",   512, NULL, 3, NULL);
    xTaskCreate(vUART_ParseTask, "UART_Parse", 1024, NULL, 2, NULL);
    xTaskCreate(vKeypadTask,     "Keypad",    configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vButtonsTask,    "Buttons",   configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vRGBTask,        "RGB",       configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vDisplayTask,    "Display",   configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);

    vTaskStartScheduler();
    while (1);
    return 0;
}

// ---- UART ---- (from Lab 2 Part 1)
static void uart_init(void)
{
    XUartPs_Config *cfg = XUartPs_LookupConfig(UART_BASEADDR);
    if (!cfg) while (1) {}
    if (XUartPs_CfgInitialize(&UartPs, cfg, cfg->BaseAddress) != XST_SUCCESS)
        while (1) {}
    XUartPs_SetBaudRate(&UartPs, 115200);
}

static int uart_poll_rx(uint8_t *b)
{
    if (XUartPs_IsReceiveData(UartPs.Config.BaseAddress)) {
        *b = XUartPs_ReadReg(UartPs.Config.BaseAddress, XUARTPS_FIFO_OFFSET);
        return 1;
    }
    return 0;
}

static int uart_tx_byte_nonblocking(uint8_t b)
{
    if (XUartPs_IsTransmitFull(UartPs.Config.BaseAddress)) return 0;
    XUartPs_WriteReg(UartPs.Config.BaseAddress, XUARTPS_FIFO_OFFSET, b);
    return 1;
}

static void vUART_RX_Task(void *pvParameters)
{
    uint8_t byte;
    for (;;) {
        if (uart_poll_rx(&byte))
            xQueueSend(q_rx_byte, &byte, 0);
        else
            vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void vUART_TX_Task(void *pvParameters)
{
    char c;
    static char pending = '\0';
    int has_pending = 0;
    for (;;) {
        if (!has_pending && xQueueReceive(q_tx, &c, 0) == pdTRUE) {
            pending = c;
            has_pending = 1;
        }
        if (has_pending && uart_tx_byte_nonblocking((uint8_t)pending))
            has_pending = 0;
        vTaskDelay(pdMS_TO_TICKS(POLL_DELAY_MS));
    }
}

void receive_string(char *buf, size_t buf_len)
{
    uint8_t recvd;
    size_t idx = 0;
    buf[0] = '\0';
    while (1) {
        if (xQueueReceive(q_rx_byte, &recvd, 0) == pdTRUE) {
            if (recvd == '\r') { buf[idx] = '\0'; return; }
            if (idx < buf_len - 1) { buf[idx++] = (char)recvd; buf[idx] = '\0'; }
        } else {
            vTaskDelay(pdMS_TO_TICKS(POLL_DELAY_MS));
        }
    }
}

void print_string(const char *str)
{
    while (*str) {
        char c = *str++;
        xQueueSend(q_tx, &c, portMAX_DELAY);
    }
}

// Parse UART string and send to RGB or SSD queue
static void parse_and_send(const char *buf)
{
    uart_cmd_t cmd;
    int a, b;

    if (buf[0] == 'B' && sscanf(buf + 1, "%d", &a) == 1) {
        cmd.type = CMD_RGB_BRIGHT;
        cmd.param1 = (u32)(a < 0 ? 0 : (a > 20 ? 20 : a));
        xQueueOverwrite(q_uart_to_rgb, &cmd);
    } else if (buf[0] == 'C' && sscanf(buf + 1, "%d", &a) == 1) {
        cmd.type = CMD_RGB_COLOR;
        cmd.param1 = (u32)(a & 0x07);  // 0-7
        xQueueOverwrite(q_uart_to_rgb, &cmd);
    } else if (buf[0] == 'D' && sscanf(buf + 1, "%1d%1d", &a, &b) == 2) {
        cmd.type = CMD_SSD_DISPLAY;
        cmd.param1 = (u32)('0' + (a % 10));  // left digit ASCII
        cmd.param2 = (u32)('0' + (b % 10));  // right digit ASCII
        xQueueOverwrite(q_uart_to_ssd, &cmd);
    }
}

static void vUART_ParseTask(void *pvParameters)
{
    char buf[CMD_BUF_LEN];
    print_string("Lab 2 Part 2: UART commands B<n> C<n> D<xy>\r\n");

    for (;;) {
        receive_string(buf, sizeof(buf));
        if (strlen(buf) > 0)
            parse_and_send(buf);
    }
}

// ---- Lab 1 peripherals ----
void InitializeSSD(void)
{
    XGpio_Initialize(&SSDInst, SSD_DEVICE_ID);
    XGpio_SetDataDirection(&SSDInst, 1, 0);
}

void InitializeRGB_LED(void)
{
    XGpio_Initialize(&RGB_LEDInst, RGB_LED_DEVICE_ID);
    XGpio_SetDataDirection(&RGB_LEDInst, RGB_CHANNEL, 0);
}

void InitializePush_Button(void)
{
    XGpio_Initialize(&PUSH_BUTTONInst, PUSH_BUTTON_DEVICE_ID);
    XGpio_SetDataDirection(&PUSH_BUTTONInst, 1, 0xFF);
}

void InitializeKeypad(void)
{
    KYPD_begin(&KYPDInst, KYPD_DEVICE_ID);
    KYPD_loadKeyTable(&KYPDInst, (u8*)"0FED789C456B123A");
}

// ---- Tasks ----
static void vButtonsTask(void *pvParameters)
{
    u32 button_value;
    static u32 previous_val = 0;
    while (1) {
        button_value = XGpio_DiscreteRead(&PUSH_BUTTONInst, 1);
        if (button_value != previous_val) {
            xQueueOverwrite(xButtonsRGBQueue, &button_value);
            previous_val = button_value;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static const u32 color_table[] = {
    RGB_RED, RGB_GREEN, RGB_BLUE, RGB_MAGENTA, RGB_CYAN, RGB_YELLOW, RGB_WHITE
};

static void vRGBTask(void *pvParameters)
{
    const TickType_t xPeriod = 21;
    TickType_t xOnDelay = 10;
    u32 color = RGB_MAGENTA;
    u32 button_value;

    while (1) {
        uart_cmd_t cmd;
        if (xQueueReceive(q_uart_to_rgb, &cmd, 0) == pdTRUE) {
            if (cmd.type == CMD_RGB_BRIGHT)
                xOnDelay = cmd.param1 < xPeriod ? cmd.param1 : xPeriod - 1;
            else if (cmd.type == CMD_RGB_COLOR && cmd.param1 < 7)
                color = color_table[cmd.param1];
        }

        if (xQueueReceive(xButtonsRGBQueue, &button_value, 0) == pdTRUE) {
            if (button_value == 0x08 && xOnDelay < xPeriod) xOnDelay++;
            else if (button_value == 0x01 && xOnDelay > 0) xOnDelay--;
        }

        TickType_t xOffDelay = xPeriod - xOnDelay;
        XGpio_DiscreteWrite(&RGB_LEDInst, RGB_CHANNEL, xOnDelay > 0 ? color : 0);
        vTaskDelay(xOnDelay);
        XGpio_DiscreteWrite(&RGB_LEDInst, RGB_CHANNEL, 0);
        vTaskDelay(xOffDelay);
    }
}

static void vDisplayTask(void *pvParameters)
{
    keyValues_t keypad_vals = { 'x', 'x' };
    keyValues_t received_vals;
    uart_cmd_t ssd_cmd;
    u8 left = 'x', right = 'x';
    const TickType_t xDelay = pdMS_TO_TICKS(10);

    while (1) {
        if (xQueueReceive(q_uart_to_ssd, &ssd_cmd, 0) == pdTRUE) {
            left = (u8)ssd_cmd.param1;
            right = (u8)ssd_cmd.param2;
        }
        if (xQueueReceive(xKeypadDisplayQueue, &received_vals, 0) == pdTRUE) {
            keypad_vals = received_vals;
            left = keypad_vals.previous_key;
            right = keypad_vals.current_key;
        }

        u32 ssd_value = SSD_decode(left, 0);
        XGpio_DiscreteWrite(&SSDInst, 1, ssd_value);
        vTaskDelay(xDelay);

        ssd_value = SSD_decode(right, 1);
        XGpio_DiscreteWrite(&SSDInst, 1, ssd_value);
        vTaskDelay(xDelay);
    }
}

static void vKeypadTask(void *pvParameters)
{
    u16 keystate;
    XStatus status, prev_status = KYPD_NO_KEY;
    u8 new_key, current_key = 'x', previous_key = 'x';
    keyValues_t keypad_vals;
    const TickType_t xDelay = pdMS_TO_TICKS(50);

    while (1) {
        keystate = KYPD_getKeyStates(&KYPDInst);
        status = KYPD_getKeyPressed(&KYPDInst, keystate, &new_key);

        if (status == KYPD_SINGLE_KEY && prev_status == KYPD_NO_KEY) {
            previous_key = current_key;
            current_key = new_key;
            keypad_vals.current_key = current_key;
            keypad_vals.previous_key = previous_key;
            xQueueOverwrite(xKeypadDisplayQueue, &keypad_vals);
        }
        prev_status = status;
        vTaskDelay(xDelay);
    }
}

u32 SSD_decode(u8 key_value, u8 cathode)
{
    u32 result;
    switch (key_value) {
        case 48: result = 0b00111111; break;
        case 49: result = 0b00110000; break;
        case 50: result = 0b01011011; break;
        case 51: result = 0b01111001; break;
        case 52: result = 0b01110100; break;
        case 53: result = 0b01101101; break;
        case 54: result = 0b01101111; break;
        case 55: result = 0b00111000; break;
        case 56: result = 0b01111111; break;
        case 57: result = 0b01111100; break;
        case 65: result = 0b01111110; break;
        case 66: result = 0b01100111; break;
        case 67: result = 0b00001111; break;
        case 68: result = 0b01110011; break;
        case 69: result = 0b01001111; break;
        case 70: result = 0b01001110; break;
        default: result = 0b00000000; break;
    }
    return cathode ? (result | 0b10000000) : result;
}