/*
 *	Lab 2: Part 2: UART in Interrupt Mode
 *
 *	ECE 315		: Computer Interfacing
 *  Created on	: July 27, 2021
 *  Author	: Shyama M. Gandhi, Mazen Elbaz
 *  Modified by : Antonio Andara
 *  Modified on	: January 24, 2023
 *
 *     ------------------------------------------------------------------------------------------------------------------------------
 *
 *     This is the main file that uses the Xilinx the "uart_driver.h" header file.
 *     The final objective is to implement as interrupt mode receive side and polling mode on transmit side of the UART.
 *
 *     Inside the interrupt service routine in the uart_Driver.h file, the received bytes will be written at the back of receive queue. (xRxQueue)
 *     The bytes to be transmitted later on to UART console, will be read from the front of the front of the transmit queue. (xTxQueue)
 *
 *
 *     Overview of uart_driver.h file:
 *     This driver file is responsible for Initializing the UART, setting up the Interrupt System, Implements the ISR when a receive or sent event is triggered via UART,
 *     and implementing the four driver functions myReceiveData(), MyReceiveByte(), myTransmitFull() and MySendByte().
 *     You are suppose to add the code at the commented places in this uart_driver.h file.
 *     Please read the explanations carefully for your understanding.
 *
 *     Overview of this main source .c file:
 *     The driver file does all the background work and the queue implementation, which is hidden from the user in this driver file. Is the four functions myReceiveData(), MyReceiveByte(), myTransmitFull() and MySendByte(),
 *     that will be used to play around in this file.
 *     This file will also have the logic responsible for detecting the "\r#\r" and "\r%\r" termination sequence for displaying the interrupt statistics and resetting the variables respectively.
 *     It is in this file where students will enter the logic to change the letter capitalization.
 *     Please enter your code at the commented sections in this file too.
 *
 *     ------------------------------------------------------------------------------------------------------------------------------
 */


#include "stdio.h"
#include "ctype.h"
#include "xgpio.h"
#include "xil_printf.h"
#include "xil_types.h"
// #include "xtime_l.h"

// UART driver header file
#include "uart_driver.h"

// Devices
#define SSD_DEVICE_ID   XPAR_GPIO_SSD_BASEADDR
#define BTN_DEVICE_ID	XPAR_GPIO_INPUTS_BASEADDR
#define LEDS_DEVICE_ID	XPAR_GPIO_LEDS_BASEADDR

// Device channels
#define SSD_CHANNEL		1
#define BTN_CHANNEL		1

// Other Useful Macros
#define BTN0 1
#define BTN1 2
#define BTN2 4
#define BTN3 8
#define CHAR_ESC				0x23	// '#' character is used as termination sequence
#define CHAR_CARRIAGE_RETURN	0x0D	// '\r' character is used in the termination sequence
#define SEQUENCE_LENGTH 3 				//Rolling buffer sequence length
//#define configUSE_IDLE_HOOK 1


// Device declaration
XGpio SSDInst, btnInst, swInst, ledsInst;


// Function prototypes
void vBufferReceiveTask(void *p);
void vBufferSendTask(void *p);


u8 checkBufferSequence(u8 rollingBuffer[], char* sequence);
void updateRollingBuffer(u8 rollingBuffer[], u8 receivedByte);

u32 sevenSegDecode(int digit, u8 cathode);


TaskHandle_t task_receiveuarthandle = NULL;
TaskHandle_t task_transmituarthandle = NULL;

int main()
{
    // Initialization
	int status;

	// SSD
	status = XGpio_Initialize(&SSDInst, SSD_DEVICE_ID);
	if(status != XST_SUCCESS){
		xil_printf("GPIO Initialization for SSD failed.\r\n");
		return XST_FAILURE;
	}

	// Buttons
	status = XGpio_Initialize(&btnInst, BTN_DEVICE_ID);
	if(status != XST_SUCCESS){
		xil_printf("GPIO Initialization for buttons failed.\r\n");
		return XST_FAILURE;
	}

	// LEDS
	status = XGpio_Initialize(&ledsInst, LEDS_DEVICE_ID);
	if(status != XST_SUCCESS){
		xil_printf("GPIO Initialization for LEDS failed.\r\n");
		return XST_FAILURE;
	}

	// switches
	status = XGpio_Initialize(&swInst, BTN_DEVICE_ID);
    if(status != XST_SUCCESS){
        xil_printf("GPIO Initialization for switches failed.\r\n");
        return XST_FAILURE;
    }

    // UART
    status = initializeUART();
	if (status != XST_SUCCESS){
		xil_printf("UART Initialization failed\n");
	}

	// Device data direction: 0 for output 1 for input
	XGpio_SetDataDirection(&SSDInst, SSD_CHANNEL, 0x00);
	XGpio_SetDataDirection(&btnInst, BTN_CHANNEL, 0x0F);
	XGpio_SetDataDirection(&ledsInst, 1, 0x00);
	XGpio_SetDataDirection(&swInst, 2, 0x00);



    // FreeRTOS Tasks
	xTaskCreate( vBufferReceiveTask
			   , "uart_receive_task"
			   , 1024
			   , (void*)0
			   , tskIDLE_PRIORITY+1
			   , &task_receiveuarthandle
			   );

	xTaskCreate( vBufferSendTask
			   , "uart_transmit_task"
			   , 1024
			   , (void*)0
			   , tskIDLE_PRIORITY+1
			   , &task_transmituarthandle
			   );

    // Queues
	xTxQueue = xQueueCreate( SIZE_OF_QUEUE, sizeof(u8));
	xRxQueue = xQueueCreate( SIZE_OF_QUEUE, sizeof(u8));

    // assertions
    configASSERT(vBufferReceiveTask);
    configASSERT(vBufferSendTask);
    configASSERT(xTxQueue);
	configASSERT(xTxQueue);
	configASSERT(xRxQueue);

    // initializing globals
    countRxIrq = 0;
	countTxIrq = 0;
    byteCount = 0;

	xil_printf(
	    "\n====== App Ready ======\n"
	    "Instructions:\n"
	    "- Send data via serial terminal. Press Enter to swap case of letters.\n"
	    "  (Numbers/symbols unchanged).\n"
	    "- To view interrupt count, type: '\\r#\\r'\n"
	    "- To reset interrupt count, type: '\\r%\\r'\n"
	    "- BTN0: Display Rx interrupt count on SSD.\n"
	    "- BTN1: Display Tx interrupt count on SSD.\n"
		"- BTN2: Display byte count on SSD.\n"
		"- BTN3: Reset interrupt and byte count.\n"
	    "========================\n\n"
	);

	vTaskStartScheduler();

    while(1);

	return 0;

}


void vBufferReceiveTask(void *p)
{
    int status;
    u8 pcString, cathode = 0;
    char formattedChar;
    u32 ssd_value = 1;
    unsigned int sendMethod = 0, swVal, buttonVal = 0;
    u8 rollingBuffer[SEQUENCE_LENGTH] = {0, 0, 0};

    /* Local transmit buffer for string mode */
    char txBuffer[128];
    int txIndex = 0;

    status = setupInterruptSystem(&InterruptController, &UART, UART_INT_IRQ_ID);
    if (status != XST_SUCCESS){
        xil_printf("UART PS interrupt failed\n");
    }

    while (1)
    {
        /* Wait until RX queue has data */
        while (myReceiveData() == pdFALSE){
            swVal = XGpio_DiscreteRead(&swInst, 2);
            sendMethod = (swVal == 0) ? 0 : 1;

            buttonVal = XGpio_DiscreteRead(&btnInst, 1);

            if (buttonVal == BTN0){
                ssd_value = sevenSegDecode(countRxIrq, cathode);
            } else if (buttonVal == BTN1){
                ssd_value = sevenSegDecode(countTxIrq, cathode);
            } else if (buttonVal == BTN2){
                ssd_value = sevenSegDecode(byteCount, cathode);
            } else if (buttonVal == BTN3){
                byteCount  = 0;
                countRxIrq = 0;
                countTxIrq = 0;
                ssd_value  = sevenSegDecode(88, cathode);
            } else{
                ssd_value = sevenSegDecode(0, cathode);
            }

            XGpio_DiscreteWrite(&SSDInst, SSD_CHANNEL, ssd_value);
            cathode = !cathode;
        }

        pcString = myReceiveByte();
        formattedChar = (char)pcString;

        if (formattedChar >= 'A' && formattedChar <= 'Z'){
            formattedChar = tolower(formattedChar);
        } else if (formattedChar >= 'a' && formattedChar <= 'z'){
            formattedChar = toupper(formattedChar);
        }            

        updateRollingBuffer(rollingBuffer, pcString);

        if (checkBufferSequence(rollingBuffer, "\r#\r")){
            taskYIELD();
        } else if (checkBufferSequence(rollingBuffer, "\r%\r")){
            xil_printf("Byte Count and interrupt counters reset\n\n");
            byteCount  = 0;
            countRxIrq = 0;
            countTxIrq = 0;
        } else {
            if (sendMethod == 0){
                mySendByte((u8)formattedChar);
            } else {
                if (formattedChar != '\r'){
                    if (txIndex < sizeof(txBuffer) - 1){
                        txBuffer[txIndex++] = formattedChar;
                    }
                } else {
                    txBuffer[txIndex++] = '\r';
                    txBuffer[txIndex] = '\0';

                    mySendString(txBuffer);

                    txIndex = 0;
                }
            }
        }
    }
}


void vBufferSendTask(void *p)
{
    while(1){
        char countArray[10];
        char CountRxIrqArray[10];
        char CountTxIrqArray[10];

        // Convert counts to strings
        sprintf(countArray, "%d", byteCount);
        sprintf(CountRxIrqArray, "%d", countRxIrq);
        sprintf(CountTxIrqArray, "%d", countTxIrq);

        // Build message
        char message[256];
        sprintf(message,
            "\n\nBytes received:\t%s\n"
            "Rx interrupts:\t%s\n"
            "Tx interrupts:\t%s\n\n",
            countArray,
            CountRxIrqArray,
            CountTxIrqArray);

        mySendString(message);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


/*----------------------------------------------------------------------------*/

u8 checkBufferSequence(u8 rollingBuffer[], char* sequence)
{
/*************************** Enter your code here ****************************/
	if ( rollingBuffer[0] == sequence[0]
	  && rollingBuffer[1] == sequence[1]
	  && rollingBuffer[2] == sequence[2]
	   )
	{
		return 1;
	}
	return 0;
/*****************************************************************************/
}


void updateRollingBuffer(u8 rollingBuffer[], u8 receivedByte)
{
	for (int i = 0; i < SEQUENCE_LENGTH - 1; i++){
		rollingBuffer[i] = rollingBuffer[i+1];
    }

    rollingBuffer[SEQUENCE_LENGTH - 1] = receivedByte;
}


// This function translates int values to their binary representation
u32 sevenSegDecode(int countValue, u8 cathode)
{
    u32 result;
    int digit;

    // Convert countValue to two decimal digits
    if (cathode == 0) {
        // LSD: the least significant digit
        digit = countValue % 10;
    } else {
        // MSD: the most significant digit
        digit = (countValue / 10) % 10;
    }

    // Map the digit to the corresponding 7-segment display encoding
    switch(digit){
        case 0: result = 0b00111111; break; // 0
        case 1: result = 0b00110000; break; // 1
        case 2: result = 0b01011011; break; // 2
        case 3: result = 0b01111001; break; // 3
        case 4: result = 0b01110100; break; // 4
        case 5: result = 0b01101101; break; // 5
        case 6: result = 0b01101111; break; // 6
        case 7: result = 0b00111000; break; // 7
        case 8: result = 0b01111111; break; // 8
        case 9: result = 0b01111100; break; // 9
        default: result = 0b00000000; break; // Undefined, all segments are OFF
    }

    // The cathode logic remains unchanged
    if (cathode == 1) {
        return result;
    } else {
        return result | 0b10000000;
    }
}
