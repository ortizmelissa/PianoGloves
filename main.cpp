/*
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */

/**
 * @file
 * @brief This is the application entry point.
 * 			FreeRTOS and stdio printf is pre-configured to use uart0_min.h before main() enters.
 * 			@see L0_LowLevel/lpc_sys.h if you wish to override printf/scanf functions.
 *
 */
#include "FreeRTOS.h"
#include "command_handler.hpp"
#include "handlers.hpp"
#include "tasks.hpp"
#include "task.h"
#include "timers.h"
#include "uart2.hpp"
#include "uart3.hpp"
#include "i2c2.hpp"
#include "i2c_base.hpp"
#include "scheduler_task.hpp"
#include "semphr.h"
#include "io.hpp"
#include "eint.h"
#include "stdio.h"
#include "gpio.hpp"
#include "LPC17xx.h"
#include "examples/examples.hpp"
#include "../L4_IO/fat/disk/spi_flash.h"
#include "printf_lib.h"
#include "file_logger.h"
#include "event_groups.h"
#include "ff.h"
#include "storage.hpp"
#include "time.h"
#include "bio.h"
#include "src/FileSystemObject.hpp"
#include "pthread.h"

//CMPE 146 - Lab 1
class gpio_task : public scheduler_task
{
public:
	gpio_task(uint8_t priority): scheduler_task("gpio", 2000, priority)
	{ }
	bool run(void *p)
	{
//Set GPIO Output P0.0 pin to Logical HIGH - this enables me to supply power to my external LED.
		LPC_GPIO0->FIOPIN |= (1 << 0);
		LE.on(1);
		LD.setNumber(0);
//Reading Logical Value from GPIO Input P1.19
		if(LPC_GPIO1->FIOPIN & (1 << 19))
		{
			//P1.19 is Logic 1
			if(SW.getSwitch(1))
			{
				//Push Button Switch Trigger = TRUE
				LE.on(1);
				LD.setNumber(1);
				LPC_GPIO1->FIOSET = (1<<0);
				vTaskDelay(100); //Run at home with and without this to see different CPU Usage.
			}
		}
		else
		{
			//P1.19 is Logic 0
			LPC_GPIO1->FIOCLR = (1<<0);
			vTaskDelay(100);
			LE.on(1);
			LD.setNumber(0);
			vTaskDelay(100);
		}

		return true;
	}
	bool init(void)
	{
		printf("GPIO Init");

//Multiplexing Pins - Selecting GPIO COnfiguration (00)
		LPC_PINCON->PINSEL0 &= ~ (3<<0);
		LPC_PINCON->PINSEL1 &= ~ (3<<0);

//Setting Direction of GPIO Pins
		//Inputs - P1.19
		LPC_GPIO1->FIODIR &= ~(1 << 19);

		//Outputs P1.0, P0.0
		LPC_GPIO1->FIODIR |= (1<<0);
		LPC_GPIO0->FIODIR |= (1<<0);

		return true;
	}
};

//CMPE 146 - Lab 2
class spi_task : public scheduler_task
{
public:
	spi_task(uint8_t priority): scheduler_task("ssp", 2000, priority)
	{ }
	bool run(void *p)
	{
		int selection = 0;
		int SB1, SB2;

		 menu();
		scanf("%i", &selection);

		switch(selection)
		{
		case 1:
		       LPC_GPIO0->FIOCLR = (1<<6);
		        printf("\n 0\t %x",SSP_Byte_Transfer(0x9f));
		        printf("\n 1\t %x",SSP_Byte_Transfer(0x9f));
		        printf("\n 2\t %x",SSP_Byte_Transfer(0x00));
		        printf("\n 3\t %x",SSP_Byte_Transfer(0x00));
		        printf("\n 4\t %x",SSP_Byte_Transfer(0x00));
		        printf("\n 5\t %x",SSP_Byte_Transfer(0x00));
		        printf("\n");
		        LPC_GPIO0->FIOSET = (1<<6);
		     	break;

		case 2:
		       LPC_GPIO0->FIOCLR = (1<<6);
		        printf("\n 0\t %x",SSP_Byte_Transfer(0xd7));
		        SB1 = SSP_Byte_Transfer(0x00);
		        SB2 = SSP_Byte_Transfer(0x00);
		        printf("\n Status Byte 1: \t %x", SB1);
		        printf("\n Status Byte 2: \t %x", SB2);

		        if (SB1 == 172)
		        {
		        		printf("\n\n\tSTATUS BYTE 1: ");
		        		Decimal_To_Binary(SB1);
		        		printf("\n\n");
		        		printf("Bit 7: Ready/Busy Status --> Device is ready \n");
		        		printf("Bit 6: Compare Result --> Main Memory Page Data matched buffer data \n");
		        		printf("Bits [5:2]: Destiny Code --> 16 Mbit\n");
		        		printf("Bit 1: Sector Protection Status --> Sector Protection is disabled \n");
		        		printf("Bit 0: Page Size Configuration --> Devicve Configure is configured for standard DataFlash page size (528) \n\n");
		        }

		        if (SB2 == 136)
		        {
		        		printf("\n\n\tSTATUS BYTE 2: ");
		        		Decimal_To_Binary(SB2);
		        		printf("\n\n");
		        		printf("Bit 7: Ready/Busy Status --> Device is ready \n");
		        		printf("Bit 6: Reserved for Future Use --> Reserved for Future Use \n");
		        		printf("Bit 5: Erase/Program Error --> Erase or program operation was successful\n");
		        		printf("Bit 4: Reserved for Future Use --> Reserved for Future Use \n");
		        		printf("Bit 3: Sector Lockdown Enabled --> Sector Lockdown Command is Enabled \n");
		        		printf("Bit 2: Program Suspend Status (Buffer 2) --> No program operation has been suspended while using Buffer 2 \n");
		        		printf("Bit 1: Program Suspend Status (Buffer 1)--> No program operation has been suspended while using Buffer 1 \n");
		        		printf("Bit 0: Erase Suspend --> No sectors are erase suspended \n\n");
		        }
		        LPC_GPIO0->FIOSET = (1<<6);
			break;

		case 3:
			//Read Using 512 Byte Page Size
			unsigned char *c;
            LPC_GPIO0->FIOCLR = (1<<6);
            printf("Initialize the the Flash. . .\n\n ");
            flash_initialize();

    //        printf("FLASH PAGE COUNT: %i, \n",flash_get_page_count());
    //        printf("FLASH PAGE SIZE: %i, \n",flash_get_page_size());

            printf("OP CODE 0xD2: %i \n", flash_ioctl(0xd2, &c));
            printf("OP CODE 0x01: %i \n", flash_ioctl(0x01, &c));
            printf("OP CODE 0x55: %i \n", flash_ioctl(0x55, &c));
            printf("OP CODE 0xAA: %i \n", flash_ioctl(0xAA, &c));

            	printf("Reading Flash Sectors: \n");
                printf("%i \n", flash_read_sectors(c, 0, 0));
		       LPC_GPIO0->FIOSET = (1<<6);
			break;

		case 4:
			break;

		default:
			printf("Invalid Selection. Please try again.\n");
		}

		return true;
	}

	bool init(void)
	{
/*
 * Controlling SSP Peripheral.
 * Enabling SPI interface power/clock control bit.
 */
	    LPC_SC->PCONP |= (1<<10);

/*
  * PCLKSEL Controls Rate of clock signal.
  * 01 - PCLK_peripheral = CCLK
  */
	    LPC_SC->PCLKSEL0 &= ~(3<<20);
	    LPC_SC->PCLKSEL0 |= (1<<20);

//Clearing & Setting P0.7, P0.8, and P0.9 - bits [19:14]
	    LPC_PINCON->PINSEL0 &= ~((3<<18)|(3<<16)|(3<<14));
        LPC_PINCON->PINSEL0 |= ((2<<18)|(2<<16)|(2<<14));
/*
 * Short Cut to Clearing and Setting Bits.
 * LPC_PINCON->PINSEL0 &= ~(0x3F << 14);
 * LPC_PINCON->PINSEL0 |= (0x2A << 14);
 */


/*
  * CR0: Selects  Serial Clock Rate, Bus Type, and Data Size.
  * Here, we are enabling an 8-bit Data Transfer
 */
   		LPC_SSP1->CR0 = 0x07;
	    LPC_SSP1->CR1 = (1<<1);
	    LPC_SSP1->CPSR = 8; //SCK Frequency for Continuous Array Read(Low Frequency) is 33Mhx max. here we are setting it below it.

//GPIO INIT - Clear P0.6 - Setting Function to GPIO
	    LPC_PINCON->PINSEL0 &= ~(3 << 12);
	    LPC_GPIO0->FIODIR |= (1<<6);
	    LPC_GPIO0->FIOSET = (1<<6);
		return true;
	}
	uint8_t SSP_Byte_Transfer(uint8_t send_data)
	{
		LPC_SSP1->DR = send_data;
		while(LPC_SSP1->SR & (1<<4));

		return LPC_SSP1->DR;
	}
	void Decimal_To_Binary(int number)
	{
		int i = 0;
		for(i = 7; i>= 0; i--)
		{
			if ((number & (1<<i)) != 0)
				printf("1");
			else
				printf("0");
		}
	}
	void menu(void)
	{
		printf("\n***** SPI MENU *****\n");
		printf("1. Manufacture & Device ID\n");
		printf("2. Memory Status Register\n");
		printf("3. FAT File System\n");
		printf("4. Quit\n");

		printf("Please make a selection: ");
	}
};

//CMPE 146 - Lab 3
class uart_task : public scheduler_task
{
public:
	uart_task(uint8_t priority): scheduler_task("uart", 2000, priority)
	{ }
	bool run(void *p)
	{
		char send_data = 'i';
	//	char data_received = send_data;

		U2_Put_Char(send_data);
		printf("send =%c", send_data);

		//data_received = U3_Get_Char();
		//printf("data received = %c", data_received);

			return true;
	}

	bool init()
	{
		const uint32_t baud = 9600;
/*
 * UART CONFIG
 * Table 46 - PCONP
 * Table 40 - PCLK
 * Table 270/271 - BAUD RATE REGISTER
 * Table 280/281 - DLAB TX/RX REGISTER INFO
 *
 * The below configurations are for UART2.
 */
		LPC_SC->PCONP |= (0<< 3);
		LPC_SC->PCONP |= (1<< 24);
		//lpc_pconp(pconp_uart2, true); //does the same thing as above command.

		LPC_SC->PCLKSEL1 &= ~(3 << 16);
		LPC_SC->PCLKSEL1 |=  (1 << 16);   // Uart clock = CPU / 1
		//lpc_pclk(pclk_uart2,clkdiv_1);
		LPC_PINCON->PINSEL0 &= ~(0xF << 20); // Clear values
		LPC_PINCON->PINSEL0 |= (0x5 << 20);  // Set values for UART0 Rx/Tx

		/*
		 * Could also use PINSEL4 for UART2- Ask about that.
		 * Table 83!
		 */

		uint16_t dll = (48 * 1000 * 1000) / (16*baud);

		LPC_UART2->LCR = (1 << 7);	// Enable DLAB
		LPC_UART2->DLM = (dll >> 8);
		LPC_UART2->DLL = (dll >> 0);
		LPC_UART2->LCR = 3;         // 8-bit data

//UART3 CONFIG
		LPC_SC->PCONP |= (1<<25);
		/*
		 * Trying to Set and Clear Bits [3:0]
		 */
		LPC_PINCON->PINSEL0 &= (6 << 0);
		LPC_PINCON->PINSEL0 |= (0xA << 0);
		LPC_SC->PCLKSEL1 &= ~(3 << 18);
		LPC_SC->PCLKSEL1 |=  (1 << 18);

		LPC_UART3->LCR = (1 << 7);	// Enable DLAB
		LPC_UART3->DLM = (dll >> 8);
		LPC_UART3->DLL = (dll >> 0);
		LPC_UART3->LCR = 3;         // 8-bit data
		return true;
	}

/*
 * Polling - Waiting for Bit 5 to set. This lets us know that the data transmitted is complete, so that it doesn't get over written.
 */

	//change back to void after testing complete.
	char U2_Put_Char(char c)
	{
		LPC_UART2->THR = c;
		while(!(LPC_UART2->LSR & (1 << 5)))
			return (LPC_UART2->THR);
		return LPC_UART2->THR;
	}

	char U3_Get_Char(void)
	{
		while(1)
		{
			if(LPC_UART3->LSR & (1<<0))
			break;
		}
		char c = LPC_UART3->RBR;
		printf("%c", c);
		return c;
	}

	void menu(void)
	{
		printf("\n\n***** UART MENU *****\n");
		printf("1. Send Data\n");
		printf("2. Receive Data\n");
		printf("3. Quit\n");
		printf("\nPlease make a selection: ");
	}
};

//CMPE 146 - Lab 4 ENT
class interrupt_task : public scheduler_task
{
public:

	static void callback_function_port0()
	{
		u0_dbg_printf("Enter Callback Port 0! \n");
		LE.on(1);
		LE.on(3);
		LE.off(4);
//		xSemaphoreGive(binarySemaphoreSignal);
		return;
	}

	static void callback_function_port2()
	{
		u0_dbg_printf("Enter Callback Port 2! \n");
		LE.on(4);
		LE.on(2);
		LE.off(1);
		LE.off(3);
//		xSemaphoreGive(binarySemaphoreSignal);

		return;
	}
	//const unsigned char* c = (unsigned char*)("%i, %i", time, light);	//writes 10 things

	int portNumber = 0, pinNumber = 0, clockEdge = 0;

	interrupt_task (uint8_t priority): scheduler_task("eint", 2000, priority)
	{
	}

	bool run(void *p)
	{
		printf("\n \nEnter Port Number, Pin Number, and Clock Type: ");
		scanf("%i %i %i", &portNumber, &pinNumber, &clockEdge);

//EINT For Port 0
		if(portNumber == 0)
		{
			if(clockEdge) eint3_enable_port0(pinNumber, eint_falling_edge, callback_function_port0);
			else eint3_enable_port0(pinNumber, eint_rising_edge, callback_function_port0);
		}

//EINT For Port 2
		if(portNumber == 2)
		{
			if(clockEdge) eint3_enable_port2(pinNumber, eint_falling_edge, callback_function_port2);
			else eint3_enable_port2(pinNumber, eint_rising_edge, callback_function_port2);
		}


//		if(switch_pressed) printf("You pressed a switch!\n");

/*
		if(xSemaphoreTake(binarySemaphoreSignal, portMAX_DELAY))
		{
			printf("binarySemaphoreSignal taken!\n");
		}
*/

		return true;
	}

	bool init()
	{
		printf("Enter Interrupt Initialization\n");
		NVIC_EnableIRQ(EINT3_IRQn);
		return true;
	}
};

/*
 * CMPE 146 - Lab 6 RTOS
 * Observations
 * 1.) SAME PRIORITY - it is hard to tell which task is printing out first. But I do see that they alternate accordingly each time.
 *    They are partaking in time-slicing CPU resources.
 *
 * 2.) DIFFERENT PRIORITIES -
 *
 * What if you use ZERO block time while sending an item into the queue, will that make any difference?
 *
 *
 * What is the purpose of the block time during xQueueReceive()?
 */
QueueHandle_t HANDLER = 0;

//Orientation Type Enumeration
typedef enum{
	left,		// 0
	right,		// 1
   	up,			// 2
	down,		// 3
	invalid		// 4
}orientation_t;

//ID Generated for getSharedObject() and addSharedObject()
typedef enum{
	sharedSensorQueueID,
	sharedEventGroupID
}sharedHandler_t;

class task_one : public scheduler_task
{
	public:
		task_one (uint8_t priority) : scheduler_task("one", 2048, priority)
		{
			QueueHandle_t myQueue = xQueueCreate(1, sizeof(orientation_t));
			addSharedObject(sharedSensorQueueID, myQueue);
		}
		bool run(void* p)
		{
			orientation_t orientation = invalid;
			int x, y, z;
			x = AS.getX();
			y = AS.getY();
			z = AS.getZ();

			printf("RAW VALUES: X = %i, Y = %i, Z = %i \n", x, y, z);

			if( ((x <-20) && (x>-40)) )
				orientation = invalid;
			else if ( ((x >-10) && (x < 900)) )
				orientation = left;
			else if ( ((x <-40) && (x > -820)) )
				orientation = right;
			else if (x>900)
				orientation = up;
			else
				orientation = down;

			xQueueSend(getSharedObject(sharedSensorQueueID), &orientation, 0);
			printf("Sent\n");
			vTaskDelay(1000);
			return true;
		}
};

class task_two : public scheduler_task
{
public:
	task_two (uint8_t priority): scheduler_task("two", 2048, priority)
	{
	}
	bool run(void *p)
	{
		orientation_t orientation = invalid;
		QueueHandle_t qid = getSharedObject(sharedSensorQueueID);

		if(xQueueReceive(qid, &orientation, portMAX_DELAY))
		{
			printf("orientation: %d \n", orientation);

			if( ((orientation == left) || (orientation == right)) )
			{
				LE.on(1);
				LE.on(2);
				LE.on(3);
				LE.on(4);
			}
			else
			{
				LE.off(1);
				LE.off(2);
				LE.off(3);
				LE.off(4);
			}
		}
		return true;
	}
};

//CMPE 146 - Lab 7 RTOS APP
/*
int avg100MS = 0, sample100MS = 0;
class producer_task : public scheduler_task
{
    public:

		producer_task(uint8_t priority) : scheduler_task("produce", 2048, priority)
		{
            QueueHandle_t myQueue = xQueueCreate(10, sizeof(int));
            addSharedObject(sharedSensorQueueID, myQueue);
        }

        bool run(void *p)
        {
//Compute average here, and send it to the queue once a second
        	avg100MS = computeAvg(sample100MS);
        	avg100MS /= 100;

        	printf("Sending calculated average \n");
//Note to self - Duration of portMAX_DELAY shouldn't affect operation, but if it does, try 1 sec
        	xQueueSend(getSharedObject(sharedSensorQueueID), &avg100MS, portMAX_DELAY);

        	avg100MS = 0; sample100MS = 0;
        	xEventGroupSetBits(getSharedObject(sharedEventGroupID),1); //Questionable
        	vTaskDelay(100);

        	return true;
        }

        int computeAvg(int avg)
        {
        	for (int i=0; i<100; i++)
        	{
        		avg += LS.getRawValue();
           	}
        	return avg;
        }
};

class consumer_task : public scheduler_task
{
    public:
		int LIGHT = 0, AVG = 0;
        char dataArr[15];

        consumer_task (uint8_t priority) : scheduler_task("consume", 2048, priority)
        {
        }

        bool run(void *p)
        {
            QueueHandle_t qid = getSharedObject(sharedSensorQueueID);
           // vTaskDelay(1000);

            printf("Attempt to receive.\n");
            xQueueReceive(qid, &LIGHT, portMAX_DELAY);
            sprintf(dataArr,"%i %i", time, LIGHT);
            printf("%i %i", time, LIGHT);

            int PASS1 = Storage::append("1:sensor.txt", dataArr, 3, 0);
            if(PASS1 == 0) printf("Successful SD Write 1.");


            for(int i=0; i<100;i++)
			{
				xQueueReceive(qid, &AVG, portMAX_DELAY);
				sprintf(dataArr, "AVG: %i", AVG);
				printf("AVG: %i", AVG);

				int PASS2= Storage::append("1:sensor.txt", dataArr, 5, 0);
				if (PASS2==0) puts("Successful SD Write 2");
			}
			xEventGroupSetBits(getSharedObject(sharedEventGroupID), 2); //questionable
            return true;
         	}

};

class watchdog_task : public scheduler_task
{
    public:
		long sector = 0;
    	char storeChar[30];
        watchdog_task(uint8_t priority) : scheduler_task("watchdog", 2048, priority)
        {
             We save the queue handle by using addSharedObject()
        	EventGroupHandle_t WATCHDOG = xEventGroupCreate();
        	addSharedObject(sharedEventGroupID, WATCHDOG);
        }


        bool run(void *p)
        {
        		EventBits_t waitBITS = xEventGroupWaitBits(getSharedObject(sharedEventGroupID),0x3,pdTRUE,pdTRUE,1000);

        		printf("Accessing 'STUCK' Check\n");

// 0x03
        		if (waitBITS & ALL) printf("Producer & Consumer Bits are Set! \n");
//0x01
        		else if (!(waitBITS & PRODUCER))
        		{
        			printf("Error 1: Producer Bit NOT Set!\n");
        			sprintf(storeChar, "Warning: Producer Bit NOT Set!");
//Appending Check 1.
        			int PASS = Storage::append("1:stuck.txt", storeChar, sizeof(storeChar)-1, 0);
					if (PASS == 0) printf("Stuck data written to SD successfully");
        		}
//0x02
				if (!(waitBITS & CONSUMER))
				{	printf("Error 2: Consumer Bit NOT Set!\n");
					sprintf(storeChar, "Error 2: Consumer Bit NOT Set!");
//Appending Check 2.
					int success=Storage::append("1:stuck.txt", storeChar, sizeof(storeChar)-1, 0);
					if (success==0) puts("Stuck data written to SD successfully");
				}
        	return true;
    	}
};
*/

QueueHandle_t QH = 0;
void RX(void *p)
{
	int item = 0;
	puts("RX TASK");
	if(xQueueReceive(QH, &item, portMAX_DELAY))
	{
		puts("RX RECEIVED AN ITEM");
	}

	vTaskSuspend(0);
	puts("RX is suspended");
}

void TX(void *p)
{
	int item = 0;
	while(1)
	{
		puts("YEILD");
		taskYIELD();
		xQueueSend(QH, &item, 0);
		puts("Did I send an item?");

		xQueueSend(QH, &item, portMAX_DELAY);
		puts("I must have sent an item");
	}
}

QueueHandle_t queue_handler = 0;
void tx2(void *p)
{
	int buffer = 2;
	while(1)
	{
		xQueueSend(queue_handler, &buffer, portMAX_DELAY);
		printf("tx2: sent: %i\n", buffer);
		vTaskSuspend(0);
	}
}

void tx1(void *p)
{
	int buffer = 1;
	while(1)
	{
		xQueueSend(queue_handler, &buffer, portMAX_DELAY);
		printf("tx1: sent: %i\n", buffer);
		vTaskSuspend(0);
	}
}

void rx(void *p)
{
	int buffer = 0;
	while(1)
	{
		xQueueReceive(queue_handler, &buffer, portMAX_DELAY);
		printf("rx: received: %i\n", buffer);
	}
}









//CMPE 146 - Project Test

QueueHandle_t QUEUE = 0;
SemaphoreHandle_t BINARY = 0;
SemaphoreHandle_t MUTEX = 0;

class glove_interface : public scheduler_task
{
public:
	glove_interface(uint8_t priority): scheduler_task("glove", 2000, priority)
	{

	}



	bool run(void *p)
	{

// * Left Hand:  P0.0, P0.1, P0.29, P0.30, P1.19
// * Right Hand: P1.20, P1.22, P1.23, P1.28, P1.29

		int selection = 0;
		const int TWINKLE = 1, SUPERMARIO_LITE = 2 , BEETHOVAN = 3, SUPERMARIO_FULL = 4;

		clearALL();
		readALL();

		puts("Please select the type of Notes you would like to use: ");

//To Select Note Types:
// *
// *  SUPERMARIO: LEFT THUMB & INDEX
// *  BEETHOVAN: LEFT THUMB & MIDDLE
// *  TWINKLE: RIGHT THUNB & INDEX
// *

		if(readGPIO(0,0) && readGPIO(0,1)) selection = SUPERMARIO_LITE;
		else if(readGPIO(0,0) && readGPIO(0,29)) selection = BEETHOVAN;
		else if(readGPIO(1,20) && readGPIO(1,22)) selection = TWINKLE;
		else if(readGPIO(1,20) && readGPIO(1,29)) selection = SUPERMARIO_FULL;
		else puts("Please make a selection: ");

		switch(selection)
		{
		case TWINKLE:
			puts("CASE: TWINKLE");
//What it is supposed to sound like:
			NoteTest(261, 600, 20000); //C4
			NoteTest(261, 600, 20000);
			vTaskDelay(5000);
			NoteTest(392, 600, 20000); //G4
			NoteTest(392, 600, 20000); //G4
			vTaskDelay(5000);
			NoteTest(440, 600, 20000); //A4
			NoteTest(440, 600, 20000);
			vTaskDelay(5000);
			NoteTest(392, 600, 30000); //G4
			vTaskDelay(5000);
			NoteTest(349, 600, 20000); //F4
			NoteTest(349, 600, 20000);
			vTaskDelay(5000);
			NoteTest(330, 600, 20000); //E4
			NoteTest(330, 600, 20000);
			vTaskDelay(5000);
			NoteTest(294, 600, 20000); //D4
			NoteTest(294, 600, 20000);
			vTaskDelay(5000);
			NoteTest(261, 600, 30000); //C4
			vTaskDelay(10000);
			NoteTest(392, 600, 20000); //G4
			NoteTest(392, 600, 20000); //G4
			vTaskDelay(5000);
			NoteTest(349, 600, 20000); //F4
			NoteTest(349, 600, 20000);
			vTaskDelay(5000);
			NoteTest(330, 600, 20000); //E4
			NoteTest(330, 600, 20000);
			vTaskDelay(5000);
			NoteTest(294, 600, 30000); //D4
			vTaskDelay(10000);
			NoteTest(392, 600, 20000); //G4
			NoteTest(392, 600, 20000); //G4
			vTaskDelay(5000);
			NoteTest(349, 600, 20000); //F4
			NoteTest(349, 600, 20000);
			vTaskDelay(5000);
			NoteTest(330, 600, 20000); //E4
			NoteTest(330, 600, 20000);
			vTaskDelay(5000);
			NoteTest(294, 600, 30000); //D4
			vTaskDelay(10000);
			NoteTest(261, 600, 20000); //C4
			NoteTest(261, 600, 20000);
			vTaskDelay(5000);
			NoteTest(392, 600, 20000); //G4
			NoteTest(392, 600, 20000); //G4
			vTaskDelay(5000);
			NoteTest(440, 600, 20000); //A4
			NoteTest(440, 600, 20000);
			vTaskDelay(5000);
			NoteTest(392, 600, 30000); //G4
			vTaskDelay(5000);
			NoteTest(349, 600, 20000); //F4
			NoteTest(349, 600, 20000);
			vTaskDelay(5000);
			NoteTest(330, 600, 20000); //E4
			NoteTest(330, 600, 20000);
			vTaskDelay(5000);
			NoteTest(294, 600, 20000); //D4
			NoteTest(294, 600, 20000);
			vTaskDelay(5000);
			NoteTest(261, 600, 30000); //C4
			vTaskDelay(10000);


//Our Attempt:
			Twinkle();
			break;

		case SUPERMARIO_LITE:
			puts("CASE: SUPERMARIO LITE");
//What is supposed to sound like:
				vTaskDelay(5000);
				NoteTest(164, 600, 5000); //E3
				NoteTest(164, 600, 7500); //E3
				vTaskDelay(5000);
				NoteTest(164, 600, 5000); //E3
				vTaskDelay(5000);
				NoteTest(131, 800, 5000); //C3
				NoteTest(164, 600, 5000); //E3
				vTaskDelay(5000);
				NoteTest(196, 600, 5000); //G3
				vTaskDelay(5000);
				vTaskDelay(10000);
				vTaskDelay(5000);
				NoteTest(98, 600, 5000); //G2
				vTaskDelay(10000);
				NoteTest(131, 800, 5000); //C3
				vTaskDelay(10000);
				NoteTest(98, 600, 5000); //G2
				vTaskDelay(10000);
				NoteTest(82, 600, 5000); //E2
				vTaskDelay(10000);
//What we can play with:
				SuperMario();
			break;

		case BEETHOVAN:
			puts("CASE: BEETHOVAN");
//What its supposed to sound like:
			vTaskDelay(5000);
			NoteTest(164, 600, 10000);//E3
			vTaskDelay(5000);
			NoteTest(164, 600, 10000);//E3
			NoteTest(164, 600, 10000);//E3
			NoteTest(131, 800, 25000);//C3
			vTaskDelay(5000);
			vTaskDelay(5000);
			NoteTest(147, 600, 10000);//D3
			vTaskDelay(5000);
			NoteTest(147, 600, 10000);//D3
			NoteTest(147, 600, 10000);//D3
			NoteTest(123, 600, 35000);//B2
			vTaskDelay(5000);

//What you can play around with
			Beethoven();
			break;

		case SUPERMARIO_FULL:
//What it is supposed to sound like:
		vTaskDelay(5000);
		NoteTest(164, 600, 5000); //E3
		NoteTest(164, 600, 7500); //E3
		vTaskDelay(5000);
		NoteTest(164, 600, 5000); //E3
		vTaskDelay(5000);
		NoteTest(131, 800, 5000); //C3
		NoteTest(164, 600, 5000); //E3
		vTaskDelay(5000);
		NoteTest(196, 600, 5000); //G3
		vTaskDelay(5000);
		vTaskDelay(10000);
		vTaskDelay(5000);
		NoteTest(98, 600, 5000); //G2
		vTaskDelay(10000);
		NoteTest(131, 800, 5000); //C3
		vTaskDelay(10000);
		NoteTest(98, 600, 5000); //G2
		vTaskDelay(10000);
		NoteTest(82, 600, 5000); //E2
		vTaskDelay(10000);
		NoteTest(131, 600, 5000); //C3
		vTaskDelay(5000);
		NoteTest(147, 600, 5000); //D3
		vTaskDelay(5000);
		NoteTest(139, 600, 5000); //D3b
		NoteTest(131, 600, 5000); //C3
		vTaskDelay(5000);
		NoteTest(124, 600, 10000); //B2
		NoteTest(196, 600, 10000); //G3
		NoteTest(247, 600, 10000); //B3
		vTaskDelay(500);
		NoteTest(262, 600, 5000); //C4
		vTaskDelay(5000);
		NoteTest(220, 600, 5000); //A3
		NoteTest(247, 600, 5000); //B3
		vTaskDelay(5000);
		NoteTest(196, 600, 5000); //G3
		vTaskDelay(5000);
		NoteTest(164, 600, 5000); //E3
		vTaskDelay(100);
		NoteTest(175, 600, 5000); //F3
		NoteTest(147, 600, 5000); //D3
		vTaskDelay(10000);
//Stuff to play with
		SuperMarioFull();
			break;

		default:
			puts("Please select the type of Notes you would like to use: ");

		}

	 return true;
	}



bool init(void)
	{
		puts("GPIO Initialization");
/* Multiplexing Pins - Selecting GPIO Configuration (00)
 *
 * Left Hand:  P0.0, P0.1, P0.29, P0.30, P1.19
 * Right Hand: P1.20, P1.22, P1.23, P1.28, P1.29
 *
 */
		//LEFT
		LPC_PINCON->PINSEL0 &= ~ ((3<<0) | (3<<1) | (3<<29) | (3<<30));
		LPC_PINCON->PINSEL1 &= ~ (3<<19);

		//RIGHT
		LPC_PINCON->PINSEL1 &= ~ ((3<<20) | (3<<22) | (3<<23) | (3<<28) | (3<<29));

		puts("DAC Initialization");
/* Multiplexing Pins to Select DAC Configuration (10)
 *
 * Pin: P0.26, (DAC output pin)
 * PINSEL1 Bits 21:20 - First clear and then set the bits.
 *
 */
        LPC_SC -> PCLKSEL0 &= ~(3 << 22);
		LPC_PINCON ->PINSEL1 &= (3 << 20);
		LPC_PINCON ->PINSEL1 |= (2 << 20);

// Setting Direction of GPIO Pins - INPUTS!

		//LEFT
		LPC_GPIO0 ->FIODIR &= ~(1 << 0);
		LPC_GPIO0 ->FIODIR &= ~(1 << 1);
		LPC_GPIO0 ->FIODIR &= ~(1 << 29);
		LPC_GPIO0 ->FIODIR &= ~(1 << 30);
		LPC_GPIO1 ->FIODIR &= ~(1 << 19);

		//RIGHT
		LPC_GPIO1 ->FIODIR &= ~(1 << 20);
		LPC_GPIO1 ->FIODIR &= ~(1 << 22);
		LPC_GPIO1 ->FIODIR &= ~(1 << 23);
		LPC_GPIO1 ->FIODIR &= ~(1 << 28);
		LPC_GPIO1 ->FIODIR &= ~(1 << 29);


		return true;
	}


/* GPIO Functions & Ports Used
 *
 * Left Hand:  P0.0, P0.1, P0.29, P0.30, P1.19
 * Right Hand: P1.20, P1.22, P1.23, P1.28, P1.29
 *
 */
	void clearGPIO(uint8_t portNum, uint32_t pinNum)
	{
		if (portNum == 0)
		{
			LPC_GPIO0->FIOCLR = (1 << pinNum);
		}
		else if(portNum == 1)
		{
			LPC_GPIO1->FIOCLR = (1 << pinNum);
		}
		return;
	}

	void clearALL()
	{
//LEFT
		clearGPIO(0,0);
		clearGPIO(0,1);
		clearGPIO(0,29);
		clearGPIO(0,30);
		clearGPIO(1,19);

//RIGHT
		clearGPIO(1,20);
		clearGPIO(1,22);
		clearGPIO(1,23);
		clearGPIO(1,28);
		clearGPIO(1,29);

		return;
	}

	void readALL()
	{
	//LEFT
			printf("P0.0 = %i\n",readGPIO(0,0));
			printf("P0.1 = %i\n",readGPIO(0,1));
			printf("P0.29 = %i\n",readGPIO(0,29));
			printf("P0.30 = %i\n",readGPIO(0,30));
			printf("P0.19 = %i\n",readGPIO(1,19));

	//RIGHT
			printf("P1.20 = %i\n",readGPIO(1,20));
			printf("P1.22 = %i\n",readGPIO(1,22));
			printf("P1.23 = %i\n",readGPIO(1,23));
			printf("P1.28 = %i\n",readGPIO(1,28));
			printf("P1.29 = %i\n",readGPIO(1,29));

   //DEBUG: Status Check
			//vTaskDelay(1000);

			return;
	}

 	int readGPIO(uint8_t portNum, uint32_t pinNum)
	{
		if(portNum == 0)
		{
			if((LPC_GPIO0 -> FIOPIN & (1<<pinNum)))
				return 1;
			else
				return 0;
		}
		if(portNum == 1)
		{
			if((LPC_GPIO1 -> FIOPIN & (1<<pinNum)))
				return 1;
			else
				return 0;
		}
		else
		{
			printf("Only ports 0 and 1 are used, try again!\n");
			return 0;
		}
	}

//Note Testing
	void NoteTest(int frequency = 261, int amplitude = 768, int timeToPlay = 20000) {
			amplitude=(amplitude<1022) ? amplitude : 1022; //checks to make sure amplitude is less than or equal to 1023
			int min = 512 - amplitude / 2;
			int max = 512 + amplitude / 2;
			int delays = 44000 / frequency / 2;
			LPC_DAC->DACR &= ~(1023 << 6);
			int forLoop = timeToPlay / delays / 2;
			for (int i = 0; i < forLoop; i++) {
				LPC_DAC->DACR &= ~(1023 << 6);
				LPC_DAC->DACR |= (max << 6);
				max = (frequency / 10 * max + 512) / (frequency / 10 + 1);
				vTaskDelay(delays);
				LPC_DAC->DACR &= ~(1023 << 6);
				LPC_DAC->DACR |= (min << 6);
				min = (frequency / 10 * min + 512) / (frequency / 10 + 1);
				vTaskDelay(delays);
			}
		}

//Key Notes
	void A2() {
		int i, min = 128, max = 896;
		for (i = 0; i < 50; i++) { 		//A2  (Frequency=110 Hz)
										//i<50 (because 20k is about half of 44k and we want to play half a second
										//so we divide 20k by result of TickRateHz/frequency, in this case 400... 20k/400=50)
			LPC_DAC->DACR &= ~(1023 << 6);
			LPC_DAC->DACR |= (max << 6);
			max = (11 * max + 512) / 12;//max decreases, creates fading effect
			vTaskDelay(200); // (TickRateHz/frequency/2, in this case 44000/110/2) Divide by two for rising and falling
			LPC_DAC->DACR &= ~(1023 << 6);
			LPC_DAC->DACR |= (min << 6);
			min = (11 * min + 512) / 12;//min increases, creates fading effect
			vTaskDelay(200); // (TickRateHz/frequency/2, in this case 44000/110/2)
		}
	}

	void A4(int timeToPlay=20000)
	{
		int max=896;
		int min=128;
		int forLoop= timeToPlay/50/2;
		for (int i = 0; i < forLoop; i++) {
			LPC_DAC->DACR &= ~(1023 << 6);
			LPC_DAC->DACR |= (max << 6);
			max=(23*max+512)/24;
			vTaskDelay(50);
			LPC_DAC->DACR &= ~(1023 << 6);
			LPC_DAC->DACR |= (min << 6);
			min=(23*min+512)/24;
			vTaskDelay(50);
		}
	}

	void E2(int timeToPlay=20000) {
			int max = 896;
			int min = 128;
			int forLoop= timeToPlay/134/2;
			for (int i = 0; i < forLoop; i++) { 				//E2 modified
				LPC_DAC->DACR &= ~(1023 << 6);
				LPC_DAC->DACR |= (max << 6);
				max = (11 * max + 512) / 12;
				vTaskDelay(134);
				LPC_DAC->DACR &= ~(1023 << 6);
				LPC_DAC->DACR |= (min << 6);
				min = (11 * min + 512) / 12;
				vTaskDelay(134);
			}
		}

	void D2(int timeToPlay=20000) {
			int max = 896;
			int min = 128;
			int forLoop= timeToPlay/150/2;
			for (int i = 0; i < forLoop; i++) {				//D2 modified
				LPC_DAC->DACR &= ~(1023 << 6);
				LPC_DAC->DACR |= (max << 6);
				max = (11 * max + 512) / 12;
				vTaskDelay(150);
				LPC_DAC->DACR &= ~(1023 << 6);
				LPC_DAC->DACR |= (min << 6);
				min = (11 * min + 512) / 12;
				vTaskDelay(150);
			}
		}

	void B1(int timeToPlay=20000) {
			int max = 896;
			int min = 128;
			while (LPC_GPIO0 -> FIOPIN & (1 << 1)) { 				//B1 modified
				LPC_DAC->DACR &= ~(1023 << 6);
				LPC_DAC->DACR |= (max << 6);
				max = (11 * max + 512) / 12;
				vTaskDelay(179);
				LPC_DAC->DACR &= ~(1023 << 6);
				LPC_DAC->DACR |= (min << 6);
				min = (11 * min + 512) / 12;
				vTaskDelay(179);
			}
		}

	void C2(int timeToPlay=20000) {
			int max = 896;
			int min = 128;
			int forLoop= timeToPlay/168/2;
			for (int i = 0; i < forLoop; i++) {			//C2 modified
				LPC_DAC->DACR &= ~(1023 << 6);
				LPC_DAC->DACR |= (max << 6);
				max = (11 * max + 512) / 12;
				vTaskDelay(168);
				LPC_DAC->DACR &= ~(1023 << 6);
				LPC_DAC->DACR |= (min << 6);
				min = (11 * min + 512) / 12;
				vTaskDelay(168);
			}
		}

	void E3(int timeToPlay=20000) {
		int max = 896;
		int min = 128;
		int forLoop= timeToPlay/134/2;
		for (int i = 0; i < forLoop; i++) {				//E3
			LPC_DAC->DACR &= ~(1023 << 6);
			LPC_DAC->DACR |= (max << 6);
			max = (11 * max + 512) / 12;
			vTaskDelay(134);
			LPC_DAC->DACR &= ~(1023 << 6);
			LPC_DAC->DACR |= (min << 6);
			min = (11 * min + 512) / 12;
			vTaskDelay(134);
		}
	}

	void D3(int timeToPlay=20000) {
		int max = 896;
		int min = 128;
		int forLoop= timeToPlay/150/2;
		for (int i = 0; i < forLoop; i++) {				//D3
			LPC_DAC->DACR &= ~(1023 << 6);
			LPC_DAC->DACR |= (max << 6);
			max = (11 * max + 512) / 12;
			vTaskDelay(150);
			LPC_DAC->DACR &= ~(1023 << 6);
			LPC_DAC->DACR |= (min << 6);
			min = (11 * min + 512) / 12;
			vTaskDelay(150);
		}
	}

	void B2(int timeToPlay=20000) {
		int max = 896;
		int min = 128;
		int forLoop= timeToPlay/179/2;
		for (int i = 0; i < forLoop; i++) {				//B2
			LPC_DAC->DACR &= ~(1023 << 6);
			LPC_DAC->DACR |= (max << 6);
			max = (11 * max + 512) / 12;
			vTaskDelay(179);
			LPC_DAC->DACR &= ~(1023 << 6);
			LPC_DAC->DACR |= (min << 6);
			min = (11 * min + 512) / 12;
			vTaskDelay(179);
		}
	}

	void C3(int timeToPlay=20000) {
		int max = 896;
		int min = 128;
		int forLoop= timeToPlay/168/2;
		for (int i = 0; i < forLoop; i++) {				//C3
			LPC_DAC->DACR &= ~(1023 << 6);
			LPC_DAC->DACR |= (max << 6);
			max = (11 * max + 512) / 12;
			vTaskDelay(168);
			LPC_DAC->DACR &= ~(1023 << 6);
			LPC_DAC->DACR |= (min << 6);
			min = (11 * min + 512) / 12;
			vTaskDelay(168);
		}
	}

	void C4(int timeToPlay=20000) {
		int max = 896;
		int min = 128;
		int forLoop= timeToPlay/84/2;
		for (int i = 0; i < forLoop; i++) {				//C4
			LPC_DAC->DACR &= ~(1023 << 6);
			LPC_DAC->DACR |= (max << 6);
			max = (11 * max + 512) / 12;
			vTaskDelay(84);
			LPC_DAC->DACR &= ~(1023 << 6);
			LPC_DAC->DACR |= (min << 6);
			min = (11 * min + 512) / 12;
			vTaskDelay(84);
		}
	}

	void D4(int timeToPlay=20000)
	{
		int max=896;
		int min=128;
		int forLoop= timeToPlay/75/2;
		for (int i = 0; i < forLoop; i++) {
			LPC_DAC->DACR &= ~(1023 << 6);
			LPC_DAC->DACR |= (max << 6);
			max=(11*max+512)/12;
			vTaskDelay(75);
			LPC_DAC->DACR &= ~(1023 << 6);
			LPC_DAC->DACR |= (min << 6);
			min=(11*min+512)/12;
			vTaskDelay(75);
		}
	}

	void F4(int timeToPlay=20000)
	{
		int max=896;
		int min=128;
		int forLoop= timeToPlay/63/2;
		for (int i = 0; i < forLoop; i++) {  	//F4
			LPC_DAC->DACR &= ~(1023 << 6);
			LPC_DAC->DACR |= (max << 6);
			max=(11*max+512)/12;
			vTaskDelay(63);
			LPC_DAC->DACR &= ~(1023 << 6);
			LPC_DAC->DACR |= (min << 6);
			min=(11*min+512)/12;
			vTaskDelay(63);
		}
	}

	void G4(int timeToPlay=20000)
	{
		int max=896;
		int min=128;
		int forLoop= timeToPlay/56/2;
		for (int i = 0; i < forLoop; i++) {
			LPC_DAC->DACR &= ~(1023 << 6);
			LPC_DAC->DACR |= (max << 6);
			max=(11*max+512)/12;
			vTaskDelay(56);
			LPC_DAC->DACR &= ~(1023 << 6);
			LPC_DAC->DACR |= (min << 6);
			min=(11*min+512)/12;
			vTaskDelay(56);
		}
	}

/* Notes to self
 * Left Hand:  P0.0, P0.1, P0.29, P0.30, P1.19
 * Right Hand: P1.20, P1.22, P1.23, P1.28, P1.29
 *
 */

//Songs
	void Beethoven() //E31-E32-E32-C3-D3-D3-B2
	{

		while(1){
		readALL();

		if(readGPIO(0,0)) { //LEFT INDEX
			NoteTest(164, 600, 10000); //E31
			puts("E31");
			vTaskDelay(5000);
		}

		if(readGPIO(0,1)){ //LEFT THUMB
			NoteTest(164, 600, 10000);//E32
			puts("E32");
			vTaskDelay(2000);
		}

		if(readGPIO(0,29)){ //LEFT MIDDLE
			NoteTest(131, 800, 25000);//C3
			puts("C3");
			vTaskDelay(5000);
		}

		if(readGPIO(0,30)){ //LEFT RING
			NoteTest(147, 600, 10000);//D31
			puts("D31");
			vTaskDelay(5000);
		}

		if(readGPIO(1,19)){ //LEFT PINKY
			NoteTest(147, 600, 10000);//D32
			puts("D32");
			vTaskDelay(2000);
		}

		if(readGPIO(1,20)){ //RIGHT THUMB
			NoteTest(123, 600, 35000);//B2
			puts("B2");
			vTaskDelay(5000);
		}

//EXIT CONDITION
		if(readGPIO(1,29)) break;

		}

		return;
	}

//TO PLAY: E31-E32-E31-C3-E31-G3-G2-C3-G2-E2
	void SuperMario()
	{
		while(1){
//LEFT
		readALL();

		if(readGPIO(0,0)) { //INDEX
			NoteTest(164, 600, 5000);  //E3-1
			puts("E3-1");
			vTaskDelay(10000);
		}
		if(readGPIO(0,1))  { //THUMB
			NoteTest(164, 600, 7500);  //E3-2
			puts("E3-2");
			vTaskDelay(10000);
		}
		if(readGPIO(0,29)){ //MIDDLE
			NoteTest(131, 800, 5000);  //C3
			puts("C3");
			vTaskDelay(10000);
		}
		if(readGPIO(0,30)){ //RING
			NoteTest(196, 600, 5000);  //G3
			puts("G3");
			vTaskDelay(10000);
		}
		if(readGPIO(1,19)){ // PINKY
			NoteTest(98, 600, 5000);   //G2
			puts("G2");
			vTaskDelay(10000);
		}

//RIGHT
		if(readGPIO(1,20)){ //THUMB
			NoteTest(82, 600, 5000); //E2
			puts("E2");
			vTaskDelay(10000);
		}

/* EXIT Condition
 *
 * To Activate:
 * RIGHT PINKY
 *
 */
		if(readGPIO(1,29)) break;

		}

		return;
	}

/* TO PLAY:
 *
 * E31-E32-E31-C3-E31-G3-G2-C3-G2-E2
 * C32-D3-D3B
 *
 *  We can only play the above tune due to the limited avalibly of fingers
 */
	void SuperMarioFull()
	{
		while(1){
//LEFT
		readALL();

				if(readGPIO(0,0)) { //INDEX
					NoteTest(164, 600, 5000);  //E3-1
					puts("E3-1");
					vTaskDelay(10000);
				}
				if(readGPIO(0,1))  { //THUMB
					NoteTest(164, 600, 7500);  //E3-2
					puts("E3-2");
					vTaskDelay(10000);
				}
				if(readGPIO(0,29)){ //MIDDLE
					NoteTest(131, 800, 5000);  //C3
					puts("C3");
					vTaskDelay(10000);
				}
				if(readGPIO(0,30)){ //RING
					NoteTest(196, 600, 5000);  //G3
					puts("G3");
					vTaskDelay(10000);
				}
				if(readGPIO(1,19)){ // PINKY
					NoteTest(98, 600, 5000);   //G2
					puts("G2");
					vTaskDelay(10000);
				}

//RIGHT
				if(readGPIO(1,20)){ //THUMB
					NoteTest(82, 600, 5000); //E2
					puts("E2");
					vTaskDelay(10000);
				}


				if(readGPIO(1,22)){ //INDEX
					NoteTest(131, 600, 5000); //C32
					puts("C32");
					vTaskDelay(10000);
				}

				if(readGPIO(1,23)){ //MIDDLE
					NoteTest(147, 600, 5000); //D3
					puts("D3");
					vTaskDelay(10000);
				}

				if(readGPIO(1,28)){ //RING
					NoteTest(139, 600, 5000); //D3B
					puts("D3B");
					vTaskDelay(10000);
				}


				if(readGPIO(1,29)) break;

		}
		return;
	}

	void Twinkle()
	{

	}

};

//Multiple Note Test, Queues only make it such that they constantly context switch back
//And forth, no real over lap

void NoteTest(int frequency = 261, int amplitude = 768, int timeToPlay = 20000) {
			amplitude=(amplitude<1022) ? amplitude : 1022; //checks to make sure amplitude is less than or equal to 1023
			int min = 512 - amplitude / 2;
			int max = 512 + amplitude / 2;
			int delays = 44000 / frequency / 2;
			LPC_DAC->DACR &= ~(1023 << 6);
			int forLoop = timeToPlay / delays / 2;
			for (int i = 0; i < forLoop; i++) {
				LPC_DAC->DACR &= ~(1023 << 6);
				LPC_DAC->DACR |= (max << 6);
				max = (frequency / 10 * max + 512) / (frequency / 10 + 1);
				vTaskDelay(delays);
				LPC_DAC->DACR &= ~(1023 << 6);
				LPC_DAC->DACR |= (min << 6);
				min = (frequency / 10 * min + 512) / (frequency / 10 + 1);
				vTaskDelay(delays);
			}
		}

void playSong(void *p)
{
	int sendAck[4] = {0};
	while(1)
	{

		if(xQueueSend(QUEUE, &sendAck[0], 1000))
		{
			puts("123");
			if(SW.getSwitch(2)){
				NoteTest(147, 600, 10000);//D3

			}
		}

		if(xQueueSend(QUEUE, &sendAck[1], 1000))
		{
			puts("456");
			if(SW.getSwitch(3)){
				NoteTest(392, 600, 20000); //G4
		}
	}
}
}

void playAnotherSong(void *p)
{
	int Ack[3] = {0};
	while(1)
	{
		if(xQueueReceive(QUEUE, &Ack[0], 1000))
		{
			puts("789");
			if(SW.getSwitch(1)) {NoteTest(131, 800, 5000); //C3

			}
		}

		if(xQueueReceive(QUEUE, &Ack[1], 1000))
		{
			puts("000");

			if(SW.getSwitch(4))	NoteTest(440, 600, 20000); //A4; //C3
		}

	}
}


int main(void)
{
//CmpE 146 Project, GPIO Read Test, ADD Bluetooth and another terminal task! ! !
#if 1
QUEUE = xQueueCreate(1, sizeof(int));
scheduler_add_task(new glove_interface(PRIORITY_HIGH));
//xTaskCreate(playSong, "playSong", 1024, NULL, PRIORITY_HIGH, NULL);
//xTaskCreate(playAnotherSong, "playAnotherSong", 1024, NULL, PRIORITY_HIGH, NULL);

#endif

#if 0
	queue_handler = xQueueCreate(1, sizeof(int));
	xTaskCreate(tx1, "tx1", 1024, NULL, PRIORITY_HIGH, NULL);
	xTaskCreate(tx2, "tx2", 1024, NULL, PRIORITY_MEDIUM, NULL);
	xTaskCreate(rx, "rx", 1024, NULL, PRIORITY_LOW, NULL);
	vTaskStartScheduler();
#endif


#if 0
	QH = xQueueCreate(1, sizeof(int));
	xTaskCreate(RX, "RX", 1024, NULL, PRIORITY_HIGH, NULL);
	xTaskCreate(TX, "TX", 1024, NULL, PRIORITY_LOW, NULL);
	vTaskStartScheduler();
#endif

#if 0
	const uint32_t max_count = 3, initial_count = 1;
	counting_sem = xSemaphoreCreateCounting(max_count, initial_count);

	xTaskCreate(consumer, "comsumer1", 1024, NULL, PRIORITY_HIGH, NULL);
	xTaskCreate(consumer, "comsumer2", 1024, NULL, PRIORITY_MEDIUM, NULL);
	xTaskCreate(producer, "producer", 1024, NULL, PRIORITY_LOW, NULL);
	vTaskStartScheduler();
#endif

//CMPE 146 Lab 5
/*
	 I2C2 &I2C2 = I2C2::getInstance();
	 const uint8_t slaveAddr = 0xC0;

	uint8_t BUFFER_SIZE = 10;
	uint8_t DATA_BUFFER[BUFFER_SIZE];
	uint8_t prev = DATA_BUFFER[0];
	I2C2.init_Slave(slaveAddr, BUFFER_SIZE, &DATA_BUFFER[0]);



		if(prev != DATA_BUFFER[0])
		{

			LE.toggle(1);
			LE.toggle(2);
			LE.toggle(3);
			vTaskDelay(100);

			prev = DATA_BUFFER[0];
		}

		LE.off(1);
		LE.off(2);
		LE.off(3);
*/


//	vSemaphoreCreateBinary(switch_signal);
    /**
     * A few basic tasks for this bare-bone system :
     *      1.  Terminal task provides gateway to interact with the board through UART terminal.
     *      2.  Remote task allows you to use remote control to interact with the board.
     *      3.  Wireless task responsible to receive, retry, and handle mesh network.
     *
     * Disable remote task if you are not using it.  Also, it needs SYS_CFG_ENABLE_TLM
     * such that it can save remote control codes to non-volatile memory.  IR remote
     * control codes can be learned by typing the "learn" terminal command.
     */
    scheduler_add_task(new terminalTask(PRIORITY_HIGH));

//CMPE 146 Lab 1: GPIO
	#if 0
  scheduler_add_task(new gpio_task(PRIORITY_HIGH));
	#endif
//CMPE 146 Lab 2: SPI
	#if 0
    scheduler_add_task(new spi_task(PRIORITY_HIGH));
	#endif
//CMPE 146 Lab 3: UART
	#if 0
  scheduler_add_task(new uart_task(PRIORITY_HIGH));
	#endif

//CMPE 146 Lab 4: EINT
//  vSemaphoreCreateBinary(signal);
  	#if 0
    scheduler_add_task(new interrupt_task(PRIORITY_HIGH));
  	#endif

  //CMPE 146 Lab 6: RTOS
	#if 0
  scheduler_add_task(new task_one(PRIORITY_HIGH));
  scheduler_add_task(new task_two(PRIORITY_HIGH));
	#endif


  //CMPE 146 Lab 7: RTOS APP
	#if 0
  scheduler_add_task(new producer_task(PRIORITY_MEDIUM));
  scheduler_add_task(new consumer_task(PRIORITY_MEDIUM));
  scheduler_add_task(new watchdog_task(PRIORITY_HIGH));
	#endif




//--------------------------------------------------243 + 244 Section -----------------------------------------------------------
  //CMPE 243 - Lab 1
	#if 0
  scheduler_add_task(new GPIO_task(PRIORITY_HIGH));
  scheduler_add_task(new LED_task(PRIORITY_HIGH));
	#endif
    /* Consumes very little CPU, but need highest priority to handle mesh network ACKs */
	#if 0
  scheduler_add_task(new wirelessTask(PRIORITY_CRITICAL));
	#endif
    /* Change "#if 0" to "#if 1" to run period tasks; @see period_callbacks.cpp */

//Periodic Scheduler
  #if 0
  scheduler_add_task(new periodicSchedulerTask());
    #endif


    /* The task for the IR receiver */
    // scheduler_add_task(new remoteTask  (PRIORITY_LOW));

    /* Your tasks should probably used PRIORITY_MEDIUM or PRIORITY_LOW because you want the terminal
     * task to always be responsive so you can poke around in case something goes wrong.
     */

    /**
     * This is a the board demonstration task that can be used to test the board.
     * This also shows you how to send a wireless packets to other boards.
     */
    #if 0
        scheduler_add_task(new example_io_demo());
    #endif

    /**
     * Change "#if 0" to "#if 1" to enable examples.
     * Try these examples one at a time.
     */
    #if 0
        scheduler_add_task(new example_task());
        scheduler_add_task(new example_alarm());
        scheduler_add_task(new example_logger_qset());
        scheduler_add_task(new example_nv_vars());
    #endif

    /**
	 * Try the rx / tx tasks together to see how they queue data to each other.
	 */
    #if 0
        scheduler_add_task(new queue_tx());
        scheduler_add_task(new queue_rx());
    #endif

    /**
     * Another example of shared handles and producer/consumer using a queue.
     * In this example, producer will produce as fast as the consumer can consume.
     */
    #if 0
        scheduler_add_task(new producer());
        scheduler_add_task(new consumer());
    #endif

    /**
     * If you have RN-XV on your board, you can connect to Wifi using this task.
     * This does two things for us:
     *   1.  The task allows us to perform HTTP web requests (@see wifiTask)
     *   2.  Terminal task can accept commands from TCP/IP through Wifly module.
     *
     * To add terminal command channel, add this at terminal.cpp :: taskEntry() function:
     * @code
     *     // Assuming Wifly is on Uart3
     *     addCommandChannel(Uart3::getInstance(), false);
     * @endcode
     */
    #if 0
        Uart3 &u3 = Uart3::getInstance();
        u3.init(WIFI_BAUD_RATE, WIFI_RXQ_SIZE, WIFI_TXQ_SIZE);
        scheduler_add_task(new wifiTask(Uart3::getInstance(), PRIORITY_LOW));
    #endif

    scheduler_start(); ///< This shouldn't return
    return -1;
}

