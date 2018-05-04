/*
 * main.c
 */
#include <stdint.h>
#include <stdbool.h>
//#include <math.h>
//#include <stdlib.h>

#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "driverlib/pin_map.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/systick.h"
#include "utils/ringbuf.h"
#include "driverlib/ssi.h"
#include "driverlib/uDMA.h"

#define TARGET_IS_BLIZZARD_RB1
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

#include "mySerial.h"
#include "serial.h"
#include "myFIFO.h"
#include "myTimer.h"
#include "myIO.h"

#include "numManipulate.h"

#include "SDcard/diskio.h"


void task_100Hz();
void task_50Hz();
void task_20Hz();

void communication();
void display_com();
void task_SSI();



union {
	struct{
		unsigned char D0:1;
		unsigned char D1:1;
		unsigned char D2:1;
		unsigned char D3:1;
		unsigned char D4:1;
		unsigned char D5:1;
		unsigned char D6:1;
		unsigned char D7:1;
		unsigned char D8:1;
		unsigned char D9:1;
		unsigned char D10:1;
		unsigned char D11:1;
		unsigned char shdn:1;
		unsigned char ga:1;
		unsigned char buf:1;
		unsigned char writeEnable:1;
	}bitData;
	uint16_t data16;
}SSI0_data;

void SSI_init(){
	//
	// Enable Peripheral Clocks
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	//
	// Enable pin PA4 for SSI0 SSI0RX
	//
	GPIOPinConfigure(GPIO_PA4_SSI0RX);
	GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_4);

	//
	// Enable pin PA2 for SSI0 SSI0CLK
	//
	GPIOPinConfigure(GPIO_PA2_SSI0CLK);
	GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2);

	//
	// Enable pin PA3 for SSI0 SSI0FSS
	//
	GPIOPinConfigure(GPIO_PA3_SSI0FSS);
	GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_3);

	//
	// Enable pin PA5 for SSI0 SSI0TX
	//
	GPIOPinConfigure(GPIO_PA5_SSI0TX);
	GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5);

	//
	//
	//

	SSIConfigSetExpClk(SSI0_BASE,SysCtlClockGet(),SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,2000000,8);
	//
	// Enable the SSI module.
	//	SSI0_CR1_R|=SSI_CR1_EOT;
	//	SSIIntEnable(SSI0_BASE,SSI_TXFF);
	SSIEnable(SSI0_BASE);
	//	IntMasterEnable(); //enable processor interrupts
	//	IntEnable(INT_SSI0); //enable the SSI0 interrupt
	SSI0_data.bitData.writeEnable=0;//0 is enable, 1 ignoge this command
	SSI0_data.bitData.buf=0;// 0 is unbuffered
	SSI0_data.bitData.ga=1;// 1- output gain is x1 ; 0 - output gain is x2
	SSI0_data.bitData.shdn=1;// 1 - active mode operation; 0- shutdown the device
	SSI0_data.bitData.D0=0;
	SSI0_data.bitData.D1=0;
	SSI0_data.bitData.D2=0;
	SSI0_data.bitData.D3=0;
	SSI0_data.bitData.D4=0;
	SSI0_data.bitData.D5=0;
	SSI0_data.bitData.D6=0;
	SSI0_data.bitData.D7=0;

	SSI0_data.bitData.D8=0;
	SSI0_data.bitData.D9=0;
	SSI0_data.bitData.D10=0;
	SSI0_data.bitData.D11=1;
}







unsigned char SDbuff[512];

struct{
	unsigned char enable;
	unsigned char ID;
	unsigned char drv;
	unsigned char count;
	unsigned char Stat;
	unsigned char CardType;
	unsigned long sector;
	unsigned char dataRxAvail;
	unsigned int lastData;
	unsigned int indexCount;
	unsigned char readDone;
}FlagSSI;

#define SSI_ID_CONFIG 1
#define SSI_ID_CMD_WFR 2
#define SSI_ID_CMD_SEND 3
#define SSI_ID_CMD_RESPONE 4
#define SSI_ID_READ_WFR 5
#define SSI_ID_READ_GET 6
#define SSI_ID_RESET 7
#define SSI_ID_READ_END_1 8
#define SSI_ID_READ_END_2 9
#define SSI_ID_IDLE 0
unsigned char numofRX_TX=0;
unsigned char SSITimer=0;



// The control table used by the uDMA controller.  This table must be aligned to a 1024 byte boundary.
#pragma DATA_ALIGN(pui8ControlTable, 1024)
uint8_t pui8ControlTable[1024];

#define UART_RXBUF_SIZE         512
static uint8_t g_pui8RxPing[UART_RXBUF_SIZE];

// Define errors counters
static uint32_t g_ui32DMAErrCount = 0;
static uint32_t g_ui32BadISR = 0;
// Define transfer counter
static uint32_t g_ui32MemXferCount = 0;
static uint8_t dataTX=0xFF;
void DMA_init(void){
	//
	//DMA init
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
	IntEnable(INT_UDMAERR);
	ROM_uDMAEnable();
	ROM_uDMAControlBaseSet(pui8ControlTable);
	//
	// Enable Peripheral Clocks
	//
}


void SSI_DMA_init(){


//		SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
//		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
//
//		//
//		// Enable pin PA4 for SSI0 SSI0RX
//		//
//		GPIOPinConfigure(GPIO_PA4_SSI0RX);
//		GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_4);
//
//		//
//		// Enable pin PA2 for SSI0 SSI0CLK
//		//
//		GPIOPinConfigure(GPIO_PA2_SSI0CLK);
//		GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2);
//
//		//
//		// Enable pin PA3 for SSI0 SSI0FSS
//		//
//		GPIOPinConfigure(GPIO_PA3_SSI0FSS);
//		GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_3);
//
//		//
//		// Enable pin PA5 for SSI0 SSI0TX
//		//
//		GPIOPinConfigure(GPIO_PA5_SSI0TX);
//		GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5);
//
//		SSIConfigSetExpClk(SSI0_BASE,SysCtlClockGet(),SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,2000000,8);
//	//
//	// Enable the SSI module.
//		SSIEnable(SSI0_BASE);

	IntMasterEnable(); //enable processor interrupts
	IntEnable(INT_SSI0); //enable the SSI0 interrupt



	// Place the uDMA channel attributes in a known state. These should already be disabled by default.
	ROM_uDMAChannelAttributeEnable(UDMA_CHANNEL_SSI0TX, UDMA_ATTR_USEBURST);

	ROM_uDMAChannelControlSet(UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT,
			UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_NONE |
			UDMA_ARB_4);

	ROM_uDMAChannelControlSet(UDMA_CHANNEL_SSI0RX | UDMA_PRI_SELECT,
			UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_SRC_INC_8 |
			UDMA_ARB_1);

	ROM_uDMAChannelTransferSet(UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT,
			UDMA_MODE_BASIC, &dataTX,(void *)(SSI0_BASE +SSI_O_DR) ,
			512);

	ROM_uDMAChannelTransferSet(UDMA_CHANNEL_SSI0RX | UDMA_PRI_SELECT,
			UDMA_MODE_BASIC,(void *)(SSI0_BASE +SSI_O_DR),g_pui8RxPing,
			512);

//		SSIDMAEnable(SSI0_BASE,SSI_DMA_TX|SSI_DMA_RX);
//		ROM_uDMAChannelEnable(UDMA_CHANNEL_SSI0TX);
//		ROM_uDMAChannelEnable(UDMA_CHANNEL_SSI0RX);

}

void
uDMAErrorHandler(void)
{
	uint32_t ui32Status;

	// Check for uDMA error bit
	ui32Status = ROM_uDMAErrorStatusGet();

	// If there is a uDMA error, then clear the error and increment the error counter.
	if(ui32Status)
	{
		ROM_uDMAErrorStatusClear();
		g_ui32DMAErrCount++;
	}
}

void SSI0_Interrupt_Handler(void){
	//	uint32_t interrupt_status = ROM_SSIIntStatus(SSI0_BASE,1);
	//	SSIIntClear(SSI0_BASE, interrupt_status);
	//
	//	uint32_t ui32Mode = ROM_uDMAChannelModeGet(UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT);

	//	if(!ROM_uDMAChannelIsEnabled(UDMA_CHANNEL_SSI0TX))
	//	{
	//		ROM_uDMAChannelTransferSet(UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT,
	//				UDMA_MODE_BASIC, &dataTX,(void *)(SSI0_BASE +SSI_O_DR) ,
	//				10);
	//		ROM_uDMAChannelEnable(UDMA_CHANNEL_SSI0TX);
	//	}

	//	if(!ROM_uDMAChannelIsEnabled(UDMA_CHANNEL_SSI0TX)){
	//		toggle_led[0]^=1;
	//		led(LED_RED,toggle_led[0]);
	//	}
	//
	if(!ROM_uDMAChannelIsEnabled(UDMA_CHANNEL_SSI0RX)){

		FlagSSI.readDone=1;
		toggle_led[0]^=1;
		led(LED_RED,toggle_led[0]);

	}





	if(ROM_uDMAChannelModeGet(UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT)==UDMA_MODE_STOP){
//		ROM_uDMAChannelTransferSet(UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT,
//					UDMA_MODE_BASIC, &dataTX,(void *)(SSI0_BASE +SSI_O_DR) ,
//					10);
//		ROM_uDMAChannelEnable(UDMA_CHANNEL_SSI0TX);
	}




}



void main(){
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ); //40Mhz
	myIO_init();
	led(LED_RED,0);
	led(LED_BLUE,0);
	led(LED_GREEN,0);
	Timer0_init();
	SysTick_Init();
	UART0_Init();
	//	SSI_init();
	//	UART3_Init();
	//	UART5_Init();
	//	UART1_Init();
	SerialPutStrLn(UART_PC_,"init SD card ...");
	unsigned char status=disk_initialize(0);
	while(status!=0){
		status=disk_initialize (0);
		switch(status)
		{
		case STA_NOINIT:
			SerialPutStrLn(UART_PC_,"init fail");
			break;
		case STA_NODISK:
			SerialPutStrLn(UART_PC_," no disk");
			break;
		case STA_PROTECT:
			SerialPutStrLn(UART_PC_,"protected SD");
			break;
		default:
			break;
		}
	}
	SerialPutStrLn(UART_PC_,"init successful");
	DMA_init();
	SSI_DMA_init();

	SerialPutStrLn(UART_PC_,"config done!");
	FlagSSI.enable=1;
	FlagSSI.ID=SSI_ID_CONFIG;
	SysCtlDelay(SysCtlClockGet()/3/100);
	while(1)
	{
		//						disk_read(0,SDbuff,57350+8192,1);
		//		if(FlagSSI.ID==SSI_ID_IDLE)
		//		{
		//			FlagSSI.enable=1;
		//			FlagSSI.ID=SSI_ID_CONFIG;
		//		}
		communication();
		task_20Hz();
		task_50Hz();
		task_100Hz();
		task_SSI();
		if(FlagSSI.ID!=SSI_ID_READ_GET){
			while((SSI0_SR_R&SSI_SR_RNE)==SSI_SR_RNE)
			{
				if(SSIDataGetNonBlocking(SSI0_BASE, &FlagSSI.lastData)){
					FlagSSI.dataRxAvail=1;
					numofRX_TX--;
				}
			}
		}
	}
}
struct{
	unsigned char display:1;
}Flag;



void task_SSI(){
	if(FlagSSI.enable){
		switch (FlagSSI.ID){
		case SSI_ID_IDLE:
			//			FlagSSI.enable=0;
//			FlagSSI.ID=SSI_ID_CONFIG;
			break;
		case SSI_ID_CONFIG: //wait for ready
			selectSSI();            /* CS = L */
			FlagSSI.ID=SSI_ID_CMD_WFR;
			FlagSSI.sector=57350+8192;
			xmit_spi_my(0xFF);
			SSITimer=50;
			break;
		case SSI_ID_CMD_WFR:
			//sending command - waif for ready
			// try 50 time send 0xFF, check received data;
			// if receive 0xFF then pass, if time out and not recive 0xFF, then finish with error
			if(FlagSSI.dataRxAvail){
				FlagSSI.dataRxAvail=0;
				if (FlagSSI.lastData==0xFF)
				{
					FlagSSI.ID=SSI_ID_CMD_SEND;
					send_cmd_my((0x40+17) ,FlagSSI.sector);
				}
				else
					if(SSITimer==0)
						FlagSSI.ID=SSI_ID_RESET;
					else{
						xmit_spi_my(0xFF);
						SSITimer--;
					}
			}
			break;
		case SSI_ID_CMD_SEND:
			//send 6 byte of command
			//and pass
			if(numofRX_TX==0)
			{
				FlagSSI.ID=SSI_ID_CMD_RESPONE;
				SSITimer=10;
				xmit_spi_my(0xFF);
			}
			break;
		case SSI_ID_CMD_RESPONE:
			// try 10 time send dummy (0xFF) and get the respone
			// if respone is  (res & 0x80) ~=0xFF, then pass, if not, error
			//
			if(FlagSSI.dataRxAvail){
				FlagSSI.dataRxAvail=0;
				if (FlagSSI.lastData&0x80)
				{
					if(SSITimer==0)
						FlagSSI.ID=SSI_ID_RESET;
					else{
						xmit_spi_my(0xFF);
						SSITimer--;
					}
				}
				else{
					FlagSSI.ID=SSI_ID_READ_WFR;
					SSITimer=200;
					xmit_spi_my(0xFF);
				}
			}
			break;
		case SSI_ID_READ_WFR:
			// wait for ready to read
			// try 100ms send dummy data and get the respone
			//  if the respone is 0xFE then pass, if not then error
			if(FlagSSI.dataRxAvail){
				FlagSSI.dataRxAvail=0;
				if (FlagSSI.lastData==0xFE)
				{
					FlagSSI.ID=SSI_ID_READ_GET;
					//					xmit_spi_my(0xFF);
					//					toggle_led[1]^=1;
					//					led(LED_BLUE,toggle_led[1]);
//					ROM_uDMAChannelTransferSet(UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT,
//							UDMA_MODE_BASIC, &dataTX,(void *)(SSI0_BASE +SSI_O_DR) ,
//							512);
//					ROM_uDMAChannelTransferSet(UDMA_CHANNEL_SSI0RX | UDMA_PRI_SELECT,
//							UDMA_MODE_BASIC,(void *)(SSI0_BASE +SSI_O_DR),g_pui8RxPing,
//							512);
					SSIDMAEnable(SSI0_BASE,SSI_DMA_TX|SSI_DMA_RX);
					ROM_uDMAChannelTransferSet(UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT,
							UDMA_MODE_BASIC, &dataTX,(void *)(SSI0_BASE +SSI_O_DR) ,
							512);

					ROM_uDMAChannelTransferSet(UDMA_CHANNEL_SSI0RX | UDMA_PRI_SELECT,
							UDMA_MODE_BASIC,(void *)(SSI0_BASE +SSI_O_DR),g_pui8RxPing,
							512);

					ROM_uDMAChannelEnable(UDMA_CHANNEL_SSI0TX);
					ROM_uDMAChannelEnable(UDMA_CHANNEL_SSI0RX);


				}
				else{
					if(SSITimer==0)
						FlagSSI.ID=SSI_ID_RESET;
					else{
						xmit_spi_my(0xFF);
						SSITimer--;
					}
				}
			}
			break;
		case SSI_ID_READ_GET:
			// get all data from sector until end
			// end process and back to IDLE
			//			if(FlagSSI.dataRxAvail){
			//				FlagSSI.dataRxAvail=0;
			//				SDbuff[FlagSSI.indexCount++]=FlagSSI.lastData;
			//				if(FlagSSI.indexCount==512)
			//				{
			//					FlagSSI.ID=SSI_ID_READ_END_1;
			//				}
			//				//toggle_led[1]^=1;
			//				//led(LED_BLUE,toggle_led[1]);
			//				xmit_spi_my(0xFF);
			//			}
			if(FlagSSI.readDone){
				FlagSSI.readDone=0;
				FlagSSI.ID=SSI_ID_READ_END_1;
				ROM_uDMAChannelDisable(UDMA_CHANNEL_SSI0TX);
				ROM_uDMAChannelDisable(UDMA_CHANNEL_SSI0RX);
				SSIDMADisable(SSI0_BASE,SSI_DMA_TX|SSI_DMA_RX);
				xmit_spi_my(0xFF);
			}
			break;
		case SSI_ID_READ_END_1:
			if(FlagSSI.dataRxAvail){
				FlagSSI.dataRxAvail=0;
				xmit_spi_my(0xFF);
				FlagSSI.ID=SSI_ID_READ_END_2;
				//				toggle_led[1]^=1;
				//				led(LED_BLUE,toggle_led[1]);
			}
			break;
		case SSI_ID_READ_END_2:
			if(FlagSSI.dataRxAvail){
				FlagSSI.dataRxAvail=0;
				FlagSSI.ID=SSI_ID_RESET;
				SSITimer=0;
				deSelectSSI();
				xmit_spi_my(0xFF);
				//				toggle_led[1]^=1;
				//				led(LED_BLUE,toggle_led[1]);
			}
			break;
		case SSI_ID_RESET:
			FlagSSI.dataRxAvail=0;
			FlagSSI.ID=SSI_ID_IDLE;
			FlagSSI.indexCount=0;
			break;
		default:
			break;
		}
	}
}
uint32_t ui32RcvDat;
void task_20Hz(){
	if(FlagTimer.Hz_20)
	{
		FlagTimer.Hz_20=0;
		/*
		 * Your code begin from here
		 */
		//	    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
		toggle_led[1]^=1;
		led(LED_BLUE,toggle_led[1]);
		//		SSIDataPut(SSI0_BASE, 0xFF);
		//		SSIDataPut(SSI0_BASE, 0xFA);
		//		SSIDataPut(SSI0_BASE, 0xFE);
		//SSIDataPut(SSI0_BASE, 0xFE);
		//SSIDataGet(SSI0_BASE, &ui32RcvDat);
		//		toggle_led[1]^=1;
		//		led(LED_BLUE,toggle_led[1]);
		//		 ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
		//		SSIDataPut(SSI0_BASE, SSI0_data.data16);
		//		led(LED_RED,0);
		//		SysCtlDelay(2);
		//		led(LED_RED,1);

//		SSIDMAEnable(SSI0_BASE,SSI_DMA_TX|SSI_DMA_RX);
//		ROM_uDMAChannelTransferSet(UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT,
//				UDMA_MODE_BASIC, &dataTX,(void *)(SSI0_BASE +SSI_O_DR) ,
//				512);
//
//		ROM_uDMAChannelTransferSet(UDMA_CHANNEL_SSI0RX | UDMA_PRI_SELECT,
//				UDMA_MODE_BASIC,(void *)(SSI0_BASE +SSI_O_DR),g_pui8RxPing,
//				512);
//
//		ROM_uDMAChannelEnable(UDMA_CHANNEL_SSI0TX);
//		ROM_uDMAChannelEnable(UDMA_CHANNEL_SSI0RX);




	}
}

void task_100Hz(){
	if(FlagTimer.Hz_100)
	{
		FlagTimer.Hz_100=0;
		/*
		 * Your code begin from here
		 */
	}
}
void task_50Hz(){
	if(FlagTimer.Hz_50)
	{
		FlagTimer.Hz_50=0;
		/*
		 * Your code begin from here
		 */

	}
}


void communication(){
	if(serialPC.Flag_receive){
		serialPC.Flag_receive=0;
		if(serialPC.Command_Data[0]==PC_HEADER_)
		{
			if(serialPC.Command_Data[1]&0x80)
			{
				switch(serialPC.Command_Data[1]){
				case  PC_SAM_SP_MODE_1_:
					SerialPutStrLn(UART_PC_,"sm1");

					break;
				case PC_SAM_SP_MODE_2_:
					SerialPutStrLn(UART_PC_,"sm2");
					break;
				default:
					break;

				}
			}
			else{
				if(serialPC.dataIndex==PC_SAM_MODE_1_DATALENGTH_)
				{
					switch(serialPC.Command_Data[1]>>5)
					{
					case PC_SAM_MODE_1_:
						SerialPutStrLn(UART_PC_,"M1");
						break;
					case PC_SAM_MODE_2_:
						SerialPutStrLn(UART_PC_,"M2");

						break;
					case PC_SAM_MODE_3_:
						SerialPutStrLn(UART_PC_,"M3");
						break;
					case PC_SAM_MODE_4_:
						SerialPutStrLn(UART_PC_,"M4");
						break;
					default:
						break;
					}

				}
				else
				{
					SerialPutStrLn(UART_PC_,"e_l_1");
				}

			}
		}
		else
		{
			serialPC.dataIndex=0;
			SerialPutStrLn(UART_PC_,"e_h");
		}
	}
}

unsigned char display_mode=COM2CTL_DISPLAY_MODE_1_;

void display_com(){
	char buffer[20];

	if(display_mode==COM2CTL_DISPLAY_MODE_0_)
	{

	}
	else if(display_mode==COM2CTL_DISPLAY_MODE_1_)
	{

		/*
		 * khao sat PID_z
		 */
		//		int2num(sam1.id,buffer);
		//		SerialPutStr_NonTer(UART_PC_,"id = ");
		//		SerialPutStr_NonTer(UART_PC_,buffer);
		//
		//		int2num(sam1.mode,buffer);
		//		SerialPutStr_NonTer(UART_PC_,"mode = ");
		//		SerialPutStr_NonTer(UART_PC_,buffer);
		//
		//		int2num(sam1.position12,buffer);
		//		SerialPutStr_NonTer(UART_PC_,"position12 = ");
		//		SerialPutStr_NonTer(UART_PC_,buffer);

	}
	SerialTerminator(UART_PC_);
}


