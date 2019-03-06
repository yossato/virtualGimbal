/**************************************************************************//**
 * (C) Copyright 2014 Silicon Labs Inc,
 * http://developer.silabs.com/legal/version/v11/Silicon_Labs_Software_License_Agreement.txt
 *
 *****************************************************************************/

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include <SI_C8051F380_Register_Enums.h>                // SI_SFR declarations
#include "VCPXpress.h"
#include "descriptor.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include "NAND.h"
#include <string.h>
#include "inc/ringBuffer.h"
#include "inc/mpu9250.h"
#define M_PI 3.14159265358
#include "math.h"
#include "inc/util.h"
#include "SPI-NAND.h"
#include <stdbool.h>
#include "inc/LED.h"
//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------



#define SYSCLK             48000000UL        // SYSCLK frequency in Hz

#define PACKET_SIZE 64
#define FRAME_POSITIONS

uint16_t xdata InCount;                   // Holds size of received packet
uint16_t xdata OutCount;                  // Holds size of transmitted packet
uint8_t xdata RX_Packet[PACKET_SIZE];     // Packet received from host
uint8_t xdata TX_Packet[PACKET_SIZE];     // Packet to transmit to host

NMX_uint8  pArray[2048+128];

SBIT(LED_blue, SFR_P0, 1);                // LED_blue=1 means ON
SBIT(LED_green, SFR_P0, 0);               // LED_green=1 means ON
SBIT(power_enable_bit, SFR_P2, 1);        // power_enable_bit=1 means POWER ON
SBIT(tact_switch, SFR_P2, 7);             // Tact Switch tact_switch == 1 means switch is pressed, ON
SBIT(PWM_bit, SFR_P0,2);                  // Motor driver gate bit. PWM_bit = 1 means drive a motor.

volatile uint8_t readyToWriteVCP = 0;	//VCPでwriteする準備が整っている時は1、準備が整ってないときは0とする。
volatile uint8_t readyToReadVCP = 0;
volatile uint8_t readyToPrintf = 0;
//extern FLASH_DEVICE_OBJECT *fdo;
code char welcome[] = "\n"
		"***********************************************\n"
		"* virtualGimbal rev.4.1                       *\n"
		"***********************************************\n";

code char help[] = "Press key.\n"
		"\n"
		"e:Erase all flash block\n"
		"u:Unlock Flash\n"
		"r:Read Flash\r"
		"w:Write Flash\n"
		"!:Reset Flash\n"
		"s:Spin for 1 second\n"
		"h:Show help\n"
		"t:Read angular velocity from flash\n"
		"g:Show Acceleration\n"
		"f:Show Angular velocity";

enum Status {
	recordingAngularVelocityInInterrupt	= 0x0001 << 0,//各速度記録中
};
//uint16_t status = 0x0000;
int C;
uint32_t na;
uint32_t nb;
uint32_t next;


#define TEST_FUNC(A,B) \
		C=A;\
		do{\
			B=A;\
		}while(1);\
		B=C;

typedef struct _VG_STATUS{
	volatile uint16_t status;	//!<状態。enum Statusの中から選ぶ
	volatile uint32_t write_col;		//!<書き込みポインタ。次回の書き込み位置を示す
	volatile uint32_t read_page;		//!<読み取りポインタ。次回の読み取り位置を示す
	volatile uint32_t write_page;
	volatile uint32_t time_ms;	//!<時間[ms],フレームレート毎に更新されるので精度が低いことに注意
	volatile FrameData acceleration;//!<加速度
	volatile FrameData angular_velocity;//!<角速度[rad/s]
	volatile uint32_t initial_read_page;
}VG_STATUS;

VG_STATUS xdata d;
FrameData frame_data;
//FrameData *p_frame_data;
// Function Prototypes
//-----------------------------------------------------------------------------
void Delay (void);
void Sysclk_Init (void);
static void Port_Init (void);
//void Timer0_Init ();
void timer_init();
void myAPICallback(void);
void spin();
int vcpPrintf(const char* format, ...);
void resetSystemStatus(VG_STATUS *st);
ReturnType Build_Address(NMX_uint16 block, NMX_uint8 page, NMX_uint16 col, NMX_uint32* addr);


bool printReturnType(ReturnType return_value){
	switch (return_value) {
	case Flash_Success:
		vcpPrintf("Flash_Success\n");
		return true;
	case Flash_AddressInvalid:
		vcpPrintf("Flash_AddressInvalid\n");
		break;
	case Flash_RegAddressInvalid:
		vcpPrintf("Flash_RegAddressInvalid\n");
		break;
	case Flash_MemoryOverflow:
		vcpPrintf("Flash_MemoryOverflow\n");
		break;
	case Flash_BlockEraseFailed:
		vcpPrintf("Flash_BlockEraseFailed\n");
		break;
	case Flash_PageNrInvalid:
		vcpPrintf("Flash_PageNrInvalid\n");
		break;
	case Flash_SubSectorNrInvalid:
		vcpPrintf("Flash_SubSectorNrInvalid\n");
		break;
	case Flash_SectorNrInvalid:
		vcpPrintf("Flash_SectorNrInvalid\n");
		break;
	case Flash_FunctionNotSupported:
		vcpPrintf("Flash_FunctionNotSupported\n");
		break;
	case Flash_NoInformationAvailable:
		vcpPrintf("Flash_NoInformationAvailable\n");
		break;
	case Flash_OperationOngoing:
		vcpPrintf("Flash_OperationOngoing\n");
		break;
	case Flash_OperationTimeOut:
		vcpPrintf("Flash_OperationTimeOut\n");
		break;
	case Flash_ProgramFailed:
		vcpPrintf("Flash_ProgramFailed\n");
		break;
	case Flash_SectorProtected:
		vcpPrintf("Flash_SectorProtected\n");
		break;
	case Flash_SectorUnprotected:
		vcpPrintf("Flash_SectorUnprotected\n");
		break;
	case Flash_SectorProtectFailed:
		vcpPrintf("Flash_SectorProtectFailed\n");
		break;
	case Flash_SectorUnprotectFailed:
		vcpPrintf("Flash_SectorUnprotectFailed\n");
		break;
	case Flash_SectorLocked:
		vcpPrintf("Flash_SectorLocked\n");
		break;
	case Flash_SectorUnlocked:
		vcpPrintf("Flash_SectorUnlocked\n");
		break;
	case Flash_SectorLockDownFailed:
		vcpPrintf("Flash_SectorLockDownFailed\n");
		break;
	case Flash_WrongType:
		vcpPrintf("Flash_WrongType\n");
		break;
	default:
		vcpPrintf("Undefined Error\n");
		break;
	}
	return false;
}

void reset(){
	RSTSRC |= RSTSRC_SWRSF__BMASK;
}

void spinAsSdCard(uint32_t spin_time_ms);
void resetRpToBeginAddress(VG_STATUS *st);


/**************************************************************************//**
 * @breif Main loop
 *
 * The main loop sets up the device and then waits forever. All active tasks
 * are ISR driven.
 *
 *****************************************************************************/
void main (void)
{
	PCA0MD &= ~PCA0MD_WDTE__BMASK;             // Disable watchdog timer

	VDM0CN = VDM0CN_VDMEN__ENABLED;            // Enable VDD Monitor
	Delay ();                                  // Wait for VDD Monitor to stabilize
	RSTSRC = RSTSRC_PORSF__SET;                // Enable VDD Monitor as a reset source

	Sysclk_Init ();                            // Initialize system clock
	Port_Init ();                              // Initialize crossbar and GPIO
	timer_init();

	resetSystemStatus(&d);

	IE_EA = 0;
//	d.write_col = findNext(d.WRp_init_value,MAX_FRAMES-2);
//	na = END_PAGE_OF_WRITABLE_REGION-1;
	d.write_page = findBeginOfWritablePages(BEGIN_PAGE_OF_WRITABLE_REGION,END_PAGE_OF_WRITABLE_REGION-1,pArray);
	d.write_col = 0;

	IE_EA = 1;

	//Initialize a gyro sensor.
	mpu9250_init();				//ジャイロセンサを初期化

	//Check VBUS to check that USB is connected or not.
	if(REG01CN & REG01CN_VBSTAT__BMASK){
		//USB is connected.
		char key = 0;

		// VCPXpress Initialization
		USB_Init(&InitStruct);

		// Enable VCPXpress API interrupts
		API_Callback_Enable(myAPICallback);

		IE_EA = 1;       // Enable global interrupts

		//Wait for opening virtual com port on USB.
		while(readyToWriteVCP==0);
		vcpPrintf(welcome);

		//LED on
		turnOnBlueLED();

		vcpPrintf(help);
		while(1){
			bool page_continue;
			uint16_t record = 0;
			uAddrType byte_addr;
			uAddrType row_addr;
			NMX_uint16 block;
			NMX_uint8 page;
			NMX_uint16 col;
			NMX_uint32 addr;
			ReturnType return_value;
			uint8_t character;
			uint8_t regs[4];
			int16_t i;
			int srand_value;
			bool skip_vcpPrint = false;
			volatile bool read_page_is_empty = true;
			uint8_t value;

			key = getchar();//Keyboard input
			vcpPrintf("\n");

			switch (key) {
			case 'p':   //VCP Printf Verification
			case 'P':
				vcpPrintf("Print test character string for vcpPrintf\n");
				for(character = 0x20;character<0x7f;++character){
					vcpPrintf("%c,",character);
//					wait_ms(1);
				}
				vcpPrintf("\n");
				FlashGetFeature(SPI_NAND_BLKLOCK_REG_ADDR,&regs[0]);
				FlashGetFeature(SPI_NAND_CONFIGURATION_REG_ADDR,&regs[1]);
				FlashGetFeature(SPI_NAND_STATUS_REG_ADDR,&regs[2]);
				FlashGetFeature(SPI_NAND_DIE_SELECT_REC_ADDR,&regs[3]);
				vcpPrintf("SPI_NAND_BLKLOCK_REG_ADDR:%x\n",(int)regs[0]);
				vcpPrintf("SPI_NAND_CONFIGURATION_REG_ADDR:%x\n",(int)regs[1]);
				vcpPrintf("SPI_NAND_STATUS_REG_ADDR:%x\n",(int)regs[2]);
				vcpPrintf("SPI_NAND_DIE_SELECT_REC_ADDR:%x\n",(int)regs[3]);

				break;
			case '!':
				FlashReset();
				vcpPrintf("Flash reseted.\n");
				break;
			case 'u':	//Disable Volatile Block Protection. This operation is required before erase or programming.
			case 'U':
				FlashUnlockAll();
				FlashGetFeature(SPI_NAND_BLKLOCK_REG_ADDR,&character);
				vcpPrintf("SPI_NAND_BLKLOCK_REG_ADDR:%x\n");
				vcpPrintf("All blocks are unlocked.\n");
				break;
			case 'e':	//Erase
			case 'E':
				vcpPrintf("\nDo you really want to erase data?\nYes:y No:N\n");
				key = getchar();
				if((key == 'y') || (key == 'Y')){
					//Erase
					vcpPrintf("Erasing...\n");
					byte_addr = 0;

					for(row_addr = 0; row_addr < (0x01UL<<29); row_addr+=(0x01UL<<18)){
						return_value = FlashBlockErase(row_addr);
						if(Flash_Success != return_value){
							printReturnType(return_value);
							vcpPrintf("FlashBlockErase failed at %ld\n",row_addr);
							vcpPrintf("Did you unlocked flash?\n");
							reset();
						}
						vcpPrintf("Erased row_addr:%ld\n",row_addr);
					}
					vcpPrintf("Erase complete.\n");
				}else{
					vcpPrintf("Canceled.\n");
				}

				resetSystemStatus(&d);
				break;

			case 'r':	//Read data from a flash memory
			case 'R':
				IE_EA = 0;
//				Build_Address(0,BEGIN_PAGE_OF_WRITABLE_REGION,0,&addr);
				d.read_page = BEGIN_PAGE_OF_WRITABLE_REGION;
				return_value = nandReadFramePage(&(d.read_page),pArray);
				IE_EA = 1;
				if(Flash_Success != return_value){
					printReturnType(return_value);
					vcpPrintf("FlashPageRead failed at %ld\n",BEGIN_PAGE_OF_WRITABLE_REGION);
				}
				else{
					for(i=0;i<PAGE_DATA_SIZE/sizeof(FrameData);++i){
						FrameData *p_frame_data = (FrameData*)pArray;
						vcpPrintf("%d:%d,%d,%d\n",i,p_frame_data[i].x,p_frame_data[i].y,p_frame_data[i].z);
					}
				}
				break;
			case 'w':	//Write data to a flash memory
			case 'W':
//				Build_Address(0,0x0c,0,&addr);
				IE_EA = 0;
//				p_frame_data = (FrameData*)pArray;
				for(i=0;i<PAGE_DATA_SIZE/sizeof(FrameData);++i){	//Generate data to write
					frame_data.x = i;
					frame_data.y = i;
					frame_data.z = i;
					nandWriteFramePage(&(d.write_col),&(d.write_page),&frame_data,pArray);
				}
//				return_value = nandWriteFramePage(addr,pArray,10);
				IE_EA = 1;
				if(Flash_Success != return_value){
					printReturnType(return_value);
					vcpPrintf("FlashPageProgram failed\n",addr);
					vcpPrintf("Did you unlocked flash?\n");
				}
				break;

			case 'a':	//Erase, Program and Read all byte. Verify flash memory driver.
			case 'A':
				//Erase
				vcpPrintf("Erasing...\n");
				byte_addr = 0;

				for(row_addr = 0; row_addr < (0x01UL<<29); row_addr+=(0x01UL<<18)){
					return_value = FlashBlockErase(row_addr);
					if(Flash_Success != return_value){
						printReturnType(return_value);
						vcpPrintf("FlashBlockErase failed at %ld\n",row_addr);
						vcpPrintf("Did you unlocked flash?\n");
						reset();
					}
					vcpPrintf("Erased row_addr:%ld\n",row_addr);
				}
				vcpPrintf("Erase complete.\n");

				//Program
				vcpPrintf("Programming...\n");
				srand_value = d.time_ms;
				srand(srand_value);


				for(block = 0;block<20;++block){
					vcpPrintf("Programming block %d ...\n",block);
					for(page = 0;page<64;++page){
						//Skip ECC spare, parameters and OTP regions.
						if(0 == block){
							if(0x0c > page){
								continue;
							}
						}
						for(col = 0;col<2048;++col){
							pArray[col] = (uint8_t)rand();
						}
						Build_Address(block,page,0,&addr);
						IE_EA = 0;
						return_value = FlashPageProgram(addr,pArray,2048);
						IE_EA = 1;
						if(Flash_Success != return_value){
							printReturnType(return_value);
							vcpPrintf("FlashPageProgram failed at %ld\n",addr);
						}
					}

				}
				vcpPrintf("Program complete.\n\n");

				//Read
				vcpPrintf("Reading...\n");
				srand(srand_value);

				for(block = 0;block<20;++block){
					vcpPrintf("Reading block %d ...\n",block);
					for(page = 0;page<64;++page){
						//Skip ECC spare, parameters and OTP regions.
						if(0 == block){
							if(BEGIN_PAGE_OF_WRITABLE_REGION > page){
								continue;
							}
						}
						Build_Address(block,page,0,&addr);
						IE_EA = 0;
						return_value = FlashPageRead(addr,pArray);
						IE_EA = 1;
						if(Flash_Success != return_value){
							printReturnType(return_value);
							vcpPrintf("FlashPageRead failed at %ld\n",addr);
						}
						skip_vcpPrint = false;
						for(col = 0;col<2048;++col){
							value = (uint8_t)rand();
							if(value!=pArray[col]){
								if(skip_vcpPrint != true){
									vcpPrintf("pArray:%d value:%d\n",(int16_t)pArray[col],(int16_t)value);
									vcpPrintf("Verify faild ad block:%u page:%u col:%u\n",block,(uint16_t)page,col);
									skip_vcpPrint = true;
								}

							}
						}
					}

				}
				vcpPrintf("Reading complete.\n");

				break;
			case 'h':
			case 'H':
				vcpPrintf(help);
				break;
			case 's':
			case 'S':

				vcpPrintf("Spin as a SD Card...\n");
				EIE1 &= ~EIE1_EUSB0__BMASK;
				spinAsSdCard(6000UL);
				EIE1 |= EIE1_EUSB0__BMASK;
				vcpPrintf("Spin ends.\n\n");
				break;

			case 't'://TODO:データを順番に出力
				resetRpToBeginAddress(&d);
				read_page_is_empty = false;
				page_continue = true;
				EIE1 &= ~0x80;
				while(page_continue){//レコードのループ
					uint32_t num=0;
#ifdef FRAME_POSITIONS
					vcpPrintf("Record %u",record);
					vcpPrintf("\n");
#endif
					while(!read_page_is_empty){
						IE_EA = 0;
						read_page_is_empty = isEmpty(d.read_page,pArray);
						IE_EA = 1;
						if(true == read_page_is_empty){
							break;
						}
						IE_EA = 0;
						nandReadFramePage(&(d.read_page),pArray);
						IE_EA = 1;

						for(i=0;i<PAGE_DATA_SIZE/sizeof(FrameData);++i){
							FrameData *p_frame_data = (FrameData*)pArray;
							vcpPrintf("%lu:%d,%d,%d\n",num++,p_frame_data[i].x,p_frame_data[i].y,p_frame_data[i].z);
						}

						if (d.read_page >= END_PAGE_OF_WRITABLE_REGION){
							page_continue = false;
							break;
						}
					}

					if(false == page_continue){
						break;
					}

					// Find next record
					IE_EA = 0;
					if (!(d.read_page >= END_PAGE_OF_WRITABLE_REGION-1)){
						if(!isEmpty(d.read_page+1,pArray)){ // separator is one page only
							++record;
							d.read_page += 1;
							read_page_is_empty = false;
							IE_EA = 1;
							continue;
						}
					}
					if (!(d.read_page >= END_PAGE_OF_WRITABLE_REGION-2)){
						if(!isEmpty(d.read_page+2,pArray)){ // separator is two pages
							++record;
							d.read_page += 2;
							read_page_is_empty = false;
							IE_EA = 1;
							continue;
						}
					}
					// Reached the end of records
					page_continue = false;
					IE_EA = 1;
					break;
				}
				EIE1 |= 0x80;
				//TODO:Flashメモリが満タンの時の処理

				break;
			case 'j':	//Output angular velocity as JSON format.
				resetRpToBeginAddress(&d);
				read_page_is_empty = false;
				page_continue = true;

				EIE1 &= ~0x80;
				vcpPrintf("{\n");
				vcpPrintf("    \"coefficient_adc_raw_value_to_rad_per_sec\":%0.10f,\n",1.f/16.4f*M_PI/180.0f);
				vcpPrintf("    \"frequency\":%f,\n",240.f);
				vcpPrintf("    \"angular_velocity\":[\n");
				while(page_continue){//レコードのループ
					vcpPrintf("        [");
					while(!read_page_is_empty){
						FrameData *p_frame_data = (FrameData*)pArray;
						IE_EA = 0;
						read_page_is_empty = isEmpty(d.read_page,pArray);
						IE_EA = 1;
						if(true == read_page_is_empty){
							break;
						}
						IE_EA = 0;
						nandReadFramePage(&(d.read_page),pArray);
						IE_EA = 1;

						vcpPrintf("%d,%d,%d",p_frame_data[0].x,p_frame_data[0].y,p_frame_data[0].z); //TODO; Support escape sequence.
						for(i=1;i<PAGE_DATA_SIZE/sizeof(FrameData);++i){
							vcpPrintf(",%d,%d,%d",p_frame_data[i].x,p_frame_data[i].y,p_frame_data[i].z);
						}

						if (d.read_page >= END_PAGE_OF_WRITABLE_REGION){
							page_continue = false;
							break;
						}

						IE_EA = 0;
						read_page_is_empty = isEmpty(d.read_page,pArray);
						IE_EA = 1;
						if(false == read_page_is_empty){
							vcpPrintf(",");
						}
					}

					if(false == page_continue){
						break;
					}

					// Find next record
					IE_EA = 0;
					if (!(d.read_page >= END_PAGE_OF_WRITABLE_REGION-1)){
						if(!isEmpty(d.read_page+1,pArray)){ // separator is one page only
							++record;
							d.read_page += 1;
							read_page_is_empty = false;
							IE_EA = 1;
							vcpPrintf("],\n");
							continue;
						}
					}
					if (!(d.read_page >= END_PAGE_OF_WRITABLE_REGION-2)){
						if(!isEmpty(d.read_page+2,pArray)){ // separator is two pages
							++record;
							d.read_page += 2;
							read_page_is_empty = false;
							IE_EA = 1;
							vcpPrintf("],\n");
							continue;
						}
					}
					// Reached the end of records
					page_continue = false;
					IE_EA = 1;
					vcpPrintf("]\n");
					break;
				}

				vcpPrintf("    ]\n");
				vcpPrintf("}\n");
				EIE1 |= 0x80;
				break;
			case 'g':
				//Show gravity
				mpu9250_pollingAcceleration(&frame_data);
				vcpPrintf("%ld,%ld,%ld\n",(int32_t)frame_data.x,(int32_t)frame_data.y,(int32_t)frame_data.z);
				break;
			case 'f':
				mpu9250_polling2(&frame_data);
				vcpPrintf("%ld,%ld,%ld\n",(int32_t)frame_data.x,(int32_t)frame_data.y,(int32_t)frame_data.z);
				break;
			default:
				;
			}
		}

	}else{


    	spinAsSdCard(0);//Infinite spin
	}
}

void turnOnBlueLED(){
	//LED on
	LED_blue = 1;
}

void turnOffBlueLED(){
	//LED on
	LED_blue = 0;
}

void turnOnGreenLED(){
	//LED on
	LED_green = 1;
}

void turnOffGreenLED(){
	//LED on
	LED_green = 0;
}

void spinAsSdCard(uint32_t spin_time_ms){
	static uint32_t endTime = 0;
	static uint32_t startTime;
	FlashUnlockAll();

	//Turned on as SD Card.
	//Record anguler velocity to Flash memory.

	d.status |= recordingAngularVelocityInInterrupt;//録画開始
	endTime = d.time_ms + spin_time_ms;
	while(1){
		//Sync 60 Hz
		startTime = d.time_ms;
		while(startTime==d.time_ms);
		if((0 != endTime) && (d.time_ms >= endTime)){
			break;
		}
	}
	d.status &= ~recordingAngularVelocityInInterrupt;
}

// Interrupt Service Routines
//-----------------------------------------------------------------------------

/**************************************************************************//**
 * @brief VCPXpress callback
 *
 * This function is called by VCPXpress. In this example any received data
 * is immediately transmitted to the UART. On completion of each write the
 * next read is primed.
 *
 *****************************************************************************/
VCPXpress_API_CALLBACK(myAPICallback)
{
	uint32_t INTVAL = Get_Callback_Source();

	if (INTVAL & DEVICE_OPEN)
	{
		readyToWriteVCP = 1;
		readyToPrintf = 1;					// Enable vcpPrintf after USB VCP is connected.
	}

	if (INTVAL & RX_COMPLETE)                          // USB Read complete
	{
		readyToReadVCP = 1;
	}

	if (INTVAL & TX_COMPLETE)                          // USB Write complete
	{
		readyToWriteVCP = 1;
	}

	if(INTVAL & DEVICE_CLOSE)
	{
		//Reset me if USB VCP is disconnected.
		//This reset may occurs when a terminal software on pc is closed.
		RSTSRC |= RSTSRC_SWRSF__BMASK;
	}
}


/**************************************************************************//**
 * @ clock initialization
 *
 * Slowest System clock: 1.5Mhz (48mhz/32) MHz  USB Clock: 48 MHz
 *****************************************************************************/
void Sysclk_Init (void)
{
	OSCICN |= 0x03;                     // Configure internal oscillator for
	CLKSEL  = 0x01;                     // External Oscillator
}


/**************************************************************************//**
 * @breif Port initialization
 *
 *
 *****************************************************************************/
static void Port_Init (void)
{
	P0MDIN 	= 0xFF;		// Flash memory SPI(P0.4-P0.7), P0.0-P0.1 (UART1) and P0.3 (XTAL2) are digital.
	P0MDOUT = 0xB7;		// Enable Flash Memory and UART
	P0 		= 0xE8;		// Enable SPI, disable UART1 (output low). An adapter circuit has two LEDs that connected to UART lines.
                        // If you set UART lines as low, LEDs are turned off.

	P1MDIN	= 0x00;		// All pins on P1 are analog.
	P1MDOUT	= 0x00;		// All pins are open-drain.
	P1		= 0xFF;		// All pins are high (open).

	P2MDIN = 0x3C;		// IMU_CS, IMU_SCK, IMU_MOSI and IMU_MISO are digital.
	P2MDOUT	= 0x1C;		// IMU_CS, IMU_SCK and IMU_MOSI are push-pull.
	P2		= 0xFF;	    // All pins are high.

	P3MDIN	|= 0x01;    // P3.0 is digital
	P3MDOUT	= 0x00;
	P3		|= 0x01;

	//Skip
	P0SKIP = 0xFF; //If you use UART 1, Set this value to 0xF3.
	P1SKIP = 0xFF;
	P2SKIP = 0xFF;

	XBR1    = 0x40;                     // Enable crossbar and enable

	//UART1
//	//	XBR2 = 0x01;                        // route UART 1 to crossbar

}

void assert(bool condition){
	if(true == condition){
		return;
	}
	while(1){//LED点滅の無限ループ
		//					LED1 = 1;
		turnOnBlueLED();
		wait_ms(1000);
		turnOffBlueLED();
		wait_ms(1000);
	}
}

//-----------------------------------------------------------------------------
// Initialization Subroutines
//-----------------------------------------------------------------------------
/**
 * @brief 120Hz周期の割り込み。呼び出し2回辺り1回、ジャイロセンサのデータをPCへUART経由で送信する
 *
 **/
INTERRUPT(Timer3_ISR, TIMER3_IRQn) {
	static uint8_t Times=0;
	static uint16_t oldStatus = 0;
	ReturnType return_value;
	turnOnBlueLED();

//	if((++Times) & 0x01){	//2回に1回実行、60Hz
		//フラッシュメモリへの波形書き込み
		if(d.status & recordingAngularVelocityInInterrupt){

			mpu9250_polling2(&d.angular_velocity);
			mpu9250_pollingAcceleration(&d.acceleration);

			// If writing position reaches the end page on flash, stop writing.
			assert(END_PAGE_OF_WRITABLE_REGION > d.write_page);//If データの末尾がFlashメモリの末尾-1(すなわちMAX_FRAMES-2)ならデータの区切りが必要なのでここで無限ループ then

			//Write angular velocity to the flash memory
			return_value = nandWriteFramePage(&(d.write_col),&(d.write_page),&(d.angular_velocity),pArray);
			assert(Flash_Success == return_value);


			d.time_ms += 4;//17ms追加

		}

		// Fill in remaining area with large data when the falling edge is detected.
		if((oldStatus & recordingAngularVelocityInInterrupt) && (!(d.status & recordingAngularVelocityInInterrupt))){
			//
			d.angular_velocity.x = 32767;
			d.angular_velocity.y = 32767;
			d.angular_velocity.z = 32767;
			while(d.write_col != 0){
				//Write angular velocity to the flash memory
				return_value = nandWriteFramePage(&(d.write_col),&(d.write_page),&(d.angular_velocity),pArray);
				assert(Flash_Success == return_value);
			}
			d.write_page += 2;	// Leaves two pages to empty. These empty pages indicates a separator between recording data.
		}

		//状態の保存
		oldStatus = d.status;

//	}
	turnOFFBlueLED();

	TMR3CN &= ~0x80; // Clear interrupt
}

void timer_init(){
	TMR3CN = 0x00;    // Stop Timer3; Clear TF3;
	CKCON &= ~0xC0;   // Timer3 clocked based on T3XCLK;
	TMR3RL = (0x10000 - (SYSCLK/12/240));  // Re-initialize reload value (240Hz, 4.2ms)
	TMR3 = 0xFFFF;    // Set to reload immediately
	EIE1 |= 0x80;     // Enable Timer3 interrupts(ET3)
	TMR3CN |= 0x04;   // Start Timer3(TR3)
}

/**************************************************************************//**
 * @brief Timer initialiation
 *
 * @param counts:
 *   calculated Timer overflow rate range is positive range of integer: 0 to 32767
 *
 * Configure Timer2 to 16-bit auto-reload and generate an interrupt at
 * interval specified by <counts> using SYSCLK/48 as its time base.
 *
 *****************************************************************************/
//void Timer0_Init()
//{
//   TH0 = TIMER_RELOAD_HIGH;            // Init Timer0 High register
//   TL0 = TIMER_RELOAD_LOW;             // Init Timer0 Low register
//   TMOD = TMOD_T0M__MODE1;             // Timer0 in 16-bit mode
//   CKCON = CKCON_SCA__SYSCLK_DIV_48;   // Timer0 uses a 1:48 prescaler
//   IE_ET0 = 1;                         // Timer0 interrupt enabled
//   TCON = TCON_TR0__RUN;               // Timer0 ON
//}

/**************************************************************************//**
 * @brief delay for approximately 5us @ 48Mhz
 *
 *****************************************************************************/
void Delay (void)
{
	int16_t x;

	for (x = 0; x < 500; x)
	{
		x++;
	}
}

/**
 * @brief printf関数などの動作に必要な関数。文字を受け取りリングバッファに書き込みます
 *
 **/
char putchar (char c)  {
	uint8_t cn = 0x0d;
	if(readyToPrintf == 0) return EOF;	//Printfの準備ができていなかったらデータを捨てる。Blockしないように対策。

	if (c == '\n')  {                // check for newline character
		rbPut(0x0d);
	}

	rbPut(c);
	return c;
}

char _getkey ()  {
	//VCPXpress library is not reentrant so we disable
	//  the USB interrupt to ensure it doesn't fire during
	//  the block write call
	EIE1 &= ~EIE1_EUSB0__BMASK;
	Block_Read(RX_Packet,PACKET_SIZE,&InCount);//VCPから一文字読み取り
	EIE1 |= EIE1_EUSB0__BMASK;

	while(readyToReadVCP != 1);
	readyToReadVCP = 0;

	return (RX_Packet[0]);                        // return value received through UART1
}

static int8_t readBuff[RINGSIZE];//スタック不足が疑われるため、グローバル変数にリングバッファの本体を置く
void spin(){
	//int8_t readBuff[RINGSIZE];

	if(readyToWriteVCP != 1) return;//VCPの準備ができていなければ終了

	while(!rbIsEmpty()){//バッファに残りがある
		int32_t length = rbGet(readBuff,128);//リングバッファからデータを読み出す
		readyToWriteVCP = 0;
//			   wait_ms(5);
		EIE1 &= ~EIE1_EUSB0__BMASK;
		Block_Write(readBuff, length, &OutCount);     // Start USB Write
		EIE1 |= EIE1_EUSB0__BMASK;
		while(readyToWriteVCP != 1);//送信完了を待つ


	}
}

int vcpPrintf(const char* format, ...){
	int32_t retval;
	va_list arg;
	va_start(arg,format);
	retval = vprintf(format,arg);
	va_end(arg);
	spin();
	return retval;
}

/**
 * @brief virtualGimbalの状態を初期化します
 *
 **/
void resetSystemStatus(VG_STATUS *st){
	uint32_t addr;
	Build_Address(0,BEGIN_PAGE_OF_WRITABLE_REGION,0,&addr);
	st->initial_read_page = BEGIN_PAGE_OF_WRITABLE_REGION;
	st->write_col = 0;
	st->read_page = 0;
	st->write_page = BEGIN_PAGE_OF_WRITABLE_REGION;
	st->status = 0;
	st->time_ms = 0;
}

void resetRpToBeginAddress(VG_STATUS *st){
	st->read_page = st->initial_read_page;
}
