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
#include "stdlib.h"
#include <stdint.h>
#include <stdarg.h>
#include "NAND.h"
 #include <string.h>
#include "ringBuffer.h"
#include "mpu9250.h"
#define M_PI 3.14159265358
#include "math.h"
#include "util.h"
/**************************************************************************//**
 * VCPXpress_Echo_main.c
 *
 * Main routine for VCPXpress Echo example.
 *
 * This example simply echos any data received over USB back up to the PC.
 *
 *****************************************************************************/

//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------



//#define BLINK_RATE         10              // Timer2 Interrupts per second
#define SYSCLK             48000000UL        // SYSCLK frequency in Hz
//#define PRESCALE           64             //Timer prescaler
//#define TIMER_RELOAD -(SYSCLK / PRESCALE / BLINK_RATE)
//
//#define TIMER_RELOAD_HIGH ((TIMER_RELOAD & 0xFF00)>>8)
//#define TIMER_RELOAD_LOW (TIMER_RELOAD & 0x00FF)

#define PACKET_SIZE 64

//#define FRAME_POSITIONS

uint16_t xdata InCount;                   // Holds size of received packet
uint16_t xdata OutCount;                  // Holds size of transmitted packet
uint8_t xdata RX_Packet[PACKET_SIZE];     // Packet received from host
uint8_t xdata TX_Packet[PACKET_SIZE];     // Packet to transmit to host

SBIT(LED1, SFR_P2, 7);                  // LED1='1' means ON

uint8_t readyToWriteVCP = 0;	//VCPでwriteする準備が整っている時は1、準備が整ってないときは0とする。
uint8_t readyToReadVCP = 0;
uint8_t readyToPrintf = 0;
extern FLASH_DEVICE_OBJECT *fdo;
const char welcome[] = "\n"
					   "***********************************************\n"
					   "* VirtualGimbal  - Post-processing stabilizer *\n"
					   "***********************************************\n";
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
	volatile uint32_t Wp;		//!<書き込みポインタ。次回の書き込み位置を示す
	volatile uint32_t Rp;		//!<読み取りポインタ。次回の読み取り位置を示す
	volatile uint32_t time_ms;	//!<時間[ms],フレームレート毎に更新されるので精度が低いことに注意
	volatile FrameData acceleration;//!<加速度
}VG_STATUS;

VG_STATUS xdata d;
FrameData testFrame;
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
void initVG(VG_STATUS *st);
float norm(float a[]);
float dot(float a[],float b[]);
/**************************************************************************//**
 * @breif Main loop
 *
 * The main loop sets up the device and then waits forever. All active tasks
 * are ISR driven.
 *
 *****************************************************************************/
void main (void)
{
//	int16_t len;
//	const char *pos = &welcome;

	static ReturnType ret;				// return variable
	static FLASH_DEVICE_OBJECT mfdo;	//フラッシュメモリの構造体を準備

	PCA0MD &= ~PCA0MD_WDTE__BMASK;             // Disable watchdog timer

   VDM0CN = VDM0CN_VDMEN__ENABLED;            // Enable VDD Monitor
   Delay ();                                  // Wait for VDD Monitor to stabilize
   RSTSRC = RSTSRC_PORSF__SET;                // Enable VDD Monitor as a reset source

   Sysclk_Init ();                            // Initialize system clock
   Port_Init ();                              // Initialize crossbar and GPIO
   timer_init();

   initVG(&d);

   ret = Driver_Init(&mfdo);	//Flashメモリドライバを初期化
   if (Flash_WrongType == ret)	//Flashメモリが正常に動いているか確認
   	{
   		vcpPrintf("Sorry, no device detected.\n");//エラーが発生した場合はここで停止
   		while(1);
   	}

   	fdo = &mfdo;
//   	aaa = 1;
//   	bbb = 2;
//   	aaa = TEST_FUNC(aaa,bbb);
//   	vcpPrintf("result:%d",aaa);
   	//あらかじめFlashメモリの末端を検索しておく
   	//FIND_NEXT(na,nb,next);
   	//d.Wp = next;
   	IE_EA = 0;
   	d.Wp = findNext();
   	IE_EA = 1;
   	//nandReadFrame((MAX_FRAMES-0)/2,&testFrame);
   	//nandReadFrame((MAX_FRAMES-0)/2+1,&testFrame);
   	//nandReadFrame((MAX_FRAMES-0)/2+2,&testFrame);

   	//ジャイロセンサ初期化
   	mpu9250_init();				//ジャイロセンサを初期化

    //USB接続かHDMI接続かチェック
    if(REG01CN & REG01CN_VBSTAT__BMASK){
    	//USB接続の場合
    	char key = 0;
    	uint32_t num[3];



		// VCPXpress Initialization
		USB_Init(&InitStruct);

//		//Clock Recovery Mode
//		USB0ADR |= CLKREC_CRE__ENABLED;

		// Enable VCPXpress API interrupts
		API_Callback_Enable(myAPICallback);

		IE_EA = 1;       // Enable global interrupts

		//USBに接続されVCPが初期化されるまで待機
		while(readyToWriteVCP==0);
		vcpPrintf(welcome);

		while(1){
			uint32_t validFrame;
			uint32_t record = 0;
			vcpPrintf("Press key.\n");
//			vcpPrintf("e:Erase, r:Read & Write, q:exit,\na:Erase sector(0), s:Record angular velocity, t:Read angular velocity\ng:Show Acceleration");
			vcpPrintf("e:Erase All, t:Read angular velocity\ng:Show Acceleration");
			key = getchar();//キーボード入力

			switch (key) {
				case 'e':	//データの全消去。ダイが2個載っているので、順番に消す。
				//違うな。ダイ1は書き込みできないから、無視しよう。
				case 'E':
					vcpPrintf("\nDo you really want to erase all data?\nYes:y No:N\n");
					key = getchar();
					if((key == 'y') || (key == 'Y')){
						vcpPrintf("Eraseing. Wait for about 400 seconds...\n");
//						vcpPrintf("Die 0...");
						FlashDieErase(0);
//						vcpPrintf(" OK\nDie 1...");
//						FlashDieErase(1);
						vcpPrintf(" OK\n");
					}
					d.Wp = 0;
					break;
					/*case 'r':
					//読み書きテスト

					num[0]=rand();
					num[1]=rand();
					num[2]=rand();
					vcpPrintf("\nTEST MODE 1...\n");
					IE_EA = 0;
					FlashSectorErase(0);	//セクタ消去。忘れるとデータが化けるはず。
					nandWriteData32(0,num[0]);	//テストデータとして乱数をアドレス0から2に書き込み。
					nandWriteData32(1,num[1]);
					nandWriteData32(2,num[2]);
					//テストデータの読み込み
					vcpPrintf("Write 0:%lu 1:%lu 2:%lu\n",num[0],num[1],num[2]);
					num[0]=nandReadData32(0);
					num[1]=nandReadData32(1);
					num[2]=nandReadData32(2);
//					vcpPrintf("Read  0:%lu 1:%lu 2:%lu\n",nandReadData32(0),nandReadData32(1),nandReadData32(2));
//					vcpPrintf("Read  0:%lu 1:%lu 2:%lu\n",num[0],num[1],num[2]);
					vcpPrintf("Read  0:%lu ",num[0]);
					vcpPrintf("1:%lu ",num[1]);
					vcpPrintf("2:%lu\n",num[2]);
					IE_EA = 1;
					break;
				case 'a':
					//セクタ0を消去
					FlashSectorErase(0);	//セクタ0消去。
					vcpPrintf("Sector 0 is erased.\n");
					d.Wp = 0;//TODO:データが64KB以上あったときに、不正な処理となってしまう。。。直す。
					break;

				case 's':
					//波形記録テスト セクタ0のみ有効


					//データの末尾を検索
//					frame = findNext();
					//TODO:満タンかどうか判断
					//記録開始
//					d.Wp = findNext();
					d.status |= recordingAngularVelocityInInterrupt;//開始
					vcpPrintf("Recording started...\n");
					vcpPrintf("Press s key to stop recording.\n");
					//記録停止
					key = 'a';
					while(key != 's'){//sキーが押されるまで待機
						key = getchar();
					}
					vcpPrintf("Recording stopped.\n");
					d.status &= ~recordingAngularVelocityInInterrupt;//停止
					break;*/

				case 't'://TODO:データを順番に出力
					d.Rp = 0;
//					record = 0;
					validFrame = 1;
					EIE1 &= ~0x80;
					while(validFrame){//レコードのループ
//						vcpPrintf("Press any key to read record %lu",record);
//						getchar();
						vcpPrintf("Record %lu",record);
						vcpPrintf("\n");
						while(1){//フレームのループ
							FrameData angularVelocity;
							//フラッシュメモリからデータを読み取る
							IE_EA = 0;
							nandReadFrame(d.Rp,&angularVelocity);
							IE_EA = 1;
							if(angularVelocity.x!=0xffff || angularVelocity.y!=0xffff || angularVelocity.z!=0xffff){
								//エスケープシーケンスでなければ出力
//								vcpPrintf("%d,%d,%d\n",angularVelocity.x,angularVelocity.y,angularVelocity.z);
								vcpPrintf("%f,%f,%f\n",(float)angularVelocity.y/16.4*M_PI/180.0,(float)angularVelocity.x/16.4*M_PI/180.0,-(float)angularVelocity.z/16.4*M_PI/180.0);
								++d.Rp;
								continue;
							}
							//エスケープシーケンスがあったら次のフレームをチェック
							IE_EA = 0;
							nandReadFrame(d.Rp+1,&angularVelocity);
							IE_EA = 1;
							if(angularVelocity.x==0xfffe && angularVelocity.y==0xfffe && angularVelocity.z==0xfffe){
								//(0xffff,0xffff,0xffff)
								//vcpPrintf("%d,%d,%d\n",(uint16_t)0xffff,(uint16_t)0xffff,(uint16_t)0xffff);
								vcpPrintf("%f,%f,%f\n",(float)((uint16_t)0xffff)/16.4*M_PI/180.0,(float)((uint16_t)0xffff)/16.4*M_PI/180.0,-(float)((uint16_t)0xffff)/16.4*M_PI/180.0);
								d.Rp += 2;//エスケープシーケンス分の2フレームを加算
								continue;
							}else if(angularVelocity.x==0x0000 && angularVelocity.y==0x0000 && angularVelocity.z==0x0000){
								//データの途切れ
#ifdef FRAME_POSITIONS
								vcpPrintf("Frame position:%lu\n",d.Rp);//これエスケープシーケンス分が含まれてしまっている
#endif
								//エスケープシーケンス分の2フレームを加算し移動
								d.Rp += 2;
								++record;
								break;
							}else if(angularVelocity.x==0xffff && angularVelocity.y==0xffff && angularVelocity.z==0xffff){
#ifdef FRAME_POSITIONS
								//データなし
								vcpPrintf("Frame position:%lu\n",d.Rp);//これエスケープシーケンス分が含まれてしまっている
								vcpPrintf("No data.\n");
#endif
								validFrame = 0;
								record = 0;
								break;
							}
						}


					}
					EIE1 |= 0x80;
					//TODO:Flashメモリが満タンの時の処理

					break;
				case 'g':
					//重力を表示
//					mpu9250_pollingAcceleration(&accel);
					vcpPrintf("%ld,%ld,%ld\n",(int32_t)d.acceleration.x,(int32_t)d.acceleration.y,(int32_t)d.acceleration.z);
					break;
				case 'f':
					while(1){
						static uint32_t startTime;// = d.time_ms;
						//フィルタの計算に用いるベクトル
						const float vecFront[3] = {-16384,0,0};//撮影中、カメラが正面を向いている状況
						const float vecDown[3] = {0,0,16384};//下向き。撮影中断中。
						static float accelLow[3]={0,0,16384};
						static float accelHigh[3]={0,0,16384};
						const float thF = 30.f/180.f*3.1415;//前向きの閾値
						const float thD = 30.f/180.f*3.1415;//下向きの閾値
						float tLowFront,tHighFront,tLowDown,tHighDown;
//						FrameData accel;
						//60Hzに同期させる
						startTime = d.time_ms;
						while(startTime==d.time_ms);

						//加速度計を監視
//						mpu9250_pollingAcceleration(&accel);

						//時定数の異なるLPFをかける。

						accelLow[0] = 0.005 * (float)d.acceleration.x + 0.995 * accelLow[0];
						accelLow[1] = 0.005 * (float)d.acceleration.y + 0.995 * accelLow[1];
						accelLow[2] = 0.005 * (float)d.acceleration.z + 0.995 * accelLow[2];

						accelHigh[0] = 0.1 * (float)d.acceleration.x + 0.9 * accelHigh[0];
						accelHigh[1] = 0.1 * (float)d.acceleration.y + 0.9 * accelHigh[1];
						accelHigh[2] = 0.1 * (float)d.acceleration.z + 0.9 * accelHigh[2];

						tLowFront = acos(dot(accelLow,vecFront)/(norm(accelLow)*norm(vecFront)));
						tHighFront = acos(dot(accelHigh,vecFront)/(norm(accelHigh)*norm(vecFront)));
						tLowDown = acos(dot(accelLow,vecDown)/(norm(accelLow)*norm(vecDown)));
						tHighDown = acos(dot(accelHigh,vecDown)/(norm(accelHigh)*norm(vecDown)));

						vcpPrintf("LF:%4.3f ",tLowFront/3.1415*180.0);
						vcpPrintf("HF:%4.3f ",tHighFront/3.1415*180.0);
						vcpPrintf("LD:%4.3f ",tLowDown/3.1415*180.0);
						vcpPrintf("HD:%4.3f\n",tHighDown/3.1415*180.0);

						if((tLowFront<thF)||(tHighFront<thF)){//カメラが前向きになったとき
							LED1=1;
						}else if((tLowDown<thD)&&(tHighDown<thD)){//カメラが下向きになったとき
							LED1=0;
						}
					}
					break;
				default:
//					vcpPrintf("\nExit.\n");
//					return;
					;
				}
		}

    }else{
    	//HDMI接続の場合
    	IE_EA = 1;       // Enable global interrupts
    	LED1 = 0;		//LED消灯
    	while(1){
			static uint32_t startTime;// = d.time_ms;
			//フィルタの計算に用いるベクトル
			const float vecFront[3] = {-16384,0,0};//撮影中、カメラが正面を向いている状況
			const float vecDown[3] = {0,0,16384};//下向き。撮影中断中。
			static float accelLow[3]={0,0,16384};
			static float accelHigh[3]={0,0,16384};
			const float thF = 30.f/180.f*3.1415;//前向きの閾値
			const float thD = 30.f/180.f*3.1415;//下向きの閾値
			float tLowFront,tHighFront,tLowDown,tHighDown;
//			FrameData accel;
			//60Hzに同期させる
			startTime = d.time_ms;
			while(startTime==d.time_ms);

			//加速度計を監視
//			mpu9250_pollingAcceleration(&accel);

			//時定数の異なるLPFをかける。

			accelLow[0] = 0.005 * (float)d.acceleration.x + 0.995 * accelLow[0];
			accelLow[1] = 0.005 * (float)d.acceleration.y + 0.995 * accelLow[1];
			accelLow[2] = 0.005 * (float)d.acceleration.z + 0.995 * accelLow[2];

			accelHigh[0] = 0.1 * (float)d.acceleration.x + 0.9 * accelHigh[0];
			accelHigh[1] = 0.1 * (float)d.acceleration.y + 0.9 * accelHigh[1];
			accelHigh[2] = 0.1 * (float)d.acceleration.z + 0.9 * accelHigh[2];

			tLowFront = acos(dot(accelLow,vecFront)/(norm(accelLow)*norm(vecFront)));
			tHighFront = acos(dot(accelHigh,vecFront)/(norm(accelHigh)*norm(vecFront)));
			tLowDown = acos(dot(accelLow,vecDown)/(norm(accelLow)*norm(vecDown)));
			tHighDown = acos(dot(accelHigh,vecDown)/(norm(accelHigh)*norm(vecDown)));

			vcpPrintf("LF:%4.3f ",tLowFront/3.1415*180.0);
			vcpPrintf("HF:%4.3f ",tHighFront/3.1415*180.0);
			vcpPrintf("LD:%4.3f ",tLowDown/3.1415*180.0);
			vcpPrintf("HD:%4.3f\n",tHighDown/3.1415*180.0);

			if((tLowFront<thF)||(tHighFront<thF)){//カメラが前向きになったとき
//				LED1=1;
				d.status |= recordingAngularVelocityInInterrupt;//録画開始
			}else if((tLowDown<thD)&&(tHighDown<thD)){//カメラが下向きになったとき
//				LED1=0;
				d.status &= ~recordingAngularVelocityInInterrupt;//停止
			}
		}



    }



   while (1)
   {
//	   static int n=0;
//	   printf("%d\n",n++);
//printf("-0X");
//	   len = strlen(welcome)+1;
//	   while(len > 0){
//		   Block_Write(pos, len, &OutCount);
//		   len -= OutCount;
//	   }
   }                                // Spin forever
}

float norm(float a[]){
	return sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
}
float dot(float a[],float b[]){
	return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
}

// Interrupt Service Routines
//-----------------------------------------------------------------------------

/**************************************************************************//**
 * @brief Timer2_ISR
 *
 * This routine changes the state of the LED whenever Timer2 overflows.
 *
 *****************************************************************************/
//INTERRUPT(Timer0_ISR, TIMER0_IRQn)
//{
//  TMR2CN_TF2H = 0;                    // Clear Timer2 interrupt flag
//  TH0 = TIMER_RELOAD_HIGH;            // Reload Timer0 High register
//  TL0 = TIMER_RELOAD_LOW;             // Reload Timer0 Low register
//
//  LED1 = !LED1;                       // Change state of LED
//}


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
//   uint8_t i;


   if (INTVAL & DEVICE_OPEN)
   {
	   //PC側が安定するまで待つ
//	uint32_t ae;
//	for(ae=0;ae<200000;ae++){
//		Delay ();
//	}

	   readyToWriteVCP = 1;
		readyToPrintf = 1;	//USBに接続されたら、初めてprintf関数を有効にする
//      Block_Read(RX_Packet, PACKET_SIZE, &InCount);   // Start first USB Read
   }

   if (INTVAL & RX_COMPLETE)                          // USB Read complete
   {
	   readyToReadVCP = 1;
//      for (i=0; i<InCount;i++)                        // Copy received packet to output buffer
//      {
//         TX_Packet[i] = RX_Packet[i];
//      }
//      Block_Write(TX_Packet, InCount, &OutCount);     // Start USB Write
   }

   if (INTVAL & TX_COMPLETE)                          // USB Write complete
   {

//      Block_Read(RX_Packet, PACKET_SIZE, &InCount);   // Start next USB Read
	   readyToWriteVCP = 1;
//
//	   int8_t readBuff[RINGSIZE];
//	   //VCPの仕事をこなす
//	   if((!rbIsEmpty())){//バッファに残りがある
//		   int32_t length = rbGet(readBuff,RINGSIZE-1);//リングバッファからデータを読み出す
//		   Block_Write(readBuff, length, &OutCount);     // Start USB Write
//		   //	   readyToWriteVCP = 0;
//	   }else{
//		   readyToWriteVCP = 1;
//	   }

   }

   if(INTVAL & DEVICE_CLOSE)
   {
	   //VCP接続が解除されたら、ソフトウェアでリセットする。たぶん、PC側のソフトを終了したときに発生する。
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
//	OSCICN  = OSCICN_IOSCEN__ENABLED
//            | OSCICN_IFCN__SYSCLK_DIV_2;        // Enable intosc (/4 / 2 = 6Mhz)
//  CLKSEL = CLKSEL_CLKSL__DIVIDED_HFOSC_DIV_4;   // select full speed sysclk
}

/**************************************************************************//**
 * @breif Port initialization
 *
 * P2.2   digital   push-pull    LED1
 * P2.3   digital   push-pull    LED2
 *
 *****************************************************************************/
static void Port_Init (void)
{
	P0MDIN 	= 0xFF;		// All pins on P0 are digital
	P0MDOUT = 0xB0;		// enable Flash Memory
	P0 		= 0xEF;

	P1MDIN	= 0x02;		// SW2 are digital. Set SPI(SD) pins as a analog initially.
	P1MDOUT	= 0x00;		// All pins are open-drain.
	P1		= 0xFF;		// All pins are high (open).

	P2MDIN = 0xBC;		// IMU_CS, IMU_SCK, IMU_MOSI, IMU_MISO, P2.7 are digital.
	P2MDOUT	= 0x9C;		// IMU_CS, IMU_SCK, IMU_MOSI and P2.7(LED) are push-pull.
	P2		= 0xFF;

	P3MDIN	|= 0x01;		// P3.0 is digital
	P3MDOUT	= 0x00;
	P3		|= 0x01;

	//Skip
	P0SKIP = 0xFF;
	P1SKIP = 0xFF;
	P2SKIP = 0xFF;

	//UART1
//	XBR2 = 0x01;                        // route UART 1 to crossbar

	XBR1    = 0x40;                     // Enable crossbar and enable

	/////////////
//   P2MDOUT   = P2MDOUT_B2__PUSH_PULL
//               | P2MDOUT_B3__PUSH_PULL; // P2.2 - P2.3 are push-pull
//   P2SKIP    = P2SKIP_B2__SKIPPED
//               | P2SKIP_B3__SKIPPED;    // P2.2 - P2.3 skipped
//   XBR1      = XBR1_XBARE__ENABLED;     // Enable the crossbar
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
	static FrameData angularVelocity;

	if((++Times) & 0x01){	//2回に1回実行、60Hz
		d.time_ms += 17;//17ms追加

		//フラッシュメモリへの波形書き込み
		if(d.status & recordingAngularVelocityInInterrupt){
			//Flashメモリの末尾に達したら書き込みを停止
			if(MAX_FRAMES-2 <= d.Wp){//If データの末尾がFlashメモリの末尾-1(すなわちMAX_FRAMES-2)ならデータの区切りが必要なのでここで無限ループ then
				while(1){//LED点滅の無限ループ
					LED1 = 1;
					wait_ms(100);
					LED1 = 0;
					wait_ms(100);
				}
			}//end if

			//書き込み開始時の立ち上がりエッジを監視
			if((!(oldStatus & recordingAngularVelocityInInterrupt))&&(d.Wp!=0)){//d.Wp==0の時は、前回の記録がないのでスキップ
				LED1 = 1;
				//前回の記録の末端に、記録終了の合図を書き込む
				angularVelocity.x = 0xffff;//初期値が0xffffなのだから書き込む意味ない気がする。
				angularVelocity.y = 0xffff;
				angularVelocity.z = 0xffff;
				nandWriteFrame(d.Wp,&angularVelocity);
				d.Wp += 1;

				angularVelocity.x = 0;
				angularVelocity.y = 0;
				angularVelocity.z = 0;
				nandWriteFrame(d.Wp,&angularVelocity);
				d.Wp += 1;

				LED1 = 0;
			}


			LED1 = 1;
	//		mpu9250_polling();
			mpu9250_polling2(&angularVelocity);
			//フラッシュメモリにデータを書き込む
			nandWriteFrame(d.Wp,&angularVelocity);
			d.Wp += 1;		//書き込みポインタを進める

			//書き込んだデータが万が一エスケープシーケンス(0xffff,0xffff,0xffff)なら、続けて1フレーム付加する
			if(angularVelocity.x==0xffff && angularVelocity.y==0xffff && angularVelocity.z==0xffff){
				angularVelocity.x = 0xfffe;
				angularVelocity.y = 0xfffe;
				angularVelocity.z = 0xfffe;
				nandWriteFrame(d.Wp,&angularVelocity);
				d.Wp += 1;		//書き込みポインタを進める
			}

			//LEDを消灯
			LED1 = 0;
			//TODO:Flashメモリが満タンの時の処理が必要だな。。。。
		}

		//状態の保存
		oldStatus = d.status;

		//加速度の計測
		mpu9250_pollingAcceleration(&d.acceleration);
	}
	TMR3CN &= ~0x80; // Clear interrupt
}

void timer_init(){
	TMR3CN = 0x00;    // Stop Timer3; Clear TF3;
	CKCON &= ~0xC0;   // Timer3 clocked based on T3XCLK;
	TMR3RL = (0x10000 - (SYSCLK/12/120));  // Re-initialize reload value (120Hz, 8.3ms)
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
	   while(readyToWriteVCP != 1);//送信完了を待つ
//	   wait_ms(5);
	   EIE1 &= ~EIE1_EUSB0__BMASK;
	   Block_Write(readBuff, length, &OutCount);     // Start USB Write
		EIE1 |= EIE1_EUSB0__BMASK;
	   readyToWriteVCP = 0;

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
void initVG(VG_STATUS *st){
	st->Wp = 0;
	st->Rp = 0;
	st->status = 0;
	st->time_ms = 0;
}
