/*
 * NAND.c
 *
 *  Created on: 2016/03/19
 *      Author: Yoshiaki
 */

#include <SI_C8051F380_Register_Enums.h>                // SFR declarations
#include "NAND.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#define NULL_PTR 0


uAddrType getAddress(NMX_uint32 page){
	return page << 12;
}

bool pageIsFilledWith0xFF(NMX_uint32 page, NMX_uint8 *pArray){
	int16_t col;

	FlashPageRead(getAddress(page),pArray);

	for(col = 0;col<PAGE_DATA_SIZE;++col){
		if(pArray[col] != (int8_t)0xff){
			return false;
		}
	}
	return true;
}

bool isFullPages(NMX_uint32 page, NMX_uint8 *pArray){
	// Page is bigger than begin + 1
	if(page >= BEGIN_PAGE_OF_WRITABLE_REGION+1){
		if (!pageIsFilledWith0xFF(page-1,pArray)){
			return false;
		}
	}
	if (!pageIsFilledWith0xFF(page,pArray)){
		return false;
	}
	return true;
}

uint32_t findBeginOfWritablePages(uint32_t begin_page, uint32_t end_page, NMX_uint8 *pArray){
	uint32_t nm;

	while(1){
		nm=(begin_page+end_page)/2;//仮の解
		if(isFullPages(nm,pArray)){
			begin_page=nm;
			if((end_page-begin_page)<=1){//終了条件
				return end_page;
			}
		}else{
			if((end_page-begin_page)<=1){//終了条件
				return nm;
			}
			end_page=nm;
		}
	}
}

ReturnType nandWriteFramePage(uint32_t *frame, uint32_t *page, FrameData *angularVelocity, NMX_uint8 *pArray){
	code uint32_t FRAMES_IN_PAGE = PAGE_DATA_SIZE/sizeof(FrameData);
	uint32_t col = *frame*sizeof(FrameData);
	memcpy(pArray+col,angularVelocity,sizeof(FrameData));
	(*frame)++;
	if(*frame>=FRAMES_IN_PAGE){
		*frame = 0;
		(*page)++;
		return FlashPageProgram(getAddress(*page),pArray,FRAMES_IN_PAGE*sizeof(FrameData));
	}
	return Flash_Success;
}

ReturnType nandReadFramePage(uint32_t *page, NMX_uint8 *pArray){
	return FlashRead(getAddress(*page),pArray,PAGE_DATA_SIZE);
}

ReturnType nandWriteFrame(uint32_t frame, FrameData *angularVelocity){
	uint32_t addr = frame * sizeof(FrameData);
	addr = (0x1FFFF000UL & (addr << 1)) + (addr & 0x7FFUL); // | BLOCK 11 bits | PAGE 6 bits | DUMMU 1 bit | BYTE 11 bits |
	return FlashPageProgram(addr,(uint8_t *)angularVelocity,sizeof(FrameData));

}

ReturnType nandReadFrame(uint32_t frame, FrameData *angularVelocity){
	uint32_t addr = frame * sizeof(FrameData);
	addr = (0x1FFFF000UL & (addr << 1)) + (addr & 0x7FFUL); // | BLOCK 11 bits | PAGE 6 bits | DUMMU 1 bit | BYTE 11 bits |
	return FlashRead(addr,(uint8_t *)angularVelocity,sizeof(FrameData));
}

/**
 * @brief フレームにデータがあるかどうか確認します
 * @param [in] frame フレーム位置
 * @retval false:未使用 true:使用済み
 **/
int16_t isFull(uint32_t frame){
	//TODO:領域外の時はどうする？？？
	static FrameData f;
//	uint32_t position16 = frame*1*3;//フレーム数×16bit×3軸
	//3軸のデータを取り出す
	nandReadFrame(frame,&f);
	++frame;
//	f.x = nandReadData16(position16);
//	f.y = nandReadData16(position16+1);
//	f.z = nandReadData16(position16+2);
	//全軸が0xFFFFかどうか確認
	if(f.x==0xffff && f.y==0xffff && f.z==0xffff){
		//未使用の可能性が高いが、次のフレームも併せて確認する
		//次のフレームも確認
		nandReadFrame(frame,&f);
		++frame;
//		f.x = nandReadData16(position16+3);
//		f.y = nandReadData16(position16+4);
//		f.z = nandReadData16(position16+5);
		if(f.x==0xffff && f.y==0xffff && f.z==0xffff){
			//未使用
			return 0;
		}else{
			//使用済み
			return 1;
		}
	}else{
		//使用済み
		return 1;
	}
}

/**
 * @brief 二分法でデータ末尾の次の位置を探します。書き込みはこの位置から実行してください。
 * 0xffff x 3 + 0xfefe x 3の2フレームで0xffffのデータを表す。0xffff x 6で不定（基本的に空き領域）。0xffff x 3 + 0x0000 x 3でデータの区切り
 *
 **/


uint32_t findNext(uint32_t begin, uint32_t end){


	uint32_t nm;

	while(1){
		nm=(begin+end)/2;//仮の解
		if(isFull(nm)){
			begin=nm;
			if((end-begin)<=1){//終了条件,たぶんNANDがすべて空っぽの時
				return end;
			}
		}else{
			if((end-begin)<=1){//終了条件
				return nm;
			}
			end=nm;
		}

	}//while((nb-begin)>1);//本当？
//	return begin;
}

void nand_wait_8n6clk(uint8_t i){
  while(i--);
}
#define nand_cs_wait() //nand_wait_8n6clk(0) 	// No wait required because a flash memory is enough fast.
#define nand_clk_wait() //nand_wait_8n6clk(2) 	// No wait required because a flash memory is enough fast.

//ピンアサイン
#define nand_clk_up()      (P0 |=  0x10)
#define nand_clk_down()    (P0 &= ~0x10)
#define nand_out_up()      (P0 |=  0x20)
#define nand_out_down()    (P0 &= ~0x20)
#define nand_cs_assert()   (P0 &= ~0x80)
#define nand_cs_deassert() (P0 |=  0x80)

#define nand_is_in_up()    (P0 & 0x40)
#define nand_MOSI_is_in_up()	(P0 & 0x20)



void NAND_write(NMX_uint8 xdata *buf, NMX_uint16 size){
	int i=0;
  for(; size--; buf++){
    NMX_uint8 mask = 0x80;
    do{
      nand_clk_down();
      if((*buf) & mask){nand_out_up();}else{nand_out_down();}
      nand_clk_wait();
      nand_clk_up();
      nand_clk_wait();

    }while(mask >>= 1);

    nand_clk_down();
  }
}

void NAND_read(NMX_uint8 xdata *buf, NMX_uint16 size){
  for(; size--; buf++){
    NMX_uint8 temp = 0;
    NMX_uint8 mask = 0x80;
    do{
      nand_clk_down();
      nand_clk_wait();
      nand_clk_up();
      nand_clk_wait();
      if(nand_is_in_up()) temp |= mask;
    }while(mask >>= 1);
    *buf = temp;
    nand_clk_down();
  }
}

#define NAND_set(address, value) { \
  static const __code NMX_uint8 addr_value[2] = {address, value}; \
  nand_cs_assert(); \
  nand_cs_wait(); \
  NAND_write(addr_value, sizeof(addr_value)); \
  nand_cs_deassert(); \
  nand_cs_wait(); \
}

#define NAND_get2(address, vp, size) { \
  static const __code NMX_uint8 addr[1] = {0x80 | address}; \
  nand_cs_assert(); \
  nand_cs_wait(); \
  NAND_write(addr, sizeof(addr)); \
  NAND_read(vp, size); \
  nand_cs_deassert(); \
  nand_cs_wait(); \
}

#define NAND_get(address, value) NAND_get2(address, (NMX_uint8 *)&(value), sizeof(value))

#define EXT_MOD



/*******************************************************************************
Function:     ConfigureSpi(SpiConfigOptions opt)
Arguments:    opt configuration options, all acceptable values are enumerated in
              SpiMasterConfigOptions, which is a typedefed enum.
Return Values:There is no return value for this function.
Description:  This function can be used to properly configure the SPI master
              before and after the transfer/receive operation
Pseudo Code:
   Step 1  : perform or skip select/deselect slave
   Step 2  : perform or skip enable/disable transfer
   Step 3  : perform or skip enable/disable receive
*******************************************************************************/

void ConfigureSpi(SpiConfigOptions opt)
{
	switch (opt)
	{
	case OpsWakeUp:
		nand_cs_assert();
		nand_cs_wait();
		break;
	case OpsInitTransfer:
		break;
	case OpsEndTransfer:
		nand_cs_wait();
		nand_cs_deassert();
		nand_cs_wait();
		break;
	default:
		break;
	}
}

void ResetNAND(void){
//	NMX_uint8  cRDID = SPI_FLASH_INS_REN;
//	NMX_uint16 tx_len = 1;
//	ConfigureSpi(OpsWakeUp);
//
//	NAND_write(&cRDID,tx_len);
//	ConfigureSpi(OpsEndTransfer);
//
//	cRDID = SPI_FLASH_INS_RMEM;
//	ConfigureSpi(OpsWakeUp);
//		NAND_write(&cRDID,tx_len);
//		ConfigureSpi(OpsEndTransfer);
}


/*******************************************************************************
Function:     Serialize(const CharStream* char_stream_send,
					CharStream* char_stream_recv,
					SpiMasterConfigOptions optBefore,
					SpiMasterConfigOptions optAfter
				)
Arguments:    char_stream_send, the char stream to be sent from the SPI master to
              the Flash memory, usually contains instruction, address, and data to be
              programmed.
              char_stream_recv, the char stream to be received from the Flash memory
              to the SPI master, usually contains data to be read from the memory.
              optBefore, configurations of the SPI master before any transfer/receive
              optAfter, configurations of the SPI after any transfer/receive
Return Values:TRUE
Description:  This function can be used to encapsulate a complete transfer/receive
              operation
Pseudo Code:
   Step 1  : perform pre-transfer configuration
   Step 2  : perform transfer/ receive
   Step 3  : perform post-transfer configuration
*******************************************************************************/
SPI_STATUS Serialize_SPI(const CharStream* char_stream_send,
                         CharStream* char_stream_recv,
                         SpiConfigOptions optBefore,
                         SpiConfigOptions optAfter
                        )
{

	NMX_uint8 *char_send, *char_recv;

	NMX_uint16 rx_len = 0, tx_len = 0;

//	int i;
#ifdef ENABLE_PRINT_DEBUG
	int i;
	printf("\nSEND: ");
	for(i=0; i<char_stream_send->length; i++)
		printf(" 0x%x ", char_stream_send->pChar[i]);
	printf("\n");
#endif

	tx_len = char_stream_send->length;
	char_send = char_stream_send->pChar;

	if (NULL_PTR != char_stream_recv)
	{
		rx_len = char_stream_recv->length;
		char_recv = char_stream_recv->pChar;
	}

	ConfigureSpi(optBefore);

	if(tx_len > 0){
		NAND_write(char_send, tx_len);
	}

	if (rx_len > 0){
		NAND_read(char_recv, rx_len);
	}

#ifdef ENABLE_PRINT_DEBUG
	printf("\nRECV: ");
	for(i=0; i<char_stream_recv->length; i++)
		printf(" 0x%x ", char_stream_recv->pChar[i]);
	printf("\n");
#endif

	ConfigureSpi(optAfter);

	return RetSpiSuccess;
}


