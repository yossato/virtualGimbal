/*
 * NAND.h
 *
 *  Created on: 2016/03/19
 *      Author: Yoshiaki
 */

#ifndef NAND_H_
#define NAND_H_

#include <SI_C8051F380_Register_Enums.h>                // SFR declarations
#include "SPI-NAND.h"

uAddrType getAddress(NMX_uint32 page);
bool pageIsFilledWith0xFF(NMX_uint32 page, NMX_uint8 *pArray);
bool isEndOfRecordedPages(NMX_uint32 page);
bool isFullPages(NMX_uint32 page);
void NAND_write(NMX_uint8 xdata *buf, NMX_uint16 size);
void NAND_read(NMX_uint8 xdata *buf, NMX_uint16 size);


#define FOUR_BYTE_ENABLE \
   WR_R(SPICR, RD_R(SPICR) | SPICR_4B);

#define FOUR_BYTE_DISABLE \
   WR_R(SPICR, RD_R(SPICR) & ~SPICR_4B);

int16_t isFull(uint32_t frame);
uint32_t findNext(uint32_t begin, uint32_t end);


#define MAX_FRAMES 2048UL*1024*1024/8/sizeof(FrameData)-1	//2048[Mb]/8[bit/byte]/sizeof(Frame)[byte/Frame]



typedef enum
{
	RetSpiError,
	RetSpiSuccess
} SPI_STATUS;

// char stream definition for
typedef struct _structCharStream
{
	NMX_uint8 xdata* pChar;                                // buffer address that holds the streams
	NMX_uint32 length;                               // length of the stream in bytes
} CharStream;


// Acceptable values for SPI master side configuration
typedef enum _SpiConfigOptions
{
	OpsNull,  			// do nothing
	OpsWakeUp,			// enable transfer
	OpsInitTransfer,
	OpsEndTransfer,

} SpiConfigOptions;

typedef struct _structFrame
{
	int16_t x;
	int16_t y;
	int16_t z;
} FrameData;


SPI_STATUS Serialize_SPI(const CharStream* char_stream_send,
                         CharStream* char_stream_recv,
                         SpiConfigOptions optBefore,
                         SpiConfigOptions optAfter
                        );
void four_byte_addr_ctl(int enable);
void ConfigureSpi(SpiConfigOptions opt);
void ResetNAND(void);
ReturnType nandWriteFrame(uint32_t frame, FrameData *angularVelocity);
ReturnType nandReadFrame(uint32_t frame, FrameData *angularVelocity);



#endif /* NAND_H_ */
