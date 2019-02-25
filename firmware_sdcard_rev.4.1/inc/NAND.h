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

typedef struct _structFrame
{
	int16_t x;
	int16_t y;
	int16_t z;
} FrameData;

uAddrType getAddress(NMX_uint32 page);
bool isEmpty(NMX_uint32 page, NMX_uint8 *pArray);
bool isEndOfRecordedPages(NMX_uint32 page);
bool isInEmptyRegions(NMX_uint32 page, NMX_uint8 *pArray);
void NAND_write(NMX_uint8 xdata *buf, NMX_uint16 size);
void NAND_read(NMX_uint8 xdata *buf, NMX_uint16 size);
ReturnType nandWriteFramePage(uint32_t *col, uint32_t *page, FrameData *angularVelocity, NMX_uint8 *pArray);
ReturnType nandReadFramePage(uint32_t *page, NMX_uint8 *pArray);
uint32_t findBeginOfWritablePages(uint32_t begin_page, uint32_t end_page, NMX_uint8 *pArray);

#define FOUR_BYTE_ENABLE \
   WR_R(SPICR, RD_R(SPICR) | SPICR_4B);

#define FOUR_BYTE_DISABLE \
   WR_R(SPICR, RD_R(SPICR) & ~SPICR_4B);

int16_t isFull(uint32_t frame);
uint32_t findNext(uint32_t begin, uint32_t end);






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
