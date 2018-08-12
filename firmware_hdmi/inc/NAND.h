/*
 * NAND.h
 *
 *  Created on: 2016/03/19
 *      Author: Yoshiaki
 */

#ifndef NAND_H_
#define NAND_H_

#include <SI_C8051F380_Register_Enums.h>                // SFR declarations
#include "N25Q.h"

void NAND_write(NMX_uint8 *buf, NMX_uint16 size);
void NAND_read(NMX_uint8 *buf, NMX_uint16 size);


#define FOUR_BYTE_ENABLE \
   WR_R(SPICR, RD_R(SPICR) | SPICR_4B);

#define FOUR_BYTE_DISABLE \
   WR_R(SPICR, RD_R(SPICR) & ~SPICR_4B);

int16_t isFull(uint32_t frame);
uint32_t findNext();


#define MAX_FRAMES 256UL*1024*1024/8/sizeof(FrameData)	//512[Mb]/8[bit/byte]/sizeof(Frame)[byte/Frame]-1=11184810-1=11184809[Frame]。-1はエスケープシーケンス分？


/*#define FIND_NEXT(na,nb,next)\
		na = 0;\
		nb = MAX_FRAMES-1;\
		while(1){\
			next=(na+nb)/2;\
			if(isFull(next)){\
				na=next;\
				if((nb-na)<=1){\
					next = nb;\
					break;\
				}\
			}else{\
				if((nb-na)<=1){\
					break;\
				}\
				nb=next;\
			}\
		}*/

typedef enum
{
	RetSpiError,
	RetSpiSuccess
} SPI_STATUS;

// char stream definition for
typedef struct _structCharStream
{
	NMX_uint8* pChar;                                // buffer address that holds the streams
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
ReturnType nandWriteData32(NMX_uint32 n, NMX_uint32 data2write);
NMX_uint32 nandReadData32(NMX_uint32 n);
ReturnType nandWriteData16(NMX_uint32 n, NMX_uint16 data2write);
NMX_uint16 nandReadData16(NMX_uint32 n);
bool nandWriteFrame(uint32_t frame, FrameData *angularVelocity);
bool nandReadFrame(uint32_t frame, FrameData *angularVelocity);



#endif /* NAND_H_ */
