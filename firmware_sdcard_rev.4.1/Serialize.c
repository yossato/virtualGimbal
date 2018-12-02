/**********************  DRIVER FOR SPI CONTROLLER  **********************

   Filename:     Serialize.c
   Description:  This files is aimed at giving a basic example of
				 SPI Controller to drive the SPI serial interface.

   Version:    0.2
   Date:       Decemb. 2011
   Authors:    Micron S.r.l. Arzano (Napoli)


   THIS PROGRAM IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER
   EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTY
   OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. THE ENTIRE RISK
   AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM IS WITH YOU. SHOULD THE
   PROGRAM PROVE DEFECTIVE, YOU ASSUME THE COST OF ALL NECESSARY SERVICING,
   REPAIR OR CORRECTION.
   
********************************************************************************

*******************************************************************************/
#include "Serialize.h"

/* this function is called only from Serialize_SPI() */
SPI_STATUS ConfigureSpi(SpiConfigOptions opt);

/*******************************************************************************
     SPI Controller Init  
	 
Function:        SpiCtlInit()
Arguments:       There is no argument for this function
Return Values:   Return SpiSuccess
Description:     This function has to be called at the beginnning to configure 
				 the port.   

/******************************************************************************/
SPI_STATUS SpiCtlInit(void)
{
	/* Clean the Controller Register */
	WR_R(SPICR, RD_R(SPICR) & ZERO_MASK);
	
	/* 
	   Set clock frequency to 25 MHz, set 8 DUMMY cycles, disable WP and 
	   HOLD modes 
	 */
	WR_R(SPICR, RD_R(SPICR) | SPICR_CLK(0x1) | SPICR_DUMMY(8) | SPICR_WP | SPICR_HOLD);
	
#ifdef SPI_NAND
	/* Set SPI_NAND_ENABLE bit in the SPI Controller Register */
	WR_R(SPICR, RD_R(SPICR) | SPI_NAND_EN);
#endif 

#ifdef EXT_MOD
	/* enable extended mode */
	EXT_ENABLE;
#endif

	/*Set byte size transfer*/	
	SET_TXRXSIZE(8);
	
	return RetSpiSuccess;

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

	uint8 *char_send, *char_recv;
	uint16 rx_len = 0, tx_len = 0;


	tx_len = char_stream_send->length;
	char_send = char_stream_send->pChar;

	if (NULL_PTR != char_stream_recv)
	{
		rx_len = char_stream_recv->length;
		char_recv = char_stream_recv->pChar;
	}



	ConfigureSpi(optBefore);


	while (tx_len-- > 0)
	{
		WR_R(SPIWRFIFO,  *(char_send++));
		CHECK_BSY;	
		RD_R(SPIRDFIFO); 	
	}

	while (rx_len-- > 0)
	{
	
		WR_R(SPIWRFIFO,  DUMMY_BYTE);
		CHECK_BSY;
		if (CHECK_RX_FIFO)
			*char_recv++ = RD_R(SPIRDFIFO);
		else
			rx_len++;
	}

	ConfigureSpi(optAfter);


	return RetSpiSuccess;
}



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

		/* to do before all operation */
		case OpsWakeUp:
			/* check if SPI controller is busy */
			CHECK_BSY;
			/* set Chip Select H-->L */
			SET_CS;
			break;


		/* to do before the data transfering */
		case OpsInitTransfer:
			/* flush the content of write and read FIFO */
			FLUSHRWFIFO;
			break;


		/* to do after the data transfering */
		case OpsEndTransfer:
			/* set Chip Select L-->H */
			CLEAR_CS;
			/* flush the content of write and read FIFO */
			FLUSHRWFIFO;
			break;


		default:
			break;
	}
}
