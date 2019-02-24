/************************** Flash Memory Driver ***********************************

   Filename:    SPI-NAND.c
   Description: Library routines for the SPI-NAND Flash Memory.

   Version:     1.0
   Author:      Micron Italia,  Arzano (Napoli)

   Copyright (c) 2011 Micron

   THIS PROGRAM IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER
   EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTY
   OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. THE ENTIRE RISK
   AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM IS WITH YOU. SHOULD THE
   PROGRAM PROVE DEFECTIVE, YOU ASSUME THE COST OF ALL NECESSARY SERVICING,
   REPAIR OR CORRECTION.
********************************************************************************

   Version History.

   Ver.		Date			Comments

   1.0		Dec 2011 		First release 

********************************************************************************

  The following functions are available in this library:

	ReturnType FlashReadStatusRegister(NMX_uint8 *ucpStatusRegister);
	ReturnType FlashWriteEnable(void);
	ReturnType FlashWriteDisable(void);
	ReturnType FlashBlockErase(uAddrType udBlockAddr);
	ReturnType FlashPageRead(uAddrType udAddr, NMX_uint8 *pArray);
	ReturnType FlashPageReadDual(uAddrType udAddr, NMX_uint8 *pArray);
	ReturnType FlashPageReadQuad(uAddrType udAddr, NMX_uint8 *pArray);
	ReturnType FlashReadDeviceIdentification(NMX_uint16 *uwpDeviceIdentification);
	ReturnType FlashPageProgram(uAddrType udAddr, NMX_uint8 *pArray);
	ReturnType FlashRandomProgram(uAddrType rowAddr, chunk* cks, NMX_uint8 num_of_chunk);
	ReturnType FlashInternalDataMove(uAddrType udSourceAddr, uAddrType udDestAddr);
	ReturnType FlashUnlock(ProtectedRows pr);
	ReturnType FlashUnlockAll(void);
	ReturnType FlashReadOTPStatus(NMX_uint8 *otp_reg_value);
	ReturnType FlashGetFeature(NMX_uint8 ucRegAddr, NMX_uint8 *ucpRegValue);
	ReturnType FlashSetFeature(Register ucRegAddr, NMX_uint8 ucpRegValue);
	ReturnType FlashReset(void);

********************************************************************************/

#include "SPI-NAND.h"
//#include "Serialize.h" /* Header file with SPI master abstract prototypes */
#include "NAND.h"

#define NULL_PTR 0

#ifdef MT29F2G01ABAGD
/*
 *
 *  ADDRESS STRUCTURE for MT29FG01x devices
 *
 * 		31                                  0
 * 		+---+-----------+------+------------+
 * 		| / |   BLOCK   | PAGE |   BYTES    |
 * 		+---+-----------+------+------------+
 *
 *		BLOCK = 11 bits
 *		PAGE = 6 bits
 *		BYTES = 12 bits
 *
 * 		BLOCK + PAGE = Row address
 * 		BYTES = Column address
 *
 *		Notes:
 *		The 12-bit column address is capable of addressing from 0 to 4095 bytes; however, only
 *		bytes 0 through 2175 are valid. Bytes 2176 through 4095 of each page are �out of
 *		bounds,� do not exist in the device, and cannot be addressed.
 *
 *
 */

/*
 * void Build_Row_Stream(NMX_uint32 udAddr, NMX_uint8 cCMD, NMX_uint8 *chars);
 * (This is not an api function)
 */
static void Build_Row_Stream(NMX_uint32 udAddr, NMX_uint8 cCMD, NMX_uint8 *chars)
{
	udAddr &= 0xFFFFF000ul; /* row mask */
	chars[0] = (NMX_uint8) cCMD;
	chars[1] = (NMX_uint8) (udAddr>>28);
	chars[2] = (NMX_uint8) (udAddr>>20);
	chars[3] = (NMX_uint8) (udAddr>>12);
	return;
}

/*
 * void Build_Column_Stream(NMX_uint32 udAddr, NMX_uint8 cCMD, NMX_uint8 *chars);
 * (This is not an api function)
 */

static void Build_Column_Stream(NMX_uint32 udAddr, NMX_uint8 cCMD, NMX_uint8 *chars)
{
	chars[0] = (NMX_uint8) cCMD;
	chars[1] = (NMX_uint8) ((udAddr>>8) & 0x0f) | ((NMX_uint8) ((udAddr>>14) & 0x10)); //3 bit column address + 1 bit plane select
	chars[2] = (NMX_uint8) (udAddr);	//8 bit column address
	return;
}

/*
 * ReturnType Build_Address(NMX_uint16 block, NMX_uint8 page, NMX_uint16 col, NMX_uint32 addr);
 * (This is not an api function)
 */

ReturnType Build_Address(NMX_uint16 block, NMX_uint8 page, NMX_uint16 col, NMX_uint32* addr)
{
	/* check if the parameters are within flash boundaries */
	if((block >= NUM_BLOCKS) || (page >= NUM_PAGE_BLOCK) || (col >= PAGE_SIZE))
		return Flash_AddressInvalid;

	/* build the address */
	*addr = 0;
	block = block & 0x07FF; // 11 bit
	page = page & 0x3F; // 6 bit
	col = col & 0x0FFF;  // 12 bit
	*addr = (((uint32_t)block << 18) | ((uint32_t)page << 12) | (uint32_t)col);
	return Flash_Success;
}
#endif

/*********************************************************************************
 *
 * Function: 		FlashTimeOut()
 * Arguments:		NMX_uint32 udSeconds
 * Return Value:	Flash_OperationTimeOut, Flash_OperationOngoing
 * Description:
 *
 * This function provides a timeout for Flash polling actions or
 * other operations which would otherwise never return. The Routine uses
 * COUNT_FOR_A_SECOND which is considered to be a loop that counts for one second.
 * It needs to be adapted to the target Hardware.
 *
 *
 *********************************************************************************/
ReturnType FlashTimeOut(NMX_uint32 udSeconds) {
	#define COUNT_FOR_A_SECOND 0xFFFFFF /* Timer Usage */
   static NMX_uint32 udCounter = 0;
   if (udSeconds == 0)  /* Set Timeout to 0 */
     udCounter = 0;

   if (udCounter == (udSeconds * COUNT_FOR_A_SECOND)) {
      udCounter = 0;
      return Flash_OperationTimeOut;
   } else {
      udCounter++;
      return Flash_OperationOngoing;
   }

}

/*******************************************************************************
 *
 * Function:     	IsFlashBusy()
 * Arguments:    	none
 * Return Value: 	TRUE, FALSE
 * Description:
 *
 * This function checks the Write In Progress (WIP) bit to
 * determine whether the Flash memory is busy with a Write, Program or
 * Erase cycle.
 *
 * Pseudo Code:
 *
 *   - Read the Status Register
 *   - Check the WIP bit
 *
 ******************************************************************************/
int IsFlashBusy()
{
    NMX_uint8 ucSR;

    /* Step 1: Read the Status Register */
    FlashReadStatusRegister(&ucSR);

    /* Step 2: Check the WIP bit */
    if(ucSR & SPI_NAND_OIP)
        return TRUE;
    else
        return FALSE;
}

/*
 * ReturnType WAIT_EXECUTION_COMPLETE(NMX_sint16 second)
 *
 * (This is not an api function)
 */
static ReturnType WAIT_EXECUTION_COMPLETE(NMX_sint16 second)
{
    FlashTimeOut(0);
    while(IsFlashBusy())
    {
        if(Flash_OperationTimeOut == FlashTimeOut(second))
            return  Flash_OperationTimeOut;
    }
    return Flash_Success;
}

/*******************************************************************************
 *
 * Function:		ReadStatusRegister()
 * Arguments:		NMX_uint8 *ucpStatusRegister
 * Return Value:	Flash_Success
 * Description:
 *
 * Return the status of the status register
 *
 * The GET FEATURES (0Fh) and SET FEATURES (1Fh) commands are used to alter the
 * device behavior from the default power-on behavior. These commands use a 1-byte feature
 * address to determine which feature is to be read or modified. Features such as OTP
 * and block locking can be enabled or disabled by setting specific bits in feature address
 * A0h and B0h (shown in the following table). The status register is mostly read, except
 * WEL, which is a writable bit with the WRITE ENABLE (06h) command.
 * When a feature is set, it remains active until the device is power cycled or the feature is
 * written to. Unless otherwise specified in the following table, once the device is set, it
 * remains set, even if a RESET (FFh) command is issued.
 *
 * Status register bits:
 *
 *     SR7 - reserved
 *     SR6 - reserved
 *     SR5 - ECC status (*)
 *     SR4 - ECC status (*)
 *     SR3 - P_Fail Program fail
 *     SR2 - E_Fail Erase fail
 *     SR1 - WEL Write enable
 *     SR0 - OIP Operation in progress
 *
 * Pseudo Code:
 *
 *    - Send Command (0x0F) get features
 *    - Send address (0xC0) to get status register content
 *
 *******************************************************************************/
ReturnType FlashReadStatusRegister(NMX_uint8 *ucpStatusRegister)
{
    CharStream char_stream_send;
    CharStream char_stream_recv;
	NMX_uint8 chars[2];

	chars[0]= (NMX_uint8) SPI_NAND_GET_FEATURE_INS;
	chars[1]= (NMX_uint8) SPI_NAND_STATUS_REG_ADDR;

    // Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
    char_stream_send.length  = 2;
    char_stream_send.pChar   = chars;
    char_stream_recv.length  = 1;
    char_stream_recv.pChar   = ucpStatusRegister;

    // Step 2: Send the packet serially, get the Status Register content
    Serialize_SPI(&char_stream_send, &char_stream_recv, OpsWakeUp, OpsEndTransfer);

    return Flash_Success;
}

/*******************************************************************************
 *
 * Function:     FlashWriteEnable()
 * Arguments:    none
 * Return Value: Flash_Success
 * Description:
 *
 * This function sets the Write Enable Latch(WEL) by sending a write
 * enable Instruction.
 *
 * Pseudo Code:
 *
 *    - Initialize the data (i.e. Instruction) packet to be sent serially
 *    - Send the packet serially
 *
 ******************************************************************************/
ReturnType FlashWriteEnable(void)
{
    CharStream char_stream_send;
    NMX_uint8  cWREN = SPI_NAND_WRITE_ENABLE;
    NMX_uint8 ucSR;

    // Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
    char_stream_send.length = 1;
    char_stream_send.pChar  = &cWREN;

    // Step 2: Send the packet serially
    Serialize_SPI(&char_stream_send,
              NULL_PTR,
              OpsWakeUp,
              OpsEndTransfer);

    // Step 3: Read the Status Register.
    do {
        FlashReadStatusRegister(&ucSR);
    } while(~ucSR & SPI_NAND_WE);
    
    return Flash_Success;
}

/*******************************************************************************
 * Function:     FlashWriteDisable()
 * Arguments:    none
 * Return Value: Flash_Success
 * Description:
 *
 * This function resets the Write Enable Latch(WEL) by sending a
 * WRDI Instruction.
 *
 * Pseudo Code:
 *
 *   - Initialize the data (i.e. Instruction) packet to be sent serially
 *   - Send the packet serially
 *
 ******************************************************************************/
ReturnType FlashWriteDisable(void)
{
    CharStream char_stream_send;
    NMX_uint8  cWRDI = SPI_NAND_WRITE_DISABLE;
    NMX_uint8 ucSR;
 
    // Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
    char_stream_send.length = 1;
    char_stream_send.pChar  = &cWRDI;

    // Step 2: Send the packet serially
    Serialize_SPI(&char_stream_send,
              NULL_PTR,
              OpsWakeUp,
              OpsEndTransfer);

    // Step 3: Read the Status Register.
    do {
        FlashReadStatusRegister(&ucSR);
    } while(ucSR & SPI_NAND_WE);

    return Flash_Success;
}

/*******************************************************************************
 * Function:		FlashBlockErase()
 * Arguments:		uAddrType udBlockAddr
 * Return Value:	Flash_OperationOngoing,
 *					Flash_OperationTimeOut,
 *					Flash_Success,
 *					Flash_PageEraseFailed
 * Description:
 *
 * This function resets the Write Enable Latch(WEL) by sending a
 * WRDI Instruction.
 *
 * The BLOCK ERASE (D8h) command is used to erase at the block level.
 * For the MT29FG01 The blocks are organized as 64 pages per block, 2176 bytes
 * per page (2048 + 128 bytes). Each block is 132 Kbytes. The BLOCK ERASE
 * command (D8h) operates on one block at a time. The command sequence for the
 * BLOCK ERASE operation is as follows:
 *
 * � 06h (WRITE ENBALE command)
 * � D8h (BLOCK ERASE command)
 * � 0Fh (GET FEATURES command to read the status register)
 *
 * Prior to performing the BLOCK ERASE operation, a WRITE ENABLE (06h) command
 * must be issued. As with any command that changes the memory contents, the WRITE
 * ENABLE command must be executed in order to set the WEL bit. If the WRITE ENABLE
 * command is not issued, then the rest of the erase sequence is ignored. A WRITE ENABLE
 * command must be followed by a BLOCK ERASE (D8h) command. This command
 * requires a 24-bit address consisting of 7 dummy bits followed by an 17-bit row address.
 * After the row address is registered, the control logic automatically controls timing and
 * erase-verify operations. The device is busy for tERS time during the BLOCK ERASE operation.
 * The GET FEATURES (0Fh) command can be used to monitor the status of the
 * operation.
 *
 *
 * Pseudo Code:
 *
 *    - Send Write Enable
 *    - Construct the address (24bit): 8dummy bit + 16 row address bit
 *    - Send block erase command 0xD8
 *    - Send address
 *    - Check Status register
 *
 ******************************************************************************/
ReturnType FlashBlockErase(uAddrType udBlockAddr)
{
    CharStream char_stream_send;
	NMX_uint8 chars[4];
	NMX_uint8 status_reg;

    // Step 1: Check whether any previous Write, Program or Erase cycle is on going
    if(IsFlashBusy()) return Flash_OperationOngoing;

    // Step 2: Enable WE
    FlashWriteEnable(); 
	
    // Step 3: Initialize the data (Instruction & address) packet to be sent serially
	Build_Row_Stream(udBlockAddr, SPI_NAND_BLOCK_ERASE_INS, chars);
    char_stream_send.length   = 4;
    char_stream_send.pChar    = chars;

    // Step 4: Send the packet (Instruction & address) serially
    Serialize_SPI(&char_stream_send, NULL_PTR, OpsWakeUp, OpsEndTransfer);

    // Step 5: Wait until the operation completes or a timeout occurs.
    WAIT_EXECUTION_COMPLETE(SE_TIMEOUT);

    // Step 6: Check if the erase fails
    FlashReadStatusRegister(&status_reg);
    if (status_reg & SPI_NAND_EF)
    	return Flash_BlockEraseFailed;

    return Flash_Success;

}

ReturnType FlashRead(uAddrType udAddr, NMX_uint8 *pArray, NMX_uint32 udNrOfElementsInArray)
{
    CharStream char_stream_send;
    CharStream char_stream_recv;
    NMX_uint8  chars[4];
	NMX_uint8  cReadFromCacheCMD;

    // Step 1: Validate address input
	if(ADDRESS_2_COL(udAddr) >= PAGE_SIZE)
		return Flash_AddressInvalid;

    // Step 2: Initialize the data (i.e. Instruction) packet to be sent serially
	Build_Row_Stream(udAddr, SPI_NAND_PAGE_READ_INS, chars);
    char_stream_send.length   = 4;
    char_stream_send.pChar    = chars;

    // Step 3: Send the packet serially, and fill the buffer with the data being returned
    Serialize_SPI(&char_stream_send, NULL_PTR, OpsWakeUp, OpsEndTransfer);

	// Step 4: Wait until the operation completes or a timeout occurs.
	WAIT_EXECUTION_COMPLETE(SE_TIMEOUT);

	// Standard read
	cReadFromCacheCMD = SPI_NAND_READ_CACHE_INS;

    // Step 5: Initialize the data (i.e. Instruction) packet to be sent serially
	Build_Column_Stream(udAddr, cReadFromCacheCMD, chars);
    char_stream_send.length   = 4;
    char_stream_send.pChar    = chars;
	char_stream_recv.length   = udNrOfElementsInArray;
    char_stream_recv.pChar    = pArray;

    // Step 6: Send the packet serially, and fill the buffer with the data being returned
    Serialize_SPI(&char_stream_send, &char_stream_recv, OpsWakeUp, OpsEndTransfer);

    return Flash_Success;
}

/******************************************************************************
 *
 * Function:		FlashPageRead()
 * Arguments:		uAddrType udAddr, NMX_uint8 *pArray
 * Return Value:	Flash_AddressInvalid, Flash_Success
 * Description:
 *
 * The PAGE READ (13h) command transfers the data from the NAND Flash array to the
 * cache register. The command sequence is follows:
 *
 * � 13h (PAGE READ to cache)
 * � 0Fh (GET FEATURES command to read the status)
 * � 0Bh or 03h (Random data read)
 *
 * The PAGE READ command requires a 24-bit address consisting of 7 dummy bits followed
 * by a 17-bit block/page address. After the block/page addresses are registered, the
 * device starts the transfer from the main array to the cache register, and is busy for tRD
 * time. During this time, the GET FEATURE (0Fh) command can be issued to monitor the
 * status of the operation. Following a status of successful completion, the RANDOM DATA
 * READ (03h or 0Bh) command must be issued in order to read the data out of the
 * cache. The RANDOM DATA READ command requires 3 dummy bits, followed by a
 * plane select and a 12-bit column address for the starting byte address. The starting byte
 * address can be 0 to 2111, but after the end of the cache register is reached, the data does
 * not wrap around and SO goes to a High-Z state.
 *
 *
 ******************************************************************************/
ReturnType FlashPageRead(uAddrType udAddr, NMX_uint8 *pArray)
{
    CharStream char_stream_send;
    CharStream char_stream_recv;
    NMX_uint8  chars[4];
	NMX_uint8  cReadFromCacheCMD;

    // Step 1: Validate address input
	if(ADDRESS_2_COL(udAddr) >= PAGE_SIZE)
		return Flash_AddressInvalid;

    // Step 2: Initialize the data (i.e. Instruction) packet to be sent serially
	Build_Row_Stream(udAddr, SPI_NAND_PAGE_READ_INS, chars);
    char_stream_send.length   = 4;
    char_stream_send.pChar    = chars;

    // Step 3: Send the packet serially, and fill the buffer with the data being returned
    Serialize_SPI(&char_stream_send, NULL_PTR, OpsWakeUp, OpsEndTransfer);

	// Step 4: Wait until the operation completes or a timeout occurs.
	WAIT_EXECUTION_COMPLETE(SE_TIMEOUT);

	// Standard read
	cReadFromCacheCMD = SPI_NAND_READ_CACHE_INS;

    // Step 5: Initialize the data (i.e. Instruction) packet to be sent serially
	Build_Column_Stream(udAddr, cReadFromCacheCMD, chars);
    char_stream_send.length   = 4;
    char_stream_send.pChar    = chars;
	char_stream_recv.length   = FlashPageSize - (udAddr & FlashAddressMask) & FlashAddressMask;
    char_stream_recv.length   = PAGE_SIZE - ADDRESS_2_COL(udAddr);
    char_stream_recv.pChar    = pArray;

    // Step 6: Send the packet serially, and fill the buffer with the data being returned
    Serialize_SPI(&char_stream_send, &char_stream_recv, OpsWakeUp, OpsEndTransfer);

    return Flash_Success;
}

/******************************************************************************
 *
 * Function:		FlashPageReadDual()
 * Arguments:		uAddrType udAddr, NMX_uint8 *pArray
 * Return Value:	Flash_AddressInvalid, Flash_Success
 * Description:
 *
 * The PAGE READ (13h) command transfers the data from the NAND Flash array
 * to the cache register with x2 wire I/O.
 *
 *****************************************************************************/
ReturnType FlashPageReadDual(uAddrType udAddr, NMX_uint8 *pArray)
{
    CharStream char_stream_send;
    CharStream char_stream_recv;
    NMX_uint8  chars[4];
	NMX_uint8  cReadFromCacheCMD;

    // Step 1: Validate address input
	if(ADDRESS_2_COL(udAddr) >= PAGE_SIZE)
		return Flash_AddressInvalid;

    // Step 2: Initialize the data (i.e. Instruction) packet to be sent serially
	Build_Row_Stream(udAddr, SPI_NAND_PAGE_READ_INS, chars);
    char_stream_send.length   = 4;
    char_stream_send.pChar    = chars;

    // Step 3: Send the packet serially, and fill the buffer with the data being returned
    Serialize_SPI(&char_stream_send, NULL_PTR, OpsWakeUp, OpsEndTransfer);

	// Step 4: Wait until the operation completes or a timeout occurs.
	WAIT_EXECUTION_COMPLETE(SE_TIMEOUT);

	// Spi Dual Read
	cReadFromCacheCMD = SPI_NAND_READ_CACHE_X2_INS;

    // Step 5: Initialize the data (i.e. Instruction) packet to be sent serially
	Build_Column_Stream(udAddr, cReadFromCacheCMD, chars);
    char_stream_send.length   = 4;
    char_stream_send.pChar    = chars;
	char_stream_recv.length   = FlashPageSize - (udAddr & FlashAddressMask) & FlashAddressMask;
    char_stream_recv.pChar    = pArray;

    // Step 6: Send the packet serially, and fill the buffer with the data being returned
    Serialize_SPI(&char_stream_send, &char_stream_recv, OpsWakeUp, OpsEndTransfer);

    return Flash_Success;
}

/******************************************************************************
 *
 * Function:		FlashPageReadDual()
 * Arguments:		uAddrType udAddr, NMX_uint8 *pArray
 * Return Value:	Flash_AddressInvalid, Flash_Success
 * Description:
 *
 * The PAGE READ (13h) command transfers the data from the NAND Flash array
 * to the cache register with x4 wire I/O.
 *
 *****************************************************************************/
ReturnType FlashPageReadQuad(uAddrType udAddr, NMX_uint8 *pArray, PageReadMode Mode)
{
    CharStream char_stream_send;
    CharStream char_stream_recv;
    NMX_uint8  chars[4];
	NMX_uint8  cReadFromCacheCMD;

    // Step 1: Validate address input
	if(ADDRESS_2_COL(udAddr) >= PAGE_SIZE)
		return Flash_AddressInvalid;

    // Step 2: Initialize the data (i.e. Instruction) packet to be sent serially
	Build_Row_Stream(udAddr, SPI_NAND_PAGE_READ_INS, chars);
    char_stream_send.length   = 4;
    char_stream_send.pChar    = chars;

    // Step 3: Send the packet serially, and fill the buffer with the data being returned
    Serialize_SPI(&char_stream_send, NULL_PTR, OpsWakeUp, OpsEndTransfer);

	// Step 4: Wait until the operation completes or a timeout occurs.
	WAIT_EXECUTION_COMPLETE(SE_TIMEOUT);

	// Spi Quad Read
	cReadFromCacheCMD = SPI_NAND_READ_CACHE_X4_INS;

    // Step 5: Initialize the data (i.e. Instruction) packet to be sent serially
	Build_Column_Stream(udAddr, cReadFromCacheCMD, chars);
    char_stream_send.length   = 4;
    char_stream_send.pChar    = chars;
	char_stream_recv.length   = FlashPageSize - (udAddr & FlashAddressMask) & FlashAddressMask;
    char_stream_recv.pChar    = pArray;
	
    // Step 6: Send the packet serially, and fill the buffer with the data being returned
    Serialize_SPI(&char_stream_send, &char_stream_recv, OpsWakeUp, OpsEndTransfer);

    return Flash_Success;
}

/******************************************************************************
 *
 * Function:		FlashReadDeviceIdentification()
 * Arguments:		NMX_uint16 *uwpDeviceIdentification
 * Return Value:	Flash_Success
 * Description:
 *
 * The READ ID command is used to read the 2 bytes of identifier code programmed into
 * the SPI NAND Flash device. The READ ID command reads a 2-byte table (see below) that
 * includes the Manufacturer ID and the device configuration.
 *
 ******************************************************************************/
ReturnType FlashReadDeviceIdentification(NMX_uint16 *uwpDeviceIdentification)
{
    CharStream char_stream_send;
    CharStream char_stream_recv;
    NMX_uint8  chars[2];
    NMX_uint8  pIdentification[2];

	chars[0]= (NMX_uint8) SPI_NAND_READ_ID;
	chars[1]= 0x00; /* dummy byte */
	
    // Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
    char_stream_send.length = 2;
    char_stream_send.pChar  = chars;
    char_stream_recv.length = 2;
    char_stream_recv.pChar  = &pIdentification[0];

    // Step 2: Send the packet serially
    Serialize_SPI(&char_stream_send, &char_stream_recv, OpsWakeUp, OpsEndTransfer);

    // Step 3: Device Identification is returned ( memory type + memory capacity )
    *uwpDeviceIdentification = char_stream_recv.pChar[0];
    *uwpDeviceIdentification <<= 8;
    *uwpDeviceIdentification |= char_stream_recv.pChar[1];

    return Flash_Success;
}


/******************************************************************************
 *
 * Function:		FlashPageProgram()
 * Arguments:		uAddrType udAddr,
 * 					NMX_uint8 *pArray,
 * 					NMX_uint32 udNrOfElementsInArray
 * Return Value:	Flash_AddressInvalid,
 * 					Flash_OperationOngoing,
 * 					Flash_Success,
 * 					Flash_ProgramFailed
 * Description:
 *
 * The PAGE PROGRAM operation sequence programs 1 byte to 2176 bytes of data within
 * a page. The page program sequence is as follows:
 *
 * � 06h (WRITE ENABLE
 * � 02h (PROGRAM LOAD)
 * � 10h (PROGRAM EXECUTE)
 * � 0Fh (GET FEATURE command to read the status)
 *
 * Prior to performing the PROGRAM LOAD operation, a WRITE ENABLE (06h) command
 * must be issued. As with any command that changes the memory contents, the WRITE
 * ENABLE must be executed in order to set the WEL bit. If this command is not issued,
 * then the rest of the program sequence is ignored. WRITE ENABLE must be followed by
 * a PROGRAM LOAD (02h) command. PROGRAM LOAD consists of an 8-bit Op code, followed
 * by 3 dummy bits, a plane select and a 12-bit column address, then the data bytes
 * to be programmed. The data bytes are loaded into a cache register that is 2176 bytes
 * long. __Only four partial-page programs are allowed on a single page__. If more than 2176
 * bytes are loaded, then those additional bytes are ignored by the cache register. The command
 * sequence ends when CS goes from LOW to HIGH.
 * After the data is loaded, a PROGRAM EXECUTE (10h) command must be issued to initiate
 * the transfer of data from the cache register to the main array. PROGRAM EXECUTE
 * consists of an 8-bit Op code, followed by a 24-bit address (7 dummy bits and a 17-bit page/
 * block address). After the page/block address is registered, the memory device starts the
 * transfer from the cache register to the main array, and is busy for tPROG time.
 * During this busy time, the status register can be polled to monitor the status of the
 * operation (refer to Status Register (page 29)). When the operation completes successfully,
 * the next series of data can be loaded with the PROGRAM LOAD command.
 *
 * Pseudo code:
 *
 *    - Send Write enable Command
 *    - Send Program Load Command (0x02)
 *    - Send address (16bit): 3 dummy bit + 1 bit plane select + 12 bit column address
 *    - Send data (2176 bytes)
 *    - CS: Low->High
 *    - Send Program Execute command (0x10)
 *    - Send address (24bit): 8 bit dummy + 16 bit address (page/Block)
 *    - Check status register
 *
 ******************************************************************************/
ReturnType FlashPageProgram(uAddrType udAddr, NMX_uint8 *pArray, NMX_uint32 udNrOfElementsInArray)
{
    CharStream char_stream_send;
    NMX_uint8 chars[4];
    NMX_uint8 status_reg;

    // Step 1: Validate address input
	if(ADDRESS_2_COL(udAddr) >= PAGE_DATA_SIZE)
		return Flash_AddressInvalid;

    // Step 2: Check whether any previous Write, Program or Erase cycle is on going
    if(IsFlashBusy()) return Flash_OperationOngoing;

    // Step 3: Disable Write protection
    FlashWriteEnable();

    // Step 4: Initialize the data (Instruction & address only) packet to be sent serially
	Build_Column_Stream(udAddr, SPI_NAND_PROGRAM_LOAD_INS, chars);
    char_stream_send.length   = 3;
    char_stream_send.pChar    = chars;

    // Step 5: Send the packet serially, and fill the buffer with the data being returned
    Serialize_SPI(&char_stream_send, NULL_PTR, OpsWakeUp, OpsInitTransfer);

    // Step 6: Initialize the data (data to be programmed) packet to be sent serially
    char_stream_send.length   = udNrOfElementsInArray;
    char_stream_send.pChar    = pArray;

    // Step 7: Send the packet (data to be programmed) serially
    Serialize_SPI(&char_stream_send, NULL_PTR, OpsWakeUp, OpsEndTransfer);
	
    // Step 8: Initialize the data (i.e. Instruction) packet to be sent serially
	Build_Row_Stream(udAddr, SPI_NAND_PROGRAM_EXEC_INS, chars);
    char_stream_send.length   = 4;
    char_stream_send.pChar    = chars;

    // Step 9: Send the packet (data to be programmed) serially
    Serialize_SPI(&char_stream_send, NULL_PTR, OpsWakeUp, OpsEndTransfer);
	
    // Step 10: Wait until the operation completes or a timeout occurs.
    WAIT_EXECUTION_COMPLETE(SE_TIMEOUT);

    // Step 11: Check if the program fails
    FlashReadStatusRegister(&status_reg);
    if (status_reg & SPI_NAND_PF)
    	return Flash_ProgramFailed;

    return Flash_Success;
}

/******************************************************************************
 *
 * Function:		FlashPageProgram()
 * Arguments:		uAddrType udAddr, chunk* cks, NMX_uint8 num_of_chunk
 * Return Value:	Flash_AddressInvalid, Flash_OperationOngoing, Flash_Success
 * Description:
 *
 * The RANDOM DATA PROGRAM sequence programs or replaces data in a page with
 * existing data. The random data program sequence is as follows:
 * � 06h (WRITE ENABLE
 * � 84h (PROGRAM LOAD RANDOM DATA)
 * � 10h (PROGRAM EXECUTE)
 * � 0Fh (GET FEATURE command to read the status)
 * Prior to performing a PROGRAM LOAD RANDOM DATA operation, a WRITE ENABLE
 * (06h) command must be issued to change the contents of the memory array. Following
 * a WRITE ENABLE (06) command, a PROGRAM LOAD RANDOM DATA (84h) command
 * must be issued. This command consists of an 8-bit Op code, followed by 3 dummy bits,
 * a plane select bit, and a 12-bit column address. New data is loaded in the column address
 * provided with the 12 bits. If the random data is not sequential, then another
 * PROGRAM LOAD RANDOM DATA (84h) command must be issued with a new column
 * address. After the data is loaded, a PROGRAM EXECUTE (10h) command can be issued
 * to start the programming operation.
 *
 * Pseudo code:
 *
 *    - Send Write enable Command (0x06)
 *    - Send Program Load Random Data Command (0x84)
 *    - Send address (16bit): 3 dummy bit + 1 bit plane select + 12 bit column address
 *    - Send data () If the random data is not sequential, then another
 *    -  PROGRAM LOAD RANDOM DATA (84h) command must be issued with a new column
 *    -  address
 *    - CS: Low->High
 *    - Send Program Execute command (0x10)
 *    - Send address (24bit): 8 bit dummy + 16 bit address (page/Block)
 *    - Check status register
 *
 *
 ******************************************************************************/
ReturnType FlashRandomProgram(uAddrType udAddr, chunk* cks, NMX_uint8 num_of_chunk)
{
    CharStream char_stream_send;
    NMX_uint8  chars[4], current_chunk;

    // Step 1: Validate address input
	if(ADDRESS_2_COL(udAddr) >= PAGE_SIZE)
		return Flash_AddressInvalid;

    // Step 2: Check whether any previous Write, Program or Erase cycle is on-going
    if(IsFlashBusy()) return Flash_OperationOngoing;

    // Step 3: Disable Write protection
    FlashWriteEnable();

    // Step 4: Initialize the data (Instruction & address only) packet to be sent serially
    for(current_chunk = 0; current_chunk < num_of_chunk; current_chunk++)
    {
		// Step 5: Initialize the data (i.e. Instruction) packet to be sent serially
		Build_Column_Stream(cks[current_chunk].address, SPI_NAND_PROGRAM_LOAD_RANDOM_INS, chars);
		char_stream_send.length   = 3;
		char_stream_send.pChar    = chars;

		// Step 6: Send the packet serially, and fill the buffer with the data being returned
		Serialize_SPI(&char_stream_send, NULL_PTR, OpsWakeUp, OpsInitTransfer);

		// Step 7: Initialize the data (data to be programmed) packet to be sent serially
		char_stream_send.length   = cks[current_chunk].chunkSize;
		char_stream_send.pChar    = cks[current_chunk].buffer;

		// Step 8: Send the packet (data to be programmed) serially
		Serialize_SPI(&char_stream_send, NULL_PTR, OpsWakeUp, OpsEndTransfer);
    }

    // Step 9: Initialize the data (i.e. Instruction) packet to be sent serially
	Build_Row_Stream(udAddr, SPI_NAND_PROGRAM_EXEC_INS, chars);
    char_stream_send.length   = 4;
    char_stream_send.pChar    = chars;

    // Step 10: Send the packet (data to be programmed) serially
    Serialize_SPI(&char_stream_send, NULL_PTR, OpsWakeUp, OpsEndTransfer);

    // Step 11: Wait until the operation completes or a timeout occurs.
    return WAIT_EXECUTION_COMPLETE(SE_TIMEOUT);
}


/******************************************************************************
 *
 * Function:		FlashInternalDataMove()
 * Arguments:		uAddrType udSourceAddr, uAddrType udDestAddr
 * Return Value:	Flash_AddressInvalid
 * Description:
 *
 * The INTERNAL DATA MOVE command sequence programs or replaces data in a page
 * with existing data. The INTERNAL DATA MOVE command sequence is as follows:
 * � 13h (PAGE READ to cache)
 *
 * � 06h (WRITE ENABLE
 * � 84h (PROGRAM LOAD RANDOM DATA)
 * � 10h (PROGRAM EXECUTE)
 * � 0Fh (GET FEATURE command to read the status)
 *
 * Prior to performing an internal data move operation, the target page content must be
 * read into the cache register. This is done by issuing a PAGE READ (13h) command (see
 * Figure 11).
 *
 * The PAGE READ command requires a 24-bit address consisting of 8 dummy bits followed
 * by a 16-bit block/page address. After the block/page addresses are registered, the
 * device starts the transfer from the main array to the cache register, and is busy for tRD
 * time. During this time, the GET FEATURE (0Fh) command can be issued to monitor the
 * status of the operation.
 *
 * The PAGE READ command must be followed with a WRITE ENABLE (06h)
 * command in order to change the contents of memory array. After the WRITE ENABLE
 * command is issued, the PROGRAM LOAD RANDOM DATA (84h) command can be issued.
 * This command consists of an 8-bit Op code, followed by 3 dummy bits, a plane
 * select, and a 12-bit column address. New data is loaded in the 12-bit column address. If
 * the random data is not sequential, another PROGRAM LOAD RANDOM DATA (84h) command
 * must be issued with the new column address. After the data is loaded, a PROGRAM
 * EXECUTE (10h) command can be issued to start the programming operation.
 * _INTERNAL DATA MOVE is not supported across internal planes._
 *
 * Pseudo code:
 *
 *    - Send Page read to cache Command (0x13)
 *    - Send address (24bit): 8 dummy bit + 16 bit address (page/Block)
 *    - Check status register
 *    - Send Write enable Command (0x06)
 *    - Send Program Load Random Data Command (0x84)
 *    - Send address (16bit): 3 dummy bit + 1 bit plane select + 12 bit column address
 *    - Send data () If the random data is not sequential, then another
 *    - PROGRAM LOAD RANDOM DATA (84h) command must be issued with a new column address
 *    - CS: Low->High
 *    - Send Program Execute command (0x10)
 *    - Send address (24bit): 8 bit dummy + 16 bit address (page/Block)
 *    - Check status register
 *
 ******************************************************************************/
ReturnType FlashInternalDataMove(uAddrType udSourceAddr, uAddrType udDestAddr)
{
    CharStream char_stream_send;
    NMX_uint8  chars[4];
	
    // Step 1: Validate address input
	if((ADDRESS_2_COL(udSourceAddr) >= PAGE_SIZE) || (ADDRESS_2_COL(udDestAddr) >= PAGE_SIZE))
		return Flash_AddressInvalid;
	
	// INTERNAL DATA MOVE is not supported across internal planes - check the bit plane
    if((ADDRESS_2_BLOCK(udSourceAddr) & 0x0001) != (ADDRESS_2_BLOCK(udDestAddr) & 0x0001))
    	return Flash_AddressInvalid;

	// Step 2: Initialize the data (i.e. Instruction) packet to be sent serially
	Build_Row_Stream(udSourceAddr, SPI_NAND_PAGE_READ_INS, chars);
    char_stream_send.length   = 4;
    char_stream_send.pChar    = chars;

    // Step 3: Send the packet serially, and fill the buffer with the data being returned
    Serialize_SPI(&char_stream_send, NULL_PTR, OpsWakeUp, OpsEndTransfer);
	
	// Wait until execution is complete
    WAIT_EXECUTION_COMPLETE(SE_TIMEOUT);

    // Step 4: Enable write
	FlashWriteEnable();
	
	Build_Column_Stream(udDestAddr, SPI_NAND_PROGRAM_LOAD_RANDOM_INS, chars);
    char_stream_send.length   = 3;
    char_stream_send.pChar    = chars;

    // Step 5: Send the packet serially, and fill the buffer with the data being returned
    // CS: Low->High
    Serialize_SPI(&char_stream_send, NULL_PTR, OpsWakeUp, OpsEndTransfer);
	
    // Step 6: Initialize the data (i.e. Instruction) packet to be sent serially
	Build_Row_Stream(udDestAddr, SPI_NAND_PROGRAM_EXEC_INS, chars);
    char_stream_send.length   = 4;
    char_stream_send.pChar    = chars;

    // Step 7: Send the packet (data to be programmed) serially
    Serialize_SPI(&char_stream_send, NULL_PTR, OpsWakeUp, OpsEndTransfer);
	
    return WAIT_EXECUTION_COMPLETE(SE_TIMEOUT);
}

/******************************************************************************
 *
 * Function:		FlashUnlock()
 * Arguments:		ProtectedRows pr
 * Return Value:	Flash_Success
 * Description:
 *
 * The block lock feature provides the ability to protect the entire device, or ranges of
 * blocks, from the PROGRAM and ERASE operations. After power-up, the device is in the
 * �locked� state, i.e., bits 3, 4, and 5 of the block lock register are set to 1. To unlock all the
 * blocks, or a range of blocks, the SET FEATURES command must be issued with the A0h
 * feature address, including the data bits shown in Table 6. The operation for the SET
 * FEATURES command is shown in Figure 10 on page 15. When BRWD is set and WP is
 * LOW, none of the writable bits (3, 4, 5, and 7) in the block lock register can be set. Also,
 * when a PROGRAM/ERASE command is issued to a locked block, a status of 00h is returned.
 * When an ERASE command is issued to a locked block, the erase failure, 04h, is
 * returned. When a PROGRAM command is issued to a locked block, program failure,
 * 08h, is returned.
 *
 *
 * Block Lock bits:
 *
 *   BL7 - BRWD1 (*)
 *   BL6 - reserved
 *   BL5 - BP2 (*)
 *   BL4 - BP1 (*)
 *   BL3 - BP0 (*)
 *   BL2 - reserved
 *   BL1 - reserved
 *   BL0 - reserved
 *
 * (*) If BRWD (block register write disable) is enabled and WP# is LOW,
 * then the block lock register cannot be changed
 *
 * (*) Tables of Protected Rows (BP2, BP1, BP0):
 *
 *    0 0 0 -> None�all unlocked
 *    0 0 1 -> Upper 1/64 locked
 *    0 1 0 -> Upper 1/32 locked
 *    0 1 1 -> Upper 1/16 locked
 *    1 0 0 -> Upper 1/8 locked
 *    1 0 1 -> Upper 1/4 locked
 *    1 1 0 -> Upper 1/2 locked
 *    1 1 1 -> All locked (default)
 *
 * Pseudo code:
 *
 *   - Send Command 0x1F SET features
 *   - Send address 0xA0 Block Lock status register
 *
 ******************************************************************************/
//ReturnType FlashUnlock(ProtectedRows pr)
//{
//    CharStream char_stream_send;
//	NMX_uint8 chars[3];
//
//	chars[0]= (NMX_uint8) SPI_NAND_SET_FEATURE;
//	chars[1]= (NMX_uint8) SPI_NAND_BLKLOCK_REG_ADDR;
//	chars[2]= (NMX_uint8) pr << 3;
//
//    // Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
//    char_stream_send.length  = 3;
//    char_stream_send.pChar   = chars;
//
//    // Step 2: Send the packet serially, get the Status Register content
//    Serialize_SPI(&char_stream_send, NULL_PTR, OpsWakeUp, OpsEndTransfer);
//
//    return Flash_Success;
//}


/******************************************************************************
 *
 * Function:		FlashUnlock()
 * Arguments:		void
 * Return Value:	Flash_Success
 * Description:		Unlock the whole device
 *
 ******************************************************************************/
ReturnType FlashUnlockAll(void)
{
    CharStream char_stream_send;
	NMX_uint8 chars[3];

	chars[0]= (NMX_uint8) SPI_NAND_SET_FEATURE;
	chars[1]= (NMX_uint8) SPI_NAND_BLKLOCK_REG_ADDR;
	chars[2]= (NMX_uint8) 0x00;	// unlock the whole flash

    // Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
    char_stream_send.length  = 3;
    char_stream_send.pChar   = chars;

    // Step 2: Send the packet serially, get the Status Register content
    Serialize_SPI(&char_stream_send, NULL_PTR, OpsWakeUp, OpsEndTransfer);

    return Flash_Success;
}


/******************************************************************************
 *
 *
 * Function:		OTPAccess()
 * Arguments:
 * Return Value:
 * Description:
 *
 * The serial device offers a protected, one-time programmable NAND Flash memory
 * area. Ten full pages (2176 bytes per page) are available on the device, and the entire
 * range is guaranteed to be good. Customers can use the OTP area any way they want;
 * typical uses include programming serial numbers, or other data, for permanent storage.
 * To access the OTP feature, the user must issue the SET FEATURES command, followed
 * by feature address B0h. When the OTP is ready for access, pages 02h�0Bh can be programmed
 * in sequential order. The PROGRAM LOAD (02h) and PROGRAM EXECUTE
 * (10h) commands can be used to program the pages. Also, the PAGE READ (13h) command
 * can be used to read the OTP area. The data bits used in feature address B0h to
 * enable OTP access are shown in the table below.
 *
 * OTP Access
 * To access OTP, perform the following command sequence:
 *
 * � Issue the SET FEATURES register write (1Fh)
 * � Issue the OTP feature address (B0h)
 * � Issue the PAGE PROGRAM or PAGE READ command
 *
 * It is important to note that after bits 6 and 7 of the OTP register are set by the user, the
 * OTP area becomes read-only and no further programming is supported. For OTP states,
 * see the following table.
 *
 *
 * Pseudo code:
 *
 *   - Send Write Register (0x1F)
 *   - Send OTP feature address (0xB0h)
 *
 *******************************************************************************/

/*
 *
 *
 * OTPAccess() code HERE
 *
 *
 *
 */


/******************************************************************************
 *
 * THIS FUNCTION HAS NOT BEEN IMPLEMENTED!
 *
 * Function:		FlashReadOTPStatus()
 * Arguments:		NMX_uint8 *otp_reg_value
 * Return Value:	Flash_Success
 * Description:
 *
 *  OTP address: B0h
 *
 *  OTP Register bits:
 *
 *    bit7 - OTP Protect bit (*)
 *    bit6 - OTP Enable bit (*)
 *    bit5 - reserved
 *    bit4 - ECC Enable bit
 *    bit3 - reserved
 *    bit2 - reserved
 *    bit1 - reserved
 *    bit0 - reserved
 *
 *
 * OTP Protect Bit / OTP Enable Bit State (bit 7 / bit 6)
 *
 *  00b = Normal operation
 *  01b = Access OTP space
 *  10b = Not applicable
 *  11b = Use PROGRAM EXECUTE (10h) to lock OTP
 *
 ******************************************************************************/

ReturnType FlashReadOTPStatus(NMX_uint8 *otp_reg_value)
{
	return FlashGetFeature(SPI_NAND_CONFIGURATION_REG_ADDR, otp_reg_value);
}

/******************************************************************************
 *
 *
 *  Function:		ECCProtection()
 *  Arguments:
 *  Return Value:
 *  Description:
 *
 * The serial device offers data corruption protection by offering 4-bit internal ECC.
 * READs and PROGRAMs with internal ECC can be enabled or disabled by setting the
 * ECC bit in the OTP register. ECC is enabled after device power up, so the default READ
 * and PROGRAM commands operate with internal ECC in the �active� state.
 * To enable/disable ECC, perform the following command sequence:
 *
 * � Issue the SET FEATURES register write (1Fh).
 * � Issue the OTP feature address (B0h).
 * � Then:
 *
 * � To enable ECC: Set Bit 4 (ECC Enable) to 1.
 * � To disable ECC: Clear Bit 4 (ECC Enable) to 0.
 *
 * During a PROGRAM operation, the device calculates an ECC code on the 2k page in the
 * cache register, before the page is written to the NAND Flash array. The ECC code is stored
 * in the spare area of the page.
 * During a READ operation, the page data is read from the array to the cache register,
 * where the ECC code is calculated and compared with the ECC code value read from the
 * array. If a 1- to 4-bit error is detected, the error is corrected in the cache register. Only
 * corrected data is output on the I/O bus. The ECC status bit indicates whether or not the
 * error correction was successful. The ECC Protection table below shows the ECC protection
 * scheme used throughout a page.
 * With internal ECC, the user must accommodate the following:
 * � Spare area definitions provided in the ECC Protection table below.
 * � WRITEs to ECC are supported for main and spare areas 0, and 1. WRITEs to the ECC
 * area are prohibited (see the ECC Protection table below).
 * � When using partial-page programming, the following conditions must both be met:
 * � In the main user area and in user meta data area I, single partial-page programming
 * operations must be used (see the ECC Protection table below).
 * � Within a page, the user can perform a maximum of four partial-page programming
 * operations.
 *
 *
 * System software should initially check the first spare area location for non-FFh data on
 * the first page of each block prior to performing any program or erase operations on the
 * NAND Flash device. Black blocks are marked with 0x00.
 *
 ******************************************************************************/

/*
 *
 *
 * OTPStatus() code HERE
 *
 *
 *
 */


/******************************************************************************
 *
 *  Function:		FlashGetFeature()
 *  Arguments:		NMX_uint8 ucRegAddr, NMX_uint8 *ucpRegValue
 *  Return Value:	Flash_RegAddressInvalid, Flash_Success
 *  Description:
 *
 ******************************************************************************/
ReturnType FlashGetFeature(NMX_uint8 ucRegAddr, NMX_uint8 *ucpRegValue)
{
    CharStream char_stream_send;
    CharStream char_stream_recv;
	NMX_uint8 chars[2];

	// Step 1: Check the register address
	if (ucRegAddr != SPI_NAND_BLKLOCK_REG_ADDR &&
	    ucRegAddr != SPI_NAND_CONFIGURATION_REG_ADDR &&
		ucRegAddr != SPI_NAND_STATUS_REG_ADDR &&
		ucRegAddr != SPI_NAND_DIE_SELECT_REC_ADDR)
		return Flash_RegAddressInvalid;

	chars[0]= (NMX_uint8) SPI_NAND_GET_FEATURE_INS;
	chars[1]= (NMX_uint8) ucRegAddr;

    // Step 2: Initialize the data (i.e. Instruction) packet to be sent serially
    char_stream_send.length  = 2;
    char_stream_send.pChar   = chars;
    char_stream_recv.length  = 1;
    char_stream_recv.pChar   = ucpRegValue;

    // Step 3: Send the packet serially, get the Status Register content
    Serialize_SPI(&char_stream_send, &char_stream_recv, OpsWakeUp, OpsEndTransfer);

    return Flash_Success;
}

 /******************************************************************************
  *
  *  Function:		FlashSetFeature()
  *  Arguments:		NMX_uint8 ucRegAddr, NMX_uint8 *ucpRegValue
  *  Return Value:	Flash_RegAddressInvalid, Flash_Success
  *  Description:
  *
  ******************************************************************************/
ReturnType FlashSetFeature(Register ucRegAddr, NMX_uint8 ucpRegValue)
{
    CharStream char_stream_send;
	NMX_uint8 chars[3];

	// Step 1: Check the register address
	if (ucRegAddr != SPI_NAND_BLKLOCK_REG_ADDR &&
	    ucRegAddr != SPI_NAND_CONFIGURATION_REG_ADDR &&
		ucRegAddr != SPI_NAND_STATUS_REG_ADDR &&
		ucRegAddr != SPI_NAND_DIE_SELECT_REC_ADDR)
		return Flash_RegAddressInvalid;

	chars[0]= (NMX_uint8) SPI_NAND_SET_FEATURE;
	chars[1]= (NMX_uint8) ucRegAddr;
	chars[2]= (NMX_uint8) ucpRegValue;

    // Step 2: Initialize the data (i.e. Instruction) packet to be sent serially
    char_stream_send.length  = 3;
    char_stream_send.pChar   = chars;

    // Step 3: Send the packet serially, get the Status Register content
    Serialize_SPI(&char_stream_send, NULL_PTR, OpsWakeUp, OpsEndTransfer);

    return Flash_Success;
}


/******************************************************************************
 *
 *  Function:		FlashReset()
 *  Arguments:		none
 *  Return Value:	Flash_Success
 *  Description:
 *
 ******************************************************************************/
ReturnType FlashReset(void)
{
    CharStream char_stream_send;
    NMX_uint8  cRST = SPI_NAND_RESET;

    // Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
    char_stream_send.length = 1;
    char_stream_send.pChar  = &cRST;

    // Step 2: Send the packet serially
    Serialize_SPI(&char_stream_send, NULL_PTR, OpsWakeUp, OpsEndTransfer);
    
    return Flash_Success;
}
