/*
 * Copyright (c) 2015, M.Naruoka (fenrir)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of the naruoka.org nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
//#include "c8051f380.h"
#include <SI_C8051F380_Register_Enums.h>                // SFR declarations
//#include "main.h"

#include <string.h>

#include "mpu9250.h"

#include "util.h"
//#include "type.h"
//#include "data_hub.h"
#include <stdio.h>
#define cs_wait() wait_8n6clk(50)
#define clk_wait() wait_8n6clk(5)

typedef enum {
  SELF_TEST_X_GYRO = 0x00,
  SELF_TEST_Y_GYRO = 0x01,
  SELF_TEST_Z_GYRO = 0x02,
  SELF_TEST_X_ACCEL = 0x0D,
  SELF_TEST_Y_ACCEL = 0x0E,
  SELF_TEST_Z_ACCEL = 0x0F,
  XG_OFFSET_H = 0x13,
  XG_OFFSET_L = 0x14,
  YG_OFFSET_H = 0x15,
  YG_OFFSET_L = 0x16,
  ZG_OFFSET_H = 0x17,
  ZG_OFFSET_L = 0x18,
  SMPLRT_DIV = 0x19,
  CONFIG = 0x1A,
  GYRO_CONFIG = 0x1B,
  ACCEL_CONFIG = 0x1C,
  ACCEL_CONFIG2 = 0x1D,
  LP_ACCEL_ODR = 0x1E,
  WOM_THR = 0x1F,
  FIFO_EN = 0x23,
  I2C_MST_CTRL = 0x24,
  I2C_SLV0_ADDR = 0x25,
  I2C_SLV0_REG = 0x26,
  I2C_SLV0_CTRL = 0x27,
  I2C_SLV1_ADDR = 0x28,
  I2C_SLV1_REG = 0x29,
  I2C_SLV1_CTRL = 0x2A,
  I2C_SLV2_ADDR = 0x2B,
  I2C_SLV2_REG = 0x2C,
  I2C_SLV2_CTRL = 0x2D,
  I2C_SLV3_ADDR = 0x2E,
  I2C_SLV3_REG = 0x2F,
  I2C_SLV3_CTRL = 0x30,
  I2C_SLV4_ADDR = 0x31,
  I2C_SLV4_REG = 0x32,
  I2C_SLV4_DO = 0x33,
  I2C_SLV4_CTRL = 0x34,
  I2C_SLV4_DI = 0x35,
  I2C_MST_STATUS = 0x36,
  INT_PIN_CFG = 0x37,
  INT_ENABLE = 0x38,
  INT_STATUS = 0x3A,
  ACCEL_OUT_BASE = 0x3B,
  TEMP_OUT_BASE = 0x41,
  GYRO_OUT_BASE = 0x43,
  EXT_SENS_DATA_BASE = 0x49,
  I2C_SLV0_DO = 0x63,
  I2C_SLV1_DO = 0x64,
  I2C_SLV2_DO = 0x65,
  I2C_SLV3_DO = 0x66,
  I2C_MST_DELAY_CTRL = 0x67,
  SIGNAL_PATH_RESET = 0x68,
  MOT_DETECT_CTRL = 0x69,
  USER_CTRL = 0x6A,
  PWR_MGMT_1 = 0x6B,
  PWR_MGMT_2 = 0x6C,
  FIFO_COUNTH = 0x72,
  FIFO_COUNTL = 0x73,
  FIFO_R_W = 0x74,
  WHO_AM_I = 0x75,
  XA_OFFSET_H = 0x77,
  XA_OFFSET_L = 0x78,
  YA_OFFSET_H = 0x7A,
  YA_OFFSET_L = 0x7B,
  ZA_OFFSET_H = 0x7D,
  ZA_OFFSET_L = 0x7E,
} address_t;

typedef enum {
  AK8963_WIA    = 0x00, // Device ID
  AK8963_INFO   = 0x01, // Information
  AK8963_ST1    = 0x02, // Status 1
  AK8963_HXL    = 0x03, // Measurement data
  AK8963_HXH    = 0x04,
  AK8963_HYL    = 0x05,
  AK8963_HYH    = 0x06,
  AK8963_HZL    = 0x07,
  AK8963_HZH    = 0x08,
  AK8963_ST2    = 0x09, // Status 2
  AK8963_CNTL1  = 0x0A, // Control 1
  AK8963_CNTL2  = 0x0B, // Control 2
  AK8963_RSV    = 0x0B, // Reserved
  AK8963_ASTC   = 0x0C, // Self-test
  AK8963_TS1    = 0x0D, // Test 1
  AK8963_TS2    = 0x0E, // Test 2
  AK8963_I2CDIS = 0x0F, // I2C disable
  AK8963_ASAX   = 0x10, // X-axis sensitivity adjustment value
  AK8963_ASAY   = 0x11, // Y-axis sensitivity adjustment value
  AK8963_ASAZ   = 0x12, // Z-axis sensitivity adjustment value
} ak8963_address_t;

#define AK8963_I2C_ADDR 0x0C

/*
 * MPU-9250
 *
 * === Connection ===
 * C8051         MPU-9250
 *  P1.4(OUT) =>  -CS
 *  P1.5(OUT) =>  SCK
 *  P1.6(OUT) =>  MOSI
 *  P1.7(IN)  <=  MISO
 */

#ifdef USE_ASM_FOR_SFR_MANIP
#define clk_up()      {__asm orl _P1,SHARP  0x20 __endasm; }
#define clk_down()    {__asm anl _P1,SHARP ~0x20 __endasm; }
#define out_up()      {__asm orl _P1,SHARP  0x40 __endasm; }
#define out_down()    {__asm anl _P1,SHARP ~0x40 __endasm; }
#define cs_assert()   {__asm anl _P1,SHARP ~0x10 __endasm; }
#define cs_deassert() {__asm orl _P1,SHARP  0x10 __endasm; }
#else
#define clk_up()      (P2 |=  0x08)
#define clk_down()    (P2 &= ~0x08)
#define out_up()      (P2 |=  0x10)
#define out_down()    (P2 &= ~0x10)
#define cs_assert()   (P2 &= ~0x04)
#define cs_deassert() (P2 |=  0x04)
#endif
#define is_in_up()    (P2 & 0x20)

static void mpu9250_write(uint8_t *buf, uint8_t size){
  for(; size--; buf++){
    uint8_t mask = 0x80;
    do{
      clk_down();
      if((*buf) & mask){out_up();}else{out_down();}
      clk_wait();
      clk_up();
      clk_wait();
    }while(mask >>= 1);
  }
}

static void mpu9250_read(uint8_t *buf, uint8_t size){
  for(; size--; buf++){
    uint8_t temp = 0;
    uint8_t mask = 0x80;
    do{
      clk_down();
      clk_wait();
      clk_up();
      if(is_in_up()) temp |= mask;
      clk_wait();
    }while(mask >>= 1);
    *buf = temp;
  }
}

#define mpu9250_set(address, value) { \
  static const  uint8_t addr_value[2] = {address, value}; \
  cs_assert(); \
  cs_wait(); \
  mpu9250_write(addr_value, sizeof(addr_value)); \
  cs_deassert(); \
  cs_wait(); \
}

#define mpu9250_get2(address, vp, size) { \
  static const  uint8_t addr[1] = {0x80 | address}; \
  cs_assert(); \
  cs_wait(); \
  mpu9250_write(addr, sizeof(addr)); \
  mpu9250_read(vp, size); \
  cs_deassert(); \
  cs_wait(); \
}

#define mpu9250_get(address, value) mpu9250_get2(address, (uint8_t *)&(value), sizeof(value))

volatile uint8_t mpu9250_capture = false;
static uint8_t mpu9250_available = false;

void mpu9250_init(){
  uint8_t v;

  cs_deassert();
  clk_up();
  mpu9250_set(PWR_MGMT_1, 0x80); // Chip reset
  wait_ms(100);
  mpu9250_set(PWR_MGMT_1, 0x01); // Wake up device and select the best available clock
  
  mpu9250_set(USER_CTRL, 0x10); // SPI mode only
  mpu9250_set(GYRO_CONFIG, (3 << 3)); // FS_SEL = 3 (2000dps)

  mpu9250_get(WHO_AM_I, v); // expecting 0x71
  if(v == 0x71){
    mpu9250_available = true;
  }
}

int8_t mpu9250_polling2(FrameData *val){
	if(!mpu9250_available){return 1;}

	mpu9250_get(GYRO_OUT_BASE, *val);

	return 0;
}

int8_t mpu9250_pollingAcceleration(FrameData *val){
	if(!mpu9250_available){return 1;}

	mpu9250_get(ACCEL_OUT_BASE, *val);

	return 0;
}

void mpu9250_polling(){
	uint16_t gbuf[3];
	if(!mpu9250_available){return;}

	mpu9250_get(GYRO_OUT_BASE, gbuf);

	printf("%d,%d,%d\n",gbuf[0],gbuf[1],gbuf[2]);
	return;

}
