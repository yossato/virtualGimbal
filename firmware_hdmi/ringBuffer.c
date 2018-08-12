/*
 * ringBuffer.c
 *
 *  Created on: 2017/02/03
 *      Author: yoshiaki
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "ringBuffer.h"

static uint8_t Ring[RINGSIZE];  /* リングバッファ */
static uint32_t Rp = 0;          /* 読み込みポインタ */
static uint32_t Wp = 0;          /* 書き込みポインタ */
static bool putingFailed = false;	//Put関数が失敗したらtrue、異常がなければfalse

/**
 * @brief リングバッファに1文字書き込みます
 **/
void rbPut(int8_t val){
	uint32_t next = (Wp + 1) & (RINGSIZE-1);
	if(next == Rp){//リングバッファが満タンの時は警告を出してデータを廃棄。
		putingFailed = true;
		return;
	}
//	if( next == Rp ) {
//		fprintf( stderr, "リングバッファが満杯です。読まない限りデータを落します\n" );
//		return;
//	}
	Ring[Wp] = val;
	Wp = next;
}

/**
 * @brief リングバッファから文字列を読み出します
 * @param [out] buf			文字列を格納する関数
 * @param [in]  maxLength	リングバッファから読み取る最大の文字列の長さ
 * @retval 読み取れた文字列の長さ
 **/
uint32_t rbGet(int8_t *buf, int32_t maxLength){
	uint32_t retval = 0;
	const uint8_t warning[] = "ring buf is full!\r\n";

	//一度putに失敗した形跡があれば警告を出す
	if(putingFailed == true){
		while(retval<maxLength){
			if(retval==strlen(warning)){//文字列の末尾
				putingFailed = false;
				break;
			}
			buf[retval] = warning[retval];
			retval++;
		}
	}

	while((Rp != Wp)&&(retval<maxLength)) {    /* リングバッファが空でない */
		buf[retval] = Ring[Rp];
		Rp = (Rp + 1) & (RINGSIZE-1);
		retval++;
	}
//	if( Rp != Wp ) {    /* リングバッファが空でない */
//		*buf = Ring[Rp];
//		Rp = (Rp + 1) & (RINGSIZE-1);
//		return 0;
//	}else {            /* リングバッファが空 */
//		return -1;
//	}
	return retval;
}

int rbIsEmpty( )
{
   return  (Rp==Wp)?(1):(0);
}
