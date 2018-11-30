/*
 * ringBuffer.h
 *
 *  Created on: 2017/02/03
 *      Author: yoshiaki
 */

#ifndef INC_RINGBUFFER_H_
#define INC_RINGBUFFER_H_

#define RINGSIZE  256	//2‚Ì™p‚Æ‚·‚é(—á128,256...)

void rbPut(int8_t val);
uint32_t rbGet(int8_t *buf, int32_t maxLength);
int rbIsEmpty( );
#endif /* INC_RINGBUFFER_H_ */
