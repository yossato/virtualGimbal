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
#include "inc/ringBuffer.h"

static uint8_t Ring[RINGSIZE];  /* �����O�o�b�t�@ */
static uint32_t Rp = 0;          /* �ǂݍ��݃|�C���^ */
static uint32_t Wp = 0;          /* �������݃|�C���^ */
static bool putingFailed = false;	//Put�֐������s������true�A�ُ킪�Ȃ����false

/**
 * @brief �����O�o�b�t�@��1�����������݂܂�
 **/
void rbPut(int8_t val){
	uint32_t next = (Wp + 1) & (RINGSIZE-1);
	if(next == Rp){//�����O�o�b�t�@�����^���̎��͌x�����o���ăf�[�^��p���B
		putingFailed = true;
		return;
	}
//	if( next == Rp ) {
//		fprintf( stderr, "�����O�o�b�t�@�����t�ł��B�ǂ܂Ȃ�����f�[�^�𗎂��܂�\n" );
//		return;
//	}
	Ring[Wp] = val;
	Wp = next;
}

/**
 * @brief �����O�o�b�t�@���當�����ǂݏo���܂�
 * @param [out] buf			��������i�[����֐�
 * @param [in]  maxLength	�����O�o�b�t�@����ǂݎ��ő�̕�����̒���
 * @retval �ǂݎ�ꂽ������̒���
 **/
uint32_t rbGet(int8_t *buf, int32_t maxLength){
	uint32_t retval = 0;
	const uint8_t warning[] = "ring buf is full!\r\n";

	//��xput�Ɏ��s�����`�Ղ�����Όx�����o��
	if(putingFailed == true){
		while(retval<maxLength){
			if(retval==strlen(warning)){//������̖���
				putingFailed = false;
				break;
			}
			buf[retval] = warning[retval];
			retval++;
		}
	}

	while((Rp != Wp)&&(retval<maxLength)) {    /* �����O�o�b�t�@����łȂ� */
		buf[retval] = Ring[Rp];
		Rp = (Rp + 1) & (RINGSIZE-1);
		retval++;
	}
//	if( Rp != Wp ) {    /* �����O�o�b�t�@����łȂ� */
//		*buf = Ring[Rp];
//		Rp = (Rp + 1) & (RINGSIZE-1);
//		return 0;
//	}else {            /* �����O�o�b�t�@���� */
//		return -1;
//	}
	return retval;
}

int rbIsEmpty( )
{
   return  (Rp==Wp)?(1):(0);
}
