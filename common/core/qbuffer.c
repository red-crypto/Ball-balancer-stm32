/*
 * qbuffer.c
 *
 *  Created on: May 2, 2024
 *      Author: user
 */
//typedef struct
//{
//	uint32_t in;
//	uint32_t out;
//	uint32_t len;
//
//	uint8_t *p_buf;
//} qbuffer_t;


#include "qbuffer.h"

void qbufferInit(void)
{

}
bool qbufferCreate(qbuffer_t *p_node, uint8_t *p_buf, uint32_t length)
{
	bool ret = true;

	p_node->in    = 0;
	p_node->out   = 0;
	p_node->len   = length;
	p_node->p_buf = p_buf;

	return ret;

}
bool qbufferWrite(qbuffer_t *p_node, uint8_t *p_data, uint32_t length)
{
	bool ret = true;
	// p_data에 값 입력, in증가
	//조건은?
	for(int i=0; i<length; i++)
	{
	p_node->p_buf[p_node->in] = p_data[i];
	p_node->in = (p_node->in + 1) % p_node->len;
	}

	return ret;

}
bool qbufferRead(qbuffer_t *p_node, uint8_t *p_data, uint32_t length)
{
	bool ret = true;
	//조건하에
	//out에 해당하는 데이터를 p_data에 읽고, out 증가?
	for(int i=0; i<length; i++)
	{
	p_data[i] = p_node->p_buf[p_node->out];
	p_node->out = p_node->out + 1 % p_node->len;
	}


	return ret;

}
uint32_t qbufferAvailable(qbuffer_t *p_node)
{
	uint32_t ret = 0;

	ret = (p_node->out - p_node->in) % p_node->len;

	return ret;

}
void qbufferFlush(qbuffer_t *p_node)
{
	p_node->in = 0;
	p_node->out =0;

}





