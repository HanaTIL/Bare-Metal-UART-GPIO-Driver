/*
 * Time_Out.h
 *
 *  Created on: Sep 26, 2025
 *      Author: tilou
 */

#ifndef TIMEOUT_H_
#define TIMEOUT_H_

#include "stdint.h"

typedef enum
{

	OK=0,
	Error=1

}TypedefStatus;

void Ticks_Init(uint32_t freq);

uint32_t get_Ticks();

void delay(uint32_t delay_ms);


#endif /* TIMEOUT_H_ */


/* TIME_OUT_H_ */
