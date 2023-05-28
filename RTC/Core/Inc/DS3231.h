/*
 * DS3231.h
 *
 *  Created on: May 26, 2023
 *      Author: David
 */

#ifndef INC_DS3231_H_
#define INC_DS3231_H_
#include "i2c.h"

#define ADD 0x68
#define SEC_ADD 0x00

typedef struct{
	int sec; //Seconds
	int min; //Minutes
	int hr;  //Hours
	int dow;  //Day of Week
	int dom;  //Day of Month
	int mon;  // Month
	int yr;  //Year

} Time;

void Set_Time(Time* Current_Time);
void Get_Time(Time* time);
uint8_t dectoBCD(int num);
int BCDtodec(uint8_t num);


#endif /* INC_DS3231_H_ */
