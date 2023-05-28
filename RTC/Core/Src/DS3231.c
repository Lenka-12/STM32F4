/*
 * DS3231.c
 *
 *  Created on: May 26, 2023
 *      Author: David
 */

#include "DS3231.h"

uint8_t dectoBCD(int num){
	return (uint8_t)(((num/10)<<4) | (num%10));
}

int BCDtodec(uint8_t num){
	return (int)(((num>>4)*10)+(num&0x0F));
}

void Set_Time(Time* Current_Time){
	uint8_t buffer[8];
	buffer[0] = SEC_ADD;
	buffer[1] = dectoBCD(Current_Time-> sec);
	buffer[2] = dectoBCD(Current_Time-> min);
	buffer[3] = dectoBCD(Current_Time-> hr);
	buffer[4] = dectoBCD(Current_Time-> dow);
	buffer[5] = dectoBCD(Current_Time-> dom);
	buffer[6] = dectoBCD(Current_Time-> mon);
	buffer[7] = dectoBCD(Current_Time-> yr);
	I2C_Master_Transmit(ADD,buffer, 8);

}
void Get_Time(Time* time){
	uint8_t current_time[7];
	I2C_Mem_Read(ADD, SEC_ADD, 1, current_time, 7);

	time->sec = BCDtodec( current_time[0]);
	time->min = BCDtodec( current_time[1]);
	time->hr= BCDtodec( current_time[2]);
	time->dow = BCDtodec( current_time[3]);
	time->dom = BCDtodec( current_time[4]);
	time->mon = BCDtodec( current_time[5]);
	time->yr = BCDtodec( current_time[6]);


}


