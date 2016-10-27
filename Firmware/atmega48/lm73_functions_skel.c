// lm73_functions.c       
// Roger Traylor 11.28.10

#include <util/twi.h>
#include "lm73_functions.h"
#include <util/delay.h>

//TODO: remove volatile type modifier?  I think so.
//TODO: initalize with more resolution and disable the smb bus timeout
//TODO: write functions to change resolution, alarm etc.

uint8_t lm73_wr_buf[2];
uint8_t lm73_rd_buf[2];

//********************************************************************************
char int_to_char(uint16_t number)
{
	switch(number)
	{
		case 0: return '0';
		case 1: return '1';
		case 2: return '2';
		case 3: return '3';
		case 4: return '4';
		case 5: return '5';
		case 6: return '6';
		case 7: return '7';
		case 8: return '8';
		case 9: return '9';
		default: return '#';
	}
	return '#';
}
//******************************************************************************
uint8_t lm73_temp_convert(char temp_digits[], uint16_t lm73_temp, uint8_t f_not_c){
//given a temperature reading from an LM73, the address of a buffer
//array, and a format (deg F or C) it formats the temperature into ascii in 
//the buffer pointed to by the arguement.
//TODO:Returns what???(uint8_t)??? 																																																																																										Probably a BUG?

//Yeah, this is for you to do! ;^)
	uint8_t negative_flag = 0;
	uint16_t temp = 0;
 
	if(lm73_temp & (1<<15)) //if value is negative
	{
		negative_flag = 1; //set negative flag
		lm73_temp &= ~(1<<15); //clear negative bit
		lm73_temp = lm73_temp >> 5; //shift number to the right 5 times
		lm73_temp = ~(lm73_temp); //invert bits
		lm73_temp &= 0b0000011111111111; //clear upper bits
		lm73_temp++; //add one
		lm73_temp = lm73_temp << 5; //shift back into place, it is now in positive format
	}

	temp = (lm73_temp>>5) & 0x0003; //get fractional bits
	temp = temp * 25; //multiply by 25 (0.25 degrees celsius)
	temp = temp + (((lm73_temp>>7) & 0x003F) * 100); //get non-fractional bits, multiply by 100, add to fractional result	

	
	if(f_not_c) //fahrenheit mode
	{
		temp = ((temp * 1.8)) + 3200; //eqn: fahrenheit = celsius*9/5 + 32, 9/5 = 1.8, 32*100 = 3200	
	} 

	temp_digits[0] = int_to_char((temp / 10000) % 10); //hundreds
	temp_digits[1] = int_to_char((temp / 1000) % 10); //tens
	temp_digits[2] = int_to_char((temp / 100) % 10); //ones
	temp_digits[3] = '.'; //decimal point
	temp_digits[4] = int_to_char((temp / 10) % 10); //(decimal)tens
	temp_digits[5] = int_to_char(temp % 10); //(decimal)ones

	if(temp_digits[0] == '0') //if hundreds is 0
	{
		if(negative_flag) //if negative
		{
			temp_digits[0] = '-'; //set hundreds char to negative symbol
			negative_flag = 0; //clear negative flag
		}
		else //it was not negative
		{
			temp_digits[0] = ' '; //set hundreds char to blank space
		}
		if(temp_digits[1] == '0') //if tens char is also zero
		{
			if(temp_digits[0] == '-') //if hundreds char was set to negative symbol
			{
				temp_digits[0] = ' '; //set hundreds char to blank space
				temp_digits[1] = '-'; //set tens char to negative symbol
			}
			else //it was not negative
			{
				temp_digits[1] = ' '; //set hundreds char to blank space
			}
		}
	}

	return negative_flag; //return 1 if negative

}//lm73_temp_convert
//******************************************************************************


