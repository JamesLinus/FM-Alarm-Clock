// lab5.c 
// Marc Ohlmann
// 12/1/2015

//  HARDWARE SETUP: (Atmega48)
/*
	

*/
//  ...

#define F_CPU 8000000UL // cpu speed in hertz 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "twi_master.h"
#include "uart_functions.h"
#include "lm73_functions.h"

//twi_master.c
extern volatile uint8_t  *twi_buf;      //pointer to the buffer we are xferred from/to
extern volatile uint8_t  twi_msg_size;  //number of bytes to be xferred
extern volatile uint8_t  twi_bus_addr;  //address of device on TWI bus 
extern volatile uint8_t  twi_state;     //status of transaction

//uart_functions.c
extern char uart_tx_buf[40];      //holds string to send to crt
extern char uart_rx_buf[40];      //holds string that recieves data from uart

//lm73
extern uint8_t lm73_wr_buf[2];
extern uint8_t lm73_rd_buf[2];


volatile char temp[6] = {'0','3', '7', '.', '0', '0'};
volatile uint16_t lm73_temp;

//***********************************************************************************
int main()
{	
	//**** initialization ****//

	//port
	//PORTC = 0b00110000; //set pin 4,5 pullup resistor

	//initialize SPI (encoder / bargraph access)
	SPCR |= (1<<SPE) | (1<<MSTR) | (0<<SPIE); //enable SPI, set master mode, no interrupt 
	SPSR |= (1<<SPI2X);	//double speed

	//TWI
	init_twi();

	//USART
	uart_init();

	uint8_t i;
	char rx_request;

	//initiate lm73 temperature sensor over twi
	lm73_wr_buf[0] = LM73_PTR_TEMP;   //load lm73_wr_buf[0] with temperature pointer address
	twi_start_wr(LM73_ADDRESS, lm73_wr_buf, 1);   //start the TWI write process (twi_start_wr())
	//_delay_ms(2); //wait for it to finish

	sei();
	
	//************************//	
	while(1)
	{
		_delay_ms(10);
		//Get data through TWI (I2C)
		twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 4);  //read temperature data from LM73 (2 bytes)
		_delay_ms(2);    //wait for it to finish
		lm73_temp = lm73_rd_buf[0]; //save high temperature byte into lm73_temp
  		lm73_temp = lm73_temp << 8; //shift it into upper byte 
  		lm73_temp |= lm73_rd_buf[1]; //"OR" in the low temp byte to lm73_tem
		
		rx_request = uart_getc();
		if(rx_request != 0) //if request made
		{
			uart_putc(lm73_rd_buf[0]); //send data
			uart_putc(lm73_rd_buf[1]); //send data
			/*
			if(lm73_temp_convert(temp, lm73_temp, 0)) //convert to string (celsius), check if negative
			{
				uart_putc('-'); //send negative sign
			}
			else			{
				uart_putc(' '); //send blank space
			}
			for(i = 0; i < 6; i++){ uart_putc(temp[i]); } //send string across USART
			*/
		}
		
		/*
		if(rx_request == 'f') //if request made for fahrenheit
		{
			uart_putc('k'); //send acknowledgment
			if(lm73_temp_convert(temp, lm73_temp, 1)) //convert to string (fahrenheit), check if negative
			{
				uart_putc('-'); //send negative sign
			}
			else
			{
				uart_putc(' '); //send blank space
			}
			for(i = 0; i < 6; i++){ uart_putc(temp[i]); } //send string across USART
		}*/
	}
	
	return 0;
}
//***********************************************************************************

//*********************************************//
//	Interrupt Routine = USART Rx complete  //
//*********************************************//
/*
ISR(USART0_RXC_vect)
{
	uint8_t i;
	if(UDR0 == 'c') //if request made for celsius
	{
		uart_putc('k'); //send acknowledgment
		if(lm73_temp_convert(temp, lm73_temp, 0)) //convert to string (celsius), check if negative
		{
			uart_putc('-'); //send negative sign
		}
		else
		{
			uart_putc(' '); //send blank space
		}
		for(i = 0; i < 6; i++){ uart_putc(temp[i]); } //send string across USART
	}
	else if(UDR0 == 'f') //if request made for fahrenheit
	{
		uart_putc('k'); //send acknowledgment
		if(lm73_temp_convert(temp, lm73_temp, 1)) //convert to string (fahrenheit), check if negative
		{
			uart_putc('-'); //send negative sign
		}
		else
		{
			uart_putc(' '); //send blank space
		}
		for(i = 0; i < 6; i++){ uart_putc(temp[i]); } //send string across USART
	}

}*/




