// lab5.c 
// Marc Ohlmann
// 12/1/2015

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTB bit 7 goes to the PWM transistor base.

#define F_CPU 16000000UL // cpu speed in hertz 
#define TRUE 1 //true = 1
#define FALSE 0 //false = 0

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "hd44780.h"
#include "twi_master.h"
#include "uart_functions.h"
#include "lm73_functions.h"
#include "si4734.h"

#define DEBUG 0 //set to one for testing mode
#define LED_OFF 0b11111111 //binary code for turning an 7 seg LED off
#define LED_COLON_CLOCK 0b00000010 //binary code for setting the colon at AM time
#define LED_COLON_CLOCK_PM 0b00000000 //binary code for setting the colon at PM time
#define ALARM_ONOFF 2 //button index for turning alarm on / off
#define TEMPERATURE_MODE 5 //button index for changing temperature setting
#define HOUR_MODE 6 //button index for changing hour setting
#define SNOOZE_ONOFF 7 //button index for activating snooze
#define CLOCK_SET 0 //button index for changing the clock time
#define ALARM_SET 1 //button index for changing the alarm time
#define ALARM_MODE 3 //button index for setting the alarm mode
#define RADIO_ONOFF 4 //button index for turning radio on / off
#define CONTROL_BITS 0b00000011 //determines which buttons are "setting modes" which are mutually exclusive operating modes
#define LCD_CYCLE_SECONDS 5 //number of seconds to wait before cycling the lcd
#define LCD_DISPLAY_PAGES 2 //number of "pages" to cycle through on the LCD
#define BTN_COUNT 8 //determines the number of buttons that are checked
#define USER_ACTIVITY_TIMER 6 //number of seconds before the clock will reset control setting after no user input
#define SNOOZE_TIME 10 //number of seconds that the snooze function will wait for before resetting alarm
#define DISPLAY_TIME 0 //index for display time mode
#define DISPLAY_CLOCKSET 1 //index for display clock set mode
#define DISPLAY_ALARMSET 2 //index for display alarm set mode
#define DISPLAY_VOLUME 3 //index for display volume mode
#define DISPLAY_RADIO 4 //index for display radio frequency mode

//*******************************************
//**********     ACCELEROMETER     **********
//*******************************************
//Address definitions
//*******************
#define ACCEL_ADDRESS 0x3A //I2C slave address for RedBot Sensor Accelerometer
#define ACCEL_PADDR_XREGH 0x01 //pointer address to x high (read only)	*note* reading 6 bytes from XREGH will auto-increment
#define ACCEL_PADDR_XREGL 0x02 //pointer address to x low  (read only)		through to ZREGL effectively reading all
#define ACCEL_PADDR_YREGH 0x03 //pointer address to y high (read only)		acceleration data registers
#define ACCEL_PADDR_YREGL 0x04 //pointer address to y low  (read only)
#define ACCEL_PADDR_ZREGH 0x05 //pointer address to z high (read only)	data format: H 0b11111111 , 1  = readable
#define ACCEL_PADDR_ZREGL 0x06 //pointer address to z low  (read only)  (12 bit res) L 0b11110000 , 0  = reserved (always 0)
#define ACCEL_PADDR_SMOD 0x0B //pointer address to system mode register  
#define ACCEL_PADDR_CTRL1 0x2A //pointer address to ctrl register 1. *note* writing 2 bytes to ctrl register 1 will 
#define ACCEL_PADDR_CTRL2 0x2B //pointer address to ctrl register 2. 		 auto-increment second byte to ctrl register 2
#define ACCEL_PADDR_RNG 0x0E //pointer address to output range setting register
#define ACCEL_PADDR_WHOAMI 0x0D //pointer address to "who am i" device id register (read only)

//************************************
//control register setting definitions
//************************************
#define ACCEL_SETTING_RNG_2G 0x00 //rng register to set output setting to 2g (min range span, max resolution)
#define ACCEL_SETTING_RNG_8G 0x02 //rng register to set output setting to 8g (max range span, min resolution)
#define ACCEL_SETTING_CTRL2_RESET 0x40 //control register2 setting to reset, no need to touch control register1
#define ACCEL_SETTING_CTRL1_INIT 0x08 //control register1 setting for setup, high byte:control1 (send to wr_buffer[0])
#define ACCEL_SETTING_CTRL2_INIT 0x00 //control register2 setting for setup, low byte:control2 (send to wr_buffer[1])
#define ACCEL_SETTING_CTRL1_ACTIVATE 0x09 //control register1 setting to activate, send to wr_buffer[0]
#define ACCEL_SETTING_SMOD_WAKE 0x01 //smod register setting for wake mode

//************************************
//register setting descriptions
//************************************
   //smod       -  bits
		//  1:0 - system mode (default 00 standby)
		

   //rng	-  bits						     
		//   4  - output high pass filter (default 0 no hpf) 
		//  1:0 - full scale range (default 00 2g) 	     

   //control 1	-  bits
		//  7:6 - sample rate during sleep mode (default 50Hz)
		//  5:3 - data rate (default 800Hz)
		//   2  - low noise mode (default 0 disabled)
		//   1  - 8-bit mode (default 0 disabled)
		//   0  - active/standby (default 0 standby)

   //control 2  -  bits
		//   7  - self-test (default 0 disabled)
		//   6  - software reset (default 0 no reset)
		//   5  - no use
		//  4:3 - sleep mode (default 00 normal mode)
		//   2  - sleep enable (default 0 auto sleep disabled)
		//  1:0 - active mode

//****************************************

//function declarations
uint8_t dec_to_sevseg(uint16_t dec, uint8_t dp); //takes a decimal number and converts to 7seg led binary code, only works for 0-9
uint8_t convert_accel(char accel_digits[], uint16_t accel_data); //takes accelerometer data and converts to char form
char int_to_char(uint16_t number); //converts an integer to a char, only works for 0-9

//kellen_music.c function declarations
void song0(uint16_t note); //Beaver Fight Song
void song1(uint16_t note); //Tetris Theme (A)
void song2(uint16_t note); //Mario Bros Theme
void song3(uint16_t note); 
void play_song(uint8_t song, uint8_t note);
void play_rest(uint8_t duration);
void play_note(char note, uint8_t flat, uint8_t octave, uint8_t duration);
void music_off(void);
void music_on(void);      
void music_init(void);

//global variables
uint8_t accel_rd_buf[8]; //read buffer for accelerometer
uint8_t accel_wr_buf[8]; //write buffer for accelerometer
uint8_t segment_data[5]; //display data for segment board, logic zero turns segment on
uint16_t mode = 0;  //8 bits sent to bar graph display
uint8_t control = 0; //determines what the encoders change
uint8_t display_digit = 0; //number of digit to display on seg board [0 - 4] (0,1,3,4 = digit, 2 = colon)
volatile uint8_t display_mode = 0; //determines what value the LED will display (volume, radio, time, alarm set, clock set, etc.)
uint8_t clock_minutes = 0; //minutes for time of day
uint8_t clock_hours = 11; //hours for time of day
volatile uint8_t clock_seconds = 0; //seconds for time of day
uint16_t clock_time = 0; //value sent to display representing time of day
uint16_t alarm_time = 0; //value sent to display representing alarm trigger time
volatile uint8_t blink_flag = 0; //determines whether or not to display led for 1 second blinking implementation
volatile uint8_t colon_blink_flag = 0; //used to toggle colon every 1 sec
uint16_t radio = 999; //stores value sent to display representing radio setting, initially 99.9 (KRKT CORVALLIS)
uint16_t radio_cnt = 59; //user changable value used to calculate radio freq selection
uint16_t radio_cnt_last = 59; //value to compare against to command radio to tune
uint8_t alarm_minutes = 0; //minutes for alarm trigger
uint8_t alarm_hours = 11; //hours for alarm trigger
volatile uint8_t activity_count; //tracks time for last user input
volatile uint8_t volume; //volume setting
volatile uint8_t snooze_count = 0; //timer for snooze to go off
uint8_t encoder_raw, encoder1_current, encoder2_current; //storage place for encoder raw data
uint8_t encoder1_last, encoder2_last; //storage place for previous encoder data
uint8_t firstrun_flag = 1; //flag set after the first run through main
char accel_char_x[2]; //holds x accel data in char format
char accel_char_y[2]; //holds y accel data in char format
char accel_char_z[2]; //holds z accel data in char format
char temperature_local[6]; //holds temperature in char format from lm73
char temperature_remote[6]; //holds temperature in char format from atmega48
uint8_t temperature_mode = 1; //temperature setting for celsius(0) or fahrenheit(1)
uint8_t data_collect_flag = 0; //flag set every second to gather temperature data
uint8_t alarm_mode = 0; //setting for what sound the alarm event will trigger
volatile uint8_t alarm_flag = 0; //flag set when alarm is going off
volatile uint8_t snooze_flag = 0; //flag set when alarm is snoozed
uint8_t lcd_display_mode = 0;
char lcd_master_string_array[3][32];
char lcd_initial_chars1[32] = { 'A', 'l', 'a', 'r', 'm', ':', ' ', 'B', //alarm mode
			      'u', 'z', 'z', ' ', ' ', ' ', ' ', 'Q', //local temperature
			      'S', 'i', 'g', 'n', 'a', 'l', ':', ' ', //signal strength
			      '*', '*', '*', '*', '*', '*', '*', 'Q'}; //remote temperature
char lcd_initial_chars2[32] = { 'L', 'o', 'c', '(', 'F', ')', ':', ' ', //alarm mode
			      'x', 'x', 'x', 'x', 'x', 'x', ' ', 'Q', //local temperature
			      'R', 'e', 'm', '(', 'F', ')', ':', ' ', //signal strength
			      'x', 'x', 'x', 'x', 'x', 'x', ' ', 'Q'}; //remote temperature
char lcd_initial_chars3[32] = { 'A', 'c', 'c', 'e', 'l', 'e', 'r', 'o', //alarm mode
			      'm', 'e', 't', 'e', 'r', ' ', ' ', 'Q', //local temperature
			      'X', ' ', 'x', 'x', ' ', 'Y', ' ', 'x', //signal strength
			      'x', ' ', 'Z', ' ', 'x', 'x', ' ', 'Q'}; //remote temperature
//***** External Variables *****//
//hd44780.c (LCD Driver)
extern char lcd_string_array[32];  //holds two strings to refresh the LCD
extern char  lcd_str[16];  //holds string to send to lcd  

//kellen_music.c
extern volatile uint8_t song;  //setting for which song to play
extern volatile uint16_t beat; 
extern volatile uint16_t max_beat;
extern volatile uint8_t  notes; //current note position in song

//twi_master.c 
extern volatile uint8_t  *twi_buf;      //pointer to the buffer we are xferred from/to
extern volatile uint8_t  twi_msg_size;  //number of bytes to be xferred
extern volatile uint8_t  twi_bus_addr;  //address of device on TWI bus 
extern volatile uint8_t  twi_state;     //status of transaction  

//uart_functions.c
extern char uart_tx_buf[40];      //holds string to send to crt
extern char uart_rx_buf[40];      //holds string that recieves data from uart

//lm73_functions.c (temperature sensor)
extern uint8_t lm73_wr_buf[2];
extern uint8_t lm73_rd_buf[2];

//si4734.c (radio)
extern uint8_t si4734_wr_buf[9];          //buffer for holding data to send to the si4734 
extern uint8_t si4734_rd_buf[15];         //buffer for holding data recieved from the si4734
extern uint8_t si4734_tune_status_buf[8]; //buffer for holding tune_status data  
extern uint8_t si4734_revision_buf[16];   //buffer for holding revision  data  

extern volatile enum radio_band current_radio_band;

extern volatile uint8_t STC_interrupt;  //flag bit to indicate tune or seek is done

extern uint16_t eeprom_fm_freq;
extern uint16_t eeprom_am_freq;
extern uint16_t eeprom_sw_freq;
extern uint8_t  eeprom_volume;

extern uint16_t current_fm_freq;
extern uint16_t current_am_freq;
extern uint16_t current_sw_freq;
extern uint8_t  current_volume;

//Used in debug mode for UART1
extern char uart1_tx_buf[40];      //holds string to send to crt
extern char uart1_rx_buf[40];      //holds string that recieves data from uart
//******************************//

//******************************************************************************
//						dec_to_sevseg
//converts a decimal to a binary code for a seven segment display. Assumes active low
//so 0 turns on a segment and 1 turns it off. Only uses the one's digit for any number 
//passed in when considering the code to display. For example passing 103 returns the same 
//code as passing 13 or 3.
uint8_t dec_to_sevseg(uint16_t dec, uint8_t dp){
//******************************************************************************
  switch(dp)
  {
    case 0: //decimal point off
  	switch(dec)
  	{
		case 0:
			return 0b11000000;//binary code to display '0'
		case 1:
			return 0b11111001;//binary code to display '1'
		case 2:
			return 0b10100100;//binary code to display '2'
		case 3:
			return 0b10110000;//binary code to display '3'
		case 4:
			return 0b10011001;//binary code to display '4'
		case 5:
			return 0b10010010;//binary code to display '5'
		case 6:
			return 0b10000010;//binary code to display '6'
		case 7:
			return 0b11111000;//binary code to display '7'
		case 8:
			return 0b10000000;//binary code to display '8'
		case 9:
			return 0b10011000;//binary code to display '9'
		default:
			return 0b11111111; //blank by default
	}
	break;
    case 1: //decimal point on
	switch(dec)
  	{
		case 0:
			return 0b01000000;//binary code to display '0'
		case 1:
			return 0b01111001;//binary code to display '1'
		case 2:
			return 0b00100100;//binary code to display '2'
		case 3:
			return 0b00110000;//binary code to display '3'
		case 4:
			return 0b00011001;//binary code to display '4'
		case 5:
			return 0b00010010;//binary code to display '5'
		case 6:
			return 0b00000010;//binary code to display '6'
		case 7:
			return 0b01111000;//binary code to display '7'
		case 8:
			return 0b00000000;//binary code to display '8'
		case 9:
			return 0b00011000;//binary code to display '9'
		default:
			return 0b01111111; //blank by default
	}
	break;
    default:
	break;
  }
  return 0b11111111; //(logically unreachable)
	
}//dec_to_sevseg
//***********************************************************************************


//******************************************************************************
//                            chk_buttons                                      
//Checks the state of the button number passed to it. It shifts in ones till   
//the button is pushed. Function returns a 1 only once per debounced button    
//push so a debounce and toggle function can be implemented at the same time.  
//Adapted to check all buttons from Ganssel's "Guide to Debouncing"            
//Expects active low pushbuttons on PINA port.  Debounce time is determined by 
//external loop delay times 12. 
//
uint8_t chk_buttons(uint8_t button) {
//******************************************************************************
  static uint16_t state[8] = {0,0,0,0,0,0,0,0}; //hold state for each button
  asm("nop");
  asm("nop");	
  state[button] = (state[button] << 1) | (! bit_is_clear(PINA, button)) | 0xE000;
	
  if (state[button] == 0xF000)
  {
	return 1;
  }
	
  return 0;

}//chk_buttons
//***********************************************************************************


//******************************************************************************
//                            convert_accel                                      
// 
//
uint8_t convert_accel(char accel_digits[], uint16_t accel_data) {
//******************************************************************************
  	uint8_t negative_flag = 0;
	uint16_t temp = 0;
 
	if(accel_data & (1<<15)) //if value is negative
	{
		negative_flag = 1; //set negative flag
		accel_data &= ~(1<<15); //clear negative bit
		accel_data = ~(accel_data); //invert bits
		accel_data += 16; //add one, it is now in positive format
	}

	temp = accel_data / 3276.75; //maps [0, (2^16) - 1] to [0, 20]
	//temp = accel_data / 102.4; 

	accel_digits[0] = int_to_char((temp / 10) % 10); // tens
	accel_digits[1] = int_to_char(temp % 10); // ones

	if(accel_digits[0] == '0') //if tens is 0
	{
		if(negative_flag) //if negative
		{
			accel_digits[0] = '-'; //set tens char to negative symbol
			negative_flag = 0; //clear negative flag
		}
		else //it was not negative
		{
			accel_digits[0] = ' '; //set tens char to blank space
		}
	}

	return negative_flag; //return 1 if need to insert negative symbol before temp_digits occurence


}//convert_accel
//***********************************************************************************


//***********************************************************************************
//                                   segment_sum                                    
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit 
//BCD segment code in the array segment_data for display.                       
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
void segsum(uint16_t sum) {
  //determine how many digits there are 
  uint8_t num_digits = 1; //stores number of digits to illuminate, always at least 1
  
  if(sum >= 10) //if the count is 10 or larger
  {
	num_digits = 2; //need two digits to display
  }
  
  if(sum >= 100) //if the count is 100 or larger
  {
	num_digits = 3; //need three digits to display
  }

  if(sum >= 1000) //if the count is 1000 or larger
  {
	num_digits = 4; //need four digits to display
  }
  
  //break up decimal sum into 4 digit-segments
  //blank out leading zero digits 
  //now move data to right place for misplaced colon position
  switch(display_mode)
  {
	case DISPLAY_TIME:
  		switch(num_digits) //perform a set of operations determined entirely by the value of num_digits
  		{
			case 1: //display only digit0
				segment_data[0] = sum % 10; //ones
				segment_data[1] = 0; //off
				if(colon_blink_flag){ segment_data[2] = LED_OFF; }
				else
				{ 
					if( !(mode & (1<<HOUR_MODE)) ) //12 hr mode
					{
						if(clock_hours >= 11 && clock_hours < 23){ segment_data[2] = LED_COLON_CLOCK_PM; } 
						else{ segment_data[2] = LED_COLON_CLOCK; }
					}
					else //24 hr mode
					{
						if(clock_hours > 11){ segment_data[2] = LED_COLON_CLOCK_PM; } 
						else{ segment_data[2] = LED_COLON_CLOCK; }
					}
				}
				segment_data[3] = 0; //off
				segment_data[4] = LED_OFF; //off
				return;
			case 2: //display digit0 and digit1
				segment_data[0] = sum % 10; //ones
				segment_data[1] = (sum / 10) % 10; //tens
				if(colon_blink_flag){ segment_data[2] = LED_OFF; }
				else
				{ 
					if( !(mode & (1<<HOUR_MODE)) ) //12 hr mode
					{
						if(clock_hours >= 11 && clock_hours < 23){ segment_data[2] = LED_COLON_CLOCK_PM; } 
						else{ segment_data[2] = LED_COLON_CLOCK; }
					}
					else //24 hr mode
					{
						if(clock_hours > 11){ segment_data[2] = LED_COLON_CLOCK_PM; } 
						else{ segment_data[2] = LED_COLON_CLOCK; }
					}
				}
				segment_data[3] = 0; //off
				segment_data[4] = LED_OFF; //off
				return;
			case 3: //display digit0, digit1, and digit2
				segment_data[0] = sum % 10; //ones
				segment_data[1] = (sum / 10) % 10; //tens
				if(colon_blink_flag){ segment_data[2] = LED_OFF; }
				else
				{ 
					if( !(mode & (1<<HOUR_MODE)) ) //12 hr mode
					{
						if(clock_hours >= 11 && clock_hours < 23){ segment_data[2] = LED_COLON_CLOCK_PM; } 
						else{ segment_data[2] = LED_COLON_CLOCK; }
					}
					else //24 hr mode
					{
						if(clock_hours > 11){ segment_data[2] = LED_COLON_CLOCK_PM; } 
						else{ segment_data[2] = LED_COLON_CLOCK; }
					}
				}
				segment_data[3] = (sum / 100) % 10; //hundreds	
				segment_data[4] = LED_OFF; //off			
				return;
			case 4: //display digit0, digit1, digit2, and digit3
				segment_data[0] = sum % 10; //ones
				segment_data[1] = (sum / 10) % 10; //tens
				if(colon_blink_flag){ segment_data[2] = LED_OFF; }
				else
				{ 
					if( !(mode & (1<<HOUR_MODE)) ) //12 hr mode
					{
						if(clock_hours >= 11 && clock_hours < 23){ segment_data[2] = LED_COLON_CLOCK_PM; } 
						else{ segment_data[2] = LED_COLON_CLOCK; }
					}
					else //24 hr mode
					{
						if(clock_hours > 11){ segment_data[2] = LED_COLON_CLOCK_PM; } 
						else{ segment_data[2] = LED_COLON_CLOCK; }
					}
				}
				segment_data[3] = (sum / 100) % 10; //hundreds
				segment_data[4] = (sum / 1000) % 10; //thousands
				return;
			default: //default to all digits off
				segment_data[0] = LED_OFF; //off
				segment_data[1] = LED_OFF; //off
				segment_data[2] = LED_OFF; //off
				segment_data[3] = LED_OFF; //off
				segment_data[4] = LED_OFF; //off
				return;
  		}
		break;	
  	case DISPLAY_CLOCKSET:
  		switch(num_digits) //perform a set of operations determined entirely by the value of num_digits
  		{
			case 1: //display only digit0
				if(blink_flag)
				{
					segment_data[0] = sum % 10; //ones
					segment_data[1] = 0; //off
					if( !(mode & (1<<HOUR_MODE)) ) //12 hr mode
					{
						if(clock_hours >= 11 && clock_hours < 23){ segment_data[2] = LED_COLON_CLOCK_PM; } 
						else{ segment_data[2] = LED_COLON_CLOCK; }
					}
					else //24 hr mode
					{
						if(clock_hours > 11){ segment_data[2] = LED_COLON_CLOCK_PM; } 
						else{ segment_data[2] = LED_COLON_CLOCK; }
					}
					segment_data[3] = 0; //off
					segment_data[4] = LED_OFF; //off 
				}
				else
				{ 
					segment_data[0] = LED_OFF;
					segment_data[1] = LED_OFF;
					segment_data[2] = LED_OFF;
					segment_data[3] = LED_OFF;
					segment_data[4] = LED_OFF;
				}
				return;
			case 2: //display digit0 and digit1
				if(blink_flag)
				{
					segment_data[0] = sum % 10; //ones
					segment_data[1] = (sum / 10) % 10; //tens
					if( !(mode & (1<<HOUR_MODE)) ) //12 hr mode
					{
						if(clock_hours >= 11 && clock_hours < 23){ segment_data[2] = LED_COLON_CLOCK_PM; } 
						else{ segment_data[2] = LED_COLON_CLOCK; }
					}
					else //24 hr mode
					{
						if(clock_hours > 11){ segment_data[2] = LED_COLON_CLOCK_PM; } 
						else{ segment_data[2] = LED_COLON_CLOCK; }
					}
					segment_data[3] = 0; //off
					segment_data[4] = LED_OFF; //off 
				}
				else
				{ 
					segment_data[0] = LED_OFF;
					segment_data[1] = LED_OFF;
					segment_data[2] = LED_OFF;
					segment_data[3] = LED_OFF;
					segment_data[4] = LED_OFF;
				}
				return;
			case 3: //display digit0, digit1, and digit2
				if(blink_flag)
				{
					segment_data[0] = sum % 10; //ones
					segment_data[1] = (sum / 10) % 10; //tens
					if( !(mode & (1<<HOUR_MODE)) ) //12 hr mode
					{
						if(clock_hours >= 11 && clock_hours < 23){ segment_data[2] = LED_COLON_CLOCK_PM; } 
						else{ segment_data[2] = LED_COLON_CLOCK; }
					}
					else //24 hr mode
					{
						if(clock_hours > 11){ segment_data[2] = LED_COLON_CLOCK_PM; } 
						else{ segment_data[2] = LED_COLON_CLOCK; }
					}
					segment_data[3] = (sum / 100) % 10; //hundreds
					segment_data[4] = LED_OFF; //off
				}
				else
				{ 
					segment_data[0] = LED_OFF;
					segment_data[1] = LED_OFF;
					segment_data[2] = LED_OFF;
					segment_data[3] = LED_OFF;
					segment_data[4] = LED_OFF;
				}		
				return;
			case 4: //display digit0, digit1, digit2, and digit3
				if(blink_flag)
				{
					segment_data[0] = sum % 10; //ones
					segment_data[1] = (sum / 10) % 10; //tens
					if( !(mode & (1<<HOUR_MODE)) ) //12 hr mode
					{
						if(clock_hours >= 11 && clock_hours < 23){ segment_data[2] = LED_COLON_CLOCK_PM; } 
						else{ segment_data[2] = LED_COLON_CLOCK; }
					}
					else //24 hr mode
					{
						if(clock_hours > 11){ segment_data[2] = LED_COLON_CLOCK_PM; } 
						else{ segment_data[2] = LED_COLON_CLOCK; }
					}
					segment_data[3] = (sum / 100) % 10; //hundreds
					segment_data[4] = (sum / 1000) % 10; //thousands
				}
				else
				{ 
					segment_data[0] = LED_OFF;
					segment_data[1] = LED_OFF;
					segment_data[2] = LED_OFF;
					segment_data[3] = LED_OFF;
					segment_data[4] = LED_OFF;
				}
				return;
			default: //default to all digits off
				segment_data[0] = LED_OFF;//off
				segment_data[1] = LED_OFF;//off
				segment_data[2] = LED_OFF;//off
				segment_data[3] = LED_OFF;//off
				segment_data[4] = LED_OFF;//off
				return;
  		}
		break;
	case DISPLAY_ALARMSET:
  		switch(num_digits) //perform a set of operations determined entirely by the value of num_digits
  		{
			case 1: //display only digit0
				if(blink_flag)
				{
					segment_data[0] = sum % 10; //ones
					segment_data[1] = 0; //off
					if( !(mode & (1<<HOUR_MODE)) ) //12 hr mode
					{
						if(alarm_hours >= 11 && alarm_hours < 23){ segment_data[2] = LED_COLON_CLOCK_PM; } 
						else{ segment_data[2] = LED_COLON_CLOCK; }
					}
					else //24 hr mode
					{
						if(alarm_hours > 11){ segment_data[2] = LED_COLON_CLOCK_PM; } 
						else{ segment_data[2] = LED_COLON_CLOCK; }
					}
					segment_data[3] = 0; //off
					segment_data[4] = LED_OFF; //off 
				}
				else
				{ 
					segment_data[0] = LED_OFF;
					segment_data[1] = LED_OFF;
					segment_data[2] = LED_OFF;
					segment_data[3] = LED_OFF;
					segment_data[4] = LED_OFF;
				}
				return;
			case 2: //display digit0 and digit1
				if(blink_flag)
				{
					segment_data[0] = sum % 10; //ones
					segment_data[1] = (sum / 10) % 10; //tens
					if( !(mode & (1<<HOUR_MODE)) ) //12 hr mode
					{
						if(alarm_hours >= 11 && alarm_hours < 23){ segment_data[2] = LED_COLON_CLOCK_PM; } 
						else{ segment_data[2] = LED_COLON_CLOCK; }
					}
					else //24 hr mode
					{
						if(alarm_hours > 11){ segment_data[2] = LED_COLON_CLOCK_PM; } 
						else{ segment_data[2] = LED_COLON_CLOCK; }
					}
					segment_data[3] = 0; //off
					segment_data[4] = LED_OFF; //off 
				}
				else
				{ 
					segment_data[0] = LED_OFF;
					segment_data[1] = LED_OFF;
					segment_data[2] = LED_OFF;
					segment_data[3] = LED_OFF;
					segment_data[4] = LED_OFF;
				}
				return;
			case 3: //display digit0, digit1, and digit2
				if(blink_flag)
				{
					segment_data[0] = sum % 10; //ones
					segment_data[1] = (sum / 10) % 10; //tens
					if( !(mode & (1<<HOUR_MODE)) ) //12 hr mode
					{
						if(alarm_hours >= 11 && alarm_hours < 23){ segment_data[2] = LED_COLON_CLOCK_PM; } 
						else{ segment_data[2] = LED_COLON_CLOCK; }
					}
					else //24 hr mode
					{
						if(alarm_hours > 11){ segment_data[2] = LED_COLON_CLOCK_PM; } 
						else{ segment_data[2] = LED_COLON_CLOCK; }
					}
					segment_data[3] = (sum / 100) % 10; //hundreds
					segment_data[4] = LED_OFF; //off
				}
				else
				{ 
					segment_data[0] = LED_OFF;
					segment_data[1] = LED_OFF;
					segment_data[2] = LED_OFF;
					segment_data[3] = LED_OFF;
					segment_data[4] = LED_OFF;
				}		
				return;
			case 4: //display digit0, digit1, digit2, and digit3
				if(blink_flag)
				{
					segment_data[0] = sum % 10; //ones
					segment_data[1] = (sum / 10) % 10; //tens
					if( !(mode & (1<<HOUR_MODE)) ) //12 hr mode
					{
						if(alarm_hours >= 11 && alarm_hours < 23){ segment_data[2] = LED_COLON_CLOCK_PM; } 
						else{ segment_data[2] = LED_COLON_CLOCK; }
					}
					else //24 hr mode
					{
						if(alarm_hours > 11){ segment_data[2] = LED_COLON_CLOCK_PM; } 
						else{ segment_data[2] = LED_COLON_CLOCK; }
					}
					segment_data[3] = (sum / 100) % 10; //hundreds
					segment_data[4] = (sum / 1000) % 10; //thousands
				}
				else
				{ 
					segment_data[0] = LED_OFF;
					segment_data[1] = LED_OFF;
					segment_data[2] = LED_OFF;
					segment_data[3] = LED_OFF;
					segment_data[4] = LED_OFF;
				}
				return;
			default: //default to all digits off
				segment_data[0] = LED_OFF;//off
				segment_data[1] = LED_OFF;//off
				segment_data[2] = LED_OFF;//off
				segment_data[3] = LED_OFF;//off
				segment_data[4] = LED_OFF;//off
				return;
  		}
		break;
  	case DISPLAY_VOLUME:
	case DISPLAY_RADIO:
	default:
		switch(num_digits) //perform a set of operations determined entirely by the value of num_digits
 		{
			case 1: //display only digit0
				segment_data[0] = sum % 10; //ones	
				segment_data[1] = LED_OFF; //off
				segment_data[2] = LED_OFF;
				segment_data[3] = LED_OFF; //off
				segment_data[4] = LED_OFF; //off
				return;
			case 2: //display digit0 and digit1
				segment_data[0] = sum % 10; //ones
				segment_data[1] = (sum / 10) % 10; //tens
				segment_data[2] = LED_OFF;
				segment_data[3] = LED_OFF; //off
				segment_data[4] = LED_OFF; //off
				return;
			case 3: //display digit0, digit1, and digit2
				segment_data[0] = sum % 10; //ones
				segment_data[1] = (sum / 10) % 10; //tens
				segment_data[2] = LED_OFF;
				segment_data[3] = (sum / 100) % 10; //hundreds
				segment_data[4] = LED_OFF; //off		
				return;
			case 4: //display digit0, digit1, digit2, and digit3
				segment_data[0] = sum % 10; //ones
				segment_data[1] = (sum / 10) % 10; //tens
				segment_data[2] = LED_OFF;
				segment_data[3] = (sum / 100) % 10; //hundreds
				segment_data[4] = (sum / 1000) % 10; //thousands
				return;
			default: //default to all digits off
				segment_data[0] = LED_OFF;//off
				segment_data[1] = LED_OFF;//off
				segment_data[2] = LED_OFF;//off
				segment_data[3] = LED_OFF;//off
				segment_data[4] = LED_OFF;//off
				return;
  		}
		break;
  	}

  return; //(logically unreachable)

}//segment_sum
//***********************************************************************************


//***********************************************************************************
int main()
{
	//set ports
	DDRB = 0xFF; //all outputs: 
	DDRE = 0xC8; //bit 3, 6, 7 output: E4 = gpo2/int (radio)
	DDRD = 0x30; //bit 4, 5, output: 
	DDRC = 0x82; //bit 1, 7 output: C1 = reset (radio)
	PORTC |= 0x02; //reset on at init 
	PORTE = 0xC0; 
	PORTD = 0x03; //enable pull up resistor pind 0,1
	
	//interrupt 4
	EIMSK |= (1<<4); //enable int4 external interrupt
	EICRB |= (1<<1) | (1<<0); //low to high change generates interrupt

	//initialize SPI (encoder / bargraph access)
	SPCR |= (1<<SPE) | (1<<MSTR) | (0<<SPIE); //enable SPI, set master mode, no interrupt 
	SPSR |= (1<<SPI2X);	//double speed

	//TWI
	init_twi();

	//USART
	uart_init();

	//initialize LCD (alarm display)
	lcd_init();

	//initialize music (initialize TCNT1 for alarm tone)
	music_init();
	song = 0;

	//initialize ADC (light sensor)
	ADMUX |= (1<<REFS1) | (1<<REFS0) | (1<<ADLAR) | (0<<MUX0); //internal 2.56 reference voltage, left justified, single ended input (PF0)
	ADCSRA |= (1<<ADEN) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); //enable adc, enable input capture interrupt, 128 prescale

	//initialize TCNT0 (real time)
	ASSR   |=  (1<<AS0);  //ext osc TOSC
	TIMSK |= (1<<TOIE0);	//enable overflow interrupt
	TCCR0 |= (0<<CS02) | (0<<CS01) | (1<<CS00); //normal mode, no prescale

	//initialize TCNT1 (alarm tone)
	//TCCR1B |= (0<<WGM13) | (1<<WGM12) | (0<<CS12) | (1<<CS11) | (1<CS10); //use ocr1a as source for top, prescale 64
	OCR1A = 0x0164;
	
	//initialize TCNT2 (dimming)
	TCCR2 |= (1<<WGM21) | (1<<WGM20) | (1<<COM21) | (1<<COM20) | (1<<CS20) | (1<<CS22); //fast pwm, set@match, clear@top, 1024 prescale
	TIMSK |= (1<<TOIE2);	//enable overflow interrupt
 	OCR2 = 0x00; //initial brightness set: 0x30 = 80%, 0xC0 = 20% brightness, 0xFF = 0%, 0x00 = 100%,

	//initialize TCNT3 (volume)
	TCCR3A |= (1<<COM3A1) | (1<<COM3A0) | (1<<WGM30) | (0<<WGM31); //fast pwm, set on match, clear at top
	TCCR3B |= (0<<WGM33) | (1<<WGM32) | (1<<CS32) | (0<<CS31) | (1<<CS30); //1024 prescale, freq = 61Hz
	OCR3A = 0x00FF; //0x0066 = 60%, 0x00CC = 20%, 0x00FF = 0%, 0x0000 = 100%

	uint8_t turns1 = 4; //right encoder
	uint8_t turns2 = 4; //left encoder
	uint8_t main_count = 0, bnew, aold; //main counter, encoder comparison temporary storages
	uint16_t lm73_temp; //storage for data read from lm73 temperature sensor
	uint16_t atmega48_temp; //storage for data read from atmega48
	uint16_t accel_x; //storage for x data read from accelerometer
	uint16_t accel_y; //storage for y data read from accelerometer
	uint16_t accel_z; //storage for z data read from accelerometer

	uint8_t i;
	for(i = 0; i < 32; i++){lcd_master_string_array[0][i] = lcd_initial_chars1[i]; } //load lcd initial string1
	for(i = 0; i < 32; i++){lcd_master_string_array[1][i] = lcd_initial_chars2[i]; } //load lcd initial string2
	for(i = 0; i < 32; i++){lcd_master_string_array[2][i] = lcd_initial_chars3[i]; } //load lcd initial string3

	sei();	//enable interrupts, allows twi wr to finish

	while(1)
 	{
		main_count++;
		if(firstrun_flag) //if this is the first run through main-while(1)
		{			//this branch will never be taken again
			//reset radio board
			PORTE &= ~(1<<PE4); //set GPO2/INT to zero
			DDRE |= (1<<PE4); //set GPO2/INT to output mode
			PORTC |= (1<<PC1); //set reset to 1
			_delay_us(200); //wait 30 us
			PORTC &= (1<<PC1); //set reset to 0
			_delay_us(30); //wait for 5v to 3.3v translators
			DDRE &= ~(1<<PE4); //set GPO2/INT to input mode
			//PORTE |= (1<<PE4); //activate pull up resistor
			while(twi_busy()){}
			fm_pwr_up();
			while(twi_busy()){}
			_delay_ms(110);
			current_fm_freq = 10*radio; //set current freq using radio value
			fm_tune_freq(); //tune radio station
			while(twi_busy()){}

			//initiate lm73 temperature sensor over twi
			lm73_wr_buf[0] = LM73_PTR_TEMP;   //load lm73_wr_buf[0] with temperature pointer address
			twi_start_wr(LM73_ADDRESS, lm73_wr_buf, 1);   //start the TWI write process (twi_start_wr())
			_delay_ms(2); //wait for it to finish

			/*
			//initiate accelerometer over twi
			//start with a reset
			accel_wr_buf[0] = ACCEL_PADDR_CTRL2; //load wr buffer with pointer address to ctrl2 reg
			accel_wr_buf[1] = ACCEL_SETTING_CTRL2_RESET;   //insert data to send to register for reset
			twi_start_wr(ACCEL_ADDRESS, accel_wr_buf, 2);   //start the TWI write process (2 bytes)
			_delay_ms(2); //wait for it to finish
			
			//set control registers
			accel_wr_buf[0] = ACCEL_PADDR_CTRL1; //load wr buffer with pointer address to ctrl1 reg
			accel_wr_buf[1] = ACCEL_SETTING_CTRL1_INIT; //insert data to send to register1 for init
			accel_wr_buf[2] = ACCEL_SETTING_CTRL2_INIT; //insert data to send to register2 for init
			twi_start_wr(ACCEL_ADDRESS, accel_wr_buf, 3); //start the TWI write process (3 bytes)
			_delay_ms(2); //wait for it to finish

			//set accel output scale
			accel_wr_buf[0] = ACCEL_PADDR_RNG; //load wr buffer with pointer address to smod reg
			accel_wr_buf[1] = ACCEL_SETTING_RNG_8G; //insert data to send to smod reg for wake mode
			twi_start_wr(ACCEL_ADDRESS, accel_wr_buf, 2); //start the twi write process (2 bytes)
			_delay_ms(2); //wait for it to finish

			//activate accelerometer
			accel_wr_buf[0] = ACCEL_PADDR_CTRL1; //load wr buffer with pointer address to ctrl1 reg
			accel_wr_buf[1] = ACCEL_SETTING_CTRL1_ACTIVATE; //insert data to send to register1 for activation
			twi_start_wr(ACCEL_ADDRESS, accel_wr_buf, 2); //start the TWI write process (2 bytes)
			_delay_ms(2); //wait for it to finish
			*/
		}

		if(data_collect_flag) //if request for temperature data is made
		{
			//Get temperature data through TWI (I2C)
			twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 2);  //read temperature data from LM73 (2 bytes)
			_delay_ms(2);    //wait for it to finish
			lm73_temp = lm73_rd_buf[0]; //save high temperature byte into lm73_temp
  			lm73_temp = lm73_temp << 8; //shift it into upper byte 
  			lm73_temp |= lm73_rd_buf[1]; //"OR" in the low temp byte to lm73_temp			
			if(lm73_temp_convert(temperature_local, lm73_temp, temperature_mode)) //convert to string, check if negative
			{
				lcd_master_string_array[1][7] = '-'; //insert negative sign
			}
			else
			{
				lcd_master_string_array[1][7] = ' '; //blank space
			}
			lcd_master_string_array[1][8] = temperature_local[0]; //100's place
			lcd_master_string_array[1][9] = temperature_local[1]; //10's place
			lcd_master_string_array[1][10] = temperature_local[2]; //1's place
			lcd_master_string_array[1][11] = temperature_local[3]; //decimal point
			lcd_master_string_array[1][12] = temperature_local[4]; //1/10's place
			lcd_master_string_array[1][13] = temperature_local[5]; //1/100's place

			/*
			//Get accelerometer data through TWI (I2C)
			accel_rd_buf[0] = ACCEL_PADDR_XREGH; //load wr buffer with pointer address for X register 1
			accel_twi_start_rd(ACCEL_ADDRESS, accel_rd_buf, 7); //start the twi write process
			_delay_ms(2); //wait for it to finish
			accel_x = accel_rd_buf[1]; //save high byte
  			accel_x = accel_x << 8; //shift it into upper byte 
  			accel_x |= accel_rd_buf[2]; //"OR" in the low byte
			accel_y = accel_rd_buf[3]; //save high byte
  			accel_y = accel_y << 8; //shift it into upper byte 
  			accel_y |= accel_rd_buf[4]; //"OR" in the low byte
			accel_z = accel_rd_buf[5]; //save high byte
  			accel_z = accel_z << 8; //shift it into upper byte 
  			accel_z |= accel_rd_buf[6]; //"OR" in the low byte

			if(convert_accel(accel_char_x, accel_x)) //convert to string, check if negative
			{
				lcd_master_string_array[2][17] = '-'; //insert negative space
			}
			else
			{
				lcd_master_string_array[2][17] = ' '; //insert blank space
			}
			lcd_master_string_array [2][18] = accel_char_x[0]; //10's place
			lcd_master_string_array [2][19] = accel_char_x[1]; //1's place

			if(convert_accel(accel_char_y, accel_y)) //convert to string, check if negative
			{
				lcd_master_string_array[2][22] = '-'; //insert negative space
			}
			else
			{
				lcd_master_string_array[2][22] = ' '; //insert blank space
			}
			lcd_master_string_array [2][23] = accel_char_y[0]; //10's place
			lcd_master_string_array [2][24] = accel_char_y[1]; //1's place
			
			if(convert_accel(accel_char_z, accel_z)) //convert to string, check if negative
			{
				lcd_master_string_array[2][27] = '-'; //insert negative space
			}
			else
			{
				lcd_master_string_array[2][27] = ' '; //insert blank space
			}
			lcd_master_string_array [2][28] = accel_char_z[0]; //10's place
			lcd_master_string_array [2][29] = accel_char_z[1]; //1's place

			//Send request for data via USART
			
			uart_putc('1');
			atmega48_temp = uart_getc();
			if(atmega48_temp != 0)
			{
				atmega48_temp = ((atmega48_temp) | ((uart_getc()) << 8));
				atmega48_temp += (45 << 6);
				if(lm73_temp_convert(temperature_remote, atmega48_temp, temperature_mode)) //convert, check negative
				{
					lcd_master_string_array[1][23] = ' '; //insert negative sign
				}
				else
				{
					lcd_master_string_array[1][23] = ' '; //blank space
				}
				lcd_master_string_array[1][24] = temperature_remote[0]; //100's place
				lcd_master_string_array[1][25] = temperature_remote[1]; //10's place
				lcd_master_string_array[1][26] = temperature_remote[2]; //1's place
				lcd_master_string_array[1][27] = temperature_remote[3]; //decimal point
				lcd_master_string_array[1][28] = temperature_remote[4]; //1/10's place
				lcd_master_string_array[1][29] = temperature_remote[5]; //1/100's place
				
			}
			*/
			
			//if(temperature_mode){ uart_putc('f'); } //send cmd to atmega48 to request temp (fahrenheit)
			//else{ uart_putc('c'); } //send cmd to atmega48 to request temp (celsius)
			/*
			if(uart_getc() == 'k')
			{
				lcd_master_string_array[1][23] = uart_getc();
				lcd_master_string_array[1][24] = uart_getc();
				lcd_master_string_array[1][25] = uart_getc();
				lcd_master_string_array[1][26] = uart_getc();
				lcd_master_string_array[1][27] = uart_getc();
				lcd_master_string_array[1][28] = uart_getc();
				lcd_master_string_array[1][29] = uart_getc();
			}*/

			//Update LCD
			clear_display(); //reset lcd display
			_delay_ms(2);
			set_cursor(1, 0); //set lcd cursor to row 1 column 1
			_delay_us(40); //required delay for cursor movement
			for(i=0;i<15;i++){ lcd_str[i] = lcd_master_string_array[lcd_display_mode][i]; }
			string2lcd(lcd_str); //send first 16 characters to lcd
			set_cursor(2, 0);; //set lcd cursor to row 2 column 1
			_delay_us(40); //required delay for cursor movement
			for(i=0;i<15;i++){ lcd_str[i] = lcd_master_string_array[lcd_display_mode][i + 16]; }
			string2lcd(lcd_str); //send second 16 characters to lcd	
			
			//reset flag
			data_collect_flag = 0;			
		}
		if(main_count % 100 == 0)
		{
			//Radio
			if(radio_cnt_last != radio_cnt)
			{
				current_fm_freq = 10*radio; //set current freq using radio value
				fm_tune_freq(); //tune radio station
				//get_int_status();
				radio_cnt_last = radio_cnt; //store new station as last
			}

			//*********  Encoders & Bargraph *********//
			//Initialize
			 PORTD &= ~(1<<PD5);//enable bar graph
 			 PORTE &= ~((1<<PE6) | (1<<PE7)); //enable encoder, enable parallel load
			 PORTE |= (1<<PE7); //disable parallel load	

  			//Check Encoders
 			SPDR = mode; //send mode to spi data register
  			while(bit_is_clear(SPSR, SPIF)){}//wait for transmission complete
		
			//update bar graph
			PORTD |=  (1<<PD4); //rising edge - creating clock signal
  			PORTD &=  ~(1<<PD4); //falling edge

			PORTD |= (1<<PD5); //disable bar graph
			PORTE |= (1<<PE6); //disable encoder

	  		//parse encoder data and check
	 		encoder_raw = SPDR; //read encoder data from SPI data register
			encoder1_current = encoder_raw & 0b00000011; //lowest two bits are from encoder1
			encoder2_current = encoder_raw & 0b00001100; //next two lowest bits are from encoder2
			encoder2_current = encoder2_current >> 2; //shift encoder2 to match encoder1 (so conditions can be reused)

			//*****  Push-Button Switches  *****//
  			//Initialize Push-Button Switches
  			//make PORTA an input port with pullups 
			DDRA = 0x00; //0 = input mode
			PORTA = 0xFF; //1 = pullup mode

		 	 //enable tristate buffer for pushbutton switches
			PORTB = 0x5F;
			_delay_us(10);
		
		  	//Check Push-Button Switches
			int i;
			for(i = 0; i < 8; i++)
			{
				if(chk_buttons(i)) //if switch i is pressed
				{
					activity_count = 0;
					switch(i)
					{
						case CLOCK_SET: //button for setting clock time
							mode &= ~(1<<ALARM_SET); //clear other setting mode buttons
							mode ^= (1<<CLOCK_SET); //toggle 0
							if(mode & (1<<CLOCK_SET)){ display_mode = DISPLAY_CLOCKSET;} //if clock was set to 1
							else{ display_mode = DISPLAY_TIME;}
							break;
						case ALARM_SET: //button for setting alarm time
							mode &= ~(1<<CLOCK_SET); //clear other setting mode buttons
							mode ^= (1<<ALARM_SET); //toggle 1
							if(mode & (1<<ALARM_SET)){ display_mode = DISPLAY_ALARMSET;} //if clock was set to 1
							else{ display_mode = DISPLAY_TIME;}
							break;
						case ALARM_ONOFF: //button for setting alarm mode {on, off}
							if(alarm_flag) //if alarm is going off
							{
								alarm_flag = 0; //clear alarm that is going off
								music_off();
							}
							mode ^= 1 << ALARM_ONOFF; //toggle alarm on/off
							break;
						case TEMPERATURE_MODE: //change temperature setting between c / f

							if(DEBUG) //if debug mode set
							{
								//	
							}
							alarm_flag ^= 1; //DEBUG: toggle alarm trigger 
							if(alarm_flag){ music_on(); } //DEBUG: if turned on, turn music on
							temperature_mode ^= 1; //toggle temperature mode flag
							if(temperature_mode)
							{
								lcd_master_string_array[1][4] = 'F';
								lcd_master_string_array[1][20] = 'F';
							}
							else
							{
								lcd_master_string_array[1][4] = 'C';
								lcd_master_string_array[1][20] = 'C';
							}
							break;
						case ALARM_MODE: //Sets what the alarm will trigger
							alarm_mode++;
							if(alarm_mode > 5){ alarm_mode = 0; }
							switch(alarm_mode)
							{
								case 0:
									lcd_master_string_array[0][7] = 'B';
									lcd_master_string_array[0][8] = 'u';
									lcd_master_string_array[0][9] = 'z';
									lcd_master_string_array[0][10] = 'z';
									lcd_master_string_array[0][11] = ' ';
									OCR1A = 0x0164;
									break;
								case 1:
									lcd_master_string_array[0][7] = 'R';
									lcd_master_string_array[0][8] = 'a';
									lcd_master_string_array[0][9] = 'd';
									lcd_master_string_array[0][10] = 'i';
									lcd_master_string_array[0][11] = 'o';
									break;
								case 2:
									lcd_master_string_array[0][7] = 'S';
									lcd_master_string_array[0][8] = 'o';
									lcd_master_string_array[0][9] = 'n';
									lcd_master_string_array[0][10] = 'g';
									lcd_master_string_array[0][11] = '0';
									song = 0;
									notes = 0;
									break;
								case 3:
									lcd_master_string_array[0][7] = 'S';
									lcd_master_string_array[0][8] = 'o';
									lcd_master_string_array[0][9] = 'n';
									lcd_master_string_array[0][10] = 'g';
									lcd_master_string_array[0][11] = '1';
									song = 1;
									notes = 0;
									break;
								case 4:
									lcd_master_string_array[0][7] = 'S';
									lcd_master_string_array[0][8] = 'o';
									lcd_master_string_array[0][9] = 'n';
									lcd_master_string_array[0][10] = 'g';
									lcd_master_string_array[0][11] = '2';
									song = 2;
									notes = 0;
									break;
								case 5:
									lcd_master_string_array[0][7] = 'S';
									lcd_master_string_array[0][8] = 'o';
									lcd_master_string_array[0][9] = 'n';
									lcd_master_string_array[0][10] = 'g';
									lcd_master_string_array[0][11] = '3';
									song = 3;
									notes = 0;
									break;
								default: break;
							}
							break;
						case RADIO_ONOFF://button for turning the radio on/off
							mode ^= 1 << RADIO_ONOFF; //toggle radio on/off
							break;
						case HOUR_MODE: //button for switching between 12/24 hour modes
							mode ^= 1 << HOUR_MODE; //toggle 12/24 hr mode
							break;
						case SNOOZE_ONOFF: //button for turning snooze on / off
							if(alarm_flag) //if alarm is going off
							{
								mode |= (1<<SNOOZE_ONOFF); //set led on bar graph
								snooze_flag = 1; //set snooze flag
								alarm_flag = 0; //clear alarm from going off
								music_off();
							}
							else
							{ 
								snooze_flag = 0; //clear snooze flag
								mode &= ~(1<<SNOOZE_ONOFF); //clear led on bar graph 
							}
							break;
						default:
							//mode ^= 1 << i; //toggle button at position i
							break;
					}
				} 
			}
			control = mode & CONTROL_BITS; //determines what the encoders currently change
		
		 	 //disable tri-state buffer
			PORTB = 0xFF;

		}

		//***** Send Count to 7-Seg Board *****//
		//break up the disp_value to 4, BCD digits in the array: call (segsum)
  		switch(display_mode)
		{
			case DISPLAY_TIME:
			case DISPLAY_CLOCKSET:
				segsum(clock_time);
				break;
			case DISPLAY_VOLUME:
				segsum(volume);
				break;
			case DISPLAY_ALARMSET: 
				segsum(alarm_time);
				break;
			case DISPLAY_RADIO: 
				segsum(radio);
				break;
			default:
				break;
		}
		
		//make PORTA an output
  		DDRA = 0xFF; //1 = output mode

		PORTA = LED_OFF; //clear LED to prevent ghosting

		//update number of digit to display
		display_digit++;

		//bound a counter (0-4) to keep track of digit to display 
 		if(display_digit > 4){display_digit = 0;}

		//send PORTB the number of the digit to display
		PORTB = display_digit << 4; //shift value of display into the correct position
	
 		//send 7 segment code to LED segments
		if(display_mode == DISPLAY_RADIO && display_digit == 1)
		{
			PORTA = dec_to_sevseg(segment_data[display_digit], 1); //convert decimal to seven seg binary code
		}
		else
		{
 			PORTA = dec_to_sevseg(segment_data[display_digit], 0); //convert decimal to seven seg binary code
		}

		//************************************//

		//update timers: snooze, activity, real time
		if(snooze_count >= SNOOZE_TIME) //snooze for duration
		{
			snooze_count = 0; //reset counter
			snooze_flag = 0; //clear snooze flag
			mode &= ~(1<<SNOOZE_ONOFF); //clear snooze led
			alarm_flag = 1; //turn alarm on again
			music_on();
		}

		if(activity_count >= USER_ACTIVITY_TIMER)//no user input for 5 seconds
		{
			activity_count = 0; //reset counter;
			mode &= ~((1<<CLOCK_SET) | (1<<ALARM_SET)); //reset to startup controls
			display_mode = DISPLAY_TIME;
		}
		if(clock_seconds >= 60)
		{
			clock_seconds -= 60; //reset counter
			clock_minutes++; //increment clock seconds counter
			if(clock_minutes >= 60)
			{
				clock_minutes -= 60; //reset counter
				clock_hours++; //increment clock hours counter
				if(clock_hours >= 24)
				{
					clock_hours -= 24; //reset counter
				}
			}
			if((clock_minutes == alarm_minutes) && (clock_hours == alarm_hours) && (mode & (1<<ALARM_ONOFF)))
			{ //if alarm is set and real time matches alarm time
				alarm_flag = 1;//set alarm
				if(volume == 0){ volume = 1; } //turn on volume if its off (want the alarm to go off even if forgot to set vol)
				switch(alarm_mode)
				{
					case 0: //nothing needs to be done
						break; 
					case 1: //do radio stuff
						break;
					case 2: //play song0
					case 3: //play song1
					case 4: //play song2
					case 5: //play song3
						music_on();
						break;
					default: //nothing needs to be done
						break;
				}
			}
		}

		//convert time values to single int
		if( !(mode & (1<<HOUR_MODE)) ) //12 hr mode	
		{
			if(clock_hours < 12){ clock_time = ((clock_hours + 1) * 100) + clock_minutes; }
			if(clock_hours >= 12){ clock_time = ((clock_hours - 11) * 100) + clock_minutes; }
			if(alarm_hours < 12){ alarm_time = ((alarm_hours + 1) * 100) + alarm_minutes; }
			if(alarm_hours >= 12){ alarm_time = ((alarm_hours - 11) * 100) + alarm_minutes; }
		}
		else//army time, 24 hr mode
		{
			clock_time = (clock_hours * 100) + clock_minutes; 
			alarm_time = (alarm_hours * 100) + alarm_minutes; 
		} 
		
		//check encoder input
		if(!firstrun_flag) //if this is NOT the first time the encoders were checked
		{
			
			if(encoder1_current != encoder1_last)
			{
				activity_count = 0; //reset counter to indicate that user recently gave input
				bnew = encoder1_current & 0b00000001; //B bit of the current state
				aold = encoder1_last & 0b00000010; //A bit of the last state
				switch(bnew ^ aold) //determine direction by XOR'ing B of current with A of last
				{
					case 0:
						turns1++; //electrical tick to the right
						break;
					case 1: 
						turns1--; //electrical tick to the left
						break; 
					default:
						break;
				}
			}
			if(encoder2_current != encoder2_last)
			{
				activity_count = 0; //reset counter to indicate that user recently gave input
				bnew = encoder2_current & (1<<0); //B bit of the current state
				aold = encoder2_last & (1<<1); //A bit of the last state
				switch(bnew ^ aold) //determine direction by XOR'ing B of current with A of last
				{
					case 0:
						turns2++; //electrical tick to the right
						break;
					case 1: 
						turns2--; //electrical tick to the left
						break; 
					default:
						break;
				}
			}
			if(turns1 == 2) //sufficient number of electrical ticks for a left turn
			{	
					switch(control)
					{
						case 1: //clock set mode
							if(clock_minutes > 0){ clock_minutes--; }
							else
							{
								clock_minutes = 59;
								if(clock_hours > 0){ clock_hours--; }
								else{ clock_hours = 23; }
							}
							break;
						case 0: //volume mode
							display_mode = DISPLAY_VOLUME;
							if(volume > 1)
							{
								volume--; 
								OCR3A = 232 - (volume * 13); //set pwm based on volume setting
							}
							else
							{ 
								volume = 0;
								OCR3A = 255; //0 volts
							}
							break;
						case 2: //alarm set mode
							if(alarm_minutes > 0){ alarm_minutes--; }
							else
							{
								alarm_minutes = 59;
								if(alarm_hours > 0){ alarm_hours--; }
								else{ alarm_hours = 23; }
							}
							break;
						default:
							break;
					}
					turns1 = 4; //reset turn count
			}		 	
			if(turns1 == 6) //sufficient number of electrical ticks for a right turn
			{					
					switch(control)
					{
						case 1: //clock set mode
							if(clock_minutes < 59){ clock_minutes++; }
							else
							{
								clock_minutes = 0;
								if(clock_hours < 23){ clock_hours++; }
								else{ clock_hours = 0; }
							}
							break;
						case 0: //volume mode
							display_mode = DISPLAY_VOLUME;
							if(volume < 10){ volume++; }
							else{ volume = 10; }
							OCR3A = 232 - (volume * 13); //set pwm based on volume setting
							break;
						case 2: //alarm set mode
							if(alarm_minutes < 59){ alarm_minutes++; }
							else
							{
								alarm_minutes = 0;
								if(alarm_hours < 23){ alarm_hours++; }
								else{ alarm_hours = 0; }
							}
							break;
						default:
							break;
					}
					turns1 = 4; //reset turn count		
			}
			if(turns2 == 2) //sufficient number of electrical ticks for a left turn
			{
					switch(control)
					{
						case 1: //clock set mode
							if(clock_hours > 0){ clock_hours--; }
							else{ clock_hours = 23; }
							break;
						case 0: //radio mode
							display_mode = DISPLAY_RADIO;
							radio_cnt--;
							if(radio_cnt < 1){ radio_cnt = 100; }
							radio = 881 + 2 * (radio_cnt - 1);
							break;
						case 2: //alarm set mode
							if(alarm_hours > 0){ alarm_hours--; }
							else{ alarm_hours = 23; }
							break;
						default:
							break;
					} 
					turns2 = 4; //reset turn count	
			}
			if(turns2 == 6) //sufficient number of electrical ticks for a right turn
			{
					switch(control)
					{
						case 1: //clock set mode
							if(clock_hours < 23){ clock_hours++; }
							else{ clock_hours = 0; }
							break;
						case 0: //volume/radio mode
							display_mode = DISPLAY_RADIO;
							radio_cnt++;
							if(radio_cnt > 100){ radio_cnt = 1; }
							radio = 881 + 2 * (radio_cnt - 1);
							break;
						case 2: //alarm set mode
							if(alarm_hours < 23){ alarm_hours++; }
							else{ alarm_hours = 0; }
							break;

						default:
							break;	
					}
					turns2 = 4; //reset turn count
			}
		}
		else //first run through main (skips comparison of encoder_current to encoder_last)
		{ 
			firstrun_flag = 0; //clear first run flag, this branch will never be taken again
		}
	
		encoder1_last = encoder1_current; //store state to check against later	
		encoder2_last = encoder2_current; //store state to check against later

	}//while(1)
}//main

//*********************************************//
//	Interrupt Routine = ADC Capture        //
//*********************************************//
ISR(ADC_vect)
{
	OCR2 = ADCH; //set pwm based on cds cell voltage read through adc
}
//*********************************************//
//	Interrupt Routine = Timer 0 Overflow   //
//*********************************************//
ISR(TIMER0_OVF_vect)
{
	static uint16_t count_7ms = 0;        //holds 7ms tick count in binary
  	count_7ms++;                //increment count every 7.8125 ms
	if( (count_7ms % 8 == 0) && (alarm_flag) )
	{
    		//for note duration (64th notes) 
		beat++;
	}
	if((count_7ms % 64) < 8) 
	{
		blink_flag = 0; 
	} 
	if((count_7ms % 64) > 16)
	{
		blink_flag = 1;
	}
  	if(count_7ms % 128 == 0) //equals one second 
	{
		colon_blink_flag ^= 1;
		data_collect_flag = 1;
		clock_seconds++; //increment clock seconds counter
		activity_count++; //increment activity counter
		if(snooze_flag){ snooze_count++; } //if snooze is on, increment snooze counter
	}
	if( count_7ms % (128 * LCD_CYCLE_SECONDS) == 0 ) //equals two seconds
	{
		if(DEBUG) //if debug mode set
		{
			lcd_display_mode = 2; //FOR TESTING ACCELEROMETER
		}
		else
		{
			lcd_display_mode++; //cycle display mode
			if(lcd_display_mode > (LCD_DISPLAY_PAGES - 1)){ lcd_display_mode = 0; } //prevent lcd display mode overflow
		}
	}
}
/*********************************************************************/
/*                             TIMER1_COMPA                          */
/*Oscillates pin7, PORTD for alarm tone output                       */
/*********************************************************************/
#define ALARM_PIN 7 //PORTC, pin 7
ISR(TIMER1_COMPA_vect) {
	switch(alarm_mode)
	{
		case 0: //play buzzer
			if( (alarm_flag) && (volume != 0) && colon_blink_flag ){PORTC ^= (1<<ALARM_PIN);} //flip bit
			break;
		case 1: //play radio
			break;
		case 2: //play song0
		case 3: //play song1
		case 4: //play song2
		case 5: //play song3
			if( (alarm_flag) && (volume != 0) ){PORTC ^= (1<<ALARM_PIN);} //flips bit, creating a tone
			if(beat >= max_beat)   //if we've played the note long enough
			{
				notes++;               //move on to the next note
				play_song(song, notes);//and play it
			}
		default: 
			break;
	}
}
//*********************************************//
//	Interrupt Routine = Timer 2 Overflow   //
//*********************************************//
ISR(TIMER2_OVF_vect)
{
	ADCSRA |= (1<<ADSC); //start adc conversion
}

//*********************************************//
//	Interrupt Routine = INT4               //
//*********************************************//
ISR(INT4_vect) //radio GPO2/INT
{
	STC_interrupt = TRUE;
}

//*********************************************//
//	Interrupt Routine = USART Rx complete  //
//*********************************************//
/*
ISR(USART0_RX_vect)
{
	static uint8_t slot_no = 23;
	
	if(UDR0 == 'k') //if ack receieved
	{
		slot_no = 23; //reset index
	}
	else //
	{
		lcd_master_string_array[1][slot_no] = UDR0;
		slot_no++;
		if(slot_no > 29){ slot_no = 23; }
	}
	
}
*/

