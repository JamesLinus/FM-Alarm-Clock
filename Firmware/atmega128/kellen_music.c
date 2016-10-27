/*********************************************************************/
/*                   Music functions for ATMEGA128                   */
/*                        Kellen Arb 10.18.08                        */
/* This file should include everything you need to set up and use    */
/*songs for your ECE473 alarm clock.  Each function has a description*/
/*so you know how to use it, but all you should need to do is:       */
/*  0)Add the following to your Timer0 overflow interrupt (assuming  */
/*      interrupt is 128 times/second):
  ms++;
  if(ms % 8 == 0) {
    //for note duration (64th notes) 
    beat++;
  }                                                                  */
/*      ms can be changed for any counter variable, if you have one  */
/*      already. beat, however, must stay.                           */
/*  1)Change the #define values below for mute, unmute, and ALARM_PIN*/
/*      to the values needed for your setup.  If you use a different */
/*      port, as well as different pins, you'll have to manually     */
/*      change them throughout this file.                            */
/*  2)In your main function, call music_init().  Check to make sure  */
/*      there aren't any conflicts with the values it sets.          */
/*  4)Set the value of "song" to your liking. You can make this user */
/*      selectable very easily.                                      */
/*  5)Anytime you set off your alarm, add a call to music_on(). This */
/*      will start the interrupt and the song playing.               */
/*  4)Anytime you turn off your alarm, add a call to music_off().    */
/*      this stops the song playing and halts the interrupt.         */
/*                                                                   */
/* That should be all you need!  If you want to create new songs,    */
/*just read the descriptions for play_note and play_rest and copy the*/
/*format of any of the pre-done songs.  Make sure to share any good  */
/*ones with the class! Have fun!                                     */
/*             -Kellen Arb                                           */
/*********************************************************************/
#include <avr/io.h>
#define F_CPU 16000000UL //16Mhz clock
#include <string.h>

//Mute is on PORTD
//set the hex values to set and unset the mute pin
//I used PORTD-PIN2
#define mute 0x04
#define unmute 0xFB
//Alarm is also on PORTD
//set the hex value for the alarm pin
//I used PORTD-PIN7
#define ALARM_PIN 7
#define NUM_SONGS 4
//set this variable to select the song
//(0-3, unless you add more)
volatile uint8_t song;

//function prototypes defined here
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

#define C0 0x1DDC
#define Db0 0x1C30
#define D0 0x1A9A
#define Eb0 0x1919
#define E0 0x17B2
#define F0 0x165D
#define Gb0 0x151D
#define G0 0x13ED
#define Ab0 0x12CE
#define A0 0x11C0
#define Bb0 0x10C0
#define B0 0x0FD0
#define C1 0x0EED
#define Db1 0x0E16
#define D1 0x0D4C
#define Eb1 0x0C8D
#define E1 0x0BD8
#define F1 0x0B2E
#define Gb1 0x0A8D
#define G1 0x09F6
#define Ab1 0x0967
#define A1 0x08DF
#define Bb1 0x0860
#define B1 0x07E7
#define C2 0x0776
#define Db2 0x070A
#define D2 0x06A5
#define Eb2 0x0646
#define E2 0x05EB
#define F2 0x0596
#define Gb2 0x0546
#define G2 0x04FA
#define Ab2 0x04B2
#define A2 0x046F
#define Bb2 0x042F
#define B2 0x03F3
#define C3 0x03BA
#define Db3 0x0384
#define D3 0x0352
#define Eb3 0x0322
#define E3 0x02F5
#define F3 0x02CA
#define Gb3 0x02A2
#define G3 0x027C
#define Ab3 0x0258
#define A3 0x0237
#define Bb3 0x0217
#define B3 0x01F9
#define C4 0x01DC
#define Db4 0x01C1
#define D4 0x01A8
#define Eb4 0x0190
#define E4 0x017A
#define F4 0x0164
#define Gb4 0x0150
#define G4 0x013D
#define Ab4 0x012B
#define A4 0x011B
#define Bb4 0x010B
#define B4 0x00FC
#define C5 0x00ED
#define Db5 0x00E0
#define D5 0x00D3
#define Eb5 0x00C7
#define E5 0x00BC
#define F5 0x00B1
#define Gb5 0x00A7
#define G5 0x009E
#define Ab5 0x0095
#define A5 0x008D
#define Bb5 0x0085
#define B5 0x007D
#define C6 0x0076
#define Db6 0x006F
#define D6 0x0069
#define Eb6 0x0063
#define E6 0x005D
#define F6 0x0058
#define Gb6 0x0053
#define G6 0x004E
#define Ab6 0x004A
#define A6 0x0046
#define Bb6 0x0042
#define B6 0x003E
#define C7 0x003A
#define Db7 0x0037
#define D7 0x0034
#define Eb7 0x0031
#define E7 0x002E
#define F7 0x002B
#define Gb7 0x0029
#define G7 0x0026
#define Ab7 0x0024
#define A7 0x0022
#define Bb7 0x0020
#define B7 0x001E
#define C8 0x001C
#define Db8 0x001B
#define D8 0x0019
#define Eb8 0x0018
#define E8 0x0015
#define F8 0x0012
#define Gb8 0x0010
#define G8 0x000D
#define Ab8 0x000B
#define A8 0x0009
#define Bb8 0x0007
#define B8 0x0005

volatile uint16_t beat;
volatile uint16_t max_beat;
volatile uint8_t  notes;

void song0(uint16_t note) { //beaver fight song (Max and Kellen)
  switch (note) {
    case 0: play_note('F', 0, 4, 8);
       break;
    case 1: play_note('E', 0, 4, 8);
       break;
    case 2: play_note('D', 0, 4, 8);
       break;
    case 3: play_note('C', 0, 4, 8);
       break;
    case 4: play_note('A', 0, 4, 6);
       break;
    case 5: play_note('A', 1, 4, 2);
       break;
    case 6: play_note('A', 0, 4, 6);
       break;
    case 7: play_note('A', 1, 4, 2);
       break;
    case 8: play_note('A', 0, 4, 16);
       break;
    case 9: play_note('F', 0, 4, 8);
       break;
    case 10: play_note('E', 0, 4, 8);
       break;
    case 11: play_note('D', 0, 4, 8);
       break;
    case 12: play_note('C', 0, 4, 8);
       break;
    case 13: play_note('B', 1, 4, 6);
       break;
    case 14: play_note('A', 0, 4, 2);
       break;
    case 15: play_note('B', 1, 4, 6);
       break;
    case 16: play_note('A', 0, 4, 2);
       break;
    case 17: play_note('B', 1, 4, 16);
       break;
    case 18: play_note('G', 0, 4, 3);
       break;
    case 19: play_rest(1); //rest
       break;
    case 20: play_note('G', 0, 4, 7);
       break;
    case 21: play_rest(1); //rest
       break;
    case 22: play_note('G', 1, 4, 4);
       break;
    case 23: play_note('G', 0, 4, 6);
       break;
    case 24: play_note('A', 0, 4, 2);
       break;
    case 25: play_note('B', 1, 4, 8);
       break;
    case 26: play_note('A', 0, 4, 2);
       break;
    case 27: play_rest(2); 
       break;
    case 28: play_note('A', 0, 4, 8);
       break;
    case 29: play_note('A', 1, 4, 4);
       break;
    case 30: play_note('A', 0, 4, 6);
       break;
    case 31: play_note('B', 1, 4, 2);
       break;
    case 32: play_note('C', 0, 5, 4);
       break;
    case 33: play_note('D', 1, 5, 4);
       break;
    case 34: play_note('D', 0, 5, 4);
       break;
    case 35: play_note('B', 0, 4, 8);
       break;
    case 36: play_note('A', 0, 4, 4);
       break;
    case 37: play_note('G', 0, 4, 8);
       break;
    case 38: play_note('A', 0, 4, 8);
       break;
    case 39: play_note('G', 0, 4, 24);
       break;
    case 40: play_rest(8);
       break;
    case 41: play_note('F', 0, 4, 8);
       break;
    case 42: play_note('E', 0, 4, 8);
       break;
    case 43: play_note('D', 0, 4, 8);
       break;
    case 44: play_note('C', 0, 4, 8);
       break;
    case 45: play_note('A', 0, 4, 6);
       break;
    case 46: play_note('A', 1, 4, 2);
       break;
    case 47: play_note('A', 0, 4, 6);
       break;
    case 48: play_note('A', 1, 4, 2);
       break;
    case 49: play_note('A', 0, 4, 16);
       break;
    case 50: play_note('F', 0, 4, 8);
       break;
    case 51: play_note('G', 1, 4, 8);
       break;
    case 52: play_note('G', 0, 4, 8);
       break;
    case 53: play_note('D', 0, 4, 8);
       break;
    case 54: play_note('B', 1, 4, 6);
       break;
    case 55: play_note('A', 0, 4, 2);
       break;
    case 56: play_note('B', 1, 4, 6);
       break;
    case 57: play_note('A', 0, 4, 2);
       break;
    case 58: play_note('B', 1, 4, 16);
      break;//phrase
    case 59: play_note('D', 0, 4, 16);
       break;
    case 60: play_note('D', 0, 5, 16);
       break;
    case 61: play_note('A', 0, 4, 16);
       break;
    case 62: play_note('C', 0, 5, 16);
       break;
    case 63: play_note('B', 1, 4, 8);
       break;
    case 64: play_note('C', 0, 5, 4);
       break;
    case 65: play_note('D', 0, 5, 4);
       break;
    case 66: play_note('A', 0, 4, 8);
       break;
    case 67: play_note('G', 0, 4, 8);
       break;
    case 68: play_note('F', 0, 4, 24);
       break;
    case 69: play_rest(8);
       break;
    default: notes=-1;
  }
}//song0

void song1(uint16_t note) { //tetris theme (Kellen)
switch (note) {
    case 0: play_note('E', 0, 4, 8);
       break;
    case 1: play_note('B', 0, 3, 4);
       break;
    case 2: play_note('C', 0, 4, 4);
       break;
    case 3: play_note('D', 0, 4, 4);
       break;
    case 4: play_note('E', 0, 4, 2);
       break;
    case 5: play_note('D', 0, 4, 2);
       break;
    case 6: play_note('C', 0, 4, 4);
       break;
    case 7: play_note('B', 0, 3, 4);
       break;//bar
    case 8: play_note('A', 0, 3, 7);
       break;
    case 9: play_rest(1);
       break;
    case 10: play_note('A', 0, 3, 4);
       break;
    case 11: play_note('C', 0, 4, 4);
       break;
    case 12: play_note('E', 0, 4, 8);
       break;
    case 13: play_note('D', 0, 4, 4);
       break;
    case 14: play_note('C', 0, 4, 4);
       break;//bar
    case 15: play_note('B', 0, 3, 12);
       break;
    case 16: play_note('C', 0, 4, 4);
       break;
    case 17: play_note('D', 0, 4, 8);
       break;
    case 18: play_note('E', 0, 4, 8);
       break;//bar
    case 19: play_note('C', 0, 4, 8);
       break;
    case 20: play_note('A', 0, 3, 7);
       break;
    case 21: play_rest(1);
       break;
    case 22: play_note('A', 0, 3, 16);
       break;//bar phrase
    case 23: play_rest(4);
       break;
    case 24: play_note('D', 0, 4, 8);
        break;
    case 25: play_note('F', 0, 4, 4);
       break;
    case 26: play_note('A', 0, 4, 8);
       break;
    case 27: play_note('G', 0, 4, 4);
       break;
    case 28: play_note('F', 0, 4, 4);
       break;//bar
    case 29: play_note('E', 0, 4, 12);
        break;
    case 30: play_note('C', 0, 4, 4);
       break;
    case 31: play_note('E', 0, 4, 8);
        break;
    case 32: play_note('D', 0, 4, 4);
       break;
    case 33: play_note('C', 0, 4, 4);
       break;//bar
    case 34: play_note('B', 0, 3, 7);
       break;
    case 35: play_rest(1);
        break;
    case 36: play_note('B', 0, 3, 4);
       break;
    case 37: play_note('C', 0, 4, 4);
       break;
    case 38: play_note('D', 0, 4, 8);
       break;
    case 39: play_note('E', 0, 4, 8);
       break;
    case 40: play_note('C', 0, 4, 8);
       break;
    case 41: play_note('A', 0, 3, 7);
       break;
    case 42: play_rest(1);
       break;
    case 43: play_note('A', 0, 3, 8);
       break;
    case 44: play_rest(8);
       break;//bar phrase
    case 45: play_note('E', 0, 3, 16);
       break;
    case 46: play_note('C', 0, 3, 16);
       break;//bar
    case 47: play_note('D', 0, 3, 16);
       break;
    case 48: play_note('B', 0, 2, 16);
       break;//bar
    case 49: play_note('C', 0, 3, 16);
       break;
    case 50: play_note('A', 0, 2, 16);
       break;//bar
    case 51: play_note('A', 1, 2, 16);
       break;
    case 52: play_note('B', 0, 2, 8);
       break;
    case 53: play_rest(8);
       break;//bar phrase
    case 54: play_note('E', 0, 3, 16);
       break;
    case 55: play_note('C', 0, 3, 16);
       break;//bar
    case 56: play_note('D', 0, 3, 16);
       break;
    case 57: play_note('B', 0, 2, 16);
       break;//bar
    case 58: play_note('C', 0, 3, 8);
       break;
    case 59: play_note('E', 0, 3, 8);
       break;
    case 60: play_note('A', 0, 3, 16);
       break;//bar
    case 61: play_note('A', 1, 3, 16);
       break;
    case 62: play_rest(16);
      break;//bar phrase
    default: notes=-1;

 }
}//song1

void song2(uint16_t note) { //Super Mario Bros Theme (Brian)
switch (note) {
    case 0: play_note('E', 0, 4, 1);
       break;
    case 1: play_rest(1);
       break;
    case 2: play_note('E', 0, 4, 3);
       break;
    case 3: play_rest(1);
       break;
    case 4: play_note('E', 0, 4, 2);
       break;
    case 5: play_rest(2);
       break;
    case 6: play_note('C', 0, 4, 2);
       break;
    case 7: play_note('E', 0, 4, 4);
       break;
    case 8: play_note('G', 0, 4, 8);
       break;
    case 9: play_note('G', 0, 2, 8);
       break;
    case 10: play_rest(8);
       break;
    case 11: play_note('C', 0, 4, 5);
       break;
    case 12: play_note('G', 0, 3, 2);
       break;
    case 13: play_rest(4);
       break;
    case 14: play_note('E', 0, 3, 4);
       break;
    case 15: play_rest(2);
       break;
    case 16: play_note('A', 0, 3, 2);
       break;
    case 17: play_rest(2);
         break;
    case 18: play_note('B', 0, 3, 2);
       break;
    case 19: play_rest(2);
       break;
    case 20: play_note('B', 1, 3, 2);
       break;
    case 21: play_note('A', 0, 3, 4);
       break;
    case 22: play_note('G' , 0, 3, 3);
       break;
    case 23: play_note('E', 0, 4, 2);
       break;
    case 24: play_rest(1);
        break;
    case 25: play_note('G', 0, 4, 2);
       break;
    case 26: play_note('A', 0, 4, 4);
       break;
    case 27: play_note('F', 0, 4, 2);
       break;
    case 28: play_note('G', 0, 4, 2);
       break;
    case 29: play_rest(2);
        break;
    case 30: play_note('E', 0, 4, 2);
       break;
    case 31: play_rest(2);
        break;
    case 32: play_note('C', 0, 4, 2);
       break;
    case 33: play_note('D', 0, 4, 2);
       break;
    case 34: play_note('B', 0, 3, 2);
       break;
    case 35: play_rest(4);
        break;
    case 36: play_note('C', 0, 4, 5);
       break;
    case 37: play_rest(2);
       break;
    case 38: play_note('G', 0, 3, 2);
       break;
    case 39: play_rest(3);
       break;
    case 40: play_note('E', 0, 3, 4);
       break;
    case 41: play_rest(2);
       break;
    case 42: play_note('A', 0, 3, 2);
       break;
    case 43: play_rest(2);
         break;
    case 44: play_note('B', 0, 3, 2);
       break;
    case 45: play_rest(2);
       break;
    case 46: play_note('B', 1, 3, 2);
       break;
    case 47: play_note('A', 0, 3, 4); //phrase
       break;
    case 48: play_note('G' , 0, 3, 3);
       break;
    case 49: play_note('E', 0, 4, 2);
       break;
    case 50: play_rest(1);
        break;
    case 51: play_note('G', 0, 4, 2);
       break;
    case 52: play_note('A', 0, 4, 4);
       break;
    case 53: play_note('F', 0, 4, 2);
       break;
    case 54: play_note('G', 0, 4, 2);
       break;
    case 55: play_rest(2);
        break;
    case 56: play_note('E', 0, 4, 2);
       break;
    case 57: play_rest(2);
        break;
    case 58: play_note('C', 0, 4, 2);
       break;
    case 59: play_note('D', 0, 4, 2);
       break;
    case 60: play_note('B', 0, 3, 2);
       break;
    case 61: play_rest(8);
       break;
    case 62: play_note('G', 0, 4, 2);
       break;
    case 63: play_note('G', 1, 4, 2);
       break;
    case 64: play_note('F', 0, 4, 2);
       break;
    case 65: play_note('E', 1, 4, 2);
       break;
    case 66: play_rest(2);
       break;
    case 67: play_note('E', 0, 4, 2);
       break;
    case 68: play_rest(2);
       break;
    case 69: play_note('A', 1, 3, 2);
       break;
    case 70: play_note('A', 0, 3, 2);
       break;
    case 71: play_note('C', 0, 4, 2);
       break;
    case 72: play_rest(2);
       break;
    case 73: play_note('A', 0, 3, 2);
       break;
    case 74: play_note('C', 0, 4, 2);
       break;
    case 75: play_note('D', 0, 4, 2);
       break;
    case 76: play_rest(4);
       break;
    case 77: play_note('G', 0, 3, 2);
       break;
    case 78: play_note('G', 1, 3, 2);
       break;
    case 79: play_note('F', 0, 3, 2);
       break;
    case 80: play_note('E', 1, 3, 2);
       break;
    case 81: play_rest(2);
       break;
    case 82: play_note('E', 0, 3, 2);
       break;
    case 83: play_rest(2);
       break;
    case 84: play_note('G', 0, 4, 2);
       break;
    case 85: play_rest(2);
       break;
    case 86: play_note('G', 0, 4, 1);
       break;
    case 87: play_rest(1);
       break;
    case 88: play_note('G', 0, 4, 4);
       break;
    case 89: play_rest(8);
       break;
    case 90: play_note('G', 0, 4, 2);
       break;
    case 91: play_note('G', 1, 4, 2);
       break;
    case 92: play_note('F', 0, 4, 2);
       break;
    case 93: play_note('E', 1, 4, 2);
       break;
    case 94: play_rest(2);
       break;
    case 95: play_note('E', 0, 4, 2);
       break;
    case 96: play_rest(2);
       break;
    case 97: play_note('A', 1, 3, 2);
       break;
    case 98: play_note('A', 0, 3, 2);
       break;
    case 99: play_note('C', 0, 4, 2);
       break;
    case 100: play_rest(2);
       break;
    case 101: play_note('A', 0, 3, 2);
       break;
    case 102: play_note('C', 0, 4, 2);
       break;
    case 103: play_note('D', 0, 4, 2);
       break;
    case 104: play_rest(4);
       break;
    case 105: play_note('E', 1, 4, 4);
       break;
    case 106: play_rest(2);
       break;
    case 107: play_note('D', 0, 4, 2);
       break;
    case 108: play_rest(4);
        break;
    case 109: play_note('C', 0, 4, 4);
       break;
    case 110: play_rest(10);
       break;
    case 111: play_note('C', 0, 4, 2);
       break;
    case 112: play_rest(1);
       break;
    case 113: play_note('C', 0, 4, 2);
        break;
    case 114: play_rest(2);
        break;
    case 115: play_note('C', 0, 4, 2);
        break;
    case 116: play_rest(2);
        break;
    case 117: play_note('C', 0, 4, 2);
        break;
    case 118: play_note('D', 0, 4, 4);
        break;
    case 119: play_note('E', 0, 4, 2);
        break;
    case 120: play_note('C', 0, 4, 2);
        break;
    case 121: play_rest(2);
        break;
    case 122: play_note('A', 0, 3, 2);
        break;
    case 123: play_note('G', 0, 3, 4);
        break;
    case 124: play_rest(4);
       break;
    case 125: play_note('C', 0, 4, 2);
       break;
    case 126: play_rest(1);
       break;
    case 127: play_note('C', 0, 4, 2);
        break;
    case 128: play_rest(2);
        break;
    case 129: play_note('C', 0, 4, 2);
        break;
    case 130: play_rest(2);
        break;
    case 131: play_note('C', 0, 4, 2);
        break;
    case 132: play_note('D', 0, 4, 2);
        break;
    case 133: play_note('E', 0, 4, 2);
        break;
    case 134: play_rest(16);
        break;
    case 135: play_note('C', 0, 4, 2);
        break;
    case 136: play_rest(1);
        break;
    case 137: play_note('C', 0, 4, 2);
        break;
    case 138: play_rest(2);
        break;
    case 139: play_note('C', 0, 4, 2);
        break;
    case 140: play_rest(2);
        break;
    case 141: play_note('C', 0, 4, 2);
        break;
    case 142: play_note('D', 0, 4, 4);
        break;
    case 143: play_note('E', 0, 4, 2);
        break;
    case 144: play_note('C', 0, 4, 2);
        break;
    case 145: play_rest(2);
        break;
    case 146: play_note('A', 0, 3, 2);
        break;
    case 147: play_note('G', 0, 3, 4);
	break;
    case 148: play_rest(8);
        break;
    default: notes=-1;

 }
}//song2

void song3(uint16_t note) { //(Max and Kellen)
  switch (note) {
    case 0: play_note('E', 0, 4, 7);
       break;
    case 1: play_rest(1);
       break;
    case 2: play_note('E', 0, 4, 7);
       break;
    case 3: play_rest(1);
       break;
    case 4: play_note('E', 0, 4, 7);
       break;
    case 5: play_rest(1);
       break;
    case 6: play_note('E', 0, 4, 3);
       break;
    case 7: play_rest(1);
       break;
    case 8: play_note('E', 0, 4, 3);
       break;
    case 9: play_rest(5);
       break;
    case 10: play_note('E', 0, 5, 4);
       break;
    case 11: play_note('G', 1, 5, 4);
       break;
    case 12: play_note('E', 0, 5, 4);
       break;
    case 13: play_note('G', 0, 5, 8);
       break;
    case 14: play_note('E', 0, 5, 8);
       break;
       //phrase
    case 15: play_note('E', 1, 4, 7);
       break;
    case 16: play_rest(1);
       break;
    case 17: play_note('E', 1, 4, 7);
       break;
    case 18: play_rest(1);
       break;
    case 19: play_note('E', 1, 4, 7);
       break;
    case 20: play_rest(1);
       break;
    case 21: play_note('E', 1, 4, 3);
       break;
    case 22: play_rest(1);
       break;
    case 23: play_note('E', 1, 4, 3);
       break;
    case 24: play_rest(5);
       break;
    case 25: play_note('E', 1, 5, 4);
       break;
    case 26: play_note('E', 0, 5, 3);
       break;
    case 27: play_rest(1); //rest 1
       break;
    case 28: play_note('E', 0, 5, 4);
       break;
    case 29: play_note('G', 1, 5, 8);
       break;
    case 30: play_note('E', 0, 5, 8);
       break;
    default: notes=-1;
  }
}//song3

void play_song(uint8_t song, uint8_t note) {
  //if you add a song, you'll have to add it to this
  //switch statement.
  switch (song) {
  case 0: song0(note); //beaver fight song
    break;
  case 1: song1(note); //tetris theme
    break;
  case 2: song2(note); //super mario bros
    break;
  case 3: song3(note);
    break;
  default: song0(note); //defaults to beaver fight song
  }
}

void play_rest(uint8_t duration) {
  //mute for duration
  //duration is in 64th notes at 120bpm
  PORTD |= mute;
  beat=0;
  max_beat = duration;
}

void play_note(char note, uint8_t flat, uint8_t octave, uint8_t duration) {
  //pass in the note, it's key, the octave they want, and a duration
  //function sets the value of OCR1A and the timer
  //note must be A-G
  //flat must be 1 (for flat) or 0 (for natural) (N/A on C or F)
  //octave must be 0-8 (0 is the lowest, 8 doesn't sound very good)
  //duration is in 64th notes at 120bpm
  //e.g. play_note('D', 1, 0, 16)
  //this would play a Db, octave 0 for 1 quarter note
  //120 bpm (every 32ms inc beat)
  PORTD &= unmute;      //unmute (just in case)
  beat = 0;             //reset the beat counter
  max_beat = duration;  //set the max beat
  switch (octave) {
    case 0: switch (note) {
      case 'A': if(flat){OCR1A=Ab0;}
        else {OCR1A=A0;}
	break;
      case 'B': if(flat){OCR1A=Bb0;}
        else {OCR1A=B0;}
	break;
      case 'C': OCR1A=C0;
	break;
      case 'D': if(flat){OCR1A=Db0;}
        else {OCR1A=D0;}
	break;
      case 'E': if(flat){OCR1A=Eb0;}
        else {OCR1A=E0;}
	break;
      case 'F': OCR1A=F0;
	break;
      case 'G': if(flat){OCR1A=Gb0;}
        else {OCR1A=G0;}
	break;
      } 
      break;
    case 1: switch (note) {
      case 'A': if(flat){OCR1A=Ab1;}
        else {OCR1A=A1;}
	break;
      case 'B': if(flat){OCR1A=Bb1;}
        else {OCR1A=B1;}
	break;
      case 'C': OCR1A=C1;
	break;
      case 'D': if(flat){OCR1A=Db1;}
        else {OCR1A=D1;}
	break;
      case 'E': if(flat){OCR1A=Eb1;}
        else {OCR1A=E1;}
	break;
      case 'F': OCR1A=F1;
	break;
      case 'G': if(flat){OCR1A=Gb1;}
        else {OCR1A=G1;}
	break;
      } 
      break;
    case 2: switch (note) {
      case 'A': if(flat){OCR1A=Ab2;}
        else {OCR1A=A2;}
	break;
      case 'B': if(flat){OCR1A=Bb2;}
        else {OCR1A=B2;}
	break;
      case 'C': OCR1A=C2;
	break;
      case 'D': if(flat){OCR1A=Db2;}
        else {OCR1A=D2;}
	break;
      case 'E': if(flat){OCR1A=Eb2;}
        else {OCR1A=E2;}
	break;
      case 'F': OCR1A=F2;
	break;
      case 'G': if(flat){OCR1A=Gb2;}
        else {OCR1A=G2;}
	break;
      } 
      break;
    case 3: switch (note) {
      case 'A': if(flat){OCR1A=Ab3;}
        else {OCR1A=A3;}
	break;
      case 'B': if(flat){OCR1A=Bb3;}
        else {OCR1A=B3;}
	break;
      case 'C': OCR1A=C3;
	break;
      case 'D': if(flat){OCR1A=Db3;}
        else {OCR1A=D3;}
	break;
      case 'E': if(flat){OCR1A=Eb3;}
        else {OCR1A=E3;}
	break;
      case 'F': OCR1A=F3;
	break;
      case 'G': if(flat){OCR1A=Gb3;}
        else {OCR1A=G3;}
	break;
      } 
      break;
    case 4: switch (note) {
      case 'A': if(flat){OCR1A=Ab4;}
        else {OCR1A=A4;}
	break;
      case 'B': if(flat){OCR1A=Bb4;}
        else {OCR1A=B4;}
	break;
      case 'C': OCR1A=C4;
	break;
      case 'D': if(flat){OCR1A=Db4;}
        else {OCR1A=D4;}
	break;
      case 'E': if(flat){OCR1A=Eb4;}
        else {OCR1A=E4;}
	break;
      case 'F': OCR1A=F4;
	break;
      case 'G': if(flat){OCR1A=Gb4;}
        else {OCR1A=G4;}
	break;
      } 
      break;
    case 5: switch (note) {
      case 'A': if(flat){OCR1A=Ab5;}
        else {OCR1A=A5;}
	break;
      case 'B': if(flat){OCR1A=Bb5;}
        else {OCR1A=B5;}
	break;
      case 'C': OCR1A=C5;
	break;
      case 'D': if(flat){OCR1A=Db5;}
        else {OCR1A=D5;}
	break;
      case 'E': if(flat){OCR1A=Eb5;}
        else {OCR1A=E5;}
	break;
      case 'F': OCR1A=F5;
	break;
      case 'G': if(flat){OCR1A=Gb5;}
        else {OCR1A=G5;}
	break;
      } 
      break;
    case 6: switch (note) {
      case 'A': if(flat){OCR1A=Ab6;}
        else {OCR1A=A6;}
	break;
      case 'B': if(flat){OCR1A=Bb6;}
        else {OCR1A=B6;}
	break;
      case 'C': OCR1A=C6;
	break;
      case 'D': if(flat){OCR1A=Db6;}
        else {OCR1A=D6;}
	break;
      case 'E': if(flat){OCR1A=Eb6;}
        else {OCR1A=E6;}
	break;
      case 'F': OCR1A=F6;
	break;
      case 'G': if(flat){OCR1A=Gb6;}
        else {OCR1A=G6;}
	break;
      } 
      break;
    case 7: switch (note) {
      case 'A': if(flat){OCR1A=Ab7;}
        else {OCR1A=A7;}
	break;
      case 'B': if(flat){OCR1A=Bb7;}
        else {OCR1A=B7;}
	break;
      case 'C': OCR1A=C7;
	break;
      case 'D': if(flat){OCR1A=Db7;}
        else {OCR1A=D7;}
	break;
      case 'E': if(flat){OCR1A=Eb7;}
        else {OCR1A=E7;}
	break;
      case 'F': OCR1A=F7;
	break;
      case 'G': if(flat){OCR1A=Gb7;}
        else {OCR1A=G7;}
	break;
      } 
      break;
    case 8: switch (note) {
      case 'A': if(flat){OCR1A=Ab8;}
        else {OCR1A=A8;}
	break;
      case 'B': if(flat){OCR1A=Bb8;}
        else {OCR1A=B8;}
	break;
      case 'C': OCR1A=C8;
	break;
      case 'D': if(flat){OCR1A=Db8;}
        else {OCR1A=D8;}
	break;
      case 'E': if(flat){OCR1A=Eb8;}
        else {OCR1A=E8;}
	break;
      case 'F': OCR1A=F8;
	break;
      case 'G': if(flat){OCR1A=Gb8;}
        else {OCR1A=G8;}
	break;
      } 
      break;
    default: OCR1A=0x0000;
  }
}

void music_off(void) {
  //this turns the alarm timer off
  notes=0;
  TCCR1B &= ~((1<<CS11)|(1<<CS10));
  //and mutes the output
  PORTD |= mute;
}

void music_on(void) {
  //this starts the alarm timer running
  notes=0;
  TCCR1B |= (1<<CS11)|(1<<CS10);
  //unmutes the output
  PORTD &= unmute;
  //and starts the selected song
  play_song(song, notes);
}

void music_init(void) {
  //initially turned off (use music_on() to turn on)
  TIMSK |= (1<<OCIE1A);  //enable timer interrupt 1 on compare
  TCCR1A = 0x00;         //TCNT1, normal port operation
  TCCR1B |= (1<<WGM12);  //CTC, OCR1A = top, clk/64 (250kHz)
  TCCR1C = 0x00;         //no forced compare
  OCR1A = 0x0031;        //(use to vary alarm frequency)
  music_off();
  beat = 0;
  max_beat = 0;
  notes = 0;
  song = 2;              
} 
