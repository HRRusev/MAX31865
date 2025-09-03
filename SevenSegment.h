/*
 * SevenSegment.h
 *
 *  Created on: May 15, 2025
 *      Author: eng. Hristian Rusev
 */

#ifndef MYLIBRARY_SEVENSEGMENT_H_
#define MYLIBRARY_SEVENSEGMENT_H_

#define SEVENSEGMENT_LIB_VERSION "1.0"

#define MAXNUMDIGITS 	8
#define BLANK_IDX		36
#define DASH_IDX		37
#define DOT_IDX			38
#define ASTERISK_IDX	39
#define UNDERSCORE_IDX	40


#include <string.h>
#include <cstdio>
#include <cstdint>
#include "stm32f4xx_hal.h"

extern volatile bool displayRefresh;

typedef struct{
	GPIO_TypeDef* 	port;
	uint16_t 		pin;
}segmentDigit;

typedef struct{
	GPIO_TypeDef* 	port;
	int16_t 		pin;
}segmentsPin;



static const uint8_t digitCodeMap[] = {
  // GFEDCBA  Segments      7-segment map:
  0b00111111, // 0   "0"          AAA
  0b00000110, // 1   "1"         F   B
  0b01011011, // 2   "2"         F   B
  0b01001111, // 3   "3"          GGG
  0b01100110, // 4   "4"         E   C
  0b01101101, // 5   "5"         E   C
  0b01111101, // 6   "6"          DDD
  0b00000111, // 7   "7"
  0b01111111, // 8   "8"
  0b01101111, // 9   "9"
  0b01110111, // 65  'A'
  0b01111100, // 66  'b'
  0b00111001, // 67  'C'
  0b01011110, // 68  'd'
  0b01111001, // 69  'E'
  0b01110001, // 70  'F'
  0b00111101, // 71  'G'
  0b01110110, // 72  'H'
  0b00110000, // 73  'I'
  0b00001110, // 74  'J'
  0b01110110, // 75  'K'  Same as 'H'
  0b00111000, // 76  'L'
  0b00000000, // 77  'M'  NO DISPLAY
  0b01010100, // 78  'n'
  0b00111111, // 79  'O'
  0b01110011, // 80  'P'
  0b01100111, // 81  'q'
  0b01010000, // 82  'r'
  0b01101101, // 83  'S'
  0b01111000, // 84  't'
  0b00111110, // 85  'U'
  0b00111110, // 86  'V'  Same as 'U'
  0b00000000, // 87  'W'  NO DISPLAY
  0b01110110, // 88  'X'  Same as 'H'
  0b01101110, // 89  'y'
  0b01011011, // 90  'Z'  Same as '2'
  0b00000000, // 32  ' '  BLANK
  0b01000000, // 45  '-'  DASH
  0b10000000, // 46  '.'  DOT
  0b01100011, // 42 '*'   ASTERISK
  0b00001000, // 95 '_'   UNDERSCORE
};


static const uint32_t powersOf10[] =
{
	1,		//10^0
	10,		//10^2
	100,	//10^3
	1000,	//10^4
	10000, 	//10^5
	100000,	//10^6
	1000000	//10^7
};



class SevenSegment {

public:
	SevenSegment();
	SevenSegment(const segmentsPin (&pins)[8], const segmentDigit *digits, uint8_t segmentDigits_, bool commonCathode_);
	void intilization(const segmentsPin (&pins)[8], const segmentDigit *digits, uint8_t segmentDigits_, bool commonCathode_);
	~SevenSegment();
	bool message(const char *str);

	void setDash(void);
	bool setNumber(int32_t number);
	bool setNumberF(float numberF);

	void refreshDisplay();

	void operator<<(const char *str);



private:
	const segmentsPin 		*segmentPin;			//Seven segment ports and pins
	const segmentDigit		*digitsPin;					//Digits ports and pins
	uint8_t 				numberDigits;
	bool 					commonCathode;
	uint8_t					position;
	uint8_t					bufferIndex;
	char					*buffer;
	char					*symbolCodes; 				// The active setting of each segment of each digit

	void segmentON(void);
	void segmentOFF(void);
	void loadSymbol(void);
	uint8_t intToString(int number);
	uint8_t intToString(int number, char (&pins)[16]);
	void floatToString(float number);




};

#endif /* MYLIBRARY_SEVENSEGMENT_H_ */
