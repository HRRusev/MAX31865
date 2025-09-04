/*
 * MAX31865.h
 *
 *  Created on: May 7, 2024
 *      Author: eng. Hristian Rusev
 */

#ifndef INC_MAX31865_LIB_H_
#define INC_MAX31865_LIB_H_


#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "stm32f4xx_hal.h"

//-------------- Characteristic sensor PT100 ----------------------------
#define PT100_R0 (double)100.0				//resistant sensor PT100 at 0
#define REF_MAX31865 (double)400.0			//Reference resistor value

/*--------------- Coefficient ----------------------------------------*/
#define MAX31865_A (double)0.0039083
#define MAX31865_B (double)0.0000005775
/*--------------------------------------------------------------------*/
#define FILTER_SIZE 		100
#define MAX31865_TIMEOUT_MS	(uint32_t)500
enum class RTDconnecting
{
	two = 2, three, four
};

enum modeMAX31865
{
	normallyOff = 0, automat
};

enum filterMAX31865
{
	Hz50, Hz60
};

typedef struct{
//Chip select pin. Active low
	GPIO_TypeDef* 	portCS;
	uint16_t 		pinCS;
//Data ready pin. Active low
	GPIO_TypeDef* 	portDRDY;
	uint16_t 		pinDRDY;
}pinMAX31865;

typedef struct{
	float	value;
	bool	valid;
}sTemperature;

typedef struct
	{
		uint16_t resistanceRegister;
		uint16_t faultThresholdHight;
		uint16_t faultThresholdLow;
		uint8_t statusFault;
}rxData;

class MAX31865
{
public:
	MAX31865(SPI_HandleTypeDef* mHandleSPI, const pinMAX31865* mPins, modeMAX31865 mMode,
			RTDconnecting mNumWire, filterMAX31865 mfilter);
	bool initialization(void);
	uint8_t getConfiguration(void);
	sTemperature getTeperature(void);
	sTemperature getFilterTemperature(void);
	bool getIsInit() const;
	bool getIsBusy() const;


private:
	SPI_HandleTypeDef*	handleSPI;
	const pinMAX31865*	pins;
	modeMAX31865 		mode;
	RTDconnecting		numberWire;
	filterMAX31865 		filter;

	bool 				sensorError;
	bool				isInit;
	bool 				isBusy;
	uint8_t 			configurationRegister[2];
	rxData				data;
	float				buffer[FILTER_SIZE];
	uint8_t 			bufferIndex;
	uint8_t 			bufferCount;

	void csSet();
	void csReset();
	void read(void);
	float getTeperatureMathRTD();

};
/*
bool initializationMAX31865(bool automatMode, uint8_t numWrite);
uint8_t getConfigurationInfoMAX31865(void);
double getTeperatureMAX31865(void);
double getTeperatureMathPT100(double resistancePT100);*/




#endif /* INC_MAX31865_LIB_H_ */
