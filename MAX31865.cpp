/*
 * MAX31865.cpp
 *
 *  Created on: May 7, 2024
 *      Author: eng. Hristian Rusev
 */


#include <MAX31865.h>

MAX31865::MAX31865(SPI_HandleTypeDef* mHandleSPI, const cs_Pin* mCS, modeMAX31865 mMode,
		RTDconnecting mNumWire, filterMAX31865 mfilter)
{
	this->handleSPI = mHandleSPI;
	this->csPin = mCS;
	this->mode = mMode;
	this->numberWire = mNumWire;
	this->filter = mfilter;
	this->bufferCount = 0;
	this->bufferIndex = 0;

	this->csReset();

	this->isInit = this->initialization();




}

bool MAX31865::initialization(void)
{
	this->sensorError = false;
	this->configurationRegisterWrite[0] = 0x80;
	if(this->numberWire == RTDconnecting::two || this->numberWire == RTDconnecting::four)
		this->configurationRegisterWrite[1] = 0xC3;
	else
		this->configurationRegisterWrite[1] = 0xD3;

	if(this->mode == modeMAX31865::normally)
		this->configurationRegisterWrite[1] = this->configurationRegisterWrite[1] & 0xBF;
	if(this->filter == filterMAX31865::Hz60)
		this->configurationRegisterWrite[1] = this->configurationRegisterWrite[1] & 0xFE;

	this->csSet();
	HAL_SPI_Transmit(this->handleSPI, this->configurationRegisterWrite, 2, 100);
	this->csReset();

	//Check MAX31865 configuration
	uint8_t test = this->getConfiguration() & 0xD1;	//stay only bit for setting

	if(test == (this->configurationRegisterWrite[1] & 0xD1))
				return true;

	return false;
}

uint8_t MAX31865::getConfiguration(void)
{
	uint8_t readData = 0xFF;
	uint8_t configurationAddress = 0x00;
	this->csSet();
	HAL_SPI_Transmit(this->handleSPI, &configurationAddress, 1, 100);
	HAL_SPI_Receive(this->handleSPI, &readData, 1, 100);
	this->csReset();

	return readData;
}

sTemperature MAX31865::getTeperature(void)
{
	sTemperature temperature;
	this->read();
	if(this->data.statusFault != 0x00)
	{
		temperature.valid = false;
		return temperature;
	}

	temperature.valid = true;
	temperature.value = this->getTeperatureMathRTD();

	if(temperature.valid)
	{
		this->buffer[this->bufferIndex] = temperature.value;
		this->bufferIndex = (this->bufferIndex + 1) % FILTER_SIZE;
		if(this->bufferCount < FILTER_SIZE) ++this->bufferCount;
	}

	return temperature;
}

sTemperature  MAX31865::getFilterTemperature(void)
{
	sTemperature temperature = this->getTeperature();
	if(temperature.valid)
	{
		float sum = 0;
		for(uint8_t i = 0; i < this->bufferCount; ++i)
			sum += this->buffer[i];

		temperature.value = sum / this->bufferCount;
	}

	return temperature;
}

bool MAX31865::getIsInit() const { if(this->isInit) return true; return false; }

void MAX31865::csSet() { HAL_GPIO_WritePin(this->csPin->port, this->csPin->pin, GPIO_PIN_RESET); }
void MAX31865::csReset() { HAL_GPIO_WritePin(this->csPin->port, this->csPin->pin, GPIO_PIN_SET); }

void MAX31865::read(void)
{
	uint8_t startAddress = 0x01;
	uint8_t rxBuffer[7] = {0x00};														//for receive data
	this->csSet();
	HAL_SPI_Transmit(this->handleSPI, &startAddress, 1, 100);
	HAL_SPI_Receive(this->handleSPI, rxBuffer, 7, 100);
	this->csReset();

	this->data.resistanceRegister = (rxBuffer[0] << 8 | rxBuffer[1]) >> 1;		//value of resistance
	this->data.faultThresholdHight = (rxBuffer[2] << 8 | rxBuffer[3]);		//value of fault
	this->data.faultThresholdLow = (rxBuffer[4] << 8 | rxBuffer[5]);		//value of fault
	this->data.statusFault = rxBuffer[6];									//fault status

}

float MAX31865::getTeperatureMathRTD()
{
	float result;
	float aITS90 = 0.0039083;
	float bITS90 = -0.00000057750;
	float resistancePT100 = ((float)this->data.resistanceRegister * REF_MAX31865) / (float)32768.0;

	if(resistancePT100 > (float)100.0)
		result = ((-PT100_R0 * aITS90) + sqrt(pow(PT100_R0, 2) * pow(aITS90, 2) -
				(4 * PT100_R0 * bITS90 * (PT100_R0 - resistancePT100)))) / (2 * PT100_R0 * bITS90);
	else
		result = 0.000000000270 * pow(resistancePT100, 5) -
		 0.000000066245 * pow(resistancePT100, 4) -
		 0.000000184636 * pow(resistancePT100, 3) +
		 0.002320232987 * pow(resistancePT100, 2) +
		 2.229927824035 * resistancePT100 - 242.090854986215;

	//This is another formula with error for positive 1*10-4 and for negative -50->0.02, -100->0.2, -200->2.4
	//result = 3383.8098 - (8658.008 * sqrt(0.1758481 - (0.000231 * resistancePT100)));

	return result;
}


