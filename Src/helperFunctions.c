/*
 * helperFunctions.c
 *
 *  Created on: 05.07.2018
 *      Author: JACK
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "helperFunctions.h"
#include "sensor.h"
#include "spi.h"
/* USER CODE END Includes */


//// Resolving Two's Compliment at 8bit
//int ResolveTwosCompliment8 (uint8_t);
//// Resolving Two's Compliment at two bytes
//int ResolveTwosCompliment2_8 (uint8_t, uint8_t);
//// Send Data to SensorX
//void SPI_Tx_Sensor (SPI_HandleTypeDef*, uint8_t[], int len);
//// Read Data from SensorX
//uint8_t SPI_RxSensor (SPI_HandleTypeDef*, uint8_t);
//// Resolve Acceleration from Two's Compliment to float in mg
//float ResolveAcceleration (uint8_t, uint8_t);
//


// Resolving Two's Compliment at 8bit
int ResolveTwosCompliment8 (uint8_t data) {
	int x = (int)data;
	if (x >= 0x80) x -= 0xFF;
	return x;
}
// Resolving Two's Compliment at two bytes
int ResolveTwosCompliment2_8 (uint8_t hiByte, uint8_t loByte) {
	int x = (int)hiByte*0x100;
	if (x >= 0x8000) x -= 0xFFFF;
	x += loByte;
	return x;
}
// Send Data to SensorX
void SPI_TxSensor (SPI_HandleTypeDef *hspi1,uint8_t data[], int len){
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi1,data,(uint16_t)len,SPI_TimeOut);
	while( hspi1->State == HAL_SPI_STATE_BUSY );
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
}
// Read Data from SensorX
uint8_t SPI_RxSensor (SPI_HandleTypeDef *hspi1,uint8_t adress) {
	uint8_t rxData;
	adress += 0x80;

	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi1,&adress,1,SPI_TimeOut);
	HAL_SPI_Receive(hspi1,&rxData,1,SPI_TimeOut);
	while( hspi1->State == HAL_SPI_STATE_BUSY );
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);

	return rxData;
}
/*
uint8_t* SPI_RxSensor_M (SPI_HandleTypeDef *hspi1,uint8_t adress[], int len) {
	static uint8_t rxData[len];
	for(int i=0; i<len; i++) {
		adress[i] += 0x80;

		HAL_SPI_Transmit(hspi1,&adress[i],sizeof(uint8_t),SPI_TimeOut);
		while( hspi1->State == HAL_SPI_STATE_BUSY );
		HAL_SPI_Receive(hspi1,&rxData[i],sizeof(uint8_t),SPI_TimeOut);
	}
	return rxData;
}*/
// Resolve Acceleration from Two's Compliment to float in mg
float ResolveAcceleration (uint8_t hiByte, uint8_t loByte) {
	extern float sensorLsbValue;
	return ResolveTwosCompliment2_8(hiByte, loByte)*sensorLsbValue;
}
