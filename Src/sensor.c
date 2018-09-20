/*
 * sensor.conf
 *
 *  Created on: 02.07.2018
 *      Author: JACK
 */
#include "main.h"
#include "stm32f0xx_hal.h"
#include "sensor.h"

uint8_t sensorConf[14] ={0x21,0x40,		//Soft-reset sensor configuration
						 0x21,0x04,      //CTRL2-register adress and bit-mask
						 0x20,0x60,		//CTRL1-register adress and bit-mask
						 0x22,0x00,		//CTRL3-register adress and bit-mask
						 0x23,0x00,		//CTRL4-register adress and bit-mask
						 0x24,0x40,		//CTRL5-register adress and bit-mask
						 0x25,0xC0		//FIFO-CTRL-register adress and bit-mask
};

float sensorLsbValue = SENSOR_LSB_2G;






