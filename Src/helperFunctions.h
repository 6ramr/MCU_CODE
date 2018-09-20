/*
 * helperFunctions.h
 *
 *  Created on: 11.07.2018
 *      Author: JACK
 */

#ifndef HELPERFUNCTIONS_H_
#define HELPERFUNCTIONS_H_
// Number of Elements in any Array
#define LEN(x) (sizeof(x)/sizeof(x[0]))

// Resolving Two's Compliment at 8bit
int ResolveTwosCompliment8 (uint8_t);
// Resolving Two's Compliment at two bytes
int ResolveTwosCompliment2_8 (uint8_t, uint8_t);
// Send Data to SensorX
void SPI_TxSensor (SPI_HandleTypeDef*, uint8_t[], int len);
// Read Data from SensorX
uint8_t SPI_RxSensor (SPI_HandleTypeDef*, uint8_t);
// Resolve Acceleration from Two's Compliment to float in mg
float ResolveAcceleration (uint8_t, uint8_t);


#endif /* HELPERFUNCTIONS_H_ */
