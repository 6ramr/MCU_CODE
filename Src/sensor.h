/*
 * sensor.h
 *
 *  Created on: 11.07.2018
 *      Author: JACK
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#define SENSOR_OUT_T 0xA6
#define SENSOR_OUT_X_L 0x28
#define SENSOR_OUT_X_H 0x29
#define SENSOR_OUT_Y_L 0x2A
#define SENSOR_OUT_Y_H 0x2B
#define SENSOR_OUT_Z_L 0x2C
#define SENSOR_OUT_Z_H 0x2D

#define SENSOR_ID 0x43
#define SENSOR_WHO_AM_I 0x0F

#define SENSOR_LSB_2G 0.061

extern uint8_t sensorConf[14];
extern float sensorLsbValue;


#endif /* SENSOR_H_ */
