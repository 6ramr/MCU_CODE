/*
 * chip.h
 *
 *  Created on: Nov 15, 2018
 *      Author: j
 */

#ifndef CHIP_H_
#define CHIP_H_

#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"

#define STM32F0XX
#define STM32_PCLK1 (42000000ul)
#define STM32_TIMCLK1 (42000000ul)
#define RCC_APB1ENR_CAN1EN RCC_APB1ENR_CANEN
#define RCC_APB1RSTR_CAN1RST RCC_APB1RSTR_CANRST
#define CAN1_RX1_IRQn CAN_RX1_IRQn
#define CAN1_TX_IRQHandler CAN1_TX_IRQHandler
#define CAN1_RX0_IRQHandler CAN1_RX0_IRQHandler
#define CAN1_RX1_IRQHandler CAN1_RX1_IRQHandler
#endif /* CHIP_H_ */
