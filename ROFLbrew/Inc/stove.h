/*
 * stove.h
 *
 *  Created on: 19.05.2016
 *      Author: washed
 */

#ifndef STOVE_H_
#define STOVE_H_

#include "stm32f7xx_hal.h"
#include "gpio.h"
#include "cmsis_os.h"

#define STOVE_POWERSTEP_COUNT 11
#define STOVE_STARTING_POWERSTEP 6

#define STOVE_BUTTON_PRESS_TIME 750
#define STOVE_BUTTON_NOPRESS_TIME 250

#define STOVE_NUM_PINS 4

#define STOVE_POWER_PIN_INDEX 0
#define STOVE_POWERMODE_PIN_INDEX 1
#define STOVE_INCPOWER_PIN_INDEX 2
#define STOVE_DECPOWER_PIN_INDEX 3

osThreadId createTaskStove();
void setStovePower();
uint32_t getStovePower();

/*
 *
 *      Stove-Cable     Signal
 *
 *      Black           GND
 *      Violett         PWR
 *      Red             PWR-MODE
 *      Grey            PWR-UP
 *      Brown           PWR-DOWN
 *
 *
 *      LEMO-Stecker:
 *      White						GND
 *      Green						PWR
 *      Red							PWR-MODE
 *      Yellow					PWR-UP
 *      Grey						PWR-DOWN
 */

#endif /* STOVE_H_ */
