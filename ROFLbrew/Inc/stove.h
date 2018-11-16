/*
 * stove.h
 *
 *  Created on: 19.05.2016
 *      Author: washed
 */

#ifndef STOVE_H_
#define STOVE_H_

#include "gpio.h"

#define STOVE_POWERSTEP_COUNT 10
#define STOVE_STARTING_POWERSTEP 5

#define STOVE_BUTTON_PRESS_TIME 750
#define STOVE_BUTTON_NOPRESS_TIME 250

#define STOVE_NUM_PINS 4

#define STOVE_POWER_PIN_INDEX 0
#define STOVE_POWERMODE_PIN_INDEX 1
#define STOVE_INCPOWER_PIN_INDEX 2
#define STOVE_DECPOWER_PIN_INDEX 3

const uint32_t stove_powersteps[ STOVE_POWERSTEP_COUNT ];

extern uint8_t handle_stove_timers;

typedef enum STOVE_OP
{

  STOVE_OP_NOP = 0,
  STOVE_OP_PIN0_LOW_STOVE_ON,
  STOVE_OP_PIN0_LOW_STOVE_OFF,
  STOVE_OP_PIN0_HIGH_STOVE_ON,
  STOVE_OP_PIN0_HIGH_STOVE_OFF,
  STOVE_OP_PIN1_LOW,
  STOVE_OP_PIN1_HIGH,
  STOVE_OP_PIN2_LOW,
  STOVE_OP_PIN2_HIGH,
  STOVE_OP_PIN3_LOW,
  STOVE_OP_PIN3_HIGH

} STOVE_OP;

typedef struct STOVE
{
  uint8_t power_enabled;
  uint32_t powermode_enabled;
  uint32_t current_powerstep;
  uint32_t requested_powerstep;
  uint8_t locked;
  GPIO_TypeDef* gpio_bank[ STOVE_NUM_PINS ];
  uint16_t gpio_pin[ STOVE_NUM_PINS ];
  // Soft_TimerCatalog_TypeDef* timers;
  uint16_t current_pin;

} STOVE;

extern STOVE stove0;

void handleStove( STOVE* stove_handle );
void initStove( STOVE* stove_handle );
void switchStoveOn( STOVE* stove_handle );
void switchStoveOff( STOVE* stove_handle );
void switchStovePowerMode( STOVE* stove_handle );
void increaseStovePower( STOVE* stove_handle );
void decreaseStovePower( STOVE* stove_handle );
void setStovePower( STOVE* stove_handle );
void handleStoveOPs( STOVE* stove_handle );

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
