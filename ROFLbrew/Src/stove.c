/*
 * stove.c

 *
 *  Created on: 19.05.2016
 *      Author: washed
 */

#include "stove.h"
#include "gpio.h"
#include "stm32f4xx_hal.h"

STOVE stove0;
Soft_TimerCatalog_TypeDef stove_timers;
Soft_Timer_TypeDef stove_timer_press;
Soft_Timer_TypeDef stove_timer_nopress;

uint8_t handle_stove_timers = 0;

const uint32_t stove_powersteps[ STOVE_POWERSTEP_COUNT ] = { 300, 600, 800, 1000, 1200, 1300, 1500, 1600, 1800, 2000 };

static void stove_releaseButton( STOVE* stove_handle );
static void stove_buttonFin( STOVE* stove_handle );

void initStove( STOVE* stove_handle )
{
  stove_handle->current_powerstep = STOVE_STARTING_POWERSTEP;
  stove_handle->requested_powerstep = STOVE_STARTING_POWERSTEP;
  stove_handle->power_enabled = 0;
  stove_handle->powermode_enabled = 0;

  timer_set( &stove_timer_press, STOVE_BUTTON_PRESS_TIME, TIMER_FLAG_NONE, stove_releaseButton, (void*)stove_handle );
  timer_set( &stove_timer_nopress, STOVE_BUTTON_NOPRESS_TIME, TIMER_FLAG_NONE, stove_buttonFin, (void*)stove_handle );
  timer_register( &stove_timers, &stove_timer_press, NULL );
  timer_register( &stove_timers, &stove_timer_nopress, NULL );

  stove_handle->timers = &stove_timers;

  stove_handle->gpio_bank[ STOVE_POWER_PIN_INDEX ] = STOVE_POWER_GPIO_Port;
  stove_handle->gpio_pin[ STOVE_POWER_PIN_INDEX ] = STOVE_POWER_Pin;

  stove_handle->gpio_bank[ STOVE_POWERMODE_PIN_INDEX ] = STOVE_POWERMODE_GPIO_Port;
  stove_handle->gpio_pin[ STOVE_POWERMODE_PIN_INDEX ] = STOVE_POWERMODE_Pin;

  stove_handle->gpio_bank[ STOVE_INCPOWER_PIN_INDEX ] = STOVE_INCPOWER_GPIO_Port;
  stove_handle->gpio_pin[ STOVE_INCPOWER_PIN_INDEX ] = STOVE_INCPOWER_Pin;

  stove_handle->gpio_bank[ STOVE_DECPOWER_PIN_INDEX ] = STOVE_DECPOWER_GPIO_Port;
  stove_handle->gpio_pin[ STOVE_DECPOWER_PIN_INDEX ] = STOVE_DECPOWER_Pin;
}

void handleStove( STOVE* stove_handle )
{
  if ( handle_stove_timers == 1 )
  {
    timer_handleTimerCatalog( &stove_timers );
    handle_stove_timers = 0;
  }
  setStovePower( stove_handle );
}

static void stove_releaseButton( STOVE* stove_handle )
{
  HAL_GPIO_WritePin( stove_handle->gpio_bank[ stove_handle->current_pin ],
                     stove_handle->gpio_pin[ stove_handle->current_pin ], GPIO_PIN_RESET );
  timer_set( &stove_timer_nopress, STOVE_BUTTON_NOPRESS_TIME, TIMER_FLAG_RUN, stove_buttonFin, (void*)stove_handle );
}

static void stove_buttonFin( STOVE* stove_handle )
{
  switch ( stove_handle->current_pin )
  {
    case STOVE_POWER_PIN_INDEX:
      stove_handle->power_enabled ^= 1;
      if ( stove_handle->power_enabled == 0 )
      {
        stove_handle->powermode_enabled = 0;
        stove_handle->current_powerstep = STOVE_STARTING_POWERSTEP;
      }
      break;
    case STOVE_POWERMODE_PIN_INDEX:
      stove_handle->powermode_enabled ^= 1;
      break;
    case STOVE_INCPOWER_PIN_INDEX:
      stove_handle->current_powerstep += 1;
      if ( stove_handle->current_powerstep >= STOVE_POWERSTEP_COUNT )
        stove_handle->current_powerstep = STOVE_POWERSTEP_COUNT - 1;
      break;
    case STOVE_DECPOWER_PIN_INDEX:
      stove_handle->current_powerstep -= 1;
      if ( stove_handle->current_powerstep < 0 ) stove_handle->current_powerstep = 0;
      break;
  }

  stove_handle->locked = 0;
}
/*
 void handleStoveOPs( STOVE* stove_handle )
 {
 switch ( stove_handle->next_op )
 {
 case STOVE_OP_PIN0_LOW_STOVE_ON:
 HAL_GPIO_WritePin( stove_handle->gpio_bank[STOVE_POWER_PIN_INDEX],
 stove_handle->gpio_pin[STOVE_POWER_PIN_INDEX],
 GPIO_PIN_RESET );
 stove_handle->power_enabled = 1;
 stove_handle->next_op = STOVE_OP_NOP;
 stove_handle->wait_time = STOVE_BUTTON_PERIOD_TIME;
 break;
 case STOVE_OP_PIN0_LOW_STOVE_OFF:
 HAL_GPIO_WritePin( STOVE0_GPIO_Port, STOVE0_Pin, GPIO_PIN_RESET );
 initStove( stove_handle );
 stove_handle->power_enabled = 0;
 stove_handle->next_op = STOVE_OP_NOP;
 stove_handle->wait_time = STOVE_BUTTON_PERIOD_TIME;
 break;
 case STOVE_OP_PIN0_HIGH_STOVE_ON:
 break;
 case STOVE_OP_PIN0_HIGH_STOVE_OFF:
 break;
 case STOVE_OP_PIN1_LOW:
 HAL_GPIO_WritePin( STOVE1_GPIO_Port, STOVE1_Pin, GPIO_PIN_RESET );
 stove_handle->powermode_enabled = 1;
 stove_handle->next_op = STOVE_OP_NOP;
 stove_handle->wait_time = STOVE_BUTTON_PERIOD_TIME;
 break;
 case STOVE_OP_PIN1_HIGH:
 break;
 case STOVE_OP_PIN2_LOW:
 HAL_GPIO_WritePin( STOVE2_GPIO_Port, STOVE2_Pin, GPIO_PIN_RESET );
 stove_handle->current_powerstep--;
 if ( stove_handle->current_powerstep < 0 )
 stove_handle->current_powerstep = 0;
 stove_handle->next_op = STOVE_OP_NOP;
 stove_handle->wait_time = STOVE_BUTTON_PERIOD_TIME;
 break;
 case STOVE_OP_PIN2_HIGH:
 break;
 case STOVE_OP_PIN3_LOW:
 HAL_GPIO_WritePin( STOVE3_GPIO_Port, STOVE3_Pin, GPIO_PIN_RESET );
 stove_handle->current_powerstep++;
 if ( stove_handle->current_powerstep > STOVE_POWERSTEP_COUNT )
 stove_handle->current_powerstep = STOVE_POWERSTEP_COUNT;
 stove_handle->next_op = STOVE_OP_NOP;
 stove_handle->wait_time = STOVE_BUTTON_PERIOD_TIME;
 break;
 default:
 case STOVE_OP_NOP:
 break;
 }

 stove_handle->handle_op = 0;
 }
 */
void switchStoveOn( STOVE* stove_handle )
{
  // ON
  if ( ( !stove_handle->locked ) && ( !stove_handle->power_enabled ) )
  {
    stove_handle->locked = 1;
    stove_handle->current_pin = STOVE_POWER_PIN_INDEX;
    HAL_GPIO_WritePin( stove_handle->gpio_bank[ STOVE_POWER_PIN_INDEX ],
                       stove_handle->gpio_pin[ STOVE_POWER_PIN_INDEX ], GPIO_PIN_SET );
    timer_set( &stove_timer_press, STOVE_BUTTON_PRESS_TIME, TIMER_FLAG_RUN, stove_releaseButton, (void*)stove_handle );
  }
}

void switchStoveOff( STOVE* stove_handle )
{
  // OFF
  if ( ( !stove_handle->locked ) && ( stove_handle->power_enabled ) )
  {
    stove_handle->locked = 1;
    stove_handle->current_pin = STOVE_POWER_PIN_INDEX;
    HAL_GPIO_WritePin( stove_handle->gpio_bank[ STOVE_POWER_PIN_INDEX ],
                       stove_handle->gpio_pin[ STOVE_POWER_PIN_INDEX ], GPIO_PIN_SET );
    timer_set( &stove_timer_press, STOVE_BUTTON_PRESS_TIME, TIMER_FLAG_RUN, stove_releaseButton, (void*)stove_handle );
  }
}

void switchStovePowerMode( STOVE* stove_handle )
{
  // POWER
  if ( ( !stove_handle->locked ) && ( stove_handle->power_enabled ) && ( !stove_handle->powermode_enabled ) )
  {
    stove_handle->locked = 1;
    stove_handle->current_pin = STOVE_POWERMODE_PIN_INDEX;
    HAL_GPIO_WritePin( stove_handle->gpio_bank[ STOVE_POWERMODE_PIN_INDEX ],
                       stove_handle->gpio_pin[ STOVE_POWERMODE_PIN_INDEX ], GPIO_PIN_SET );
    timer_set( &stove_timer_press, STOVE_BUTTON_PRESS_TIME, TIMER_FLAG_RUN, stove_releaseButton, (void*)stove_handle );
  }
}

void increaseStovePower( STOVE* stove_handle )
{
  // Powerup
  if ( ( !stove_handle->locked ) && ( stove_handle->powermode_enabled ) && ( stove_handle->power_enabled ) )
  {
    stove_handle->locked = 1;
    stove_handle->current_pin = STOVE_INCPOWER_PIN_INDEX;
    HAL_GPIO_WritePin( stove_handle->gpio_bank[ STOVE_INCPOWER_PIN_INDEX ],
                       stove_handle->gpio_pin[ STOVE_INCPOWER_PIN_INDEX ], GPIO_PIN_SET );
    timer_set( &stove_timer_press, STOVE_BUTTON_PRESS_TIME, TIMER_FLAG_RUN, stove_releaseButton, (void*)stove_handle );
  }
}

void decreaseStovePower( STOVE* stove_handle )
{
  // Powerdown
  if ( ( !stove_handle->locked ) && ( stove_handle->powermode_enabled ) && ( stove_handle->power_enabled ) )
  {
    stove_handle->locked = 1;
    stove_handle->current_pin = STOVE_DECPOWER_PIN_INDEX;
    HAL_GPIO_WritePin( stove_handle->gpio_bank[ STOVE_DECPOWER_PIN_INDEX ],
                       stove_handle->gpio_pin[ STOVE_DECPOWER_PIN_INDEX ], GPIO_PIN_SET );
    timer_set( &stove_timer_press, STOVE_BUTTON_PRESS_TIME, TIMER_FLAG_RUN, stove_releaseButton, (void*)stove_handle );
  }
}

void setStovePower( STOVE* stove_handle )
{
  if ( stove_handle->current_powerstep == stove_handle->requested_powerstep ) return;

  if ( ( stove_handle->requested_powerstep < 0 ) || ( stove_handle->requested_powerstep > STOVE_POWERSTEP_COUNT ) )
    return;

  if ( stove_handle->current_powerstep > stove_handle->requested_powerstep )
    decreaseStovePower( stove_handle );
  else if ( stove_handle->current_powerstep < stove_handle->requested_powerstep )
    increaseStovePower( stove_handle );
}
