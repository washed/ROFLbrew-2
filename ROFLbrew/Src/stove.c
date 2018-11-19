/*
 * stove.c

 *
 *  Created on: 19.05.2016
 *      Author: washed
 */

#include "stm32f7xx_hal.h"
#include "stove.h"
#include "gpio.h"
#include "cmsis_os.h"

const uint32_t stove_powersteps[ STOVE_POWERSTEP_COUNT ] = {
  0, 300, 600, 800, 1000, 1200, 1300, 1500, 1600, 1800, 2000
};

const uint32_t stove_powersteps[ STOVE_POWERSTEP_COUNT ];

typedef enum STOVE_STATE
{
  STOVE_STATE_RESET = 0,
  STOVE_STATE_READY,
  STOVE_STATE_BUSY
} STOVE_STATE;

typedef struct STOVE
{
  uint32_t current_powerstep;
  uint32_t requested_powerstep;
  STOVE_STATE state;
  GPIO_TypeDef* gpio_bank[ STOVE_NUM_PINS ];
  uint16_t gpio_pin[ STOVE_NUM_PINS ];
} STOVE;

#define STACK_SIZE 0x2000

uint32_t stoveTaskBuffer[ STACK_SIZE ];
osStaticThreadDef_t stoveControlBlock;

static void handleStove( STOVE* stove_handle );
static void stovePowerButtonPress( STOVE* stove_handle );
static void switchStovePowerMode( STOVE* stove_handle );
static void increaseStovePower( STOVE* stove_handle );
static void decreaseStovePower( STOVE* stove_handle );

STOVE stove0;

void vTaskStove( void* pvParameters )
{
  stove0.state = STOVE_STATE_RESET;

  for ( ;; )
  {
    handleStove( &stove0 );
  }

  // We should never get here
  vTaskDelete( NULL );
}

osThreadId createTaskStove()
{
  osThreadStaticDef( stove, vTaskStove, osPriorityNormal, 0, STACK_SIZE, stoveTaskBuffer, &stoveControlBlock );
  return osThreadCreate( osThread( stove ), NULL );
}

void setStovePower( uint32_t requested_powerstep )
{
  // TODO: We should probably use a queue or something for communication to this task!
  if ( requested_powerstep >= STOVE_POWERSTEP_COUNT )
  {
    return;
  }

  stove0.requested_powerstep = requested_powerstep;
}

uint32_t getStovePower()
{
  return stove_powersteps[ stove0.current_powerstep ];
}

static void initStove( STOVE* stove_handle )
{
  stove_handle->current_powerstep = 0;
  stove_handle->requested_powerstep = 0;

  stove_handle->gpio_bank[ STOVE_POWER_PIN_INDEX ] = STV_PWR_GPIO_Port;
  stove_handle->gpio_pin[ STOVE_POWER_PIN_INDEX ] = STV_PWR_Pin;

  stove_handle->gpio_bank[ STOVE_POWERMODE_PIN_INDEX ] = STV_PWR_MODE_GPIO_Port;
  stove_handle->gpio_pin[ STOVE_POWERMODE_PIN_INDEX ] = STV_PWR_MODE_Pin;

  stove_handle->gpio_bank[ STOVE_INCPOWER_PIN_INDEX ] = STV_PWR_INC_GPIO_Port;
  stove_handle->gpio_pin[ STOVE_INCPOWER_PIN_INDEX ] = STV_PWR_INC_Pin;

  stove_handle->gpio_bank[ STOVE_DECPOWER_PIN_INDEX ] = STV_PWR_DEC_GPIO_Port;
  stove_handle->gpio_pin[ STOVE_DECPOWER_PIN_INDEX ] = STV_PWR_DEC_Pin;

  stove_handle->state = STOVE_STATE_READY;
}

static void handleStove( STOVE* stove_handle )
{
  switch ( stove_handle->state )
  {
    case STOVE_STATE_RESET:
      initStove( stove_handle );
      break;

    default:
    case STOVE_STATE_BUSY:
      // Can't do anything in this case
      osThreadYield();
      break;

    case STOVE_STATE_READY:
      if ( stove_handle->requested_powerstep > STOVE_POWERSTEP_COUNT )
      {
        return;
      }

      if ( stove_handle->current_powerstep == stove_handle->requested_powerstep )
      {
        return;
      }
      else if ( stove_handle->requested_powerstep > 0 && stove_handle->current_powerstep == 0 )
      {
        stovePowerButtonPress( stove_handle );
        switchStovePowerMode( stove_handle );
      }
      else if ( stove_handle->requested_powerstep == 0 && stove_handle->current_powerstep > 0 )
      {
        stovePowerButtonPress( stove_handle );
      }
      else if ( stove_handle->current_powerstep > stove_handle->requested_powerstep )
      {
        decreaseStovePower( stove_handle );
      }
      else if ( stove_handle->current_powerstep < stove_handle->requested_powerstep )
      {
        increaseStovePower( stove_handle );
      }

      osThreadYield();
      break;
  }
}

static void stovePowerButtonPress( STOVE* stove_handle )
{
  // ON/OFF
  stove_handle->state = STOVE_STATE_BUSY;

  HAL_GPIO_WritePin( stove_handle->gpio_bank[ STOVE_POWER_PIN_INDEX ], stove_handle->gpio_pin[ STOVE_POWER_PIN_INDEX ],
                     GPIO_PIN_SET );
  osDelay( STOVE_BUTTON_PRESS_TIME );
  HAL_GPIO_WritePin( stove_handle->gpio_bank[ STOVE_POWER_PIN_INDEX ], stove_handle->gpio_pin[ STOVE_POWER_PIN_INDEX ],
                     GPIO_PIN_RESET );
  osDelay( STOVE_BUTTON_NOPRESS_TIME );

  stove_handle->state = STOVE_STATE_READY;
}

static void switchStovePowerMode( STOVE* stove_handle )
{
  // POWERMODE
  stove_handle->state = STOVE_STATE_BUSY;

  HAL_GPIO_WritePin( stove_handle->gpio_bank[ STOVE_POWERMODE_PIN_INDEX ],
                     stove_handle->gpio_pin[ STOVE_POWERMODE_PIN_INDEX ], GPIO_PIN_SET );
  osDelay( STOVE_BUTTON_PRESS_TIME );
  HAL_GPIO_WritePin( stove_handle->gpio_bank[ STOVE_POWERMODE_PIN_INDEX ],
                     stove_handle->gpio_pin[ STOVE_POWERMODE_PIN_INDEX ], GPIO_PIN_RESET );
  osDelay( STOVE_BUTTON_NOPRESS_TIME );

  stove_handle->current_powerstep = STOVE_STARTING_POWERSTEP;
  stove_handle->state = STOVE_STATE_READY;
}

static void increaseStovePower( STOVE* stove_handle )
{
  // Increase power
  stove_handle->state = STOVE_STATE_BUSY;

  if ( ( stove_handle->current_powerstep > 0 ) && ( stove_handle->current_powerstep < ( STOVE_POWERSTEP_COUNT - 1 ) ) )
  {
    HAL_GPIO_WritePin( stove_handle->gpio_bank[ STOVE_INCPOWER_PIN_INDEX ],
                       stove_handle->gpio_pin[ STOVE_INCPOWER_PIN_INDEX ], GPIO_PIN_SET );
    osDelay( STOVE_BUTTON_PRESS_TIME );
    HAL_GPIO_WritePin( stove_handle->gpio_bank[ STOVE_INCPOWER_PIN_INDEX ],
                       stove_handle->gpio_pin[ STOVE_INCPOWER_PIN_INDEX ], GPIO_PIN_RESET );
    osDelay( STOVE_BUTTON_NOPRESS_TIME );

    stove_handle->current_powerstep++;
  }

  stove_handle->state = STOVE_STATE_READY;
}

static void decreaseStovePower( STOVE* stove_handle )
{
  // Decrease power
  stove_handle->state = STOVE_STATE_BUSY;

  if ( stove_handle->current_powerstep > 1 )
  {
    HAL_GPIO_WritePin( stove_handle->gpio_bank[ STOVE_DECPOWER_PIN_INDEX ],
                       stove_handle->gpio_pin[ STOVE_DECPOWER_PIN_INDEX ], GPIO_PIN_SET );
    osDelay( STOVE_BUTTON_PRESS_TIME );
    HAL_GPIO_WritePin( stove_handle->gpio_bank[ STOVE_DECPOWER_PIN_INDEX ],
                       stove_handle->gpio_pin[ STOVE_DECPOWER_PIN_INDEX ], GPIO_PIN_RESET );
    osDelay( STOVE_BUTTON_NOPRESS_TIME );

    stove_handle->current_powerstep--;
  }

  stove_handle->state = STOVE_STATE_READY;
}
