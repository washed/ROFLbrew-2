/*
 * MAX31865.c
 *
 *  Created on: 12.06.2016
 *      Author: washed
 */

#include "MAX31865.h"
#include <math.h>
#include <string.h>
#include "gui/ugui.h"
#include "lcd_touch/touch.h"
#include "stm32f7xx_hal.h"
#include "temp_control.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#if defined( __ENABLE_SYSVIEW )
#include "SEGGER_SYSVIEW.h"
#include "SEGGER_SYSVIEW_Conf.h"
#endif

volatile uint32_t MAX31865_DEVICES_RTD_DATA[ MAX31865_MAX_DEVICES ];
int32_t MAX31865_DEVICES_TEMP[ MAX31865_MAX_DEVICES ];
float MAX31865_DEVICES_TEMP_FLOAT[ MAX31865_MAX_DEVICES ];

volatile uint8_t MAX31865_DEVICES_SAMPLE_READY[ MAX31865_MAX_DEVICES ] = { 0, 0, 0, 0 };
volatile uint32_t MAX31865_DEVICES_TIME_SINCE_LAST_READ[ MAX31865_MAX_DEVICES ] = { 0, 0, 0, 0 };

static inline void assertCS( uint32_t device_num );
static inline void deassertCS( uint32_t device_num );

const uint32_t MAX31865_DEVICES_CS_BANK_PIN[ MAX31865_MAX_DEVICES ][ 2 ] = {  //
  { MAX31865_0_CS_BANK, MAX31865_0_CS_PIN },
  { MAX31865_1_CS_BANK, MAX31865_1_CS_PIN },
  { MAX31865_2_CS_BANK, MAX31865_2_CS_PIN },
  { MAX31865_3_CS_BANK, MAX31865_3_CS_PIN }
};

const uint32_t MAX31865_DEVICES_DR_BANK_PIN[ MAX31865_MAX_DEVICES ][ 2 ] = {  //
  { MAX31865_0_DR_BANK, MAX31865_0_DR_PIN },
  { MAX31865_1_DR_BANK, MAX31865_1_DR_PIN },
  { MAX31865_2_DR_BANK, MAX31865_2_DR_PIN },
  { MAX31865_3_DR_BANK, MAX31865_3_DR_PIN }
};

#define STACK_SIZE 0x400
uint32_t MAX31865TaskBuffer[ STACK_SIZE ];
osStaticThreadDef_t MAX31865ControlBlock;

void vTaskMAX31865( void* pvParameters )
{
  for ( ;; )
  {
	  handleMAX31865Devices();
  }

  // We should never get here
  vTaskDelete(NULL);
}

osThreadId createTaskMAX31865()
{
	osThreadStaticDef(MAX31865Task, vTaskMAX31865, osPriorityNormal, 0, STACK_SIZE, MAX31865TaskBuffer, &MAX31865ControlBlock);
	return osThreadCreate(osThread(MAX31865Task), NULL);
}

void handleMAX31865Devices()
{
  float r;

  for ( uint32_t device_num = 0; device_num < MAX31865_CON_DEVICES; device_num++ )
  {
    if ( MAX31865_DEVICES_SAMPLE_READY[ device_num ] )
    {
#if defined( __ENABLE_SYSVIEW )
      SEGGER_SYSVIEW_OnTaskStartExec( SYSVIEW_TASK_MAX31865_RX );
#endif

      MAX31865_DEVICES_TIME_SINCE_LAST_READ[ device_num ] = 0;
      getRTDData_MAX31865( device_num );
      // TODO: Do any necessary post processing steps here
      if ( MAX31865_USE_CALLENDAR_VAN_DUSEN )
      {
        r = ( (float)MAX31865_DEVICES_RTD_DATA[ device_num ] * (float)MAX31865_REF_RESISTOR ) / MAX31865_RTD_DIVIDER;
        MAX31865_DEVICES_TEMP[ device_num ] =
            lrintf( (float)( ( r * ( MAX31865_CVD_A + r * ( MAX31865_CVD_B + r * MAX31865_CVD_C ) ) ) *
                             TEMP_INT_FACTOR ) ) -
            MAX31865_KELVIN_0dC;
      }
      else
      {
        // MAX31865_DEVICES_TEMP[device_num] =
        // MAX31865_DEVICES_RTD_DATA[device_num];
        MAX31865_DEVICES_TEMP[ device_num ] = lrintf(
            (float)( ( (float)MAX31865_DEVICES_RTD_DATA[ device_num ] / 32.0 ) - 256.0 ) * (float)TEMP_INT_FACTOR );
      }

      switch ( device_num )
      {
        case 0:
          addTemperatureSample( &temp_control0, MAX31865_DEVICES_TEMP[ device_num ] );
          break;
      }

#if defined( __ENABLE_SYSVIEW )
      SEGGER_SYSVIEW_OnTaskStopReady( SYSVIEW_TASK_LCD_UPDATE, 0 );
#endif
    }
  }
}

void initMAX31865()
{
  uint8_t status_reg[ MAX31865_CON_DEVICES ], fault_status[ MAX31865_CON_DEVICES ], fault_detect_running;

  ( (GPIO_TypeDef*)( MAX31865_PWR_BANK ) )->BSRR = MAX31865_PWR_PIN;

  HAL_Delay( 50 );

  setCfgReg_MAX31865( 0, ( MAX_31865_CFG_VBIAS_ON | MAX_31865_CFG_FAULT_AUTODELAY | MAX_31865_CFG_50HZ_ON ) );
  setCfgReg_MAX31865( 1, ( MAX_31865_CFG_VBIAS_ON | MAX_31865_CFG_FAULT_AUTODELAY | MAX_31865_CFG_50HZ_ON ) );

  do
  {
    fault_detect_running = 0;
    for ( uint8_t device_num = 0; device_num < MAX31865_CON_DEVICES; device_num++ )
    {
      getReg_MAX31865( device_num, MAX31865_CFG_REG_RD_ADDR, &status_reg[ device_num ], 1 );
      fault_detect_running |= ( status_reg[ device_num ] & MAX_31865_CFG_FAULT_AUTODELAY );
      if ( !( status_reg[ device_num ] & MAX_31865_CFG_FAULT_AUTODELAY ) )
      {
        fault_status[ device_num ] = getFaultStatus_MAX31865( device_num );
      }
    }
  } while ( fault_detect_running );

  for ( uint8_t device_num = 0; device_num < MAX31865_CON_DEVICES; device_num++ )
  {
    if ( fault_status[ device_num ] == 0 )
      setCfgReg_MAX31865( device_num, ( MAX_31865_CFG_VBIAS_ON | MAX_31865_CFG_CONVAUTO_ON | MAX_31865_CFG_FAULT_NONE |
                                        MAX_31865_CFG_50HZ_ON ) );
    else
      setCfgReg_MAX31865( device_num, 0 );
  }
}

void checkMAX31865WDG()
{
  for ( uint32_t device_num = 0; device_num < MAX31865_CON_DEVICES; device_num++ )
  {
    if ( MAX31865_DEVICES_TIME_SINCE_LAST_READ[ device_num ] >= MAX31865_WDG_PERIOD )
      MAX31865_DEVICES_SAMPLE_READY[ device_num ] = 1;
  }
}

void getRTDData_MAX31865( uint32_t device_num )
{
  const uint8_t tx_data[ 3 ] = { MAX31865_RTDMSB_REG_RD_ADDR, 0xFF, 0xFF };
  uint8_t rx_data[ 3 ];

  assertCS( device_num );
  HAL_SPI_TransmitReceive( MAX31865_SPI_INSTANCE_PT, (uint8_t*)&tx_data, (uint8_t*)&rx_data, 3, 100 );
  deassertCS( device_num );
  MAX31865_DEVICES_SAMPLE_READY[ device_num ] = 0;
  MAX31865_DEVICES_RTD_DATA[ device_num ] = ( ( ( rx_data[ 1 ] << 8 ) | rx_data[ 2 ] ) >> 1 ) & 0x7FFF;
}

void setCfgReg_MAX31865( uint32_t device_num, uint8_t config_flags )
{
  setReg_MAX31865( device_num, MAX31865_CFG_REG_WR_ADDR, &config_flags, 1 );
}

uint8_t getFaultStatus_MAX31865( uint32_t device_num )
{
  uint8_t fault_status;
  getReg_MAX31865( device_num, MAX31865_FLTSTAT_REG_RD_ADDR, &fault_status, 1 );
  return ( fault_status & MAX31865_FLTSTAT_REG_MASK );
}

void setReg_MAX31865( uint32_t device_num, uint8_t reg, uint8_t* p_data, uint8_t len )
{
  uint8_t tx_data[ 9 ] = { reg, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
  uint8_t rx_data[ 9 ];

  if ( ( p_data == NULL ) || ( len > 8 ) || ( len < 1 ) ) return;

  memcpy( &tx_data[ 1 ], p_data, len );
  assertCS( device_num );
  HAL_SPI_TransmitReceive( MAX31865_SPI_INSTANCE_PT, (uint8_t*)&tx_data, (uint8_t*)&rx_data, len + 1, 50 );
  deassertCS( device_num );
}

void getReg_MAX31865( uint32_t device_num, uint8_t reg, uint8_t* p_data, uint8_t len )
{
  uint8_t tx_data[ 9 ] = { reg, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
  uint8_t rx_data[ 9 ];

  if ( ( p_data == NULL ) || ( len > 8 ) || ( len < 1 ) ) return;

  assertCS( device_num );
  HAL_SPI_TransmitReceive( MAX31865_SPI_INSTANCE_PT, (uint8_t*)tx_data, (uint8_t*)rx_data, len + 1, 50 );
  deassertCS( device_num );
  memcpy( p_data, &rx_data[ 1 ], len );
}

void initSPIIdleClock()
{
  uint8_t tx_data[ 2 ] = { 0xFF, 0xFF };
  uint8_t rx_data[ 2 ];
  assertCS( 0 );
  HAL_SPI_TransmitReceive( MAX31865_SPI_INSTANCE_PT, (uint8_t*)&tx_data, (uint8_t*)&rx_data, 2, 10 );
  deassertCS( 0 );
}

static inline void assertCS( uint32_t device_num )
{
  ( (GPIO_TypeDef*)( MAX31865_DEVICES_CS_BANK_PIN[ device_num ][ 0 ] ) )->BSRR =
      ( MAX31865_DEVICES_CS_BANK_PIN[ device_num ][ 1 ] << 16UL );
}

static inline void deassertCS( uint32_t device_num )
{
  ( (GPIO_TypeDef*)( MAX31865_DEVICES_CS_BANK_PIN[ device_num ][ 0 ] ) )->BSRR =
      MAX31865_DEVICES_CS_BANK_PIN[ device_num ][ 1 ];
}

void tickMAX31865WDGTimer( uint32_t ticks )
{
  for ( uint32_t device_num = 0; device_num < MAX31865_CON_DEVICES; device_num++ )
  {
    MAX31865_DEVICES_TIME_SINCE_LAST_READ[ device_num ] += ticks;
  }
}
