/*
 * MAX31865.c
 *
 *  Created on: 12.06.2016
 *      Author: washed
 */

#include "MAX31865.h"
#include <math.h>
#include <string.h>
#include "stm32f7xx_hal.h"
#include "temp_control.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#if defined( __ENABLE_SYSVIEW )
#include "SEGGER_SYSVIEW.h"
#include "SEGGER_SYSVIEW_Conf.h"
#endif

static inline void assertCS();
static inline void deassertCS();
static inline void powerOff();
static inline void powerOn();

static void handleMAX31865Devices();
static void initMAX31865();
static uint32_t getRTDData_MAX31865();
static void setCfgReg_MAX31865( uint8_t config_flags );
static uint8_t getFaultStatus_MAX31865();
static void setReg_MAX31865( uint8_t reg, uint8_t* p_data, uint8_t len );
static void getReg_MAX31865( uint8_t reg, uint8_t* p_data, uint8_t len );
static void initSPIIdleClock();

#define MAX31865StackSize 0x400
uint32_t MAX31865TaskBuffer[ MAX31865StackSize ];
osStaticThreadDef_t MAX31865ControlBlock;
osThreadId MAX31865TaskHandle;

void vTaskMAX31865( void* pvParameters )
{
  initMAX31865();

  for ( ;; )
  {
    handleMAX31865Devices();
  }

  // We should never get here
  vTaskDelete( NULL );
}

osThreadId createTaskMAX31865()
{
  osThreadStaticDef( MAX31865Task, vTaskMAX31865, osPriorityRealtime, 0, MAX31865StackSize, MAX31865TaskBuffer,
                     &MAX31865ControlBlock );
  MAX31865TaskHandle = osThreadCreate( osThread( MAX31865Task ), NULL );
  return MAX31865TaskHandle;
}

void MAX31865_notifyDataReadyFromISR()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR( MAX31865TaskHandle, &xHigherPriorityTaskWoken );
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

static void handleMAX31865Devices()
{
  uint32_t dr_event_count = ulTaskNotifyTake( pdTRUE, 50 );

  if ( dr_event_count != 0 )
  {
    uint32_t rtd_data;
    int32_t temperature_int;

    rtd_data = getRTDData_MAX31865();

    // TODO: Do any necessary post processing steps here
    if ( MAX31865_USE_CALLENDAR_VAN_DUSEN )
    {
      float r = ( (float)rtd_data * (float)MAX31865_REF_RESISTOR ) / MAX31865_RTD_DIVIDER;
      temperature_int = lrintf( (float)( ( r * ( MAX31865_CVD_A + r * ( MAX31865_CVD_B + r * MAX31865_CVD_C ) ) ) *
                                         TEMP_INT_FACTOR ) ) -
                        MAX31865_KELVIN_0dC;
    }
    else
    {
      temperature_int = lrintf( (float)( ( (float)rtd_data / 32.0 ) - 256.0 ) * (float)TEMP_INT_FACTOR );
    }

    // TODO: Send the sample to the temp_control task with a queue
    // addTemperatureSample( &temp_control0, temperature_int );
    putTemperatureSample( temperature_int );
  }
}

static void initMAX31865()
{
  uint8_t status_reg;
  uint8_t fault_status;
  uint8_t fault_detect_running;

  powerOff();
  initSPIIdleClock();
  HAL_Delay( 10 );
  powerOn();
  HAL_Delay( 50 );

  setCfgReg_MAX31865( ( MAX_31865_CFG_VBIAS_ON | MAX_31865_CFG_FAULT_AUTODELAY | MAX_31865_CFG_50HZ_ON ) );

  do
  {
    // TODO: Add a timeout here!
    fault_detect_running = 0;
    getReg_MAX31865( MAX31865_CFG_REG_RD_ADDR, &status_reg, 1 );
    fault_detect_running |= ( status_reg & MAX_31865_CFG_FAULT_AUTODELAY );
    if ( !( status_reg & MAX_31865_CFG_FAULT_AUTODELAY ) )
    {
      fault_status = getFaultStatus_MAX31865();
    }
  } while ( fault_detect_running );

  if ( fault_status == 0 )
  {
    setCfgReg_MAX31865(
        ( MAX_31865_CFG_VBIAS_ON | MAX_31865_CFG_CONVAUTO_ON | MAX_31865_CFG_FAULT_NONE | MAX_31865_CFG_50HZ_ON ) );
  }
  else
  {
    setCfgReg_MAX31865( 0 );
  }
}

static uint32_t getRTDData_MAX31865()
{
  const uint8_t tx_data[ 3 ] = { MAX31865_RTDMSB_REG_RD_ADDR, 0xFF, 0xFF };
  uint8_t rx_data[ 3 ];

  assertCS();
  HAL_SPI_TransmitReceive( MAX31865_SPI_INSTANCE_PT, (uint8_t*)&tx_data, (uint8_t*)&rx_data, 3, 100 );
  deassertCS();
  return ( ( ( ( rx_data[ 1 ] << 8 ) | rx_data[ 2 ] ) >> 1 ) & 0x7FFF );
}

static void setCfgReg_MAX31865( uint8_t config_flags )
{
  setReg_MAX31865( MAX31865_CFG_REG_WR_ADDR, &config_flags, 1 );
}

static uint8_t getFaultStatus_MAX31865()
{
  uint8_t fault_status;
  getReg_MAX31865( MAX31865_FLTSTAT_REG_RD_ADDR, &fault_status, 1 );
  return ( fault_status & MAX31865_FLTSTAT_REG_MASK );
}

static void setReg_MAX31865( uint8_t reg, uint8_t* p_data, uint8_t len )
{
  uint8_t tx_data[ 9 ] = { reg, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
  uint8_t rx_data[ 9 ];

  if ( ( p_data == NULL ) || ( len > 8 ) || ( len < 1 ) ) return;

  memcpy( &tx_data[ 1 ], p_data, len );
  assertCS();
  HAL_SPI_TransmitReceive( MAX31865_SPI_INSTANCE_PT, (uint8_t*)&tx_data, (uint8_t*)&rx_data, len + 1, 50 );
  deassertCS();
}

static void getReg_MAX31865( uint8_t reg, uint8_t* p_data, uint8_t len )
{
  uint8_t tx_data[ 9 ] = { reg, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
  uint8_t rx_data[ 9 ];

  if ( ( p_data == NULL ) || ( len > 8 ) || ( len < 1 ) ) return;

  assertCS();
  HAL_SPI_TransmitReceive( MAX31865_SPI_INSTANCE_PT, (uint8_t*)tx_data, (uint8_t*)rx_data, len + 1, 50 );
  deassertCS();
  memcpy( p_data, &rx_data[ 1 ], len );
}

static void initSPIIdleClock()
{
  uint8_t tx_data[ 2 ] = { 0xFF, 0xFF };
  uint8_t rx_data[ 2 ];
  assertCS();
  HAL_SPI_TransmitReceive( MAX31865_SPI_INSTANCE_PT, (uint8_t*)&tx_data, (uint8_t*)&rx_data, 2, 10 );
  deassertCS();
}

static inline void assertCS()
{
  MAX31865_CS_BANK->BSRR = ( MAX31865_CS_PIN << 16UL );
}

static inline void deassertCS()
{
  MAX31865_CS_BANK->BSRR = MAX31865_CS_PIN;
}

static inline void powerOff()
{
  MAX31865_PWR_BANK->BSRR = ( MAX31865_PWR_PIN << 16UL );
}

static inline void powerOn()
{
  MAX31865_PWR_BANK->BSRR = MAX31865_PWR_PIN;
}
