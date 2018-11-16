/*
 * touch.c
 *
 *  Created on: 23.10.2016
 *      Author: washed
 */

#include "stm32f7xx_hal.h"
#include "i2c.h"
#include "lcd_touch/touch.h"
#include "lcd_touch/touch_GSL680_fw.h"


#include "SEGGER_SYSVIEW.h"
#include "SEGGER_SYSVIEW_Conf.h"

TouchEvent_TypeDef touchEvent;

// GSLX680 touch controller driver

// Private functions
static void touch_chipClearRegs();
static void touch_chipReset();
static void touch_loadFirmware();
static void touch_chipStartup();

uint8_t touch_readTouchData()
{
  uint8_t touch_data[ 24 ] = { 0 };
  uint8_t reg = 0x80;

  SEGGER_SYSVIEW_OnUserStart( SYSVIEW_EVENT_TOUCH );

  HAL_I2C_Master_Transmit( &hi2c3, WRITE_ADD, &reg, 1, 10 );
  HAL_I2C_Master_Receive( &hi2c3, READ_ADD, touch_data, 24, 10 );

  touchEvent.touch_count = *(uint32_t*)&touch_data[ 0 ];

  touchEvent.x1 = ( *(uint16_t*)&touch_data[ 4 ] ) & 0xFFF;
  touchEvent.y1 = ( *(uint16_t*)&touch_data[ 6 ] ) & 0xFFF;

  touchEvent.x2 = ( *(uint16_t*)&touch_data[ 8 ] ) & 0xFFF;
  touchEvent.y2 = ( *(uint16_t*)&touch_data[ 10 ] ) & 0xFFF;

  touchEvent.x3 = ( *(uint16_t*)&touch_data[ 12 ] ) & 0xFFF;
  touchEvent.y3 = ( *(uint16_t*)&touch_data[ 14 ] ) & 0xFFF;

  touchEvent.x4 = ( *(uint16_t*)&touch_data[ 16 ] ) & 0xFFF;
  touchEvent.y4 = ( *(uint16_t*)&touch_data[ 18 ] ) & 0xFFF;

  touchEvent.x5 = ( *(uint16_t*)&touch_data[ 20 ] ) & 0xFFF;
  touchEvent.y5 = ( *(uint16_t*)&touch_data[ 22 ] ) & 0xFFF;

  SEGGER_SYSVIEW_OnUserStop( SYSVIEW_EVENT_TOUCH );
  return 0;
}

//GSLX680_Initial
void touch_init()
{
  HAL_GPIO_WritePin( TP_RST_GPIO_Port, TP_RST_Pin, GPIO_PIN_RESET );
  HAL_Delay( 1 );  // TODO: delay(50);
  HAL_GPIO_WritePin( TP_RST_GPIO_Port, TP_RST_Pin, GPIO_PIN_SET );
  HAL_Delay( 1 );  // TODO: delay(50);

  HAL_GPIO_WritePin( TP_WAKE_GPIO_Port, TP_WAKE_Pin, GPIO_PIN_SET );
  HAL_Delay( 1 );  // TODO: delay(50);
  HAL_GPIO_WritePin( TP_WAKE_GPIO_Port, TP_WAKE_Pin, GPIO_PIN_RESET );
  HAL_Delay( 1 );  // TODO: delay(50);
  HAL_GPIO_WritePin( TP_WAKE_GPIO_Port, TP_WAKE_Pin, GPIO_PIN_SET );
  HAL_Delay( 1 );  // TODO: delay(20);
  touch_chipClearRegs();
  touch_chipReset();
  touch_loadFirmware();
  touch_chipReset();
  touch_chipStartup();
}

//GSLX680 Clear reg
static void touch_chipClearRegs()
{
  uint8_t Wrbuf[ 5 ];

  Wrbuf[ 0 ] = 0xe0;
  Wrbuf[ 1 ] = 0x88;
  HAL_I2C_Master_Transmit( &hi2c3, WRITE_ADD, Wrbuf, 2, 1000 );
  HAL_Delay( 1 );  // TODO: delay(200);
  Wrbuf[ 0 ] = 0x80;
  Wrbuf[ 1 ] = 0x01;
  HAL_I2C_Master_Transmit( &hi2c3, WRITE_ADD, Wrbuf, 2, 1000 );
  HAL_Delay( 1 );  // TODO: delay(50);
  Wrbuf[ 0 ] = 0xe4;
  Wrbuf[ 1 ] = 0x04;
  HAL_I2C_Master_Transmit( &hi2c3, WRITE_ADD, Wrbuf, 2, 1000 );
  HAL_Delay( 1 );  // TODO: delay(50);
  Wrbuf[ 0 ] = 0xe0;
  Wrbuf[ 1 ] = 0x00;
  HAL_I2C_Master_Transmit( &hi2c3, WRITE_ADD, Wrbuf, 2, 1000 );
  HAL_Delay( 1 );  // TODO: delay(50);
}

//_GSLX680 Reset
static void touch_chipReset()
{
  uint8_t Wrbuf[ 5 ];
  Wrbuf[ 0 ] = 0xe0;
  Wrbuf[ 1 ] = 0x88;
  HAL_I2C_Master_Transmit( &hi2c3, WRITE_ADD, Wrbuf, 2, 10 );
  HAL_Delay( 1 );  // TODO: delay(50);
  Wrbuf[ 0 ] = 0xe4;
  Wrbuf[ 1 ] = 0x04;
  HAL_I2C_Master_Transmit( &hi2c3, WRITE_ADD, Wrbuf, 2, 10 );
  HAL_Delay( 1 );  // TODO: delay(50);
  Wrbuf[ 0 ] = 0xbc;
  Wrbuf[ 1 ] = 0x00;
  Wrbuf[ 2 ] = 0x00;
  Wrbuf[ 3 ] = 0x00;
  Wrbuf[ 4 ] = 0x00;
  HAL_I2C_Master_Transmit( &hi2c3, WRITE_ADD, Wrbuf, 5, 10 );
  HAL_Delay( 1 );  // TODO: delay(50);
}

//GSLX680 Main Down
static void touch_loadFirmware()
{
  uint8_t Wrbuf[ 5 ];
  uint32_t source_line = 0;
  uint32_t source_len = sizeof( GSLX680_FW ) / sizeof( struct fw_data );

  for ( source_line = 0; source_line < source_len; source_line++ )
  {
    Wrbuf[ 0 ] = GSLX680_FW[ source_line ].offset;
    Wrbuf[ 1 ] = (char)( GSLX680_FW[ source_line ].val & 0x000000ff );
    Wrbuf[ 2 ] = (char)( ( GSLX680_FW[ source_line ].val & 0x0000ff00 ) >> 8 );
    Wrbuf[ 3 ] = (char)( ( GSLX680_FW[ source_line ].val & 0x00ff0000 ) >> 16 );
    Wrbuf[ 4 ] = (char)( ( GSLX680_FW[ source_line ].val & 0xff000000 ) >> 24 );

    HAL_I2C_Master_Transmit( &hi2c3, WRITE_ADD, Wrbuf, 5, 10 );
  }
}

//startup chip
static void touch_chipStartup()
{
  uint8_t Wrbuf[ 5 ];
  Wrbuf[ 0 ] = 0xe0;
  Wrbuf[ 1 ] = 0x00;
  HAL_I2C_Master_Transmit( &hi2c3, WRITE_ADD, Wrbuf, 2, 10 );
}
