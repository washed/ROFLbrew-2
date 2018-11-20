/*
 * display.h
 *
 *  Created on: 23.10.2016
 *      Author: washed
 */

#ifndef DISPLAY_H_
#define DISPLAY_H_

// ST HAL top include
#include "stm32f7xx_hal.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define USE_LCD_DMA
#define LCD_HAL_DMA_INSTANCE &hdma_memtomem_dma2_stream0

  typedef struct RgbColor
  {
    unsigned char r;
    unsigned char g;
    unsigned char b;
  } RgbColor;

  typedef struct HsvColor
  {
    unsigned char h;
    unsigned char s;
    unsigned char v;
  } HsvColor;

  osThreadId createTaskDisplayUpdate();
  void handleDisplayUpdate();

  void setDisplayBacklightFade( uint32_t brightness, uint32_t stepsize );
  void setDisplayBacklight( uint32_t brightness );
  uint8_t lcd_fillFrame( uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color );

  RgbColor HsvToRgb( HsvColor hsv );
  HsvColor RgbToHsv( RgbColor rgb );

#ifdef __cplusplus
}
#endif

#endif /* DISPLAY_H_ */
