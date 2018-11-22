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

#define LCD_MAX_BRIGHTNESS 1000
#define LCD_MIN_BRIGHTNESS 0
#define LCD_DEFAULT_FADE_TIME 1000  // ms

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

  typedef enum fade_curve_t
  {
    FADE_CURVE_LINEAR = 0,
    FADE_CURVE_EXP = 1
  } fade_curve_t;

  typedef struct fade_def_t
  {
    uint32_t brightness;
    uint32_t fade_time;
    fade_curve_t curve;
  } fade_def_t;

  osThreadId createTaskDisplayUpdate();
  void handleDisplayUpdate();

  void setDisplayBacklightFade( uint32_t brightness, uint32_t fade_time, fade_curve_t curve );
  uint8_t lcd_fillFrame( uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color );

  RgbColor HsvToRgb( HsvColor hsv );
  HsvColor RgbToHsv( RgbColor rgb );

#ifdef __cplusplus
}
#endif

#endif /* DISPLAY_H_ */
