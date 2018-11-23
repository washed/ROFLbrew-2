/*
 * display.c
 *
 *  Created on: 23.10.2016
 *      Author: washed
 */

#include "stm32f7xx_hal.h"
#include "tim.h"
#include "gui/ugui.h"
#include "gui/roflbrew_gui.h"
#include "lcd_touch/touch.h"
#include "lcd_touch/display.h"
#include "lcd_touch/SSD1963.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include <math.h>
#include <stdio.h>
#ifdef USE_LCD_DMA
#include "dma.h"
#endif

#if defined( __ENABLE_SYSVIEW )
#include "SEGGER_SYSVIEW.h"
#include "SEGGER_SYSVIEW_Conf.h"
#endif

osThreadId createTaskDisplayBacklightUpdate();

#define STACK_SIZE 0x2000
uint32_t displayUpdateTaskBuffer[ STACK_SIZE ];
osStaticThreadDef_t displayUpdateControlBlock;

#define displayBacklightUpdateStackSize 0x1000
#define BACKLIGHT_Q_ITEMS 10
uint32_t displayBacklightUpdateTaskBuffer[ displayBacklightUpdateStackSize ];
osStaticThreadDef_t displayBacklightUpdateControlBlock;
uint8_t backlightQueueBuffer[ BACKLIGHT_Q_ITEMS * sizeof( fade_def_t ) ];
osStaticMessageQDef_t backlightQueueControlblock;
osMessageQId backlightQueueHandle;
#define BACKLIGHT_UPDATE_PERIOD 5

uint32_t current_brightness = LCD_MIN_BRIGHTNESS;
int32_t stepsize_brightness = 0;

fade_def_t lcd_backlight_fade;

void vTaskDisplayUpdate( void* pvParameters )
{
  uint32_t PreviousWakeTime = osKernelSysTick();
  for ( ;; )
  {
    handleDisplayUpdate();
    osDelayUntil( &PreviousWakeTime, 20 );
  }

  // We should never get here
  vTaskDelete( NULL );
}

osThreadId createTaskDisplayUpdate()
{
  lcd_init();    // LCD initialization
  touch_init();  // Touch controller init
  gui_init();

  // Create the main display update thread
  osThreadStaticDef( displayUpdate, vTaskDisplayUpdate, osPriorityNormal, 0, STACK_SIZE, displayUpdateTaskBuffer,
                     &displayUpdateControlBlock );
  osThreadId id = osThreadCreate( osThread( displayUpdate ), NULL );

  // Create the display backlight update thread
  createTaskDisplayBacklightUpdate();

  return id;
}

static inline void update_backlight_step()
{
  float current_brightness_f = (float)current_brightness;
  float target_brightness_f = (float)lcd_backlight_fade.brightness;
  float fade_time_f = lcd_backlight_fade.fade_time;
  float period_f = (float)BACKLIGHT_UPDATE_PERIOD;

  switch ( lcd_backlight_fade.curve )
  {
    case FADE_CURVE_LINEAR:
      if ( stepsize_brightness == 0 )
      {
        stepsize_brightness =
            floorl( ( target_brightness_f - current_brightness_f ) / ( fade_time_f / period_f ) + 0.5f );
      }

      if ( lcd_backlight_fade.brightness == current_brightness )
      {
        stepsize_brightness = 0;
      }
      break;

    case FADE_CURVE_EXP:
      if ( target_brightness_f != current_brightness_f )
      {
        stepsize_brightness = floorf( ( ( target_brightness_f - current_brightness_f ) / 50.0f ) + 0.5f ) + 1;
      }
      else
      {
        stepsize_brightness = 0;
      }
      break;

    default:
      stepsize_brightness = 0;
      break;
  }
}

void vTaskDisplayBacklightUpdate( void* pvParameters )
{
  lcd_backlight_fade.brightness = LCD_MAX_BRIGHTNESS;
  lcd_backlight_fade.fade_time = LCD_DEFAULT_FADE_TIME;
  lcd_backlight_fade.curve = FADE_CURVE_EXP;
  htim4.Instance->CCR1 = current_brightness;

  // Start PWM generation for backlight control
  HAL_TIM_PWM_Start( &htim4, TIM_CHANNEL_1 );

  uint32_t PreviousWakeTime = osKernelSysTick();
  for ( ;; )
  {
    if ( pdPASS == xQueueReceive( backlightQueueHandle, &lcd_backlight_fade, 0 ) )
    {
      stepsize_brightness = 0;
    }

    update_backlight_step();

    current_brightness += stepsize_brightness;

    if ( current_brightness > LCD_MAX_BRIGHTNESS )
    {
      current_brightness = LCD_MAX_BRIGHTNESS;
    }
    else if ( current_brightness < LCD_MIN_BRIGHTNESS )
    {
      current_brightness = LCD_MIN_BRIGHTNESS;
    }

    htim4.Instance->CCR1 = current_brightness;

    osDelayUntil( &PreviousWakeTime, BACKLIGHT_UPDATE_PERIOD );
  }

  // We should never get here
  vTaskDelete( NULL );
}

osThreadId createTaskDisplayBacklightUpdate()
{
  osThreadStaticDef( displayBacklightUpdate, vTaskDisplayBacklightUpdate, osPriorityNormal, 0,
                     displayBacklightUpdateStackSize, displayBacklightUpdateTaskBuffer,
                     &displayBacklightUpdateControlBlock );
  osThreadId id = osThreadCreate( osThread( displayBacklightUpdate ), NULL );

  osMessageQStaticDef( backlightQ, BACKLIGHT_Q_ITEMS, fade_def_t, backlightQueueBuffer, &backlightQueueControlblock );
  backlightQueueHandle = osMessageCreate( &os_messageQ_def_backlightQ, NULL );

  return id;
}

void handleDisplayUpdate()
{
  if ( touchEvent.touch_count > 0 )
    UG_TouchUpdate( touchEvent.x1, touchEvent.y1, TOUCH_STATE_PRESSED );
  else
    UG_TouchUpdate( -1, -1, TOUCH_STATE_RELEASED );

  gui_update();
  //lcd_waitForVSync();
  UG_Update();
}

void setDisplayBacklightFade( uint32_t brightness, uint32_t fade_time, fade_curve_t curve )
{
  if ( ( brightness >= LCD_MIN_BRIGHTNESS ) && ( brightness <= LCD_MAX_BRIGHTNESS ) )
  {
    fade_def_t fade_def;

    fade_def.brightness = brightness;
    fade_def.fade_time = fade_time;
    fade_def.curve = 0;

    // TODO: Make it possible to call this from an interrupt context
    xQueueSend( backlightQueueHandle, &fade_def, 0 );
  }
}

uint8_t lcd_fillFrame( uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color )
{
  uint16_t dx = x2 - x1 + 1;
  uint16_t dy = y2 - y1 + 1;
  uint32_t total_pixel_count = dx * dy;
  uint16_t tx_pixel_count;

  // Invalidate cache for the "color" parameter, because DMA will bypass the cache and may otherwise write invalid data
  SCB_CleanInvalidateDCache_by_Addr( (uint32_t*)&color, sizeof( color ) );

  // Wait for vertical sync
  while ( HAL_GPIO_ReadPin( LCD_TE_GPIO_Port, LCD_TE_Pin ) == GPIO_PIN_RESET )
    ;
  while ( HAL_GPIO_ReadPin( LCD_TE_GPIO_Port, LCD_TE_Pin ) == GPIO_PIN_SET )
    ;

  // Set write area
  lcd_setPosition( x1, x2, y1, y2 );

#ifdef USE_LCD_DMA
  while ( HAL_DMA_GetState( LCD_HAL_DMA_INSTANCE ) != HAL_DMA_STATE_READY )
    ;

  while ( total_pixel_count )
  {
    if ( total_pixel_count > PIXEL_COUNT_PER_DMA )
      tx_pixel_count = PIXEL_COUNT_PER_DMA;
    else
      tx_pixel_count = total_pixel_count;

    // Start new DMA Transfer
    while ( HAL_OK != HAL_DMA_Start_IT( LCD_HAL_DMA_INSTANCE, (uint32_t)&color, (uint32_t)0x60020000, tx_pixel_count ) )
      ;

    // Wait for DMA ready
    while ( HAL_DMA_GetState( LCD_HAL_DMA_INSTANCE ) != HAL_DMA_STATE_READY )
      ;
    total_pixel_count -= tx_pixel_count;
  }
#else
  for ( uint32_t current_pixel = 0; current_pixel < total_pixel_count; current_pixel++ )
  {
    *lcd_data = color;
  }
#endif
  return 0;
}

#if false
uint8_t lcd_fillArea( uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color )
{
#if USE_LCD_DMA
  uint16_t dx = x2 - x1 + 1;
  uint16_t dy = y2 - y1 + 1;
  uint32_t total_pixel_count = dx * dy;
  uint16_t tx_pixel_count;

  // Invalidate cache for the "color" parameter, because DMA will bypass the cache and may otherwise write invalid data
  SCB_CleanInvalidateDCache_by_Addr((uint32_t*)&color, sizeof(color));

  // Wait for vertical sync
#define WAIT_FOR_VSYNC
#ifdef WAIT_FOR_VSYNC
  while ( HAL_GPIO_ReadPin( LCD_TE_GPIO_Port, LCD_TE_Pin ) == GPIO_PIN_RESET )
    ;
  while ( HAL_GPIO_ReadPin( LCD_TE_GPIO_Port, LCD_TE_Pin ) == GPIO_PIN_SET )
    ;
#endif

  // Set write area
  lcd_setPosition( x1, x2, y1, y2 );

  while ( HAL_DMA_GetState( LCD_HAL_DMA_INSTANCE ) != HAL_DMA_STATE_READY )
    ;

  while ( total_pixel_count )
  {
    if ( total_pixel_count > PIXEL_COUNT_PER_DMA )
      tx_pixel_count = PIXEL_COUNT_PER_DMA;
    else
      tx_pixel_count = total_pixel_count;

    // Start new DMA Transfer
    HAL_DMA_Start_IT( LCD_HAL_DMA_INSTANCE, (uint32_t)&color, (uint32_t)0x60020000, tx_pixel_count );
    // Wait for DMA ready
    while ( HAL_DMA_GetState( LCD_HAL_DMA_INSTANCE ) != HAL_DMA_STATE_READY )
      ;
    total_pixel_count -= tx_pixel_count;
  }
  return 0;
#else
  // TODO: Non DMA fill frame function here pls
#endif
}
#endif

// TODO: Put this color stuff somewhere else
RgbColor HsvToRgb( HsvColor hsv )
{
  RgbColor rgb;
  unsigned char region, p, q, t;
  unsigned int h, s, v, remainder;

  if ( hsv.s == 0 )
  {
    rgb.r = hsv.v;
    rgb.g = hsv.v;
    rgb.b = hsv.v;
    return rgb;
  }

  // converting to 16 bit to prevent overflow
  h = hsv.h;
  s = hsv.s;
  v = hsv.v;

  region = h / 43;
  remainder = ( h - ( region * 43 ) ) * 6;

  p = ( v * ( 255 - s ) ) >> 8;
  q = ( v * ( 255 - ( ( s * remainder ) >> 8 ) ) ) >> 8;
  t = ( v * ( 255 - ( ( s * ( 255 - remainder ) ) >> 8 ) ) ) >> 8;

  switch ( region )
  {
    case 0:
      rgb.r = v;
      rgb.g = t;
      rgb.b = p;
      break;
    case 1:
      rgb.r = q;
      rgb.g = v;
      rgb.b = p;
      break;
    case 2:
      rgb.r = p;
      rgb.g = v;
      rgb.b = t;
      break;
    case 3:
      rgb.r = p;
      rgb.g = q;
      rgb.b = v;
      break;
    case 4:
      rgb.r = t;
      rgb.g = p;
      rgb.b = v;
      break;
    default:
      rgb.r = v;
      rgb.g = p;
      rgb.b = q;
      break;
  }

  return rgb;
}

HsvColor RgbToHsv( RgbColor rgb )
{
  HsvColor hsv;
  unsigned char rgbMin, rgbMax;

  rgbMin = rgb.r < rgb.g ? ( rgb.r < rgb.b ? rgb.r : rgb.b ) : ( rgb.g < rgb.b ? rgb.g : rgb.b );
  rgbMax = rgb.r > rgb.g ? ( rgb.r > rgb.b ? rgb.r : rgb.b ) : ( rgb.g > rgb.b ? rgb.g : rgb.b );

  hsv.v = rgbMax;
  if ( hsv.v == 0 )
  {
    hsv.h = 0;
    hsv.s = 0;
    return hsv;
  }

  hsv.s = 255 * ( (long)( rgbMax - rgbMin ) ) / hsv.v;
  if ( hsv.s == 0 )
  {
    hsv.h = 0;
    return hsv;
  }

  if ( rgbMax == rgb.r )
    hsv.h = 0 + 43 * ( rgb.g - rgb.b ) / ( rgbMax - rgbMin );
  else if ( rgbMax == rgb.g )
    hsv.h = 85 + 43 * ( rgb.b - rgb.r ) / ( rgbMax - rgbMin );
  else
    hsv.h = 171 + 43 * ( rgb.r - rgb.g ) / ( rgbMax - rgbMin );

  return hsv;
}
