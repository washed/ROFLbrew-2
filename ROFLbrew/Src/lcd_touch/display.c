/*
 * display.c
 *
 *  Created on: 23.10.2016
 *      Author: washed
 */

#include "tim.h"
#include "gui/ugui.h"
#include "gui/roflbrew_gui.h"
#include "lcd_touch/touch.h"
#include "lcd_touch/display.h"
#if USE_LCD_DMA
#include "dma.h"
#endif

#if defined( __ENABLE_SYSVIEW )
#include "SEGGER_SYSVIEW.h"
#include "SEGGER_SYSVIEW_Conf.h"
#endif

volatile uint32_t update_display = 0;

volatile uint16_t* lcd_command = (volatile uint16_t*)LCD_COMMAND_ADDR;
volatile uint16_t* lcd_data = (volatile uint16_t*)LCD_DATA_ADDR;

// Private functions
static void lcd_writeCommand( uint8_t c );
static void lcd_writeDataBytes( uint8_t DH, uint8_t DL );
static void lcd_writeDataShort( uint16_t d );

static uint32_t target_brightness = 1000;
static uint32_t current_brightness = 0;
static uint32_t stepsize_brightness = LCD_STEPSIZE_BRIGHTNESS;

void handleDisplayUpdate()
{
  if ( touchEvent.touch_count > 0 )
    UG_TouchUpdate( touchEvent.x1, touchEvent.y1, TOUCH_STATE_PRESSED );
  else
    UG_TouchUpdate( -1, -1, TOUCH_STATE_RELEASED );

  if ( update_display )
  {
#if defined( __ENABLE_SYSVIEW )
    SEGGER_SYSVIEW_OnTaskStartExec( SYSVIEW_TASK_LCD_UPDATE );
#endif

    if ( target_brightness > current_brightness )
      current_brightness += stepsize_brightness;
    else if ( target_brightness < current_brightness )
      current_brightness -= stepsize_brightness;

    if ( current_brightness > LCD_MAX_BRIGHTNESS )
      current_brightness = LCD_MAX_BRIGHTNESS;
    else if ( current_brightness < LCD_MIN_BRIGHTNESS )
      current_brightness = LCD_MIN_BRIGHTNESS;

    setDisplayBacklight( current_brightness );

    gui_update();
    //lcd_waitForVSync();
    UG_Update();

    update_display = 0;

#if defined( __ENABLE_SYSVIEW )
    SEGGER_SYSVIEW_OnTaskStopReady( SYSVIEW_TASK_LCD_UPDATE, 0 );
#endif
  }
}

void setDisplayBacklightFade( uint32_t brightness, uint32_t stepsize )
{
  if ( ( brightness >= 0 ) && ( brightness <= 1000 ) ) target_brightness = brightness;
  stepsize_brightness = stepsize;
}

void setDisplayBacklight( uint32_t brightness )
{
  if ( ( brightness >= 0 ) && ( brightness <= 1000 ) ) htim4.Instance->CCR1 = brightness;
}

void pset( uint16_t x, uint16_t y, uint16_t col )
{
  lcd_setPosition( x, x + 1, y, y + 1 );
  *lcd_data = col;
}

void lcd_init()
{
  // SSD1963 initialization

  HAL_GPIO_WritePin( LCD_nRST_GPIO_Port, LCD_nRST_Pin, GPIO_PIN_RESET );
  HAL_Delay( 100 );
  HAL_GPIO_WritePin( LCD_nRST_GPIO_Port, LCD_nRST_Pin, GPIO_PIN_SET );
  HAL_Delay( 100 );

  lcd_writeCommand( 0x00E2 );    //PLL multiplier, set PLL clock to 120M
  lcd_writeDataShort( 0x0023 );  //N=0x36 for 6.5M, 0x23 for 10M crystal
  lcd_writeDataShort( 0x0002 );
  lcd_writeDataShort( 0x0054 );
  lcd_writeCommand( 0x00E0 );  // PLL enable
  lcd_writeDataShort( 0x0001 );
  HAL_Delay( 1 );  //TODO: 10us
  lcd_writeCommand( 0x00E0 );
  lcd_writeDataShort( 0x0003 );  // now, use PLL output as system clock
  HAL_Delay( 1 );                //TODO: 10us
  lcd_writeCommand( 0x0001 );    // software reset
  HAL_Delay( 1 );                //TODO: 20us
  lcd_writeCommand( 0x00E6 );    //PLL setting for PCLK, depends on resolution
  lcd_writeDataShort( 0x0003 );
  lcd_writeDataShort( 0x0033 );
  lcd_writeDataShort( 0x0033 );

  lcd_writeCommand( 0x00B0 );          //LCD SPECIFICATION
  lcd_writeDataShort( 0x0020 );        //24 bit TFT panel
  lcd_writeDataShort( 0x0000 );        //Hsync+Vsync +DE mode  TFT mode
  lcd_writeDataShort( ( 799 >> 8 ) );  //Set HDP
  lcd_writeDataShort( 799 );
  lcd_writeDataShort( 479 >> 8 );  //Set VDP
  lcd_writeDataShort( 479 );
  lcd_writeDataShort( 0x0000 );

  lcd_writeCommand( 0x00B4 );  //HSYNC
  lcd_writeDataShort( 0x04 );  //Set HT
  lcd_writeDataShort( 0x1f );
  lcd_writeDataShort( 0x00 );  //Set HPS
  lcd_writeDataShort( 0xd2 );
  lcd_writeDataShort( 0x00 );  //Set HPW
  lcd_writeDataShort( 0x00 );  //Set HPS
  lcd_writeDataShort( 0x00 );
  lcd_writeDataShort( 0x00 );

  lcd_writeCommand( 0x00B6 );  //VSYNC
  lcd_writeDataShort( 0x02 );  //Set VT
  lcd_writeDataShort( 0x0c );
  lcd_writeDataShort( 0x00 );  //Set VPS
  lcd_writeDataShort( 0x22 );
  lcd_writeDataShort( 0x00 );  //Set VPW
  lcd_writeDataShort( 0x00 );  //Set FPS
  lcd_writeDataShort( 0x00 );

  lcd_writeCommand( 0x00B8 );
  lcd_writeDataShort( 0x000f );  //GPIO is controlled by host GPIO[3:0]=output   GPIO[0]=1  LCD ON  GPIO[0]=1  LCD OFF
  lcd_writeDataShort( 0x0001 );  //GPIO0 normal

  lcd_writeCommand( 0x00BA );
  lcd_writeDataShort( 0x0001 );  //GPIO[0] out 1 --- LCD display on/off control PIN

  lcd_writeCommand( 0x0036 );    //rotation
  lcd_writeDataShort( 0x0008 );  //RGB=BGR

  lcd_writeCommand( 0x003A );    //Set the current pixel format for RGB image data
  lcd_writeDataShort( 0x0050 );  //16-bit/pixel

  lcd_writeCommand( 0x00F0 );    //Pixel Data Interface Format
  lcd_writeDataShort( 0x0003 );  //16-bit(565 format) data

  lcd_writeCommand( 0x00BC );
  lcd_writeDataShort( 0x0040 );  //contrast value
  lcd_writeDataShort( 0x0080 );  //brightness value
  lcd_writeDataShort( 0x0040 );  //saturation value
  lcd_writeDataShort( 0x0001 );  //Post Processor Enable

  HAL_Delay( 1 );  //TODO: 5us

  lcd_writeCommand( 0x0029 );  //display on
  /*
	 lcd_writeCommand(0x00BE); //set PWM for B/L
	 lcd_writeDataShort(0x0006);
	 lcd_writeDataShort(0x0080);
	 lcd_writeDataShort(0x0001);
	 lcd_writeDataShort(0x0002);
	 lcd_writeDataShort(0x0000);
	 lcd_writeDataShort(0x0000);
	 */
  lcd_writeCommand( 0x00d0 );
  lcd_writeDataShort( 0x000d );  // 0x000d

  lcd_writeCommand( 0x00BE );    //set PWM for B/L
  lcd_writeDataShort( 0x0006 );  //pwm frequency
  lcd_writeDataShort( 0x0010 );  //pwm duty cycle
  lcd_writeDataShort( 0x0009 );  // Bit 3: 0 := host; 1 := DBC Bit 0: pwm enable
  lcd_writeDataShort( 0x0002 );  // DBC manual brightness
  lcd_writeDataShort( 0x0000 );  // DBC minimum brightness
  lcd_writeDataShort( 0x0000 );  // brightness transition prescaler

  lcd_writeCommand( 0x0035 );  // set tear effect output enable
  lcd_writeDataShort( 0x0000 );
}

void lcd_setPosition( unsigned int xs, unsigned int xe, unsigned int ys, unsigned int ye )
{
  lcd_writeCommand( 0x002A );
  lcd_writeDataShort( xs >> 8 );
  lcd_writeDataShort( xs & 0x00ff );
  lcd_writeDataShort( xe >> 8 );
  lcd_writeDataShort( xe & 0x00ff );
  lcd_writeCommand( 0x002b );
  lcd_writeDataShort( ys >> 8 );
  lcd_writeDataShort( ys & 0x00ff );
  lcd_writeDataShort( ye >> 8 );
  lcd_writeDataShort( ye & 0x00ff );
  lcd_writeCommand( 0x002c );
}

void lcd_waitForVSync()
{
  // Wait for vertical sync
  while ( HAL_GPIO_ReadPin( LCD_TE_GPIO_Port, LCD_TE_Pin ) == GPIO_PIN_RESET )
    ;
  while ( HAL_GPIO_ReadPin( LCD_TE_GPIO_Port, LCD_TE_Pin ) == GPIO_PIN_SET )
    ;
}

void lcd_clear( unsigned int i )
{
#if USE_LCD_DMA

  lcd_setPosition( 0, 799, 0, 479 );
  // Wait for DMA ready
  while ( HAL_DMA_GetState( LCD_HAL_DMA_INSTANCE ) != HAL_DMA_STATE_READY )
    ;
  lcd_waitForVSync();
  for ( uint32_t j = 0; j < 6; j++ )
  {
    // Start new DMA Transfer
    HAL_DMA_Start_IT( LCD_HAL_DMA_INSTANCE, (uint32_t)&i, (uint32_t)0x60020000, 64000 );
    // Wait for DMA ready
    while ( HAL_DMA_GetState( LCD_HAL_DMA_INSTANCE ) != HAL_DMA_STATE_READY )
      ;
  }
#else
  unsigned int l = 480, w;
  lcd_setPosition( 0, 799, 0, 479 );
  while ( l-- )
  {
    for ( w = 0; w < 800; w++ )
    {
      Write_Data_int( i );
    }
  }
#endif
}

void lcd_showBMP( unsigned char p[] )  //200*120
{
  // TODO: Improve to use DMA, variable image size etc
  unsigned int i, w, l, x1, x2, y1, y2, xa, xb, ya, yb;
  unsigned char k, m;
  xa = 0;
  xb = 199;
  ya = 0;
  yb = 119;
  i = 0;
  for ( m = 0; m < 4; m++ )
  {
    for ( k = 0; k < 4; k++ )
    {
      x1 = xa;
      x2 = xb;
      y1 = ya;
      y2 = yb;
      lcd_setPosition( x1, x2, y1, y2 );
      for ( l = 0; l < 120; l++ )
      {
        for ( w = 0; w < 200; w++ )
        {  //  temp=p[i];  temp1=p[i+1];  temp=temp<<8;temp=temp|temp1 ;
           //	Write_Data_int(temp);  i+=1;
           //	Write_Data_int(p[i++]);
          lcd_writeDataBytes( p[ i ], p[ i + 1 ] );
          i += 2;
          //Write_Data_byte((p[i])>>8,p[i]);
          //	i+=1;
        }
      }
      xa += 200;
      xb += 200;
      i = 0;
    }
    xa = 0;
    xb = 199;
    ya += 120;
    yb += 120;
  }
}

uint8_t lcd_fillFrame( uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color )
{

  uint16_t dx = x2 - x1 + 1;
  uint16_t dy = y2 - y1 + 1;
  uint32_t total_pixel_count = dx * dy;
  uint16_t tx_pixel_count;

  // Invalidate cache for the "color" parameter, because DMA will bypass the cache and may otherwise write invalid data
  SCB_CleanInvalidateDCache_by_Addr((uint32_t*)&color, sizeof(color));

  // Wait for vertical sync
  while ( HAL_GPIO_ReadPin( LCD_TE_GPIO_Port, LCD_TE_Pin ) == GPIO_PIN_RESET )
    ;
  while ( HAL_GPIO_ReadPin( LCD_TE_GPIO_Port, LCD_TE_Pin ) == GPIO_PIN_SET )
    ;

  // Set write area
  lcd_setPosition( x1, x2, y1, y2 );

#if USE_LCD_DMA
  while ( HAL_DMA_GetState( LCD_HAL_DMA_INSTANCE ) != HAL_DMA_STATE_READY )
    ;

  while ( total_pixel_count )
  {
    if ( total_pixel_count > PIXEL_COUNT_PER_DMA )
      tx_pixel_count = PIXEL_COUNT_PER_DMA;
    else
      tx_pixel_count = total_pixel_count;

    // Start new DMA Transfer
    while ( HAL_OK != HAL_DMA_Start_IT( LCD_HAL_DMA_INSTANCE, (uint32_t)&color, (uint32_t)0x60020000, tx_pixel_count ));

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

static void lcd_writeCommand( unsigned char c )
{
  *lcd_command = (uint16_t)c;
}

static void lcd_writeDataBytes( uint8_t DH, uint8_t DL )
{
  *lcd_data = ( uint16_t )( ( DH << 8 ) | DL );
}

static void lcd_writeDataShort( uint16_t d )
{
  *lcd_data = d;
}

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
