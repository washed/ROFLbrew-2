#include "stm32f7xx_hal.h"
#include "lcd_touch/SSD1963.h"

volatile uint16_t* lcd_command = (volatile uint16_t*)LCD_COMMAND_ADDR;
volatile uint16_t* lcd_data = (volatile uint16_t*)LCD_DATA_ADDR;

static inline void lcd_writeCommand( unsigned char c )
{
  *lcd_command = (uint16_t)c;
}

static inline void lcd_writeDataBytes( uint8_t DH, uint8_t DL )
{
  *lcd_data = ( uint16_t )( ( DH << 8 ) | DL );
}

static inline void lcd_writeDataShort( uint16_t d )
{
  *lcd_data = d;
}

void lcd_init()
{
  // SSD1963 initialization

  HAL_GPIO_WritePin( LCD_nRST_GPIO_Port, LCD_nRST_Pin, GPIO_PIN_RESET );
  HAL_Delay( 10 );
  HAL_GPIO_WritePin( LCD_nRST_GPIO_Port, LCD_nRST_Pin, GPIO_PIN_SET );
  HAL_Delay( 10 );

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

  // Backlight PWM off
  lcd_writeCommand( 0x00BE );    //set PWM for B/L
  lcd_writeDataShort( 0x0000 );  //pwm frequency
  lcd_writeDataShort( 0x0000 );  //pwm duty cycle
  lcd_writeDataShort( 0x0000 );  // Bit 3: 0 := host; 1 := DBC Bit 0: pwm enable
  lcd_writeDataShort( 0x0000 );  // DBC manual brightness
  lcd_writeDataShort( 0x0000 );  // DBC minimum brightness
  lcd_writeDataShort( 0x0000 );  // brightness transition prescaler

  // DBC off
  lcd_writeCommand( 0x00d0 );
  lcd_writeDataShort( 0x0000 );

  // TE on
  lcd_writeCommand( 0x0035 );  // set tear effect output enable
  lcd_writeDataShort( 0x0000 );

  lcd_writeCommand( 0x0029 );  //display on
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

void pset( uint16_t x, uint16_t y, uint16_t col )
{
  lcd_setPosition( x, x + 1, y, y + 1 );
  *lcd_data = col;
}

void lcd_waitForVSync()
{
#ifdef LCD_WAIT_FOR_VSYNC
  // Wait for vertical sync
  while ( HAL_GPIO_ReadPin( LCD_TE_GPIO_Port, LCD_TE_Pin ) == GPIO_PIN_RESET )
    ;
  while ( HAL_GPIO_ReadPin( LCD_TE_GPIO_Port, LCD_TE_Pin ) == GPIO_PIN_SET )
    ;
#endif
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
