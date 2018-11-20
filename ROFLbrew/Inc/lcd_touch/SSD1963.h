#ifndef SSD1963_H_
#define SSD1963_H_

#ifdef __cplusplus
extern "C"
{
#endif

#define LCD_COMMAND_ADDR 0x60000000
#define LCD_DATA_ADDR 0x60020000

#define LCD_MAX_BRIGHTNESS 1000
#define LCD_MIN_BRIGHTNESS 0
#define LCD_STEPSIZE_BRIGHTNESS 100

#ifdef USE_LCD_DMA
#define PIXEL_COUNT_PER_DMA 64000
#endif

  extern volatile uint16_t* lcd_data;
  extern volatile uint16_t* lcd_command;

  void lcd_init();
  void pset( uint16_t x, uint16_t y, uint16_t col );

  void lcd_setPosition( unsigned int xs, unsigned int xe, unsigned int ys, unsigned int ye );
  void lcd_waitForVSync();
  // void lcd_clear( unsigned int i );
  // void lcd_showBMP( unsigned char p[] );
  // uint8_t lcd_fillFrame( uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color );

#ifdef __cplusplus
}
#endif
#endif /* SSD1963_H_ */
