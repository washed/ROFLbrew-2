/*
 * touch.h
 *
 *  Created on: 23.10.2016
 *      Author: washed
 */

#ifndef TOUCH_H_
#define TOUCH_H_

#define WRITE_ADD 0x80
#define READ_ADD 0x81

typedef struct TouchEvent_TypeDef
{
  uint16_t x1;
  uint16_t y1;
  uint16_t x2;
  uint16_t y2;
  uint16_t x3;
  uint16_t y3;
  uint16_t x4;
  uint16_t y4;
  uint16_t x5;
  uint16_t y5;
  uint32_t touch_count;
  uint8_t touch_point;
  uint8_t Key_Sta;
} TouchEvent_TypeDef;

typedef struct fw_data
{
  uint8_t offset;
  uint32_t val;

} fw_data;

extern TouchEvent_TypeDef touchEvent;

uint8_t touch_readTouchData();
void touch_init();

#endif /* TOUCH_H_ */
