/*
 * roflbrew_gui.c
 *
 *  Created on: 05.11.2016
 *      Author: washed
 */

#include "gui/roflbrew_gui.h"

#include <stdlib.h>
#include <string.h>
#include "lcd_touch/display.h"
#include "lcd_touch/SSD1963.h"
#include "lcd_touch/touch.h"
#include "stove.h"

#include "lvgl.h"

#include "gui/window_mainmenu.h"
#include "gui/window_statustop.h"

char rast_walltime_string[ 32 ];
char rast_time_string[ 32 ];
char rast_walltime_remaing_string[ 32 ];

static void gui_update_rast_walltime( uint8_t force_update );
static void gui_update_rast_walltime_remaining( uint8_t force_update );
static void gui_update_rast_time( uint8_t force_update );

void gui_init()
{
  lv_init();

  lv_disp_drv_t disp_drv;
  lv_disp_drv_init( &disp_drv );
  disp_drv.disp_flush = disp_flush;
  lv_disp_drv_register( &disp_drv );

  // TODO: Add hardware accelerated drivers!

  lv_indev_drv_t indev_drv;
  lv_indev_drv_init( &indev_drv );
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read = ex_tp_read;
  lv_indev_drv_register( &indev_drv );
}

void roflbrew_gui_init() {}

/**
 * @brief This is called before every GUI update. Use it to update realtime stuff
 */
void gui_update()
{
  // lv_task_handler();
}

#if 0
static void gui_update_rast_walltime( uint8_t force_update )
{
  static int32_t last_rast_walltime;
  if ( ( last_rast_walltime != temp_control0.rasten[ temp_control0.current_rast ].time_running ) || force_update )
  {
    convertSecondsToTimeString( rast_walltime_string, temp_control0.rasten[ temp_control0.current_rast ].time_running,
                                ':' );
    UG_TextboxSetText( &window_1, TXB_ID_9, rast_walltime_string );
    last_rast_walltime = temp_control0.rasten[ temp_control0.current_rast ].time_running;
  }
}

static void gui_update_rast_walltime_remaining( uint8_t force_update )
{
  static int32_t last_rast_walltime_remaining;
  int32_t rast_walltime_remaining = temp_control0.rasten[ temp_control0.current_rast ].time_running -
                                    temp_control0.rasten[ temp_control0.current_rast ].time;

  if ( ( last_rast_walltime_remaining != rast_walltime_remaining ) || force_update )
  {
    convertSecondsToTimeString( rast_walltime_remaing_string, rast_walltime_remaining, ':' );
    UG_TextboxSetText( &window_1, TXB_ID_11, rast_walltime_remaing_string );
    last_rast_walltime_remaining = rast_walltime_remaining;
  }
}

static void gui_update_rast_time( uint8_t force_update )
{
  static int32_t last_rast_time;
  if ( ( last_rast_time != temp_control0.rasten[ temp_control0.current_rast ].time ) || force_update )
  {
    convertSecondsToTimeString( rast_time_string, temp_control0.rasten[ temp_control0.current_rast ].time, ':' );
    UG_TextboxSetText( &window_1, TXB_ID_13, rast_time_string );
    last_rast_time = temp_control0.rasten[ temp_control0.current_rast ].time;
  }
}
#endif
