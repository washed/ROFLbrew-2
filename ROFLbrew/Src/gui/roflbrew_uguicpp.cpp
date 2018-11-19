/*
 * roflbrew_uguicpp.cpp
 *
 *  Created on: 02.11.2018
 *      Author: washed
 */



#ifdef __cplusplus

#include "uguicpp.hpp"
#include "lcd_touch/display.h"

void uguicpp_init()
{
  uguicpp::uguicpp the_gui{pset, 800, 480};

  the_gui.fill_frame(10, 10, 100, 100, uguicpp::colors::C_DARK_MAGENTA);
}

#endif

#ifdef __cplusplus
extern "C" {
#endif

void guicpp_init()
{
	/*
  UG_Init( &gui, (void ( * )( UG_S16, UG_S16, UG_COLOR ))pset, 800, 480 );
  // UG_DriverRegister( DRIVER_DRAW_LINE, (void*)_DMA_DrawLine );
  UG_DriverRegister( DRIVER_FILL_FRAME, (void*)lcd_fillFrame );
  UG_DriverRegister( DRIVER_FILL_AREA, (void*)lcd_fillFrame );
  // UG_DriverEnable( DRIVER_DRAW_LINE );
  UG_DriverEnable( DRIVER_FILL_FRAME );
  UG_DriverEnable( DRIVER_FILL_AREA );

  gui_init_mainwindow();

  UG_WindowShow( &window_1 );
  */

  uguicpp_init();
}

#ifdef __cplusplus
}
#endif


