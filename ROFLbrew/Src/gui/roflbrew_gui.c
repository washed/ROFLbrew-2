/*
 * roflbrew_gui.c
 *
 *  Created on: 05.11.2016
 *      Author: washed
 */

#include "gui/roflbrew_gui.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gui/ugui.h"
#include "lcd_touch/display.h"
#include "lcd_touch/SSD1963.h"
#include "stove.h"
// #include "strutils.h"
#include "temp_control.h"

// TODO: REMOVE
#include "MAX31865.h"

#define MAX_OBJECTS 255

/* GUI structure */
UG_GUI gui;

/* Window 1 */
UG_WINDOW window_1;
UG_OBJECT obj_buff_wnd_1[ MAX_OBJECTS ];
UG_BUTTON button1_1;
UG_BUTTON button1_2;
UG_BUTTON button1_3;
UG_BUTTON button1_4;
UG_BUTTON button1_5;
UG_BUTTON button1_6;

UG_TEXTBOX textbox1_1;
UG_TEXTBOX textbox1_2;
UG_TEXTBOX textbox1_3;
UG_TEXTBOX textbox1_4;
UG_TEXTBOX textbox1_5;
UG_TEXTBOX textbox1_6;
UG_TEXTBOX textbox1_7;
UG_TEXTBOX textbox1_8;

UG_TEXTBOX textbox2_1;
UG_TEXTBOX textbox2_2;
UG_TEXTBOX textbox2_3;
UG_TEXTBOX textbox2_4;
UG_TEXTBOX textbox2_5;
UG_TEXTBOX textbox2_6;
UG_TEXTBOX textbox2_7;
UG_TEXTBOX textbox2_8;

UG_TEXTBOX textbox99_1;
UG_TEXTBOX textbox99_2;

UG_BUTTON button3_1;
UG_BUTTON button3_2;
UG_BUTTON button3_3;

UG_TEXTBOX textbox3_1;
UG_TEXTBOX textbox3_2;
UG_TEXTBOX textbox3_3;

/* Window 2 */
UG_WINDOW window_2;
UG_OBJECT obj_buff_wnd_2[ MAX_OBJECTS ];
UG_BUTTON button20_1;

char is_temperature_string[ 32 ];
char temperature_rate_string[ 32 ];
char rast_string[ 32 ];
char set_temperature_string[ 32 ];
char rast_walltime_string[ 32 ];
char rast_time_string[ 32 ];
char rast_walltime_remaing_string[ 32 ];
char heat_power_string[ 32 ];
char systick_string[ 32 ];

static void window_1_callback( UG_MESSAGE* msg );
// static void window_2_callback( UG_MESSAGE* msg );
static void gui_update_is_temperature( uint8_t force_update );
static void gui_update_set_temperature( uint8_t force_update );
static void gui_update_rate( uint8_t force_update );
static void gui_update_rast( uint8_t force_update );
static void gui_update_rast_walltime( uint8_t force_update );
static void gui_update_rast_walltime_remaining( uint8_t force_update );
static void gui_update_rast_time( uint8_t force_update );
static void gui_update_stove_power( uint8_t force_update );
static void gui_update_systick( uint8_t force_update );

void gui_init()
{
  UG_Init( &gui, (void ( * )( UG_S16, UG_S16, UG_COLOR ))pset, 800, 480 );
  // UG_DriverRegister( DRIVER_DRAW_LINE, (void*)_DMA_DrawLine );
  UG_DriverRegister( DRIVER_FILL_FRAME, (void*)lcd_fillFrame );
  UG_DriverRegister( DRIVER_FILL_AREA, (void*)lcd_fillFrame );
  // UG_DriverEnable( DRIVER_DRAW_LINE );
  UG_DriverEnable( DRIVER_FILL_FRAME );
  UG_DriverEnable( DRIVER_FILL_AREA );

  gui_init_mainwindow();

  UG_WindowShow( &window_1 );
}

void gui_init_mainwindow()
{
  uint16_t inner_width, inner_height, h_fract, w_fract;

  // Create main window
  UG_WindowCreate( &window_1, obj_buff_wnd_1, MAX_OBJECTS, window_1_callback );
  UG_WindowSetTitleText( &window_1, "ROFLbrew 0.1a" );
  UG_WindowSetTitleTextFont( &window_1, &FONT_12X20 );
  UG_WindowSetTitleTextAlignment( &window_1, ALIGN_CENTER_LEFT );
  UG_WindowSetTitleColor( &window_1, C_LIGHT_GRAY );
  UG_WindowSetTitleTextColor( &window_1, C_DARK_CYAN );
  UG_WindowSetBackColor( &window_1, C_DARK_SLATE_GRAY );

  inner_width = UG_WindowGetInnerWidth( &window_1 );
  inner_height = UG_WindowGetInnerHeight( &window_1 );
  h_fract = inner_height / 6;
  w_fract = inner_width / 6;

  /********************************************************************************************************************/
  /** These are always visible */
  /********************************************************************************************************************/
  /* Create some Buttons */
  UG_ButtonCreate( &window_1, &button1_1, BTN_ID_0, 0, 0 * h_fract, 100, 1 * h_fract );
  UG_ButtonCreate( &window_1, &button1_2, BTN_ID_1, 0, 1 * h_fract, 100, 2 * h_fract );
  UG_ButtonCreate( &window_1, &button1_3, BTN_ID_2, 0, 2 * h_fract, 100, 3 * h_fract );
  UG_ButtonCreate( &window_1, &button1_4, BTN_ID_3, 0, 3 * h_fract, 100, 4 * h_fract );
  UG_ButtonCreate( &window_1, &button1_5, BTN_ID_4, 0, 4 * h_fract, 100, 5 * h_fract );
  UG_ButtonCreate( &window_1, &button1_6, BTN_ID_5, 0, 5 * h_fract, 100, 6 * h_fract );

  w_fract = ( inner_width - 100 ) / 4;

  UG_TextboxCreate( &window_1, &textbox99_1, TXB_ID_49, 100 + 3 * w_fract, 5 * h_fract, 100 + 4 * w_fract,
                    5.3 * h_fract );
  UG_TextboxCreate( &window_1, &textbox99_2, TXB_ID_50, 100 + 3 * w_fract, 5.3 * h_fract, 100 + 4 * w_fract,
                    6 * h_fract );

  UG_TextboxCreate( &window_1, &textbox1_1, TXB_ID_0, 100, 0, 100 + w_fract, 0.3 * h_fract );
  UG_TextboxCreate( &window_1, &textbox1_2, TXB_ID_1, 100, 0.3 * h_fract, 100 + w_fract, h_fract );

  UG_TextboxCreate( &window_1, &textbox1_3, TXB_ID_2, 100 + w_fract, 0, 100 + 2 * w_fract, 0.3 * h_fract );
  UG_TextboxCreate( &window_1, &textbox1_4, TXB_ID_3, 100 + w_fract, 0.3 * h_fract, 100 + 2 * w_fract, h_fract );

  UG_TextboxCreate( &window_1, &textbox1_5, TXB_ID_4, 100 + 2 * w_fract, 0, 100 + 3 * w_fract, 0.3 * h_fract );
  UG_TextboxCreate( &window_1, &textbox1_6, TXB_ID_5, 100 + 2 * w_fract, 0.3 * h_fract, 100 + 3 * w_fract, h_fract );

  UG_TextboxCreate( &window_1, &textbox1_7, TXB_ID_6, 100 + 3 * w_fract, 0, 100 + 4 * w_fract, 0.3 * h_fract );
  UG_TextboxCreate( &window_1, &textbox1_8, TXB_ID_7, 100 + 3 * w_fract, 0.3 * h_fract, 100 + 4 * w_fract, h_fract );

  /* Configure Button 1 */
  UG_ButtonSetFont( &window_1, BTN_ID_0, &FONT_8X14 );
  UG_ButtonSetBackColor( &window_1, BTN_ID_0, C_LIGHT_SLATE_GRAY );
  UG_ButtonSetForeColor( &window_1, BTN_ID_0, C_LIGHT_GRAY );
  UG_ButtonSetStyle( &window_1, BTN_ID_0, BTN_STYLE_NO_BORDERS );
  UG_ButtonSetText( &window_1, BTN_ID_0, "Übersicht" );

  /* Configure Button 2 */
  UG_ButtonSetFont( &window_1, BTN_ID_1, &FONT_8X14 );
  UG_ButtonSetBackColor( &window_1, BTN_ID_1, C_LIGHT_GRAY );
  UG_ButtonSetStyle( &window_1, BTN_ID_1, BTN_STYLE_NO_BORDERS );
  UG_ButtonSetText( &window_1, BTN_ID_1, "Rasten" );

  /* Configure Button 3 */
  UG_ButtonSetFont( &window_1, BTN_ID_2, &FONT_8X14 );
  UG_ButtonSetBackColor( &window_1, BTN_ID_2, C_LIGHT_GRAY );
  UG_ButtonSetStyle( &window_1, BTN_ID_2, BTN_STYLE_NO_BORDERS );
  UG_ButtonSetText( &window_1, BTN_ID_2, "Verlauf" );

  /* Configure Button 4 */
  UG_ButtonSetFont( &window_1, BTN_ID_3, &FONT_8X14 );
  UG_ButtonSetBackColor( &window_1, BTN_ID_3, C_LIGHT_GRAY );
  UG_ButtonSetStyle( &window_1, BTN_ID_3, BTN_STYLE_NO_BORDERS );
  UG_ButtonSetText( &window_1, BTN_ID_3, "++" );

  /* Configure Button 5 */
  UG_ButtonSetFont( &window_1, BTN_ID_4, &FONT_8X14 );
  UG_ButtonSetBackColor( &window_1, BTN_ID_4, C_LIGHT_GRAY );
  UG_ButtonSetStyle( &window_1, BTN_ID_4, BTN_STYLE_NO_BORDERS );
  UG_ButtonSetText( &window_1, BTN_ID_4, "--" );

  /* Configure Button 6 */
  UG_ButtonSetFont( &window_1, BTN_ID_5, &FONT_8X14 );
  UG_ButtonSetBackColor( &window_1, BTN_ID_5, C_LIGHT_GRAY );
  UG_ButtonSetStyle( &window_1, BTN_ID_5, BTN_STYLE_NO_BORDERS );
  UG_ButtonSetText( &window_1, BTN_ID_5, "Sonstiges" );

  /* Configure Textbox 1 */
  UG_TextboxSetFont( &window_1, TXB_ID_0, &FONT_10X16 );
  UG_TextboxSetText( &window_1, TXB_ID_0, "Ist-Temperatur" );
  UG_TextboxSetForeColor( &window_1, TXB_ID_0, C_SILVER );
  UG_TextboxSetBackColor( &window_1, TXB_ID_0, C_LIGHT_SLATE_GRAY );
  UG_TextboxSetAlignment( &window_1, TXB_ID_0, ALIGN_CENTER );
  /* Configure Textbox 2 */
  UG_TextboxSetFont( &window_1, TXB_ID_1, &FONT_16X26 );
  UG_TextboxSetText( &window_1, TXB_ID_1, "" );
  UG_TextboxSetForeColor( &window_1, TXB_ID_1, C_GAINSBORO );
  UG_TextboxSetBackColor( &window_1, TXB_ID_1, C_LIGHT_SLATE_GRAY );
  UG_TextboxSetAlignment( &window_1, TXB_ID_1, ALIGN_CENTER );

  /* Configure Textbox 3 */
  UG_TextboxSetFont( &window_1, TXB_ID_2, &FONT_10X16 );
  UG_TextboxSetText( &window_1, TXB_ID_2, "Soll-Temperatur" );
  UG_TextboxSetAlignment( &window_1, TXB_ID_2, ALIGN_CENTER );
  UG_TextboxSetForeColor( &window_1, TXB_ID_2, C_SILVER );
  UG_TextboxSetBackColor( &window_1, TXB_ID_2, C_LIGHT_SLATE_GRAY );
  /* Configure Textbox 4 */
  UG_TextboxSetFont( &window_1, TXB_ID_3, &FONT_16X26 );
  UG_TextboxSetText( &window_1, TXB_ID_3, "" );
  UG_TextboxSetForeColor( &window_1, TXB_ID_3, C_GAINSBORO );
  UG_TextboxSetBackColor( &window_1, TXB_ID_3, C_LIGHT_SLATE_GRAY );
  UG_TextboxSetAlignment( &window_1, TXB_ID_3, ALIGN_CENTER );

  /* Configure Textbox 5 */
  UG_TextboxSetFont( &window_1, TXB_ID_4, &FONT_10X16 );
  UG_TextboxSetText( &window_1, TXB_ID_4, "Rate" );
  UG_TextboxSetForeColor( &window_1, TXB_ID_4, C_SILVER );
  UG_TextboxSetBackColor( &window_1, TXB_ID_4, C_LIGHT_SLATE_GRAY );
  UG_TextboxSetAlignment( &window_1, TXB_ID_4, ALIGN_CENTER );
  /* Configure Textbox 6 */
  UG_TextboxSetFont( &window_1, TXB_ID_5, &FONT_16X26 );
  UG_TextboxSetText( &window_1, TXB_ID_5, "0.0 °C/s" );
  UG_TextboxSetForeColor( &window_1, TXB_ID_5, C_GAINSBORO );
  UG_TextboxSetBackColor( &window_1, TXB_ID_5, C_LIGHT_SLATE_GRAY );
  UG_TextboxSetAlignment( &window_1, TXB_ID_5, ALIGN_CENTER );

  /* Configure Textbox 7 */
  UG_TextboxSetFont( &window_1, TXB_ID_6, &FONT_10X16 );
  UG_TextboxSetText( &window_1, TXB_ID_6, "Rast" );
  UG_TextboxSetForeColor( &window_1, TXB_ID_6, C_SILVER );
  UG_TextboxSetBackColor( &window_1, TXB_ID_6, C_LIGHT_SLATE_GRAY );
  UG_TextboxSetAlignment( &window_1, TXB_ID_7, ALIGN_CENTER );
  /* Configure Textbox 8 */
  UG_TextboxSetFont( &window_1, TXB_ID_7, &FONT_16X26 );
  UG_TextboxSetText( &window_1, TXB_ID_7, "1/10" );
  UG_TextboxSetForeColor( &window_1, TXB_ID_7, C_GAINSBORO );
  UG_TextboxSetBackColor( &window_1, TXB_ID_7, C_LIGHT_SLATE_GRAY );
  UG_TextboxSetAlignment( &window_1, TXB_ID_7, ALIGN_CENTER );

  /* Configure Textbox 49 */
  UG_TextboxSetFont( &window_1, TXB_ID_49, &FONT_10X16 );
  UG_TextboxSetText( &window_1, TXB_ID_49, "Systick" );
  UG_TextboxSetForeColor( &window_1, TXB_ID_49, C_SILVER );
  UG_TextboxSetBackColor( &window_1, TXB_ID_49, C_LIGHT_SLATE_GRAY );
  UG_TextboxSetAlignment( &window_1, TXB_ID_49, ALIGN_CENTER );
  /* Configure Textbox 50 */
  UG_TextboxSetFont( &window_1, TXB_ID_50, &FONT_10X16 );
  UG_TextboxSetText( &window_1, TXB_ID_50, "0" );
  UG_TextboxSetForeColor( &window_1, TXB_ID_50, C_GAINSBORO );
  UG_TextboxSetBackColor( &window_1, TXB_ID_50, C_LIGHT_SLATE_GRAY );
  UG_TextboxSetAlignment( &window_1, TXB_ID_50, ALIGN_CENTER );

  /********************************************************************************************************************/
  /** These are for the overview-view */
  /********************************************************************************************************************/

  UG_TextboxCreate( &window_1, &textbox2_1, TXB_ID_8, 100, h_fract, 100 + w_fract, 1.3 * h_fract );
  UG_TextboxCreate( &window_1, &textbox2_2, TXB_ID_9, 100, 1.3 * h_fract, 100 + w_fract, 2 * h_fract );

  UG_TextboxCreate( &window_1, &textbox2_3, TXB_ID_10, 100 + w_fract, h_fract, 100 + 2 * w_fract, 1.3 * h_fract );
  UG_TextboxCreate( &window_1, &textbox2_4, TXB_ID_11, 100 + w_fract, 1.3 * h_fract, 100 + 2 * w_fract, 2 * h_fract );

  UG_TextboxCreate( &window_1, &textbox2_5, TXB_ID_12, 100 + 2 * w_fract, h_fract, 100 + 3 * w_fract, 1.3 * h_fract );
  UG_TextboxCreate( &window_1, &textbox2_6, TXB_ID_13, 100 + 2 * w_fract, 1.3 * h_fract, 100 + 3 * w_fract,
                    2 * h_fract );

  UG_TextboxCreate( &window_1, &textbox2_7, TXB_ID_14, 100 + 3 * w_fract, h_fract, 100 + 4 * w_fract, 1.3 * h_fract );
  UG_TextboxCreate( &window_1, &textbox2_8, TXB_ID_15, 100 + 3 * w_fract, 1.3 * h_fract, 100 + 4 * w_fract,
                    2 * h_fract );

  /* Configure Textbox 8 */
  UG_TextboxSetFont( &window_1, TXB_ID_8, &FONT_10X16 );
  UG_TextboxSetText( &window_1, TXB_ID_8, "Laufzeit" );
  UG_TextboxSetForeColor( &window_1, TXB_ID_8, C_SILVER );
  UG_TextboxSetBackColor( &window_1, TXB_ID_8, C_LIGHT_SLATE_GRAY );
  UG_TextboxSetAlignment( &window_1, TXB_ID_8, ALIGN_CENTER );
  /* Configure Textbox 9 */
  UG_TextboxSetFont( &window_1, TXB_ID_9, &FONT_16X26 );
  UG_TextboxSetText( &window_1, TXB_ID_9, "" );
  UG_TextboxSetForeColor( &window_1, TXB_ID_9, C_GAINSBORO );
  UG_TextboxSetBackColor( &window_1, TXB_ID_9, C_LIGHT_SLATE_GRAY );
  UG_TextboxSetAlignment( &window_1, TXB_ID_9, ALIGN_CENTER );

  /* Configure Textbox 10 */
  UG_TextboxSetFont( &window_1, TXB_ID_10, &FONT_10X16 );
  UG_TextboxSetText( &window_1, TXB_ID_10, "Rest-Zeit" );
  UG_TextboxSetForeColor( &window_1, TXB_ID_10, C_SILVER );
  UG_TextboxSetBackColor( &window_1, TXB_ID_10, C_LIGHT_SLATE_GRAY );
  UG_TextboxSetAlignment( &window_1, TXB_ID_10, ALIGN_CENTER );
  /* Configure Textbox 11 */
  UG_TextboxSetFont( &window_1, TXB_ID_11, &FONT_16X26 );
  UG_TextboxSetText( &window_1, TXB_ID_11, "" );
  UG_TextboxSetForeColor( &window_1, TXB_ID_11, C_GAINSBORO );
  UG_TextboxSetBackColor( &window_1, TXB_ID_11, C_LIGHT_SLATE_GRAY );
  UG_TextboxSetAlignment( &window_1, TXB_ID_11, ALIGN_CENTER );

  /* Configure Textbox 12 */
  UG_TextboxSetFont( &window_1, TXB_ID_12, &FONT_10X16 );
  UG_TextboxSetText( &window_1, TXB_ID_12, "Rast-Länge" );
  UG_TextboxSetForeColor( &window_1, TXB_ID_12, C_SILVER );
  UG_TextboxSetBackColor( &window_1, TXB_ID_12, C_LIGHT_SLATE_GRAY );
  UG_TextboxSetAlignment( &window_1, TXB_ID_12, ALIGN_CENTER );
  /* Configure Textbox 13 */
  UG_TextboxSetFont( &window_1, TXB_ID_13, &FONT_16X26 );
  UG_TextboxSetText( &window_1, TXB_ID_13, "" );
  UG_TextboxSetForeColor( &window_1, TXB_ID_13, C_GAINSBORO );
  UG_TextboxSetBackColor( &window_1, TXB_ID_13, C_LIGHT_SLATE_GRAY );
  UG_TextboxSetAlignment( &window_1, TXB_ID_13, ALIGN_CENTER );

  /* Configure Textbox 14 */
  UG_TextboxSetFont( &window_1, TXB_ID_14, &FONT_10X16 );
  UG_TextboxSetText( &window_1, TXB_ID_14, "Heiz-Stufe" );
  UG_TextboxSetForeColor( &window_1, TXB_ID_14, C_SILVER );
  UG_TextboxSetBackColor( &window_1, TXB_ID_14, C_LIGHT_SLATE_GRAY );
  UG_TextboxSetAlignment( &window_1, TXB_ID_14, ALIGN_CENTER );
  /* Configure Textbox 15 */
  UG_TextboxSetFont( &window_1, TXB_ID_15, &FONT_16X26 );
  UG_TextboxSetText( &window_1, TXB_ID_15, "" );
  UG_TextboxSetForeColor( &window_1, TXB_ID_15, C_GAINSBORO );
  UG_TextboxSetBackColor( &window_1, TXB_ID_15, C_LIGHT_SLATE_GRAY );
  UG_TextboxSetAlignment( &window_1, TXB_ID_15, ALIGN_CENTER );

  /********************************************************************************************************************/
  /** These are for the "Rasten" view */
  /********************************************************************************************************************/
  /*
   UG_ButtonCreate( &window_1, &button3_1, BTN_ID_20, 100 + 1 * w_fract, 1.5 * h_fract, 100 + 2 * w_fract, 2 * h_fract
   );

   UG_TextboxCreate( &window_1, &textbox3_1, TXB_ID_20, 100 + 0.5 * w_fract, 1 * h_fract, 100 + 1 * w_fract,
   1.5 * h_fract );
   UG_TextboxCreate( &window_1, &textbox3_2, TXB_ID_21, 100 + 1 * w_fract, 1 * h_fract, 100 + 2 * w_fract,
   1.5 * h_fract );
   UG_TextboxCreate( &window_1, &textbox3_3, TXB_ID_22, 100 + 2 * w_fract, 1 * h_fract, 100 + 3 * w_fract,
   1.5 * h_fract );

   UG_ButtonSetFont( &window_1, BTN_ID_20, &FONT_8X14 );
   UG_ButtonSetBackColor( &window_1, BTN_ID_20, C_LIGHT_SLATE_GRAY );
   UG_ButtonSetForeColor( &window_1, BTN_ID_20, C_LIGHT_GRAY );
   UG_ButtonSetStyle( &window_1, BTN_ID_20, BTN_STYLE_NO_BORDERS );
   UG_ButtonSetText( &window_1, BTN_ID_20, "ï¿½bersicht" );

   UG_TextboxSetFont( &window_1, TXB_ID_20, &FONT_8X14 );
   UG_TextboxSetText( &window_1, TXB_ID_20, "#" );
   UG_TextboxSetForeColor( &window_1, TXB_ID_20, C_SILVER );
   UG_TextboxSetBackColor( &window_1, TXB_ID_20, C_LIGHT_SLATE_GRAY );
   UG_TextboxSetAlignment( &window_1, TXB_ID_20, ALIGN_CENTER );

   UG_TextboxSetFont( &window_1, TXB_ID_21, &FONT_8X14 );
   UG_TextboxSetText( &window_1, TXB_ID_21, "Laufzeit" );
   UG_TextboxSetForeColor( &window_1, TXB_ID_21, C_SILVER );
   UG_TextboxSetBackColor( &window_1, TXB_ID_21, C_LIGHT_SLATE_GRAY );
   UG_TextboxSetAlignment( &window_1, TXB_ID_21, ALIGN_CENTER );

   UG_TextboxSetFont( &window_1, TXB_ID_22, &FONT_8X14 );
   UG_TextboxSetText( &window_1, TXB_ID_22, "Temperatur" );
   UG_TextboxSetForeColor( &window_1, TXB_ID_22, C_SILVER );
   UG_TextboxSetBackColor( &window_1, TXB_ID_22, C_LIGHT_SLATE_GRAY );
   UG_TextboxSetAlignment( &window_1, TXB_ID_22, ALIGN_CENTER );

   // Window 2
   UG_WindowCreate( &window_2, obj_buff_wnd_2, MAX_OBJECTS, window_2_callback );
   UG_WindowSetTitleHeight( &window_2, 0 );
   UG_WindowSetBackColor( &window_1, C_DARK_SLATE_GRAY );

   UG_ButtonCreate( &window_2, &button20_1, BTN_ID_0, 0, 0, 100, 60 );
   UG_ButtonSetFont( &window_2, BTN_ID_0, &FONT_8X14 );
   UG_ButtonSetBackColor( &window_2, BTN_ID_0, C_LIGHT_SLATE_GRAY );
   UG_ButtonSetForeColor( &window_2, BTN_ID_0, C_LIGHT_GRAY );
   UG_ButtonSetStyle( &window_2, BTN_ID_0, BTN_STYLE_NO_BORDERS );
   UG_ButtonSetText( &window_2, BTN_ID_0, "OK" );
   */

  gui_update_is_temperature( 1 );
  gui_update_set_temperature( 1 );
  gui_update_rate( 1 );
  gui_update_rast( 1 );
  gui_update_rast_walltime( 1 );
  gui_update_rast_walltime_remaining( 1 );
  gui_update_rast_time( 1 );
}

/**
 * @brief This is called before every GUI update. Use it to update realtime stuff
 */
void gui_update()
{
  gui_update_is_temperature( 0 );
  gui_update_set_temperature( 0 );
  gui_update_rate( 0 );
  gui_update_rast( 0 );
  gui_update_rast_walltime( 0 );
  gui_update_rast_walltime_remaining( 0 );
  gui_update_rast_time( 0 );
  gui_update_stove_power( 0 );
  gui_update_systick( 0 );
}

static void gui_update_is_temperature( uint8_t force_update )
{
  static int32_t last_is_temperature;
  float float_temp;

  if ( ( abs( last_is_temperature - temp_control0.current_temperature ) >= 100 ) || force_update )
  {
    float_temp = (float)temp_control0.current_temperature / TEMP_INT_FACTOR;
    sprintf( is_temperature_string, "%.2f °C", float_temp );
    UG_TextboxSetText( &window_1, TXB_ID_1, is_temperature_string );
    last_is_temperature = temp_control0.current_temperature;
  }
}

static void gui_update_set_temperature( uint8_t force_update )
{
  static int32_t last_set_temperature;
  float float_temp;

  if ( ( abs( last_set_temperature - temp_control0.rasten[ temp_control0.current_rast ].temperature ) >= 100 ) ||
       force_update )
  {
    float_temp = (float)temp_control0.rasten[ temp_control0.current_rast ].temperature / TEMP_INT_FACTOR;
    sprintf( set_temperature_string, "%.2f °C", float_temp );
    UG_TextboxSetText( &window_1, TXB_ID_3, set_temperature_string );
    last_set_temperature = temp_control0.rasten[ temp_control0.current_rast ].temperature;
  }
}

static void gui_update_rate( uint8_t force_update )
{
  static float last_rate;

  if ( ( fabs( last_rate - temp_control0.temperature_rate ) >= 0.01 ) || force_update )
  {
    sprintf( temperature_rate_string, "%.2f °C/s", temp_control0.temperature_rate );
    UG_TextboxSetText( &window_1, TXB_ID_5, temperature_rate_string );
    last_rate = temp_control0.temperature_rate;
  }
}

static void gui_update_rast( uint8_t force_update )
{
  static uint32_t last_rast;

  if ( ( last_rast != temp_control0.current_rast ) || force_update )
  {
    sprintf( rast_string, "%lu/%u", temp_control0.current_rast + 1, MAX_RASTEN );
    UG_TextboxSetText( &window_1, TXB_ID_7, rast_string );
    last_rast = temp_control0.current_rast;
  }
}

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

static void gui_update_stove_power( uint8_t force_update )
{
  static uint32_t last_stove_power;
  uint32_t power = getStovePower();

  if ( last_stove_power != power )
  {
    sprintf( heat_power_string, "%lu W", power );
    UG_TextboxSetText( &window_1, TXB_ID_15, heat_power_string );
    last_stove_power = power;
  }
}

static void gui_update_systick( uint8_t force_update )
{
  sprintf( systick_string, "%lu", osKernelSysTick() );
  UG_TextboxSetText( &window_1, TXB_ID_50, systick_string );
}

/* Callback function for the main menu */
static void window_1_callback( UG_MESSAGE* msg )
{
  static uint32_t active_view = 0, active_view_last = 0;
  static uint8_t toggle = 0;

  if ( msg->type == MSG_TYPE_OBJECT )
  {
    switch ( msg->id )
    {
      case OBJ_TYPE_BUTTON:
        switch ( msg->sub_id )
        {
          case BTN_ID_0: /* ï¿½bersicht */
          {
            if ( msg->event == OBJ_EVENT_PRESSED )
            {
              active_view = 0;
            }
            break;
          }
          case BTN_ID_1: /* Rasten */
          {
            if ( msg->event == OBJ_EVENT_PRESSED )
            {
              active_view = 1;
              // UG_WindowShow( &window_2 );
              if ( toggle )
              {
                UG_ButtonHide( &window_1, BTN_ID_5 );
                toggle = 0;
              }
              else
              {
                UG_ButtonShow( &window_1, BTN_ID_5 );
                toggle = 1;
              }
            }
            break;
          }
          case BTN_ID_2: /* Verlauf */
          {
            if ( msg->event == OBJ_EVENT_PRESSED )
            {
              active_view = 2;
            }
            break;
          }
          case BTN_ID_3: /* ++ */
          {
            if ( msg->event == OBJ_EVENT_PRESSED )
            {
              // active_view = 3;
              modifyRast( &temp_control0, temp_control0.current_rast, ( 0.1 * TEMP_INT_FACTOR ), 0 );
            }
            break;
          }
          case BTN_ID_4: /* -- */
          {
            if ( msg->event == OBJ_EVENT_PRESSED )
            {
              // active_view = 4;
              modifyRast( &temp_control0, temp_control0.current_rast, ( -0.1 * TEMP_INT_FACTOR ), 0 );
            }
            break;
          }
          case BTN_ID_5: /* Sonstiges */
          {
            if ( msg->event == OBJ_EVENT_PRESSED )
            {
              active_view = 5;
            }
            break;
          }
          case BTN_ID_20: /* Rast 1: Time */
          {
            if ( msg->event == OBJ_EVENT_PRESSED )
            {
              UG_WindowShow( &window_2 );
            }
            break;
          }
        }
        break;
      case OBJ_TYPE_TEXTBOX:
        switch ( msg->sub_id )
        {
          case TXB_ID_0:
            switch ( msg->event )
            {
              case OBJ_EVENT_PRERENDER:
                break;
            }
            break;
        }
        break;
    }
  }

  if ( active_view_last != active_view )
  {
    UG_ButtonSetBackColor( &window_1, active_view, C_LIGHT_SLATE_GRAY );
    UG_ButtonSetForeColor( &window_1, active_view, C_LIGHT_GRAY );

    UG_ButtonSetBackColor( &window_1, active_view_last, C_LIGHT_GRAY );
    UG_ButtonSetForeColor( &window_1, active_view_last, C_LIGHT_SLATE_GRAY );
    active_view_last = active_view;
  }
}

/*
 static void window_2_callback( UG_MESSAGE* msg )
 {
 if ( msg->type == MSG_TYPE_OBJECT )
 {
 switch ( msg->id )
 {
 case OBJ_TYPE_BUTTON:
 switch ( msg->sub_id )
 {
 case BTN_ID_0:
 if ( msg->event == OBJ_EVENT_PRESSED )
 {
 UG_WindowHide( &window_2 );
 }
 break;
 }
 break;
 case OBJ_TYPE_TEXTBOX:
 switch ( msg->sub_id )
 {

 }
 break;
 }
 }
 }
 */
