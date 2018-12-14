#if 0
#include "gui/window_statustop.h"

static void gui_callback_window_statustop( UG_MESSAGE* msg );

#define WINDOW_STATUSTOP_ITEMCOUNT 32

UG_WINDOW wnd_statustop;
UG_OBJECT obj_buff_window_statustop[ WINDOW_STATUSTOP_ITEMCOUNT ];

UG_TEXTBOX tbx_lbl_istemp;
UG_TEXTBOX tbx_val_istemp;
UG_TEXTBOX tbx_lbl_settemp;
UG_TEXTBOX tbx_val_settemp;
UG_TEXTBOX tbx_lbl_rate;
UG_TEXTBOX tbx_val_rate;
UG_TEXTBOX tbx_lbl_rast;
UG_TEXTBOX tbx_val_rast;
UG_TEXTBOX tbx_lbl_power;
UG_TEXTBOX tbx_val_power;

#define TXB_ID_LBL_ISTEMP TXB_ID_0
#define TXB_ID_VAL_ISTEMP TXB_ID_1
#define TXB_ID_LBL_SETTEMP TXB_ID_2
#define TXB_ID_VAL_SETTEMP TXB_ID_3
#define TXB_ID_LBL_RATE TXB_ID_4
#define TXB_ID_VAL_RATE TXB_ID_5
#define TXB_ID_LBL_RAST TXB_ID_6
#define TXB_ID_VAL_RAST TXB_ID_7
#define TXB_ID_LBL_POWER TXB_ID_8
#define TXB_ID_VAL_POWER TXB_ID_9

char is_temperature_string[ 32 ];
char temperature_rate_string[ 32 ];
char rast_string[ 32 ];
char set_temperature_string[ 32 ];
char heat_power_string[ 32 ];

static uint16_t h_fract;
static uint16_t w_fract;

static void gui_update_is_temperature( uint8_t force_update );
static void gui_update_set_temperature( uint8_t force_update );
static void gui_update_rate( uint8_t force_update );
static void gui_update_rast( uint8_t force_update );
static void gui_update_stove_power( uint8_t force_update );

static inline uint16_t xs( uint16_t idx )
{
  return idx * w_fract;
}

static inline uint16_t xe( uint16_t idx )
{
  return ( ( idx + 1 ) * w_fract ) - 1 - 1;
}

static inline uint16_t lbl_ys()
{
  return 0;
}

static inline uint16_t lbl_ye()
{
  return 2 * h_fract - 1;
}

static inline uint16_t val_ys()
{
  return 2 * h_fract;
}

static inline uint16_t val_ye()
{
  return 5 * h_fract - 2;
}

void gui_init_window_statustop()
{
  // Create statustop window
  UG_WindowCreate( &wnd_statustop, obj_buff_window_statustop, WINDOW_STATUSTOP_ITEMCOUNT,
                   gui_callback_window_statustop );

  UG_WindowSetBackColor( &wnd_statustop, C_DARK_SLATE_GRAY );
  UG_WindowSetStyle( &wnd_statustop, WND_STYLE_2D | WND_STYLE_HIDE_TITLE );
  UG_WindowSetXStart( &wnd_statustop, 100 );
  UG_WindowSetYStart( &wnd_statustop, 0 );
  UG_WindowSetXEnd( &wnd_statustop, 799 );
  UG_WindowSetYEnd( &wnd_statustop, 79 );

  h_fract = ( UG_WindowGetInnerHeight( &wnd_statustop ) + 1 ) / 5;
  w_fract = ( UG_WindowGetInnerWidth( &wnd_statustop ) + 1 ) / 5;

  UG_TextboxCreate( &wnd_statustop, &tbx_lbl_istemp, TXB_ID_LBL_ISTEMP, xs( 0 ), lbl_ys(), xe( 0 ), lbl_ye() );
  UG_TextboxCreate( &wnd_statustop, &tbx_val_istemp, TXB_ID_VAL_ISTEMP, xs( 0 ), val_ys(), xe( 0 ), val_ye() );

  UG_TextboxCreate( &wnd_statustop, &tbx_lbl_settemp, TXB_ID_LBL_SETTEMP, xs( 1 ), lbl_ys(), xe( 1 ), lbl_ye() );
  UG_TextboxCreate( &wnd_statustop, &tbx_val_settemp, TXB_ID_VAL_SETTEMP, xs( 1 ), val_ys(), xe( 1 ), val_ye() );

  UG_TextboxCreate( &wnd_statustop, &tbx_lbl_rate, TXB_ID_LBL_RATE, xs( 2 ), lbl_ys(), xe( 2 ), lbl_ye() );
  UG_TextboxCreate( &wnd_statustop, &tbx_val_rate, TXB_ID_VAL_RATE, xs( 2 ), val_ys(), xe( 2 ), val_ye() );

  UG_TextboxCreate( &wnd_statustop, &tbx_lbl_rast, TXB_ID_LBL_RAST, xs( 3 ), lbl_ys(), xe( 3 ), lbl_ye() );
  UG_TextboxCreate( &wnd_statustop, &tbx_val_rast, TXB_ID_VAL_RAST, xs( 3 ), val_ys(), xe( 3 ), val_ye() );

  UG_TextboxCreate( &wnd_statustop, &tbx_lbl_power, TXB_ID_LBL_POWER, xs( 4 ), lbl_ys(), xe( 4 ), lbl_ye() );
  UG_TextboxCreate( &wnd_statustop, &tbx_val_power, TXB_ID_VAL_POWER, xs( 4 ), val_ys(), xe( 4 ), val_ye() );

  // Configure Textbox "Is-Temperatur" label
  UG_TextboxSetFont( &wnd_statustop, TXB_ID_LBL_ISTEMP, &FONT_10X16 );
  UG_TextboxSetText( &wnd_statustop, TXB_ID_LBL_ISTEMP, "Temp" );
  UG_TextboxSetForeColor( &wnd_statustop, TXB_ID_LBL_ISTEMP, C_SILVER );
  UG_TextboxSetBackColor( &wnd_statustop, TXB_ID_LBL_ISTEMP, C_LIGHT_SLATE_GRAY );
  UG_TextboxSetAlignment( &wnd_statustop, TXB_ID_LBL_ISTEMP, ALIGN_CENTER );
  // Configure Textbox "Is-Temperature" value
  UG_TextboxSetFont( &wnd_statustop, TXB_ID_VAL_ISTEMP, &FONT_16X26 );
  UG_TextboxSetText( &wnd_statustop, TXB_ID_VAL_ISTEMP, "" );
  UG_TextboxSetForeColor( &wnd_statustop, TXB_ID_VAL_ISTEMP, C_GAINSBORO );
  UG_TextboxSetBackColor( &wnd_statustop, TXB_ID_VAL_ISTEMP, C_LIGHT_SLATE_GRAY );
  UG_TextboxSetAlignment( &wnd_statustop, TXB_ID_VAL_ISTEMP, ALIGN_CENTER );

  // Configure Textbox "Set-Temperature" label
  UG_TextboxSetFont( &wnd_statustop, TXB_ID_LBL_SETTEMP, &FONT_10X16 );
  UG_TextboxSetText( &wnd_statustop, TXB_ID_LBL_SETTEMP, "Set Temp" );
  UG_TextboxSetAlignment( &wnd_statustop, TXB_ID_LBL_SETTEMP, ALIGN_CENTER );
  UG_TextboxSetForeColor( &wnd_statustop, TXB_ID_LBL_SETTEMP, C_SILVER );
  UG_TextboxSetBackColor( &wnd_statustop, TXB_ID_LBL_SETTEMP, C_LIGHT_SLATE_GRAY );
  // Configure Textbox "Set-Temperature" value
  UG_TextboxSetFont( &wnd_statustop, TXB_ID_VAL_SETTEMP, &FONT_16X26 );
  UG_TextboxSetText( &wnd_statustop, TXB_ID_VAL_SETTEMP, "" );
  UG_TextboxSetForeColor( &wnd_statustop, TXB_ID_VAL_SETTEMP, C_GAINSBORO );
  UG_TextboxSetBackColor( &wnd_statustop, TXB_ID_VAL_SETTEMP, C_LIGHT_SLATE_GRAY );
  UG_TextboxSetAlignment( &wnd_statustop, TXB_ID_VAL_SETTEMP, ALIGN_CENTER );

  // Configure Textbox "Rate" label
  UG_TextboxSetFont( &wnd_statustop, TXB_ID_LBL_RATE, &FONT_10X16 );
  UG_TextboxSetText( &wnd_statustop, TXB_ID_LBL_RATE, "Rate" );
  UG_TextboxSetForeColor( &wnd_statustop, TXB_ID_LBL_RATE, C_SILVER );
  UG_TextboxSetBackColor( &wnd_statustop, TXB_ID_LBL_RATE, C_LIGHT_SLATE_GRAY );
  UG_TextboxSetAlignment( &wnd_statustop, TXB_ID_LBL_RATE, ALIGN_CENTER );
  // Configure Textbox "Rate" value
  UG_TextboxSetFont( &wnd_statustop, TXB_ID_VAL_RATE, &FONT_16X26 );
  UG_TextboxSetText( &wnd_statustop, TXB_ID_VAL_RATE, "0.0 °C/s" );
  UG_TextboxSetForeColor( &wnd_statustop, TXB_ID_VAL_RATE, C_GAINSBORO );
  UG_TextboxSetBackColor( &wnd_statustop, TXB_ID_VAL_RATE, C_LIGHT_SLATE_GRAY );
  UG_TextboxSetAlignment( &wnd_statustop, TXB_ID_VAL_RATE, ALIGN_CENTER );

  // Configure Textbox "Rast" label
  UG_TextboxSetFont( &wnd_statustop, TXB_ID_LBL_RAST, &FONT_10X16 );
  UG_TextboxSetText( &wnd_statustop, TXB_ID_LBL_RAST, "Rast" );
  UG_TextboxSetForeColor( &wnd_statustop, TXB_ID_LBL_RAST, C_SILVER );
  UG_TextboxSetBackColor( &wnd_statustop, TXB_ID_LBL_RAST, C_LIGHT_SLATE_GRAY );
  UG_TextboxSetAlignment( &wnd_statustop, TXB_ID_VAL_RAST, ALIGN_CENTER );
  // Configure Textbox "Rast" value
  UG_TextboxSetFont( &wnd_statustop, TXB_ID_VAL_RAST, &FONT_16X26 );
  UG_TextboxSetText( &wnd_statustop, TXB_ID_VAL_RAST, "1/10" );
  UG_TextboxSetForeColor( &wnd_statustop, TXB_ID_VAL_RAST, C_GAINSBORO );
  UG_TextboxSetBackColor( &wnd_statustop, TXB_ID_VAL_RAST, C_LIGHT_SLATE_GRAY );
  UG_TextboxSetAlignment( &wnd_statustop, TXB_ID_VAL_RAST, ALIGN_CENTER );

  // Configure Textbox "Power" label
  UG_TextboxSetFont( &wnd_statustop, TXB_ID_LBL_POWER, &FONT_10X16 );
  UG_TextboxSetText( &wnd_statustop, TXB_ID_LBL_POWER, "Power" );
  UG_TextboxSetForeColor( &wnd_statustop, TXB_ID_LBL_POWER, C_SILVER );
  UG_TextboxSetBackColor( &wnd_statustop, TXB_ID_LBL_POWER, C_LIGHT_SLATE_GRAY );
  UG_TextboxSetAlignment( &wnd_statustop, TXB_ID_LBL_POWER, ALIGN_CENTER );
  // Configure Textbox "Power" value
  UG_TextboxSetFont( &wnd_statustop, TXB_ID_VAL_POWER, &FONT_16X26 );
  UG_TextboxSetText( &wnd_statustop, TXB_ID_VAL_POWER, "" );
  UG_TextboxSetForeColor( &wnd_statustop, TXB_ID_VAL_POWER, C_GAINSBORO );
  UG_TextboxSetBackColor( &wnd_statustop, TXB_ID_VAL_POWER, C_LIGHT_SLATE_GRAY );
  UG_TextboxSetAlignment( &wnd_statustop, TXB_ID_VAL_POWER, ALIGN_CENTER );
}

void gui_update_window_statustop( uint8_t force_update )
{
  gui_update_is_temperature( force_update );
  gui_update_set_temperature( force_update );
  gui_update_rate( force_update );
  gui_update_rast( force_update );
  gui_update_stove_power( force_update );
}

static void gui_update_is_temperature( uint8_t force_update )
{
  static int32_t last_is_temperature;
  float float_temp;

  if ( ( abs( last_is_temperature - temp_control0.current_temperature ) >= 100 ) || force_update )
  {
    float_temp = (float)temp_control0.current_temperature / TEMP_INT_FACTOR;
    sprintf( is_temperature_string, "%.2f °C", float_temp );
    UG_TextboxSetText( &wnd_statustop, TXB_ID_VAL_ISTEMP, is_temperature_string );
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
    UG_TextboxSetText( &wnd_statustop, TXB_ID_VAL_SETTEMP, set_temperature_string );
    last_set_temperature = temp_control0.rasten[ temp_control0.current_rast ].temperature;
  }
}

static void gui_update_rate( uint8_t force_update )
{
  static float last_rate;
  float rate;

  if ( pdTRUE == getTemperatureRate( &rate ) )
  {
    if ( ( fabs( last_rate - rate ) >= 0.01 ) || force_update )
    {
      sprintf( temperature_rate_string, "%.2f °C/s", rate );

      UG_TextboxSetText( &wnd_statustop, TXB_ID_VAL_RATE, temperature_rate_string );
      last_rate = rate;
    }
  }
}

static void gui_update_rast( uint8_t force_update )
{
  static uint32_t last_rast;

  if ( ( last_rast != temp_control0.current_rast ) || force_update )
  {
    sprintf( rast_string, "%lu/%u", temp_control0.current_rast + 1, MAX_RASTEN );
    UG_TextboxSetText( &wnd_statustop, TXB_ID_VAL_RAST, rast_string );
    last_rast = temp_control0.current_rast;
  }
}

static void gui_update_stove_power( uint8_t force_update )
{
  static uint32_t last_stove_power;
  uint32_t power = getStovePower();

  if ( ( last_stove_power != power ) || ( 1 == force_update ) )
  {
    sprintf( heat_power_string, "%lu W", power );
    UG_TextboxSetText( &wnd_statustop, TXB_ID_VAL_POWER, heat_power_string );
    last_stove_power = power;
  }
}

// Callback function for window "statustop"
static void gui_callback_window_statustop( UG_MESSAGE* msg ) {}
#endif
