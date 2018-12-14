#if 0
#include "gui/window_mainmenu.h"

static void gui_callback_window_mainmenu( UG_MESSAGE* msg );

#define WINDOW_MAINMENU_ITEMCOUNT 32

UG_WINDOW wnd_mainmenu;
UG_OBJECT obj_buff_window_mainmenu[ WINDOW_MAINMENU_ITEMCOUNT ];
UG_BUTTON btn_overview;
UG_BUTTON btn_rasten;
UG_BUTTON btn_history;
UG_BUTTON btn_pp;
UG_BUTTON btn_mm;
UG_BUTTON btn_other;

uint16_t h_fract;
uint16_t window_width;

static inline uint16_t xs()
{
  return 0;
}

static inline uint16_t xe()
{
  return window_width - 1;
}

static inline uint16_t ys( uint16_t idx )
{
  return idx * h_fract;
}

static inline uint16_t ye( uint16_t idx )
{
  return ( idx + 1 ) * h_fract - 2;
}

void gui_init_window_mainmenu()
{
  // Create main menu window
  UG_WindowCreate( &wnd_mainmenu, obj_buff_window_mainmenu, WINDOW_MAINMENU_ITEMCOUNT, gui_callback_window_mainmenu );
  UG_WindowSetBackColor( &wnd_mainmenu, C_DARK_SLATE_GRAY );
  UG_WindowSetStyle( &wnd_mainmenu, WND_STYLE_2D | WND_STYLE_HIDE_TITLE );
  UG_WindowSetXStart( &wnd_mainmenu, 0 );
  UG_WindowSetYStart( &wnd_mainmenu, 0 );
  UG_WindowSetXEnd( &wnd_mainmenu, 99 );
  UG_WindowSetYEnd( &wnd_mainmenu, 479 );

  h_fract = ( UG_WindowGetInnerHeight( &wnd_mainmenu ) + 1 ) / 6;
  window_width = UG_WindowGetInnerWidth( &wnd_mainmenu );

  // Create main menu buttons
  UG_ButtonCreate( &wnd_mainmenu, &btn_overview, BTN_ID_0, xs(), ys( 0 ), xe(), ye( 0 ) );
  UG_ButtonCreate( &wnd_mainmenu, &btn_rasten, BTN_ID_1, xs(), ys( 1 ), xe(), ye( 1 ) );
  UG_ButtonCreate( &wnd_mainmenu, &btn_history, BTN_ID_2, xs(), ys( 2 ), xe(), ye( 2 ) );
  UG_ButtonCreate( &wnd_mainmenu, &btn_pp, BTN_ID_3, xs(), ys( 3 ), xe(), ye( 3 ) );
  UG_ButtonCreate( &wnd_mainmenu, &btn_mm, BTN_ID_4, xs(), ys( 4 ), xe(), ye( 4 ) );
  UG_ButtonCreate( &wnd_mainmenu, &btn_other, BTN_ID_5, xs(), ys( 5 ), xe(), ye( 5 ) );

  // Configure Button "Overview"
  UG_ButtonSetFont( &wnd_mainmenu, BTN_ID_0, &FONT_8X14 );
  UG_ButtonSetBackColor( &wnd_mainmenu, BTN_ID_0, C_LIGHT_SLATE_GRAY );
  UG_ButtonSetForeColor( &wnd_mainmenu, BTN_ID_0, C_LIGHT_GRAY );
  UG_ButtonSetStyle( &wnd_mainmenu, BTN_ID_0, BTN_STYLE_NO_BORDERS );
  UG_ButtonSetText( &wnd_mainmenu, BTN_ID_0, "Overview" );

  // Configure Button "Rasten"
  UG_ButtonSetFont( &wnd_mainmenu, BTN_ID_1, &FONT_8X14 );
  UG_ButtonSetBackColor( &wnd_mainmenu, BTN_ID_1, C_LIGHT_GRAY );
  UG_ButtonSetStyle( &wnd_mainmenu, BTN_ID_1, BTN_STYLE_NO_BORDERS );
  UG_ButtonSetText( &wnd_mainmenu, BTN_ID_1, "Rasten" );

  // Configure Button "History"
  UG_ButtonSetFont( &wnd_mainmenu, BTN_ID_2, &FONT_8X14 );
  UG_ButtonSetBackColor( &wnd_mainmenu, BTN_ID_2, C_LIGHT_GRAY );
  UG_ButtonSetStyle( &wnd_mainmenu, BTN_ID_2, BTN_STYLE_NO_BORDERS );
  UG_ButtonSetText( &wnd_mainmenu, BTN_ID_2, "History" );

  // Configure Button "++"
  UG_ButtonSetFont( &wnd_mainmenu, BTN_ID_3, &FONT_8X14 );
  UG_ButtonSetBackColor( &wnd_mainmenu, BTN_ID_3, C_LIGHT_GRAY );
  UG_ButtonSetStyle( &wnd_mainmenu, BTN_ID_3, BTN_STYLE_NO_BORDERS );
  UG_ButtonSetText( &wnd_mainmenu, BTN_ID_3, "++" );

  // Configure Button "--"
  UG_ButtonSetFont( &wnd_mainmenu, BTN_ID_4, &FONT_8X14 );
  UG_ButtonSetBackColor( &wnd_mainmenu, BTN_ID_4, C_LIGHT_GRAY );
  UG_ButtonSetStyle( &wnd_mainmenu, BTN_ID_4, BTN_STYLE_NO_BORDERS );
  UG_ButtonSetText( &wnd_mainmenu, BTN_ID_4, "--" );

  // Configure Button "Other"
  UG_ButtonSetFont( &wnd_mainmenu, BTN_ID_5, &FONT_8X14 );
  UG_ButtonSetBackColor( &wnd_mainmenu, BTN_ID_5, C_LIGHT_GRAY );
  UG_ButtonSetStyle( &wnd_mainmenu, BTN_ID_5, BTN_STYLE_NO_BORDERS );
  UG_ButtonSetText( &wnd_mainmenu, BTN_ID_5, "Other" );
}

void gui_update_window_mainmenu( uint8_t force_update ) {}

// Callback function for the main menu
static void gui_callback_window_mainmenu( UG_MESSAGE* msg )
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
          case BTN_ID_0:  // �bersicht
          {
            if ( msg->event == OBJ_EVENT_PRESSED )
            {
              active_view = 0;
            }
            break;
          }
          case BTN_ID_1:  // Rasten
          {
            if ( msg->event == OBJ_EVENT_PRESSED )
            {
              active_view = 1;
              // UG_WindowShow( &window_2 );
              if ( toggle )
              {
                UG_ButtonHide( &wnd_mainmenu, BTN_ID_5 );
                toggle = 0;
              }
              else
              {
                UG_ButtonShow( &wnd_mainmenu, BTN_ID_5 );
                toggle = 1;
              }
            }
            break;
          }
          case BTN_ID_2:  // Verlauf
          {
            if ( msg->event == OBJ_EVENT_PRESSED )
            {
              active_view = 2;
            }
            break;
          }
          case BTN_ID_3:  // ++
          {
            if ( msg->event == OBJ_EVENT_PRESSED )
            {
              // active_view = 3;
              modifyRast( &temp_control0, temp_control0.current_rast, ( 0.1 * TEMP_INT_FACTOR ), 0 );
            }
            break;
          }
          case BTN_ID_4:  // --
          {
            if ( msg->event == OBJ_EVENT_PRESSED )
            {
              // active_view = 4;
              modifyRast( &temp_control0, temp_control0.current_rast, ( -0.1 * TEMP_INT_FACTOR ), 0 );
            }
            break;
          }
          case BTN_ID_5:  // Sonstiges
          {
            if ( msg->event == OBJ_EVENT_PRESSED )
            {
              active_view = 5;
            }
            break;
          }
          case BTN_ID_20:  // Rast 1: Time
          {
            if ( msg->event == OBJ_EVENT_PRESSED )
            {
              // UG_WindowShow( &window_2 );
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
    UG_ButtonSetBackColor( &wnd_mainmenu, active_view, C_LIGHT_SLATE_GRAY );
    UG_ButtonSetForeColor( &wnd_mainmenu, active_view, C_LIGHT_GRAY );

    UG_ButtonSetBackColor( &wnd_mainmenu, active_view_last, C_LIGHT_GRAY );
    UG_ButtonSetForeColor( &wnd_mainmenu, active_view_last, C_LIGHT_SLATE_GRAY );
    active_view_last = active_view;
  }
}
#endif

