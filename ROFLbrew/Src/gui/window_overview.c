#if 0 // This is old UGUI code
static void gui_callback_window_overview( UG_MESSAGE* msg );

#define WINDOW_OVERVIEW_ITEMCOUNT 32

UG_WINDOW window_overview;
UG_OBJECT obj_buff_window_overview[ WINDOW_OVERVIEW_ITEMCOUNT ];

void gui_init_window_overview()
{
  // Create overview window
  UG_WindowCreate( &window_overview, obj_buff_window_overview, WINDOW_OVERVIEW_ITEMCOUNT,
                   gui_callback_window_overview );

  UG_WindowSetBackColor( &window_overview, C_DARK_SLATE_GRAY );
  UG_WindowSetStyle( &window_overview, WND_STYLE_2D | WND_STYLE_HIDE_TITLE );
  UG_WindowSetXStart( &window_overview, 100 );
  UG_WindowSetYStart( &window_overview, 80 );
  UG_WindowSetXEnd( &window_overview, 799 );
  UG_WindowSetYEnd( &window_overview, 459 );

  uint16_t h_fract = ( UG_WindowGetInnerHeight( &window_overview ) + 1 ) / 5;
  uint16_t window_width = UG_WindowGetInnerWidth( &window_overview );
}

void gui_update_window_overview( uint8_t force_update ) {}

// Callback function for window "overview"
static void gui_callback_window_overview( UG_MESSAGE* msg ) {}
#endif
