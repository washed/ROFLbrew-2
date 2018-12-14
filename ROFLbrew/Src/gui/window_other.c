

static void gui_update_systick( uint8_t force_update );
static void gui_update_fps( uint8_t force_update );

char systick_string[ 32 ];
char fps_string[ 32 ];

void gui_init_window_other() {}

void gui_update_window_other( uint8_t force_update )
{
  // TODO: check for visibility!
  gui_update_systick( 0 );
  gui_update_fps( 0 );
}

static void gui_update_systick( uint8_t force_update )
{
  sprintf( systick_string, "%lu", osKernelSysTick() );
  UG_TextboxSetText( &window_1, TXB_ID_50, systick_string );
}

static void gui_update_fps( uint8_t force_update )
{
  uint32_t fps = 1000 / duration;
  sprintf( fps_string, "%lu", fps );
  UG_TextboxSetText( &window_1, TXB_ID_48, fps_string );
}