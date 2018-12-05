/*
 * temp_control.c
 *
 *  Created on: 26.05.2016
 *      Author: washed
 */

#include "stm32f7xx.h"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include "arm_math.h"
#include "stove.h"
#include "temp_control.h"

#if defined( __ENABLE_SYSVIEW )
#include "SEGGER_SYSVIEW.h"
#include "SEGGER_SYSVIEW_Conf.h"
#endif

#define tempControlPeriod 1000UL
#define tempControlStackSize 0x2000
#define tempControlSampleQueueItemCount 100

uint32_t tempControlTaskBuffer[ tempControlStackSize ];
osStaticThreadDef_t tempControlControlBlock;

#define tempControlSampleCollectStackSize 0x1000

uint32_t tempControlSampleCollectTaskBuffer[ tempControlSampleCollectStackSize ];
osStaticThreadDef_t tempControlSampleCollectControlBlock;

uint8_t tempControlSampleQueueBuffer[ tempControlSampleQueueItemCount * sizeof( int32_t ) ];
osStaticMessageQDef_t tempControlSampleQueueControlblock;
osMessageQId tempControlSampleQueueHandle;
#ifdef USE_MUTEX_TEMP_RATE
SemaphoreHandle_t tempControlSampleCollectMutex;
StaticSemaphore_t tempControlSampleCollectMutexBuffer;
#endif

TEMPERATURE_CONTROL temp_control0;

char uart_log_string[ 128 ];
char uart_log_temp_string[ 64 ];

static void handleTemperatureControl( TEMPERATURE_CONTROL* temp_control_handle );
static void addTemperatureSample( TEMPERATURE_CONTROL* temp_control_handle, int32_t sample );
static void initTemperatureControl( TEMPERATURE_CONTROL* temp_control_handle );
static void setGainStage( TEMPERATURE_CONTROL* temp_control_handle );

void vTaskTempControlSampleCollect( void* pvParameters )
{
  uint32_t PreviousWakeTime = osKernelSysTick();
  for ( ;; )
  {
    int32_t sample;
    // TODO: Collect samples from queue here
    if ( pdPASS == xQueueReceive( tempControlSampleQueueHandle, &sample, 0 ) )
    {
      // TODO: Maybe we should use a binary semaphore to prevent a data race?
      addTemperatureSample( &temp_control0, sample );
    }
  }

  // We should never get here
  vTaskDelete( NULL );
}

osThreadId createTaskTempControlSampleCollector()
{
  osThreadStaticDef( tempControlSampleCollect, vTaskTempControlSampleCollect, osPriorityNormal, 0,
                     tempControlSampleCollectStackSize, tempControlSampleCollectTaskBuffer,
                     &tempControlSampleCollectControlBlock );
  osThreadId id = osThreadCreate( osThread( tempControlSampleCollect ), NULL );

  osMessageQStaticDef( tempControlSampleQ, tempControlSampleQueueItemCount, int32_t, tempControlSampleQueueBuffer,
                       &tempControlSampleQueueControlblock );
  tempControlSampleQueueHandle = osMessageCreate( &os_messageQ_def_tempControlSampleQ, NULL );

#ifdef USE_MUTEX_TEMP_RATE
  tempControlSampleCollectMutex = xSemaphoreCreateMutexStatic( &tempControlSampleCollectMutexBuffer );
#endif

  return id;
}

void vTaskTempControl( void* pvParameters )
{
  initTemperatureControl( &temp_control0 );

  // TODO: Default rast for testing, Remove!
  setRast( &temp_control0, 0, 100000, 600, RAST_TYPE_HEAT );
  modifyRunMode( &temp_control0, RUN_MODE_RUN );

  uint32_t PreviousWakeTime = osKernelSysTick();
  for ( ;; )
  {
    handleTemperatureControl( &temp_control0 );
    osDelayUntil( &PreviousWakeTime, tempControlPeriod );
  }

  // We should never get here
  vTaskDelete( NULL );
}

osThreadId createTaskTempControl()
{
  osThreadStaticDef( tempControl, vTaskTempControl, osPriorityNormal, 0, tempControlStackSize, tempControlTaskBuffer,
                     &tempControlControlBlock );
  osThreadId id = osThreadCreate( osThread( tempControl ), NULL );

  createTaskTempControlSampleCollector();

  // TODO: Create filtered q here?
  osMessageQStaticDef( tempControlSampleQ, tempControlSampleQueueItemCount, int32_t, tempControlSampleQueueBuffer,
                       &tempControlSampleQueueControlblock );
  tempControlSampleQueueHandle = osMessageCreate( &os_messageQ_def_tempControlSampleQ, NULL );

  return id;
}

static void initTemperatureControl( TEMPERATURE_CONTROL* temp_control_handle )
{
  // Init PID instance

  temp_control_handle->current_temperature = 0;
  temp_control_handle->output = 0.0;
  temp_control_handle->current_rast = 0;
  temp_control_handle->current_run_mode = RUN_MODE_STOP;
  temp_control_handle->temperature_rate = 0;
  temp_control_handle->wall_time = 0;
  temp_control_handle->temperature_slope_buffer_index = 0;
  temp_control_handle->temperature_sample_filter_factor = DEFAULT_OVERSAMPLING_FACTOR;
  temp_control_handle->temperature_slope_filter_factor = TEMPERATURE_SLOPE_SAMPLE_COUNT;

  for ( uint32_t i = 0; i < TEMPERATURE_SLOPE_SAMPLE_COUNT; i++ )
    temp_control_handle->temperature_slope_buffer[ i ] = 0;

  for ( uint32_t i = 0; i < BUFFER_MAX_SAMPLES; i++ ) temp_control_handle->temperature_samples[ i ] = 0;

  for ( uint32_t i = 0; i < MAX_RASTEN; i++ ) temp_control_handle->rasten[ i ].type = RAST_TYPE_UNUSED;
  /*
   * Used for first test
   temp_control_handle->gain_stages[0].P = 0.5;
   temp_control_handle->gain_stages[0].I = 0.0;
   temp_control_handle->gain_stages[0].D = 0.0;
   temp_control_handle->gain_stages[0].lower_temperature = 0.0;
   temp_control_handle->gain_stages[0].upper_temperature = 110.0;
   temp_control_handle->gain_stages[0].lower_error = 5.0;
   temp_control_handle->gain_stages[0].upper_error = 100;

   temp_control_handle->gain_stages[1].P = 0.25;
   temp_control_handle->gain_stages[1].I = 0.0000005;
   temp_control_handle->gain_stages[1].D = 0.0;
   temp_control_handle->gain_stages[1].lower_temperature = 0.0;
   temp_control_handle->gain_stages[1].upper_temperature = 110.0;
   temp_control_handle->gain_stages[1].lower_error = -1.0;
   temp_control_handle->gain_stages[1].upper_error = 5.0;
   */

  /*
   * Used for second test
   temp_control_handle->gain_stages[0].P = 2.0;
   temp_control_handle->gain_stages[0].I = 0.0;
   temp_control_handle->gain_stages[0].D = 0.0;
   temp_control_handle->gain_stages[0].lower_temperature = 0.0;
   temp_control_handle->gain_stages[0].upper_temperature = 110.0;
   temp_control_handle->gain_stages[0].lower_error = 2.5;
   temp_control_handle->gain_stages[0].upper_error = 100;

   temp_control_handle->gain_stages[1].P = 0.25;
   temp_control_handle->gain_stages[1].I = 0.0000005;
   temp_control_handle->gain_stages[1].D = 0.0;
   temp_control_handle->gain_stages[1].lower_temperature = 0.0;
   temp_control_handle->gain_stages[1].upper_temperature = 110.0;
   temp_control_handle->gain_stages[1].lower_error = -1.0;
   temp_control_handle->gain_stages[1].upper_error = 2.5;
   */

  temp_control_handle->gain_stages[ 0 ].P = 10;
  temp_control_handle->gain_stages[ 0 ].I = 0.0;
  temp_control_handle->gain_stages[ 0 ].D = 0.0;
  temp_control_handle->gain_stages[ 0 ].lower_temperature = 0.0;
  temp_control_handle->gain_stages[ 0 ].upper_temperature = 110.0;
  temp_control_handle->gain_stages[ 0 ].lower_error = 0.5;
  temp_control_handle->gain_stages[ 0 ].upper_error = 100;

  temp_control_handle->gain_stages[ 1 ].P = 1;
  temp_control_handle->gain_stages[ 1 ].I = 0.005;
  temp_control_handle->gain_stages[ 1 ].D = 0.01;
  temp_control_handle->gain_stages[ 1 ].lower_temperature = 0.0;
  temp_control_handle->gain_stages[ 1 ].upper_temperature = 110.0;
  temp_control_handle->gain_stages[ 1 ].lower_error = 0;
  temp_control_handle->gain_stages[ 1 ].upper_error = 0.5;

  temp_control_handle->current_gain_stage = 0;

  temp_control_handle->pid.Kp = temp_control_handle->gain_stages[ temp_control_handle->current_gain_stage ].P;
  temp_control_handle->pid.Ki = temp_control_handle->gain_stages[ temp_control_handle->current_gain_stage ].I;
  temp_control_handle->pid.Kd = temp_control_handle->gain_stages[ temp_control_handle->current_gain_stage ].D;
  arm_pid_init_f32( &temp_control_handle->pid, 1 );
}

BaseType_t getTemperatureRate( float* rate_ptr )
{
#ifdef USE_MUTEX_TEMP_RATE
  if ( pdTRUE == xSemaphoreTake( tempControlSampleCollectMutex, 1 ) )
  {
#endif
    *rate_ptr = temp_control0.temperature_rate;
#ifdef USE_MUTEX_TEMP_RATE
    xSemaphoreGive( tempControlSampleCollectMutex );
#endif
    return pdTRUE;
#ifdef USE_MUTEX_TEMP_RATE
  }
  return pdFALSE;
#endif
}

uint32_t getValidGainStage( TEMPERATURE_CONTROL* temp_control_handle )
{
  uint32_t gain_stage = 0;

  float error = (float)( ( (float)temp_control_handle->rasten[ temp_control_handle->current_rast ].temperature -
                           (float)temp_control_handle->current_temperature ) /
                         TEMP_INT_FACTOR );

  for ( ; gain_stage < MAX_GAIN_STAGES; gain_stage++ )
  {
    if ( ( abs( error ) >= temp_control_handle->gain_stages[ gain_stage ].lower_error ) &&
         ( abs( error ) < temp_control_handle->gain_stages[ gain_stage ].upper_error ) &&
         ( ( (float)temp_control_handle->current_temperature / TEMP_INT_FACTOR ) >
           temp_control_handle->gain_stages[ gain_stage ].lower_temperature ) &&
         ( ( (float)temp_control_handle->current_temperature / TEMP_INT_FACTOR ) <
           temp_control_handle->gain_stages[ gain_stage ].upper_temperature ) )
    {
      return gain_stage;
    }
  }

  return 0xFFFFFFFF;
}

static void setGainStage( TEMPERATURE_CONTROL* temp_control_handle )
{
  uint32_t new_gain_stage = getValidGainStage( temp_control_handle );

  if ( new_gain_stage == 0xFFFFFFFF )
  {
    new_gain_stage = 0;
  }

  if ( new_gain_stage != temp_control_handle->current_gain_stage )
  {
    temp_control_handle->current_gain_stage = new_gain_stage;
    temp_control_handle->pid.Kp = temp_control_handle->gain_stages[ temp_control_handle->current_gain_stage ].P;
    temp_control_handle->pid.Ki = temp_control_handle->gain_stages[ temp_control_handle->current_gain_stage ].I;
    temp_control_handle->pid.Kd = temp_control_handle->gain_stages[ temp_control_handle->current_gain_stage ].D;
    arm_pid_init_f32( &temp_control_handle->pid, 1 );
  }
}

static void handleTemperatureControl( TEMPERATURE_CONTROL* temp_control_handle )
{
  switch ( temp_control_handle->current_run_mode )
  {
    case RUN_MODE_RUN:

      temp_control_handle->wall_time++;
      temp_control_handle->rasten[ temp_control_handle->current_rast ].time_running++;

      switch ( temp_control_handle->rasten[ temp_control_handle->current_rast ].type )
      {
        case RAST_TYPE_HEAT:
          if ( temp_control_handle->current_gain_stage == 1 )
          {
            temp_control_handle->current_run_mode = RUN_MODE_NEXT;
            break;
          }
          if ( temp_control_handle->rasten[ temp_control_handle->current_rast ].time != 0 )
          {
            if ( temp_control_handle->rasten[ temp_control_handle->current_rast ].time -
                     temp_control_handle->rasten[ temp_control_handle->current_rast ].time_running <=
                 0 )
            {
              temp_control_handle->current_run_mode = RUN_MODE_NEXT;
              break;
            }
          }
          break;
        case RAST_TYPE_HOLD:
          if ( temp_control_handle->rasten[ temp_control_handle->current_rast ].time != 0 )
          {
            if ( temp_control_handle->rasten[ temp_control_handle->current_rast ].time -
                     temp_control_handle->rasten[ temp_control_handle->current_rast ].time_running <=
                 0 )
            {
              temp_control_handle->current_run_mode = RUN_MODE_NEXT;
              break;
            }
          }
          break;
        case RAST_TYPE_IDLE:
          break;
        case RAST_TYPE_PAUSE:
          break;
        case RAST_TYPE_UNUSED:
          // We should never get here with an unused rast...
          temp_control_handle->current_run_mode = RUN_MODE_STOP;
          break;
      }

      // Set current gain stage:
      setGainStage( temp_control_handle );

      // Calculate controller output according to current temperature error
      temp_control_handle->output =
          arm_pid_f32( &temp_control_handle->pid,
                       (float)( ( (float)temp_control_handle->rasten[ temp_control_handle->current_rast ].temperature -
                                  (float)temp_control_handle->current_temperature ) /
                                TEMP_INT_FACTOR ) );
// #define OUTPUT_LOG
#ifdef OUTPUT_LOG
      printTempControlState( temp_control_handle );
#endif
      // Limit the output to acceptable values
      if ( temp_control_handle->output < ( MIN_OUTPUT ) ) temp_control_handle->output = MIN_OUTPUT;

      if ( temp_control_handle->output > MAX_OUTPUT ) temp_control_handle->output = MAX_OUTPUT;

      if ( temp_control_handle->output > MIN_OUTPUT )
      {
        setStovePower( ( uint8_t )( lrintf( temp_control_handle->output ) & 0xFF ) );
      }
      else
      {
        setStovePower( 0 );
      }

      break;
    case RUN_MODE_PREVIOUS:
      break;
    case RUN_MODE_NEXT:
      if ( getUsedRasten( temp_control_handle ) - temp_control_handle->current_rast <= 0 )
      {
        initTemperatureControl( temp_control_handle );
        temp_control_handle->current_run_mode = RUN_MODE_STOP;
      }
      else
      {
        temp_control_handle->rasten[ temp_control_handle->current_rast ].time_running =
            temp_control_handle->rasten[ temp_control_handle->current_rast ].time;
        temp_control_handle->current_rast++;
        temp_control_handle->current_run_mode = RUN_MODE_RUN;
      }
      break;
    case RUN_MODE_PAUSE:
      break;
    case RUN_MODE_STOP:
      // Switch off the stove and reset the temp control
      setStovePower( 0 );
      break;
  }
}

#ifdef OUTPUT_LOG
void printTempControlState( TEMPERATURE_CONTROL* temp_control_handle )
{
  // Begin constructing the log string
  __itoa( temp_control_handle->wall_time, uart_log_temp_string, 10 );
  strncat( uart_log_temp_string, "\t", 1 );
  strncat( uart_log_string, uart_log_temp_string, strlen( uart_log_temp_string ) );

  __itoa( temp_control_handle->current_temperature, uart_log_temp_string, 10 );
  strncat( uart_log_temp_string, "\t", 1 );
  strncat( uart_log_string, uart_log_temp_string, strlen( uart_log_temp_string ) );

  __itoa( temp_control_handle->rasten[ temp_control_handle->current_rast ].temperature, uart_log_temp_string, 10 );
  strncat( uart_log_temp_string, "\t", 1 );
  strncat( uart_log_string, uart_log_temp_string, strlen( uart_log_temp_string ) );

  convertFloatToString( uart_log_temp_string, &( temp_control_handle->output ), '.' );
  strncat( uart_log_temp_string, "\t", 1 );
  strncat( uart_log_string, uart_log_temp_string, strlen( uart_log_temp_string ) );

  __itoa( temp_control_handle->current_gain_stage, uart_log_temp_string, 10 );
  strncat( uart_log_string, uart_log_temp_string, strlen( uart_log_temp_string ) );

  strncat( uart_log_string, "\r\n", 2 );
  HAL_UART_Transmit( &huart2, (uint8_t*)uart_log_string, strlen( uart_log_string ), 100 );

  memset( uart_log_string, 0, 128 );
}
#endif

void putTemperatureSample( int32_t sample )
{
  xQueueSend( tempControlSampleQueueHandle, &sample, 10 );
}

static void addTemperatureSample( TEMPERATURE_CONTROL* temp_control_handle, int32_t sample )
{
  static uint32_t sample_buffer_index = 0, temperature_valid = 0;
  int64_t sample_sum = 0;
  int32_t delta_Temperature, delta_Time;
  float rate_sample = 0.0f;
  float rate_sample_sum = 0.0f;

  if ( temp_control_handle->temperature_sample_filter_factor > 0 )
  {
    // Add sample to buffer
    temp_control_handle->temperature_samples[ sample_buffer_index++ ] = sample;
    if ( sample_buffer_index >= temp_control_handle->temperature_sample_filter_factor )
    {
      sample_buffer_index = 0;
      temperature_valid = 1;
    }
    delta_Temperature = temp_control_handle->current_temperature - temp_control_handle->last_temperature;
    delta_Time = temp_control_handle->current_timestamp - temp_control_handle->last_timestamp;
    if ( delta_Time != 0 )
    {
      rate_sample = ( (float)delta_Temperature / (float)TEMP_INT_FACTOR ) / ( (float)delta_Time / (float)1000 );
      temp_control_handle->temperature_slope_buffer[ temp_control_handle->temperature_slope_buffer_index++ ] =
          rate_sample;
      if ( temp_control_handle->temperature_slope_buffer_index >= temp_control_handle->temperature_slope_filter_factor )
        temp_control_handle->temperature_slope_buffer_index = 0;

      uint32_t current_buffer_index = temp_control_handle->temperature_slope_buffer_index;
      for ( uint32_t i = 0; i < temp_control_handle->temperature_slope_filter_factor; i++ )
      {
        rate_sample_sum += temp_control_handle->temperature_slope_buffer[ current_buffer_index++ ];
        if ( current_buffer_index > temp_control_handle->temperature_slope_filter_factor ) current_buffer_index = 0;
      }

#ifdef USE_MUTEX_TEMP_RATE
      if ( pdTRUE == xSemaphoreTake( tempControlSampleCollectMutex, 1 ) )
      {
#endif
        temp_control_handle->temperature_rate = rate_sample_sum / temp_control_handle->temperature_slope_filter_factor;
#ifdef USE_MUTEX_TEMP_RATE
        xSemaphoreGive( tempControlSampleCollectMutex );
      }
#endif
    }
    if ( temperature_valid == 1 )
    {
      sample_sum = 0;
      // Calculate new average
      uint32_t current_buffer_index = sample_buffer_index;
      for ( uint32_t i = 0; i < temp_control_handle->temperature_sample_filter_factor; i++ )
      {
        sample_sum += temp_control_handle->temperature_samples[ current_buffer_index++ ];
        if ( current_buffer_index >= temp_control_handle->temperature_sample_filter_factor ) current_buffer_index = 0;
      }

      temp_control_handle->last_timestamp = temp_control_handle->current_timestamp;
      temp_control_handle->last_temperature = temp_control_handle->current_temperature;
      temp_control_handle->current_timestamp = HAL_GetTick();
      temp_control_handle->current_temperature =
          lrintf( (float)sample_sum / (float)temp_control_handle->temperature_sample_filter_factor );
    }
  }
  else if ( temp_control_handle->temperature_sample_filter_factor == 0 )
  {
    // Process sample directly
  }
}

void modifyRast( TEMPERATURE_CONTROL* temp_control_handle, uint8_t rast_index, int32_t temperature_delta,
                 int32_t time_delta )
{
  if ( ( rast_index >= 0 ) && ( rast_index < MAX_RASTEN ) )
  {
    temp_control_handle->rasten[ rast_index ].temperature += temperature_delta;
    temp_control_handle->rasten[ rast_index ].time += time_delta;
  }
}

void setRast( TEMPERATURE_CONTROL* temp_control_handle, uint8_t rast_index, int32_t temperature, int32_t time,
              RAST_TYPES type )
{
  if ( ( rast_index >= 0 ) && ( rast_index < MAX_RASTEN ) )
  {
    temp_control_handle->rasten[ rast_index ].temperature = temperature;
    temp_control_handle->rasten[ rast_index ].time = time;
    temp_control_handle->rasten[ rast_index ].type = type;
  }
}

void modifyRunMode( TEMPERATURE_CONTROL* temp_control_handle, uint8_t run_mode_command )
{
  temp_control_handle->current_run_mode = run_mode_command;
}

uint32_t getUsedRasten( TEMPERATURE_CONTROL* temp_control_handle )
{
  uint32_t retVal = 0;
  for ( uint32_t i = 0; i < MAX_RASTEN; i++ )
  {
    if ( temp_control_handle->rasten[ i ].type != RAST_TYPE_UNUSED ) retVal++;
  }
  return retVal;
}

int32_t getTotalTimeRasten( TEMPERATURE_CONTROL* temp_control_handle )
{
  int32_t used_rasten, total_time;

  used_rasten = getUsedRasten( temp_control_handle );
  total_time = 0;

  for ( uint32_t i = 0; i < used_rasten; i++ )
  {
    total_time += temp_control_handle->rasten[ i ].time;
  }

  return total_time;
}

int32_t getTotalTimeRemainingRasten( TEMPERATURE_CONTROL* temp_control_handle )
{
  int32_t used_rasten, total_time_remaining;

  used_rasten = getUsedRasten( temp_control_handle );
  total_time_remaining = 0;

  for ( uint32_t i = 0; i < used_rasten; i++ )
  {
    total_time_remaining += ( temp_control_handle->rasten[ i ].time - temp_control_handle->rasten[ i ].time_running );
  }

  return total_time_remaining;
}

/*
 switch (temp_slope_state)
 {
 case 0:
 temperature_slope_buffer[0] = DS18B20_0.temperature_float;
 temp_slope_state = 1;
 break;
 case 1:
 temp_slope_counter++;
 if ( temp_slope_counter == 16 )
 {
 temp_slope_counter = 0;
 temp_slope_state = 2;
 }
 break;
 case 2:
 temperature_slope_buffer[1] = DS18B20_0.temperature_float;
 temp_slope = (temperature_slope_buffer[1] - temperature_slope_buffer[0]) /
 16.0*0.05;
 temp_slope = temp_slope * 75.0;
 temp_slope_state = 0;
 break;
 }
 */
