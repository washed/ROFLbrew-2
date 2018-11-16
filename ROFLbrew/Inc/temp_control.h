/*
 * temp_control.h
 *
 *  Created on: 26.05.2016
 *      Author: washed
 */

#ifndef TEMP_CONTROL_H_
#define TEMP_CONTROL_H_

#include "stm32f4xx.h"
#include "arm_math.h"

#define PID_PARAM_KP 0.5      // 2.1			/* Proportional */
#define PID_PARAM_KI 0.000001 /* Integral */
#define PID_PARAM_KD 0.00005  /* Derivative */

#define TEMPERATURE_SLOPE_SAMPLE_COUNT 128
#define MIN_OUTPUT 0
#define MAX_OUTPUT 10
#define TEMP_INT_FACTOR 10000

#define MAX_RASTEN 5
#define MAX_GAIN_STAGES 5

#define BUFFER_MAX_SAMPLES 128
#define DEFAULT_OVERSAMPLING_FACTOR 25

typedef enum RUN_MODES
{
  RUN_MODE_RUN = 0,
  RUN_MODE_PREVIOUS,
  RUN_MODE_NEXT,
  RUN_MODE_PAUSE,
  RUN_MODE_STOP
} RUN_MODES;

typedef enum GAIN_SCHEDULE_STAGES
{
  GAIN_STAGE_0 = 0,
  GAIN_STAGE_1,
  GAIN_STAGE_2,
  GAIN_STAGE_3

} GAIN_SCHEDULE_STAGES;

typedef enum RAST_TYPES
{
  RAST_TYPE_HEAT = 0,
  RAST_TYPE_HOLD,
  RAST_TYPE_IDLE,
  RAST_TYPE_PAUSE,
  RAST_TYPE_UNUSED
} RAST_TYPES;

typedef struct RAST
{
  int32_t time;
  int32_t time_running;
  int32_t temperature;
  RAST_TYPES type;
} RAST;

typedef struct GAIN_STAGE_TypeDef
{
  float P;
  float I;
  float D;
  float lower_error;
  float upper_error;
  float lower_temperature;
  float upper_temperature;
} GAIN_STAGE_TypeDef;

typedef struct TEMPERATURE_CONTROL
{
  arm_pid_instance_f32 pid;
  int32_t current_temperature;
  int32_t last_temperature;
  uint32_t current_timestamp;
  uint32_t last_timestamp;
  uint32_t current_rast;
  int32_t wall_time;
  int32_t rast_time;
  float output;
  uint8_t temperature_sample_filter_factor;
  uint32_t temperature_slope_filter_factor;
  int32_t temperature_samples[ BUFFER_MAX_SAMPLES ];
  float temperature_slope_buffer[ TEMPERATURE_SLOPE_SAMPLE_COUNT ];
  uint32_t temperature_slope_buffer_index;
  float temperature_rate;
  RAST rasten[ MAX_RASTEN ];
  RUN_MODES current_run_mode;
  uint32_t current_gain_stage;
  GAIN_STAGE_TypeDef gain_stages[ MAX_GAIN_STAGES ];

} TEMPERATURE_CONTROL;

extern TEMPERATURE_CONTROL temp_control0;
extern volatile uint8_t run_temperature_control;

void setGainStage( TEMPERATURE_CONTROL* temp_control_handle );
void initTemperatureControl( TEMPERATURE_CONTROL* temp_control_handle );
void handleTemperatureControl( TEMPERATURE_CONTROL* temp_control_handle );
void addTemperatureSample( TEMPERATURE_CONTROL* temp_control_handle, int32_t sample );
void modifyRast( TEMPERATURE_CONTROL* temp_control_handle, uint8_t rast_index, int32_t temperature_delta,
                 int32_t time_delta );
void setRast( TEMPERATURE_CONTROL* temp_control_handle, uint8_t rast_index, int32_t temperature, int32_t time,
              RAST_TYPES type );
void modifyRunMode( TEMPERATURE_CONTROL* temp_control_handle, uint8_t run_mode_command );
uint32_t getUsedRasten( TEMPERATURE_CONTROL* temp_control_handle );
int32_t getTotalTimeRasten( TEMPERATURE_CONTROL* temp_control_handle );
int32_t getTotalTimeRemainingRasten( TEMPERATURE_CONTROL* temp_control_handle );

void printTempControlState( TEMPERATURE_CONTROL* temp_control_handle );

#endif /* TEMP_CONTROL_H_ */
