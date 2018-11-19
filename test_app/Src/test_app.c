#include "FreeRTOS.h"
#include "task.h"
#include "test_app.h"

#define STACK_SIZE 200

StaticTask_t xTaskBuffer;

StackType_t xStack[ STACK_SIZE ];

void vTaskCode( void* pvParameters )
{
	TickType_t lastWakeTime;
  configASSERT( (uint32_t)pvParameters == 1UL );

  lastWakeTime = xTaskGetTickCount();
  for ( ;; )
  {
	  vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(1000));
  }

  // We should never get here
  vTaskDelete(NULL);
}

void vFunction()
{
  TaskHandle_t xHandle = NULL;

  xHandle = xTaskCreateStatic(  //
      vTaskCode,                //
      "Test-Task",              //
      STACK_SIZE,               //
      (void*)1,                 //
      tskIDLE_PRIORITY,         //
      xStack,                   //
      &xTaskBuffer              //
  );

  vTaskSuspend( xHandle );
}
