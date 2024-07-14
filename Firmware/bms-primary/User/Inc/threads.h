#ifndef THREADS_H_
#define THREADS_H_

#include "user.hpp"

void ThreadsStart();

/* Defines */


/* Periodic thread function prototypes */
void ReadVoltageThread(void *argument);
void ReadCurrentThread(void *argument);
void ReadTemperaturePeriodic(void *argument);
void BroadcastPeriodic(void *argument);

/* Regular thread function prototypes */
void ReadTemperatureThread(void *argument);
void BroadcastThread(void *argument);
void ErrorThread(void* argument);

/* Periodic threads */
extern osTimerId_t voltage_timer_id;
extern osTimerId_t current_timer_id;
extern osTimerId_t temperature_timer_id;
extern osTimerId_t broadcast_timer_id;

/* Regular task definitions */
extern osThreadId_t thermistor_thread_id;
extern osThreadId_t broadcast_thread_id;
extern osThreadId_t contactors_thread_id;

/* Event flags */
extern osEventFlagsId_t read_temperature_event;
extern osEventFlagsId_t broadcast_event;
extern osEventFlagsId_t error_event;

/* Mutexes */
extern osMutexId_t adc0_mutex_id;
extern osMutexId_t amplifier_mutex_id;
extern osMutexId_t i2c4_mutex_id;
extern osMutexId_t logger_mutex_id;
extern osMutexId_t current_integral_mutex_id;
extern osMutexId_t contactor_mutex_id;

/* Callbacks */
//extern void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void VCUFrameCallback(uint8_t *data);
void SecondaryFrame0Callback(uint8_t *data);
void DriverControls1Callback(uint8_t *data);


#endif  /* THREADS_H_ */