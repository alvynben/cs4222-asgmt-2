/*
 * Copyright (C) 2015, Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>

#include "contiki.h"
#include "sys/rtimer.h"

#include "board-peripherals.h"
#include "buzzer.h"

#include <stdint.h>

PROCESS(process_rtimer, "RTimer");
AUTOSTART_PROCESSES(&process_rtimer);

enum State
{
  _IDLE,
  _WAIT,
  _BUZZ,
  _PRE_IDLE
};

static enum State state = _PRE_IDLE;
static struct rtimer timer_rtimer;
static int counter_rtimer;
static int prev_light_reading = 0;
static bool buzzer_timer_active = false;
static bool not_first_round = false;
static rtimer_clock_t start_time;

static rtimer_clock_t timeout_rtimer = RTIMER_SECOND / 4;
static rtimer_clock_t two_second_rtimer = RTIMER_SECOND * 2;
static rtimer_clock_t four_second_rtimer = RTIMER_SECOND * 4;
#define IMU_THRESHOLD 150
#define LIGHT_DELTA_THRESHOLD 30000
#define min(a, b) ((a) < (b) ? (a) : (b))
/*---------------------------------------------------------------------------*/
static void init_opt_reading(void);
static void get_light_reading(void);
static void init_mpu_reading(void);
static void get_mpu_reading(void);

static void
print_mpu_reading(int reading)
{
  if (reading < 0)
  {
    printf("-");
    reading = -reading;
  }

  printf("%d", reading);
}

static void
print_current_time()
{
  rtimer_clock_t now = RTIMER_NOW();
  int s, ms1, ms2, ms3;
  s = now / RTIMER_SECOND;
  ms1 = (now % RTIMER_SECOND) * 10 / RTIMER_SECOND;
  ms2 = ((now % RTIMER_SECOND) * 100 / RTIMER_SECOND) % 10;
  ms3 = ((now % RTIMER_SECOND) * 1000 / RTIMER_SECOND) % 10;
  printf("Current Time: %d.%d%d%d (sec) \n", s, ms1, ms2, ms3);
}
/*---------------------------------------------------------------------------*/
void do_rtimer_timeout(struct rtimer *timer, void *ptr)
{
  print_current_time();
  if (state == _IDLE)
  {
    get_light_reading();
  }
  else if (state == _WAIT)
  {
    if (RTIMER_NOW() > start_time + four_second_rtimer)
    {
    if (!not_first_round)
    {
      not_first_round = true;
    }
      state = _BUZZ;
      buzzer_timer_active = false;
    }
    get_light_reading();
  }
  else if (state == _BUZZ)
  {
    if (RTIMER_NOW() > start_time + two_second_rtimer)
    {
      state = _WAIT;
      buzzer_timer_active = false;
    }
    get_light_reading();
  }
  else if (state == _PRE_IDLE)
  {
    get_mpu_reading();
  }
  else
  {
    init_opt_reading();
  }
}

// Light Sensor

static void
get_light_reading()
{
  int value;

  value = opt_3001_sensor.value(0);
  if (value != CC26XX_SENSOR_READING_ERROR)
  {
    printf("OPT: Light=%d.%02d lux\n", value / 100, value % 100);
    if (value - prev_light_reading > LIGHT_DELTA_THRESHOLD || prev_light_reading - value > LIGHT_DELTA_THRESHOLD)
    {
      if (state == _IDLE)
      {
        state = _BUZZ;
      }
      else if (not_first_round && (state == _WAIT || state == _BUZZ))
      {
        not_first_round = false;
        state = _PRE_IDLE;
        buzzer_timer_active = false;
        buzzer_stop();
      }
    }

    prev_light_reading = value;
  }
  else
  {
    printf("OPT: Light Sensor's Warming Up\n\n");
  }
  init_opt_reading();
}

static void
init_opt_reading(void)
{
  SENSORS_ACTIVATE(opt_3001_sensor);
}

// MPU Sensor

static void
get_mpu_reading()
{
  int value;

  // printf("MPU Gyro: X=");
  // value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_X);
  // print_mpu_reading(value);
  // printf(" deg/sec\n");

  // printf("MPU Gyro: Y=");
  // value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Y);
  // print_mpu_reading(value);
  // printf(" deg/sec\n");

  // printf("MPU Gyro: Z=");
  // value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Z);
  // print_mpu_reading(value);
  // printf(" deg/sec\n");

  // printf("MPU Acc: X=");
  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);
  // print_mpu_reading(value);
  if (value < -IMU_THRESHOLD || value > IMU_THRESHOLD)
  {
    state = _IDLE;
  }
  printf("MPU Acc: X=%d", value);
  printf("value < IMU_THRESHOLD: %d", value < IMU_THRESHOLD);
  printf("State: %d\n", state);
  // printf(" G\n");

  // printf("MPU Acc: Y=");
  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);
  // print_mpu_reading(value);
  if (value < -IMU_THRESHOLD || value > IMU_THRESHOLD)
  {
    state = _IDLE;
  }
  printf("MPU Acc: Y=%d", value);
  printf("value < IMU_THRESHOLD: %d", value < IMU_THRESHOLD);
  printf("State: %d\n", state);
  // printf(" G\n");

  // printf("MPU Acc: Z=");
  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);
  // print_mpu_reading(value);
  if (value < -IMU_THRESHOLD || value > IMU_THRESHOLD)
  {
    state = _IDLE;
  }
  printf("MPU Acc: Z=%d", value);
  printf("value < IMU_THRESHOLD: %d", value < IMU_THRESHOLD);
  printf("State: %d\n", state);
  // printf(" G\n");

  init_mpu_reading();
}

static void
init_mpu_reading(void)
{
  mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL);
}

// RTimer Process

PROCESS_THREAD(process_rtimer, ev, data)
{
  PROCESS_BEGIN();

  // Initialize sensors and buzzer
  init_opt_reading();
  buzzer_init();
  init_mpu_reading();

  while (1)
  {
    if (state == _IDLE)
    {
      rtimer_set(&timer_rtimer, RTIMER_NOW() + timeout_rtimer, 0, do_rtimer_timeout, NULL);
    }
    else if (state == _PRE_IDLE)
    {
      rtimer_set(&timer_rtimer, RTIMER_NOW() + timeout_rtimer, 0, do_rtimer_timeout, NULL);
    }
    else if (state == _WAIT)
    {
      if (!buzzer_timer_active)
      {
        buzzer_stop();
        buzzer_timer_active = true;
        start_time = RTIMER_NOW();
      }
      rtimer_set(&timer_rtimer, min(RTIMER_NOW() + timeout_rtimer, start_time + four_second_rtimer), 0, do_rtimer_timeout, NULL);
    }
    else if (state == _BUZZ)
    {
      if (!buzzer_timer_active)
      {
        buzzer_start(2093);
        buzzer_timer_active = true;
        start_time = RTIMER_NOW();
      }
      rtimer_set(&timer_rtimer, min(RTIMER_NOW() + timeout_rtimer, start_time + two_second_rtimer), 0, do_rtimer_timeout, NULL);
    }
    PROCESS_YIELD();
  }

  printf("END\n");
  PROCESS_END();
}