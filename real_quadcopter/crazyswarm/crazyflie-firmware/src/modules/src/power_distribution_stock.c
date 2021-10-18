/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * power_distribution_stock.c - Crazyflie stock power distribution code
 */
#define DEBUG_MODULE "PWR_DIST"

#include "power_distribution.h"

#include <string.h>
#include "log.h"
#include "param.h"
#include "num.h"
#include "platform.h"
#include "motors.h"
#include "debug.h"
#include "position_controller.h"

static bool motorSetEnable = false;

static struct {
  uint32_t m1;
  uint32_t m2;
  uint32_t m3;
  uint32_t m4;
} motorPower;

static struct {
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
} motorPowerSet;

// Code of Peter
static struct {
  float m1;
  float m2;
  float m3;
  float m4;
} average;
static float average_m = 0;

/*static struct {
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
} actualPowerSet;*/

static float num = 0.0;
static bool isAverage = false;
static bool isFeedForward = false;
//


void powerDistributionInit(void)
{
  motorsInit(platformConfigGetMotorMapping());
  // Code of Peter
  num = 0;
  average.m1 = 0;
  average.m2 = 0;
  average.m3 = 0;
  average.m4 = 0;
  isFeedForward = false;
  //
}

bool powerDistributionTest(void)
{
  bool pass = true;

  pass &= motorsTest();

  return pass;
}

#define limitThrust(VAL) limitUint16(VAL)

void powerStop()
{
  motorsSetRatio(MOTOR_M1, 0);
  motorsSetRatio(MOTOR_M2, 0);
  motorsSetRatio(MOTOR_M3, 0);
  motorsSetRatio(MOTOR_M4, 0);
}

void powerDistribution(const control_t *control)  // Motor power: PWM -> 0...65535
{
  #ifdef QUAD_FORMATION_X
    int16_t r = control->roll; // / 2.0f;
    int16_t p = control->pitch; // / 2.0f;
    motorPower.m1 = (control->thrust - r + p + control->yaw);
    motorPower.m2 = (control->thrust - r - p - control->yaw);
    motorPower.m3 = (control->thrust + r - p + control->yaw);
    motorPower.m4 = (control->thrust + r + p - control->yaw);
  #else // QUAD_FORMATION_NORMAL
    motorPower.m1 = limitThrust(control->thrust + control->pitch +
                               control->yaw);
    motorPower.m2 = limitThrust(control->thrust - control->roll -
                               control->yaw);
    motorPower.m3 =  limitThrust(control->thrust - control->pitch +
                               control->yaw);
    motorPower.m4 =  limitThrust(control->thrust + control->roll -
                               control->yaw);
  #endif

  // Code of Peter
  if (isAverage)
  {
    average.m1 = num/(num+1)*average.m1 + 1/(num+1)*(float)motorPower.m1;
    average.m2 = num/(num+1)*average.m2 + 1/(num+1)*(float)motorPower.m2;
    average.m3 = num/(num+1)*average.m3 + 1/(num+1)*(float)motorPower.m3;
    average.m4 = num/(num+1)*average.m4 + 1/(num+1)*(float)motorPower.m4;
    num++;
  }
  //

  if (motorSetEnable)
  {
    motorsSetRatio(MOTOR_M1, motorPowerSet.m1);
    motorsSetRatio(MOTOR_M2, motorPowerSet.m2);
    motorsSetRatio(MOTOR_M3, motorPowerSet.m3);
    motorsSetRatio(MOTOR_M4, motorPowerSet.m4);
    return;
  }

  // Code of Peter
  else if (!isAverage && isFeedForward)
  { 
    average_m = (average.m1 + average.m2 + average.m3 + average.m4)/(float)4.0;
    //setThrustBase(average_m);
    motorPower.m1 = ((int32_t)((float)(motorPower.m1) * average.m1 / average_m));
    motorPower.m2 = ((int32_t)((float)(motorPower.m2) * average.m2 / average_m));
    motorPower.m3 = ((int32_t)((float)(motorPower.m3) * average.m3 / average_m));
    motorPower.m4 = ((int32_t)((float)(motorPower.m4) * average.m4 / average_m));
    /*motorsSetRatio(MOTOR_M1, actualPowerSet.m1);
    motorsSetRatio(MOTOR_M2, actualPowerSet.m2);
    motorsSetRatio(MOTOR_M3, actualPowerSet.m3);
    motorsSetRatio(MOTOR_M4, actualPowerSet.m4);*/
  }
  //

  motorsSetRatio(MOTOR_M1, limitThrust(motorPower.m1));
  motorsSetRatio(MOTOR_M2, limitThrust(motorPower.m2));
  motorsSetRatio(MOTOR_M3, limitThrust(motorPower.m3));
  motorsSetRatio(MOTOR_M4, limitThrust(motorPower.m4));    
}

void setFeedForward(){
  isFeedForward = true;
}

PARAM_GROUP_START(motorPowerSet)
PARAM_ADD(PARAM_UINT8, enable, &motorSetEnable)
PARAM_ADD(PARAM_UINT16, m1, &motorPowerSet.m1)
PARAM_ADD(PARAM_UINT16, m2, &motorPowerSet.m2)
PARAM_ADD(PARAM_UINT16, m3, &motorPowerSet.m3)
PARAM_ADD(PARAM_UINT16, m4, &motorPowerSet.m4)
PARAM_ADD(PARAM_UINT8, isAv, &isAverage)               // set true to average the motor PWMs
PARAM_ADD(PARAM_UINT8, isFF, &isFeedForward)       // set true to feedforward the averaged PWMs
PARAM_GROUP_STOP(motorPowerSet)

LOG_GROUP_START(motor)
LOG_ADD(LOG_UINT16, m1, &motorPower.m1)
LOG_ADD(LOG_UINT16, m2, &motorPower.m2)
LOG_ADD(LOG_UINT16, m3, &motorPower.m3)
LOG_ADD(LOG_UINT16, m4, &motorPower.m4)
/*LOG_ADD(LOG_UINT16, am1, &actualPowerSet.m1)
LOG_ADD(LOG_UINT16, am2, &actualPowerSet.m2)
LOG_ADD(LOG_UINT16, am3, &actualPowerSet.m3)
LOG_ADD(LOG_UINT16, am4, &actualPowerSet.m4)*/
LOG_GROUP_STOP(motor)
