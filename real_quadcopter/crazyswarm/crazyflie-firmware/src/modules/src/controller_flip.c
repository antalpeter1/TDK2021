/*
The MIT License (MIT)

Copyright (c) 2018 Wolfgang Hoenig and James Alan Preiss

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
Controller for executing a flip maneuver.
*/

#include <math.h>

#include "param.h"
#include "log.h"
#include "math3d.h"
#include "controller_flip.h"
#include "controller_pid.h"
#include "stabilizer.h"
#include "position_controller.h"
#include "attitude_controller.h"
#include "controller_mellinger.h"
#include "controller_geom.h"
#include "crtp_localization_service.h"
#include "crtp_commander_high_level.h"

// Flip parameters
static float T0 = 1.0;
static float T1 = 0.6;

// const static float thrust_scale = 132000;

static float time = 0;
static float dt = (float)(1.0f/ATTITUDE_RATE);

static bool isFlipControl = false;
static bool wasFlipControl = false;

static int controller_type = 1; // 1: PID, 2: Mellinger

static float x_bound = 1;
static float y_bound = 1;
static float z_bound = 0.2;
static float vel_bound = 0.5;
static float recovery_setpoint = 0.6;
static float z0 = 1.4;
static bool traj_started = false;

// Logging variables

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;


void controllerFlipReset(void)
{
  time = 0;
}

void controllerFlipInit(void)
{
  controllerFlipReset();
  controllerPidInit();          // we use the PID controller
  controllerGeomInit();
}

bool controllerFlipTest(void)
{
  return true;
}

void controllerFlip(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
      return;
    }

  if (!isFlipControl) {
    if (wasFlipControl)
    {
      if(((float)fabs(state->position.x) > x_bound) || ((float)fabs(state->position.y) > y_bound || state->position.z < z_bound))
      {
        stabilizerSetEmergencyStop();
      }
      if((float)fabs(state->velocity.z) < vel_bound)
      {
        //wasFlipControl = false;
      }
      /*setpoint->position.z = recovery_setpoint;
      setpoint->mode.x = modeDisable;
      setpoint->mode.y = modeDisable;
      setpoint->mode.yaw = modeDisable;
      setpoint->attitude.roll = 0;
      setpoint->attitude.pitch = 0;*/
    }
    switch (controller_type)
    {
    case 1:
      controllerGeom(control, setpoint, sensors, state, tick);
      break;
    case 2:
      controllerPid(control, setpoint, sensors, state, tick);
    default:
      break;
    }
  } else {
    if (time > T0 + T1) {         // after last section
      isFlipControl = false;
      wasFlipControl = true;
      time = 0;
      setMode(false);
      switch (controller_type)
      {
      case 1:
        break;
      case 2:
        controllerPidInit();
      default:
        break;
      }
    } else if (time > T0) {
      if (!traj_started){
        crtpCommanderHighLevelStartTrajectory(0, 1.0f, false, false);
        traj_started = true;
        setMode(true);
      }           
      controllerGeom(control, setpoint, sensors, state, tick);
      time += dt;
    } else {
      setpoint->position.z = 0.6f + (z0-0.6f)*time;
      setpoint->attitude.roll = 0;
      setpoint->attitude.pitch = 0;
      // setpoint->attitude.yaw = 0;
      switch (controller_type)
      {
      case 1:
        controllerGeom(control, setpoint, sensors, state, tick);
        break;
      case 2:
        controllerPid(control, setpoint, sensors, state, tick);
      default:
        controllerPid(control, setpoint, sensors, state, tick); // Maybe rewrite later
        break;
      }      
      time += dt;
    }
  }
}

float getTime() {
  return time-T0;
}

PARAM_GROUP_START(ctrlFlip)
PARAM_ADD(PARAM_FLOAT, T0, &T0)
PARAM_ADD(PARAM_FLOAT, T1, &T1)
PARAM_ADD(PARAM_FLOAT, z0, &z0)
PARAM_ADD(PARAM_UINT8, isFlipControl, &isFlipControl)
PARAM_ADD(PARAM_UINT8, wasFlipControl, &wasFlipControl)
PARAM_ADD(PARAM_UINT8, controller_type, &controller_type)
PARAM_ADD(PARAM_FLOAT, rec_sp_z, &recovery_setpoint)
PARAM_ADD(PARAM_FLOAT, xmax, &x_bound)
PARAM_ADD(PARAM_FLOAT, ymax, &y_bound)
PARAM_ADD(PARAM_FLOAT, zmin, &z_bound)
PARAM_ADD(PARAM_FLOAT, vel_threshold, &vel_bound) 
PARAM_GROUP_STOP(ctrlFlip)

LOG_GROUP_START(ctrlFlip)
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
LOG_GROUP_STOP(ctrlFlip)
