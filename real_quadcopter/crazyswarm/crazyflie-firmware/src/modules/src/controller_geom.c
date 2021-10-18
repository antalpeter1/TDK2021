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
Geometric tracking controller.
*/

#include <math.h>

#include "param.h"
#include "log.h"
#include "math3d.h"
#include "stabilizer.h"
#include "physicalConstants.h"
#include "controller_geom.h"
#include "controller_flip.h"


// Inertia matrix components
static float Ixx = 0.000014;
static float Izz = 0.0000217;

static float t = 0;
static float dt = (float)(1.0f/ATTITUDE_RATE);

const static float thrust_scale = 132000; // 119460.0; TODO: find a solution (scale gains somehow)
const static float rollpitch_scale = 7345302.2678;
const static float yaw_scale = 4759362.5498;

static float kr_xy = 0.4;
static float kr_z = 1.25;
static float kv_xy = 0.2;
static float kv_z = 0.4;
static float kR_xy = 0.0095; // 70000/rollpitch_scale;
static float kR_z = 0.0126; // 60000/yaw_scale
static float kw_xy = 0.0027; // 20000/rollpitch_scale;
static float kw_z = 0.0025; // 12000/yaw_scale
static float kd_omega_rp = 0.000027; // 200/rollpitch_scale

// Logging variables

static float cmd_thrust_g;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;

static float q0;
static float q2;
static struct quat q;
static float timescale = 1;
static struct vec eR, ew, wd;
static float p_des, p;

static float prev_omega_roll;
static float prev_omega_pitch;
static float prev_setpoint_omega_roll;
static float prev_setpoint_omega_pitch;
static float err_d_roll = 0;
static float err_d_pitch = 0;

static bool mode = false;  // 0: position control, 1: attitude control
static float thrust_temp;

static float g_vehicleMass = 0.027;


void controllerGeomReset(void)
{
  t = 0;
}

void controllerGeomInit(void)
{
  controllerGeomReset();
}

bool controllerGeomTest(void)
{
  return true;
}

void controllerGeom(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
      return;
    }

  struct vec er, ev, r1, r2, r3, M;
  struct mat33 Rd;
  float yaw_des = radians(setpoint->attitude.yaw);
  struct vec setpointPos = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
  struct vec setpointVel = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);
  struct vec statePos = mkvec(state->position.x, state->position.y, state->position.z);
  struct vec stateVel = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);
  float T;

  // Position Error (ep)
  er = vsub(statePos, setpointPos);

  // Velocity Error (ev)
  ev = vsub(stateVel, setpointVel);

  struct vec target_thrust = vzero();
  target_thrust.x = -kr_xy*er.x - kv_xy*ev.x + g_vehicleMass*setpoint->acceleration.x;
  target_thrust.y = -kr_xy*er.y - kv_xy*ev.y + g_vehicleMass*setpoint->acceleration.y;
  target_thrust.z = -kr_z*er.z - kv_z*ev.z + g_vehicleMass*(setpoint->acceleration.x + GRAVITY_MAGNITUDE);
  /* if (setpoint->mode.x == modeAbs) {
    target_thrust.x = -kr_xy*er.x - kv_xy*ev.x + g_vehicleMass*setpoint->acceleration.x;
    target_thrust.y = -kr_xy*er.y - kv_xy*ev.y + g_vehicleMass*setpoint->acceleration.y;
    target_thrust.z = -kr_z*er.z - kv_z*ev.z + g_vehicleMass*(setpoint->acceleration.x + GRAVITY_MAGNITUDE);
  } else {
    target_thrust.x = -sinf(radians(setpoint->attitude.pitch));
    target_thrust.y = -sinf(radians(setpoint->attitude.roll));
    if (setpoint->mode.z == modeAbs) {
      target_thrust.z = -kr_z*er.z - kv_z*ev.z + g_vehicleMass*GRAVITY_MAGNITUDE;
    } else {
      target_thrust.z = 1;
    }
  }*/
  
  if (!mode) {      // position control
    r3 = vnormalize(target_thrust);
    r2 = vnormalize(vcross(r3,mkvec(cosf(yaw_des), sinf(yaw_des), 0)));
    r1 = vcross(r2, r3);
    Rd = mcolumns(r1, r2, r3);
    wd = mkvec(radians(setpoint->attitudeRate.roll), -radians(setpoint->attitudeRate.pitch), radians(setpoint->attitudeRate.yaw));
  } else {
    t = getTime();
    T = t/timescale;
    // flip trajectory
    q0 = 1.998f/(1.0f+expf(-20.0f*(T-0.45f)))-0.999f;
    q2 = sqrtf(1.0f-q0*q0);
    float dq0 = -(39.9f * expf(-20.0f * (T - 0.45f))) / ( (expf(-20.0f * (T - 0.45f)) + 1.0f) * (expf(-20.0f * (T - 0.45f)) + 1.0f) );
    struct quat qd = mkquat(0, q2, 0, q0);
    struct vec eul_des = quat2rpy(qd);
    p_des = eul_des.y;
    Rd = quat2rotmat(qd);
    wd = mkvec(0.0f, -2.0f*dq0/sqrtf(1.0f-q0*q0), 0.0f);
  }
  q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
  struct vec eul = quat2rpy(q);
  p = eul.y;
  struct mat33 R = quat2rotmat(q);

  struct mat33 eR1 = mmul(mtranspose(Rd),R);
  struct mat33 eR2 = mmul(mtranspose(R),Rd);
  
  float trace = eR1.m[0][0] + eR1.m[1][1] + eR1.m[2][2];
  trace = -0.75;

  struct mat33 eRm = msub(eR1,eR2);

  eR.x = 1.0f/(2.0f*sqrtf(1+trace))*eRm.m[2][1];
  eR.y = -1.0f/(2.0f*sqrtf(1+trace))*eRm.m[0][2];
  eR.z = 1.0f/(2.0f*sqrtf(1+trace))*eRm.m[1][0];

  float stateAttitudeRateRoll = radians(sensors->gyro.x);
  float stateAttitudeRatePitch = -radians(sensors->gyro.y);
  float stateAttitudeRateYaw = radians(sensors->gyro.z);

  struct vec w = mkvec(stateAttitudeRateRoll, stateAttitudeRatePitch, stateAttitudeRateYaw);

  // struct vec w_target = mvmul(eR2,wd);

  ew = vsub(w,wd);


  if (prev_omega_roll == prev_omega_roll) { /*d part initialized*/
    if (!mode){
      err_d_roll = ((radians(setpoint->attitudeRate.roll) - prev_setpoint_omega_roll) - (stateAttitudeRateRoll - prev_omega_roll)) / dt;
      err_d_pitch = (-(radians(setpoint->attitudeRate.pitch) - prev_setpoint_omega_pitch) - (stateAttitudeRatePitch - prev_omega_pitch)) / dt;
      prev_omega_roll = stateAttitudeRateRoll;
      prev_omega_pitch = stateAttitudeRatePitch;
      prev_setpoint_omega_roll = radians(setpoint->attitudeRate.roll);
      prev_setpoint_omega_pitch = radians(setpoint->attitudeRate.pitch);
    } else {
      err_d_roll = clamp(((wd.x - prev_setpoint_omega_roll) - (stateAttitudeRateRoll - prev_omega_roll)) / dt, -200, 200);
      err_d_pitch = clamp(((wd.y - prev_setpoint_omega_pitch) - (stateAttitudeRatePitch - prev_omega_pitch)) / dt, -200, 200);
      prev_omega_roll = stateAttitudeRateRoll;
      prev_omega_pitch = stateAttitudeRatePitch;
      prev_setpoint_omega_roll = wd.x;
      prev_setpoint_omega_pitch = wd.y;
    }
  }
  

  struct vec cross = vcross(w, mkvec(Ixx*w.x, Ixx*w.x, Izz*w.z));
  // cross = vzero();

  M.x = cross.x - kR_xy * eR.x - kw_xy * ew.x + kd_omega_rp * err_d_roll;
  M.y = cross.y - kR_xy * eR.y - kw_xy * ew.y + kd_omega_rp * err_d_pitch;
  M.z = -kR_z  * eR.z - kw_z  * ew.z;

  float thrust = 0;
  if (!mode){
    thrust = vdot(target_thrust, mcolumn(R,2));
  } else {
    thrust_temp = vdot(target_thrust, mcolumn(R,2))*thrust_scale;
    thrust = (0.22f * cosf(2.0f * 3.14f * t / (0.9f*timescale)) + 0.4f);
  }
  
 
  if (setpoint->mode.z == modeDisable) {
    control->thrust = setpoint->thrust;
  } else {
    control->thrust = thrust * thrust_scale;
  }
  cmd_thrust_g = thrust*thrust_scale;
  r_roll = radians(sensors->gyro.x);
  r_pitch = -radians(sensors->gyro.y);
  r_yaw = radians(sensors->gyro.z);

  if (control->thrust > 0) {
    control->roll = clamp(M.x * rollpitch_scale, -32000, 32000);
    control->pitch = clamp(M.y * rollpitch_scale, -32000, 32000);
    control->yaw = clamp(-M.z * yaw_scale, -32000, 32000);
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;
  } else {
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;
  }
}

void setMode(bool val) {
  mode = val;
}

PARAM_GROUP_START(ctrlGeom)
PARAM_ADD(PARAM_FLOAT, kr_xy, &kr_xy)
PARAM_ADD(PARAM_FLOAT, kv_xy, &kv_xy)
PARAM_ADD(PARAM_FLOAT, kr_z, &kr_z)
PARAM_ADD(PARAM_FLOAT, kv_z, &kv_z)
PARAM_ADD(PARAM_FLOAT, kR_xy, &kR_xy)
PARAM_ADD(PARAM_FLOAT, kR_z, &kR_z)
PARAM_ADD(PARAM_FLOAT, kw_xy, &kw_xy)
PARAM_ADD(PARAM_FLOAT, kw_z, &kw_z)
PARAM_ADD(PARAM_FLOAT, kd_omega_rp, &kd_omega_rp)
PARAM_ADD(PARAM_FLOAT, timescale, &timescale)
PARAM_ADD(PARAM_UINT8, mode, &mode)
PARAM_ADD(PARAM_FLOAT, mass, &g_vehicleMass)
PARAM_GROUP_STOP(ctrlGeom)

LOG_GROUP_START(ctrlGeom)
LOG_ADD(LOG_FLOAT, cmd_thrust_g, &cmd_thrust_g)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
LOG_ADD(LOG_FLOAT, t, &t)
LOG_ADD(LOG_FLOAT, thrust, &thrust_temp)
LOG_ADD(LOG_FLOAT, p_des, &p_des)
LOG_ADD(LOG_FLOAT, p, &p)
LOG_ADD(LOG_FLOAT, wdy, &wd.y)
LOG_ADD(LOG_FLOAT, err_d_pitch, &err_d_pitch)
LOG_ADD(LOG_FLOAT, qw, &q.w)
LOG_ADD(LOG_FLOAT, eRy, &eR.y)
LOG_ADD(LOG_FLOAT, ewy, &ew.y)
LOG_GROUP_STOP(ctrlGeom)
