// =====================================================================
//  Daidalos landing guidance — TVC nozzle angle and thrust-clamp command
//
//  Pure algorithm: no Arduino headers, no dynamic allocation, no I/O.
//  Compiles unchanged for the RP2350 and for the host test in ../test/,
//  which is what lets the port be verified against the Python model
//  rather than merely compiled.
//
//  Target: RP2350 (Cortex-M33, 150 MHz, SINGLE-precision FPU).
//  Everything is `float`. A stray `double` here is 10-50x slower because
//  the M33 has no double-precision hardware — see the notes in guidance.cpp.
//
//  Vehicle: solid landing motor in the nose, fired downward during a
//  nose-first descent. A thrust-diverting clamp cuts axial thrust without
//  net side force. Clamped impulse is WASTED and the ~1.7 s burn cannot
//  be extended.
//
//  THREE DIMENSIONS, TWO GIMBAL AXES. The nozzle deflects about two
//  perpendicular axes. The third rotational degree of freedom -- roll
//  about the body's own long axis -- is not actuated and is assumed to
//  stay at zero. That assumption is self-consistent rather than
//  optimistic: every torque in this model is perpendicular to the body
//  axis, so a vehicle with no roll rate never acquires one, and two axes
//  are then enough for the whole attitude problem.
//
//  If your airframe DOES roll -- a canted fin, spin left over from ascent
//  -- this code cannot correct it, and worse, the two servo axes will
//  smear across each other as the body turns underneath them. Check that
//  before flying.
// =====================================================================
#ifndef DAIDALOS_GUIDANCE_H
#define DAIDALOS_GUIDANCE_H

#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------------------
//  Vehicle / environment constants (must match the simulation model)
// ---------------------------------------------------------------------
#define DG_G              9.81f
#define DG_RHO            1.225f
#define DG_AREA           0.00865901f   // m^2, 105 mm body
#define DG_CD_AXIAL       1.0f
#define DG_CN_ALPHA       2.0f          // normal force slope, C_N = CNA*sin(a)
#define DG_L_BODY         1.40f         // m
#define DG_L_GIMBAL       0.70f         // CG -> nose nozzle, m
#define DG_L_CP           0.35f         // CG -> centre of pressure (aft), m
#define DG_M_INITIAL      3.20f         // kg, wet
#define DG_M_PROP         0.330f        // kg of grain
#define DG_I_TOTAL        286.83f       // N.s, full-throttle impulse
#define DG_BURN_T         1.75f         // s, nominal burn duration
#define DG_MASS_PER_NS    0.00115051f   // kg consumed per N.s of RAW impulse
#define DG_H_START        85.0f         // m, guidance arming altitude

// Actuators
#define DG_SERVO_MAX_DEG  10.0f         // TVC servo travel limit
#define DG_TVC_RATIO      2.0f          // servo deg per nozzle deg
#define DG_SERVO_RATE     500.0f        // deg/s
#define DG_SERVO_QUANT    0.15f         // deg
#define DG_CLAMP_MIN      0.10f
#define DG_CLAMP_MAX      1.00f
#define DG_CLAMP_RATE     12.84f        // clamp fraction per second

// Ignition
#define DG_IGN_DELAY_LO   0.10f
#define DG_IGN_DELAY_HI   0.30f

// Attitude loop gains (see tvc_gains.json / tools/precalibrate.py).
// omega_n is a BANDWIDTH in rad/s and zeta a damping ratio: both survive the
// dynamic inversion, which is why one fixed pair is valid across the whole burn.
#define DG_OMEGA_N        9.0f
#define DG_ZETA           1.0f
#define DG_KI_ATT         0.0f          // calibrated to zero: the rate loop holds
                                        // attitude against nozzle bias unaided

// Rates
#define DG_CTRL_HZ        200.0f
#define DG_CTRL_DT        (1.0f / DG_CTRL_HZ)
#define DG_PLAN_HZ        10.0f

// ---------------------------------------------------------------------
//  Interfaces
// ---------------------------------------------------------------------
typedef enum {
    DG_PHASE_IDLE = 0,     // disarmed
    DG_PHASE_COAST,        // armed, falling, motor off — NO control authority
    DG_PHASE_BURN,         // motor lit, closed loop
    DG_PHASE_DONE          // touchdown or burnout with the vehicle down
} dg_phase_t;

// Navigation solution, supplied by the fusion processor.
// Frame: WORLD, z up, x and y horizontal, origin at the pad.
// vz is NEGATIVE while descending.
//
// Attitude is the body-axis UNIT VECTOR b -- the direction thrust pushes the
// vehicle, i.e. the tail-to-nose axis reversed, pointing roughly up during the
// descent. Not Euler angles: a unit vector has no convention to get backwards,
// no wrap and no gimbal lock, and it is what the control law consumes directly.
// A fusion filter carrying a quaternion produces it in one rotation:
//     b = R(q) * (body axis expressed in body coordinates)
//
// w is the angular velocity in the SAME world frame, from the gyro, rotated out
// of body coordinates by the same R(q). Its component along b is the roll rate,
// which this vehicle cannot control; the guidance projects it out.
typedef struct {
    float h;            // m above the pad
    float vx, vy;       // m/s, world horizontal
    float vz;           // m/s, negative descending
    float bx, by, bz;   // body-axis unit vector, world frame
    float wx, wy, wz;   // rad/s, world frame (use the GYRO, not a differenced angle)
    float t;            // s, monotonic since power-up
    bool  valid;        // false blocks ignition and freezes the servos at zero
} dg_nav_t;

typedef struct {
    // Two servos. Axis 1 deflects the nozzle in the vertical plane containing
    // world +x, axis 2 in the plane containing world +y. With no roll those are
    // fixed directions in the airframe, which is what makes the mapping constant.
    float servo1_deg;   // TVC servo, x channel, clamped to +/-DG_SERVO_MAX_DEG
    float servo2_deg;   // TVC servo, y channel
    float clamp;        // axial thrust fraction command, DG_CLAMP_MIN..MAX
    bool  ignite;       // rising edge: fire the igniter
    // telemetry
    float tilt_cmd;     // rad, commanded tilt from vertical
    float tilt_err;     // rad, total angle between commanded and actual body axis
    float thrust_est;   // N
    float p_plan;       // current throttle-plan parameter, 0..4
    dg_phase_t phase;
} dg_out_t;

// ---------------------------------------------------------------------
//  API
//
//  Threading model on the RP2350:
//    core 0 : dg_step_attitude() at 200 Hz — never blocks, bounded work
//    core 1 : dg_step_planner()  as often as it likes — does ONE solver
//             iteration per call, so the load is amortised instead of
//             arriving as a multi-millisecond spike every 100 ms
//  The two share dg_state through a seqlock; no mutex in the 200 Hz path.
// ---------------------------------------------------------------------
void dg_init(void);

// Latch the entry condition. Call once, when the vehicle first passes
// DG_H_START on the way down. Selects the ignition altitude from a table.
void dg_arm(float vx0, float vz0);

// 200 Hz. Attitude control, ignition decision, actuator commands.
void dg_step_attitude(const dg_nav_t *nav, dg_out_t *out);

// Amortised planner. Safe to call at any rate >= DG_PLAN_HZ.
void dg_step_planner(const dg_nav_t *nav);

// Introspection, for the host test and for telemetry.
//
// NOTE ON THE `vx` ARGUMENTS BELOW. The guidance law and the whole trajectory
// projection are PLANAR even though the vehicle flies in 3-D, and `vx` is the
// horizontal SPEED. This is exact, not a simplification: every horizontal
// acceleration these functions apply (the -v/t_go term and drag) is anti-parallel
// to the horizontal velocity, so its direction never changes and the problem lives
// in the vertical plane containing it. The 3-D command is that planar solution
// rotated into the velocity's azimuth -- which dg_guidance_dir does, and which
// costs one square root. The expensive part, the planner, is untouched by the
// second axis.
float dg_h_ignite(void);          // commanded ignition altitude, m
float dg_thrust_ref(float tau);   // un-clamped thrust at burn fraction tau, N
float dg_guidance_tilt(float h, float vx, float vz, float mass,
                       float *sin_th, float *cos_th);   // planar; returns t_go
float dg_guidance_dir(float h, float vx, float vy, float vz, float mass,
                      float *ux, float *uy, float *uz); // 3-D; returns t_go
float dg_project_miss(float h, float vx, float vz, float tau,
                      float mass, float clamp_now, float p);
float dg_solve_p_full(float h, float vx, float vz, float tau,
                      float mass, float clamp_now);     // non-amortised, for test

// Orthonormal basis of the plane perpendicular to b: the two gimbal axes.
void  dg_gimbal_basis(float bx, float by, float bz,
                      float *u1x, float *u1y, float *u1z,
                      float *u2x, float *u2y, float *u2z);

#endif // DAIDALOS_GUIDANCE_H
