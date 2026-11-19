// =====================================================================
//  Daidalos landing guidance, version 2 — the LandSim_Light algorithm
//
//  A port of the flight algorithm of
//      https://github.com/Tomasraketak/LandSim_Light   (tvc_sim.py)
//  section "The flight algorithm, step by step" of its README. What the
//  vehicle decides, when, and with which numbers is the simulation's; the
//  tables in guidance_tables.h are GENERATED from it by
//  tools/gen_guidance_tables.py, and test/guid_test.cpp compares this file
//  against reference values computed by the simulation itself.
//
//  Pure algorithm: no Arduino headers, no dynamic allocation, no I/O. It
//  compiles unchanged for the RP2350 and for the host test.
//
//  Target: RP2350 (Cortex-M33, SINGLE-precision FPU). Everything is float.
//
//  ---------------------------------------------------------------------
//  WHAT IT DECIDES
//  ---------------------------------------------------------------------
//    200 Hz   where the thrust should point (ZEV law, drag credit, tilt cone)
//    200 Hz   two TVC servo angles (cascade attitude loop with an integral,
//             aero feedforward, gyroscopic decoupling, dynamic inversion)
//    200 Hz   four fin angles (steering the nozzle cannot do, roll damping,
//             crossflow cancellation, airbrake, stall limiter) with an
//             ON-LINE ESTIMATE of how strong the fins really are
//    10 Hz    the thrust-clamp level (receding-horizon forward simulation)
//    once     when to fire the igniter (table: apogee altitude x drift)
//
//  It also reports the two countdowns the rest of the vehicle sequences
//  on: time to the ignition command and time to touchdown.
//
//  ---------------------------------------------------------------------
//  FRAMES
//  ---------------------------------------------------------------------
//  World: z up, x and y horizontal, origin at the pad. vz is NEGATIVE while
//  descending. Attitude is two body-fixed unit vectors, never Euler angles:
//
//    b   the landing motor's thrust axis: the direction thrust pushes the
//        vehicle. Standing on the pad b = (0,0,-1); nose-first descent
//        b = (0,0,+1).
//    g   a transverse body axis marking the roll orientation: the axis TVC
//        servo 1 deflects the nozzle in, and the radial direction of fin 0.
//
//  Fin i sits at azimuth i*90 deg from g about b (r_i = cos*u1 + sin*u2,
//  u2 = b x u1). A POSITIVE deflection of fin i pushes the fin along
//  n_i = b x r_i. The fins are 0.50 m from the CG on the +b side (the tail).
// =====================================================================
#ifndef DAIDALOS_GUIDANCE_H
#define DAIDALOS_GUIDANCE_H

#include <stdint.h>
#include <stdbool.h>

#include "guidance_tables.h"

#define DG_VERSION        220          // 2.20 - thrust forecast, airbrake-aware drag, fin dCd params

// ---------------------------------------------------------------------
//  Fixed constants (the simulation's)
// ---------------------------------------------------------------------
#define DG_G              9.80665f
#define DG_RHO            1.225f

#define DG_SERVO_MAX_DEG  10.0f         // TVC servo travel limit
#define DG_TVC_RATIO      2.0f          // servo deg per nozzle deg
#define DG_SERVO_QUANT    0.15f         // deg
#define DG_CLAMP_MIN      0.10f
#define DG_CLAMP_MAX      1.00f
#define DG_CLAMP_RATE     12.84f        // clamp fraction per second
#define DG_THRUST_GATE    0.30f         // clamp held open below this fraction of peak

#define DG_H_TERM         3.0f          // terminal law below this altitude, m
#define DG_A_TERM         8.0f          // m/s^2, sets the terminal descent profile
#define DG_A_TERM_MAX     70.0f
#define DG_KP_VZ          3.0f
#define DG_T_SCHED_REF    100.0f        // N
#define DG_Q_SCHED_REF    700.0f        // Pa
#define DG_SCHED_LO       0.35f
#define DG_SCHED_HI       2.0f
#define DG_FIN_STALL_MARGIN 1.5f        // deg
#define DG_PLAN_DT        0.02f         // s, projection step in flight
#define DG_PLAN_NMAX      1500          // 30 s of projection, hard bound

#define DG_CTRL_HZ        200.0f
#define DG_CTRL_DT        (1.0f / DG_CTRL_HZ)
#define DG_PLAN_HZ        10.0f

// The Master reports "GUIDANCE" when the Navigation MCU says the ignition
// command has gone out. If it cannot hear the Navigation MCU it falls back
// on this altitude, so the Position MCU still stops trusting the barometer
// before the plume reaches it.
#define DG_H_HANDOVER_FALLBACK 60.0f

// ---------------------------------------------------------------------
//  Tunable parameters. Every one has the simulation's value as its default
//  (dg_params_default) and every one can be changed from the diagnostic
//  GUI before flight; see ariadna_params.h for names, units and limits.
// ---------------------------------------------------------------------
typedef struct {
    // vehicle
    float mass_gross;        // kg with propellant
    float mass_prop;         // kg
    // attitude loop, motor lit (nozzle)
    float wn_tvc, zeta_tvc, sched_tvc;
    float att_ki;            // 1/s, integral, nozzle only
    float att_i_lim;         // rad*s
    // attitude loop, motor unlit (fins)
    float wn_fin, zeta_fin, sched_fin;
    float roll_gain;         // rad/s
    float roll_max_deg;      // travel the roll channel may spend
    // guidance law
    float tilt_min_deg, tilt_slope, tilt_cap_deg;
    float alt_pad;           // m assumed LOWER than read until the lidar acquires
    float h_ign_bias;        // m added to the table's ignition altitude
    float h_ign_override;    // > 0: ignite at this altitude instead of the table
    // clamp planner
    float plan_target;       // m/s, touchdown speed aimed for (negative)
    float plan_margin;       // fraction of estimated thrust not counted on
    float plan_hold;         // s
    // thrust onset detection
    float onset_bump;        // m/s^2 rise of axial specific force = "motor lit"
    float onset_timeout;     // s after the command before onset is assumed
    float onset_assumed;     // s, the delay assumed in that case
    // fins
    float fin_enable;        // 0 = fins parked at zero for the whole flight
    float fin_max_deg;
    float fin_brake;         // 0 = no airbrake splay
    float fin_min_airspeed;  // m/s below which fins are not used for control
    float fin_drift_gain;    // deg of tilt per m/s of drift in free fall (0 = hold vertical)
    float fin_drift_max_deg;
    float fin_ctrl_lead;     // m above ignition where the fins hand over to the ZEV law
    float fin_ascent_mode;   // 0 parked, 1 roll damper, 2 roll + hold vertical
    float fin_adapt;         // 0 = fixed model, 1 = adapt effectiveness and trim
    float fin_gain0;         // initial effectiveness (1 = the CFD model)
    // the vehicle as the ignition search and the planner see it
    float cd_body;           // bare-body drag coefficient (fins are added from the CFD)
    float thrust_scale;      // multiplies the tabulated motor curve
    float ign_delay_pad;     // s of igniter delay the ignition altitude pads for
    float drag_adapt;        // 1 = scale the drag by what is measured in the free fall
    float plan_ts_tau;       // s, how long the planner expects a thrust deviation to last
    float fin_dcd_zero;      // fins' drag, neutral, as an addition to the body Cd
    float fin_dcd_brake;     // ... splayed as the free-fall airbrake
    float diameter;          // m
    // the ignition band table, from LandSim_Light (tbl.* parameters)
    float tbl_vz0, tbl_dvz;
    float tbl_hmin[DG_IGN_N];
    float tbl_hmax[DG_IGN_N];  // 0 = no feasible band at that fall speed
} dg_params_t;

void dg_params_default(dg_params_t *p);

// ---------------------------------------------------------------------
//  Interfaces
// ---------------------------------------------------------------------
typedef enum {
    DG_PHASE_IDLE = 0,     // on the ground / not flying
    DG_PHASE_ASCENT,       // going up: fins per fin_ascent_mode, nothing else
    DG_PHASE_DESCENT,      // falling: fins fly the airframe, ignition is armed
    DG_PHASE_IGNITING,     // igniter commanded, waiting for thrust
    DG_PHASE_BURN,         // thrust detected: closed loop on the nozzle
    DG_PHASE_DONE          // down
} dg_phase_t;

typedef struct {
    float h;            // m above the pad, as the navigation filter believes it
    float vx, vy, vz;   // m/s, world, vz negative descending
    float bx, by, bz;   // thrust-axis unit vector, world
    float gx, gy, gz;   // body-fixed transverse reference axis, world
    float wx, wy, wz;   // rad/s, world, STRAIGHT FROM THE GYRO
    float a_axial;      // m/s^2, specific force along b (what an accelerometer
                        // on the thrust axis reads: ~0 in vacuum free fall,
                        // drag/m while falling, thrust/m while burning)
    bool  lidar_ok;     // the altitude is lidar-referenced: drop the altitude pad
    bool  valid;        // false blocks ignition and parks the actuators
} dg_nav_t;

typedef struct {
    float servo1_deg;   // TVC servo, g channel, +/-DG_SERVO_MAX_DEG
    float servo2_deg;   // TVC servo, b x g channel
    float clamp;        // thrust clamp command, DG_CLAMP_MIN..MAX
    float fin_deg[4];   // fin deflections, see FRAMES
    bool  fins_live;    // false: the Master leaves the fin outputs parked
    bool  ignite;       // level: true from the ignition command on
    bool  thrust_on;    // level: thrust onset has been DETECTED (not assumed)
    dg_phase_t phase;
    // countdowns, both < 0 while unknown
    float t_to_ignition;    // s until the ignition command (DESCENT only)
    float t_to_touchdown;   // s until the planned touchdown
    // telemetry
    float h_cmd;        // m, commanded ignition altitude
    float tilt_cmd;     // rad, commanded tilt from vertical
    float tilt_err;     // rad, angle between commanded and actual axis
    float roll_rate;    // rad/s about b
    float thrust_est;   // N, estimated real post-clamp thrust
    float t_scale;      // estimated thrust / tabulated thrust
    float k_plan;       // planned clamp level
    float plan_resid;   // m/s, < 0 when the plan cannot reach its target
    float fin_gain;     // estimated fin effectiveness, 1 = the model
    float fin_trim[3];  // deg, estimated trim offsets: axis 1, axis 2, roll
    float t_burn;       // s since thrust onset, < 0 before
    float drag_scale;   // measured / modelled drag in the free fall (1 = as modelled)
    float brake_frac;   // airbrake the mixer really splayed / requested (0..1)
    float h_band_lo, h_band_hi;   // m, where THIS fall enters / leaves the ignition band
} dg_out_t;

// ---------------------------------------------------------------------
//  API
//
//    core 0 : dg_step_control() at 200 Hz — bounded work, never blocks
//    core 1 : dg_step_planner() — call it as often as you like; it solves
//             the clamp plan at most DG_PLAN_HZ times a second
// ---------------------------------------------------------------------
void dg_init(void);
void dg_set_params(const dg_params_t *p);
const dg_params_t *dg_get_params(void);

// Flight phase transitions the caller reports (it owns the mission state).
// dg_begin_descent() takes the apogee state; the ignition altitude is then
// recomputed ten times a second by dg_step_planner() from the real fall.
void dg_begin_ascent(void);                           // launch detected
void dg_begin_descent(float h_apogee, float vh_apogee);  // apogee passed

void dg_step_control(const dg_nav_t *nav, dg_out_t *out);
void dg_step_planner(const dg_nav_t *nav);

// ---- introspection: the host test and the self test ---------------------
float dg_thrust_ref(float t_burn);                 // tabulated thrust, N
float dg_burned(float t_burn);                     // propellant burned, kg
/**
 * The ignition altitude for a fall starting from (h, vz, vh) with the drag
 * scaled by `drag_scale`: LandSim_Light's find_ignition(), with the band
 * [h_min, h_max] read from the tbl.* table and the fall propagated here.
 * No bias, no override. Optional outputs: the time to reach it, the fall
 * speed there, and the two band edges along this trajectory.
 */
float dg_ign_compute(float h, float vz, float vh, float drag_scale,
                     float *t_to, float *vz_at, float *band_lo, float *band_hi);
float dg_ign_band(float vz_abs, float *h_max);     // table lookup: h_min (and h_max)
float dg_h_ignite(void);                           // what this flight will use now
float dg_guidance_dir(float h, float vx, float vy, float vz, float mass,
                      float *ux, float *uy, float *uz);   // returns t_go
void  dg_gimbal_basis(float bx, float by, float bz,
                      float gx, float gy, float gz,
                      float *u1x, float *u1y, float *u1z,
                      float *u2x, float *u2y, float *u2z);
bool  dg_terminal_active(float h, float vz, float thrust_avail, float mass);
float dg_terminal_throttle(float h, float vz, float mass, float thrust_avail);
// Projected touchdown speed under clamp level k; *t_td = time to the ground.
// Thrust forecast (LandSim_Light project_vz): with ts_tau > 0 the scale
// decays from t_scale towards ts_inf with that time constant - today's
// thrust deviation is a wander, not a property of the motor. ts_tau = 0
// holds t_scale to burnout.
float dg_project_vz(float h, float vz, float t_burn, float k, float t_scale,
                    float *t_td, float ts_inf = -1.0f, float ts_tau = 0.0f);
// The clamp level that reaches the plan target; *resid < 0 if none does.
float dg_solve_plan(float h, float vz, float t_burn, float t_scale,
                    float *resid, float *t_td, float ts_inf = -1.0f,
                    float ts_tau = 0.0f);

/**
 * The fin mixer on its own, for the host test and the bench.
 *
 * tau[3]   torque demanded of the fins, world axes, N.m
 * tau_roll torque demanded about b, N.m
 * brake    requested splay, deg (0 = stowed)
 * kappa, trim[3]  effectiveness and trim estimates (1, {0,0,0} = the model)
 */
void dg_fin_mix(const float b[3], const float u1[3], const float u2[3],
                const float v[3], const float tau[3], float tau_roll,
                float brake, float kappa, const float trim[3],
                bool control, float out_deg[4]);

/**
 * Bench polarity-check step on real IMU attitude (pyros strictly OFF).
 *
 * Drives TVC as in landing burn (0 deg vertical hold) and fins as in ascent
 * (0 deg vertical hold + roll damping) using the exact same coordinate frame,
 * gimbal basis, dynamic inversion, and fin mixer as dg_step_control(), without
 * touching flight state S.
 *
 * ref_thrust_n: reference thrust for TVC [N]; <= 0 uses the default
 *               avg(mean, median) of the reference thrust curve.
 */
void dg_step_polarity(const dg_nav_t *nav, dg_out_t *out, float ref_thrust_n = 0.0f);

#endif // DAIDALOS_GUIDANCE_H
