// Compile-time feature detection for the Voltino_TriSense IMU driver.
//
// These are templates, so they live in a header: the Arduino IDE writes a
// prototype for every function DEFINED in the .ino and puts them all above
// the first one. It cannot write one for a template - and a function this
// high up in the .ino would also drag every generated prototype above the
// structs they use (fw 3.35 build break: "'imu' was not declared",
// "'GpsFix' does not name a type"). Headers are not touched by it.
#ifndef ARIADNA_IMU_COMPAT_H
#define ARIADNA_IMU_COMPAT_H

#include <stdint.h>

// True when the driver resynchronises a misaligned FIFO and calibrates at any
// ODR (Voltino_TriSense >= 1.7.1, which added getFIFOResyncCount()); false
// with an older library. Call as imu_lib_fifo_safe(sensor.imu, 0): the int
// overload wins when it exists, the long one is the fallback.
template <typename IMU>
static inline auto imu_lib_fifo_safe(IMU &imu, int) -> decltype(imu.getFIFOResyncCount(), bool())
{ (void)imu; return true; }
template <typename IMU>
static inline bool imu_lib_fifo_safe(IMU &, long) { return false; }

// FIFO resyncs so far (0 with an older driver).
template <typename IMU>
static inline auto imu_fifo_resyncs(IMU &imu, int) -> decltype(imu.getFIFOResyncCount(), uint32_t())
{ return imu.getFIFOResyncCount(); }
template <typename IMU>
static inline uint32_t imu_fifo_resyncs(IMU &, long) { return 0; }

#endif // ARIADNA_IMU_COMPAT_H
