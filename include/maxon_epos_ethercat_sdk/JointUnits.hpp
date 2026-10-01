#pragma once

#define _USE_MATH_DEFINES  // for M_PI
#include <cmath>

namespace maxon {

/*!
 * Unit constants for the Anydrive5-style joint block (TxPdoJVPT / RxPdoJVPT and
 * the Freeze variants).
 *
 * The drive's joint VELOCITY objects are in milli-RPM, not milli-rad/s. The
 * rest of the block genuinely is milli-SI -- joint torque (0x3672) in mNm,
 * joint current (0x30D1:02) in mA -- which is why a bare *0.001 sat on the
 * velocity getter unnoticed and made every reported joint velocity ~9.55x too
 * large, in a field labelled rad/s all the way out to the dashboard.
 *
 * Confirmed against hardware, ros2_ws/chirp_logs/20260707_151050 (8 leg drives,
 * 30 s chirp, ~92k samples):
 *   - regressing reported velocity on d(actual_position)/dt gives a slope of
 *     9.71, against 60/(2*pi) = 9.549 for an RPM-read-as-rad/s mixup;
 *   - the quantisation agrees independently: the smallest nonzero reported
 *     magnitude is 36.61, and one encoder count (4096 cpr) over the drive's
 *     400 us velocity sample is 36.62 RPM. That 36.6 is a single encoder tick,
 *     which is what the dashboard was showing as "velocity noise".
 *
 * The drive states its own velocity unit at 0x60A9; milli-RPM is 0xFDB44700.
 * Maxon::readSIUnitSDO() reads that at start-up and complains if a drive on the
 * bus disagrees with the constant below.
 *
 * Note the resolution this implies: the velocity feedback cannot resolve
 * anything below ~3.8 rad/s, so it is close to useless as a measurement at
 * walking speed (peak true joint velocity in the chirp above was 0.96 rad/s).
 * Differentiate actual_position instead of reading this field.
 */
constexpr double kJointVelocityMilliRpmToRadPerSec = 2.0 * M_PI / 60000.0;

}  // namespace maxon
