/**
 * @file engagement_gate.hpp
 * @brief The INTERCEPT engagement gate and the abort logic (plan section 4.4).
 *
 * Two questions, two objects
 * --------------------------
 * The gate answers "may the interceptor engage this target at all?". The abort
 * answers "must the interceptor break off the engagement it is in?". They are
 * kept apart on purpose, because they look at different things and have
 * different consequences:
 *
 * - The gate looks at the *target*: the class of the track the interceptor is
 *   flying at, its confirmation, its altitude and the geofence. It is checked
 *   before the engagement starts, and it stays checked during it.
 * - The abort looks at the *scene*: a person or a vehicle that comes too close
 *   to the interceptor, an e-stop, or a gate condition that is violated while
 *   the engagement is running. It is the safety response of an engagement that
 *   has to stop.
 *
 * The class rule
 * --------------
 * Only a uav may ever be engaged. The rule is absolute: it is not a parameter,
 * it cannot be switched off, and a violation latches the gate closed for the
 * rest of the mission. The class is checked twice: once on the selected target
 * (/target/state) and once against the track list (/tracks), so a class that
 * arrives late, or a track whose class changes under the interceptor's feet,
 * closes the gate instead of being believed.
 *
 * A person in the *scene* is not a gate violation. A bystander is handled by
 * the abort (it aborts the engagement when it comes too close to the capture
 * point) and by the keep-out barrier (the interceptor is never allowed closer
 * than d_min to it, at any altitude). The gate is about the track that is
 * engaged, so that "a person is never engaged" is a property of the gate and
 * not of the tracker's selection.
 *
 * Latching
 * --------
 * A gate that could be re-opened by the next cycle would be no gate at all: the
 * tracker would have to be right every single frame. So:
 *
 * - A class violation latches the gate closed for the rest of the mission. The
 *   reason is kept and published, so the operator can see why the engagement
 *   is refused.
 * - A gate condition that is violated *during* an engagement aborts it (see
 *   below) and latches the gate closed with it.
 * - Conditions that are simply not met yet (the operator has not armed, the
 *   track is not confirmed yet, the intruder has not taken off) do not latch:
 *   they are the normal state of a mission that has not started.
 *
 * The abort
 * ---------
 * The abort is latched as well. It fires when
 *
 * - a person or a vehicle (a bystander, that is a ground-class track) comes
 *   within abort_ground_radius of the interceptor or of the capture point: the
 *   interceptor would have to break the safety distance to finish the capture;
 * - a gate condition is violated while the engagement is running (the intruder
 *   left the geofence, the altitude condition is not met any more, the target
 *   stopped being a uav, the track went invalid);
 * - the operator pulls the e-stop. The e-stop abort is latched as well: the
 *   engagement does not resume when the e-stop is released, a new mission (a
 *   mode change or a re-arm) does. Outside INTERCEPT there is no engagement to
 *   abort, and the e-stop is the filter's own latch.
 *
 * The abort is reported through the reason string and the aborted flag, and the
 * safety filter turns it into a brake: the vehicle holds position and the
 * guidance stops commanding a closure.
 */

#pragma once

#include <Eigen/Dense>

#include <cstdint>
#include <string>
#include <vector>

#include "safety/safety_filter.hpp"

namespace interceptor
{
namespace safety
{

/// The parameters of the engagement gate and the abort logic.
struct EngagementParams
{
  // --- The gate (plan section 4.4) ------------------------------------------
  /// The operator has to arm the mission before the interceptor may engage.
  bool require_armed = true;
  /// The intruder has to be at least this high above the ground [m]. Below it
  /// the intruder is in the clutter around the people on the ground.
  double min_engage_altitude_agl = 10.0;
  /// The interceptor may be at most this far *below* the intruder [m]. Below
  /// that band the interceptor would have to close on the intruder through the
  /// ground clutter around it.
  double engage_altitude_band = 5.0;
  /// The number of consecutive cycles the target has to be confirmed before the
  /// gate opens. At the filter's rate (50 Hz) that is 0.2 s.
  int min_confirmed_frames = 10;
  /// The geofence box the interceptor and the intruder have to be inside. The
  /// margin is the same one the flight limits use: the boundary is enforced
  /// this far before it.
  double geofence_x_min = -100.0;
  double geofence_x_max = 100.0;
  double geofence_y_min = -100.0;
  double geofence_y_max = 100.0;
  double geofence_margin = 1.0;

  // --- The abort -------------------------------------------------------------
  /// The radius around the interceptor, and around the capture point, in which
  /// a person or a vehicle aborts the engagement [m]. The distance is measured
  /// in 3D, like the safety distance it protects: a bystander 12 m below the
  /// interceptor is 12 m away, not 0. It is a margin, not the safety distance
  /// itself: the capture manoeuvre brings the interceptor to the intruder's
  /// position, and the margin leaves room for the manoeuvre's own error and
  /// for the bystander to keep walking.
  double abort_ground_radius = 15.0;
};

/// The situation the gate looks at, once per cycle.
struct EngagementContext
{
  /// The mission mode is INTERCEPT.
  bool intercept_mode = false;
  /// The operator armed the mission.
  bool armed = false;
  /// The e-stop is latched.
  bool estop = false;
  /// The selected track, from /target/state. -1 when there is none.
  int32_t target_track_id = -1;
  /// The class of the selected track.
  int target_class_id = -1;
  /// The selected track has passed the tracker's confirmation.
  bool target_confirmed = false;
  /// The selected track is still being updated.
  bool target_valid = false;
  /// The position of the selected track [m], world frame.
  Eigen::Vector3d target_position = Eigen::Vector3d::Zero();
  /// The position of the interceptor [m], world frame.
  Eigen::Vector3d interceptor_position = Eigen::Vector3d::Zero();
  /// Every track the tracker reports: the class cross-check and the bystanders.
  std::vector<KeepOutTarget> tracks;
};

/// What the gate decided.
struct EngagementDecision
{
  /// True when the interceptor may engage the selected target.
  bool engage_allowed = false;
  /// True when the engagement has been aborted. The abort is latched.
  bool aborted = false;
  /// True when the gate is latched closed for the rest of the mission.
  bool latched = false;
  /// Why the gate is closed, or why the abort fired. Empty when nominal.
  std::string reason;
};

/// The engagement gate and the abort logic.
///
/// Plain C++ and Eigen, no ROS, so the rules can be unit tested on synthetic
/// situations.
class EngagementGate
{
public:
  explicit EngagementGate(const EngagementParams & params = EngagementParams());

  const EngagementParams & params() const {return params_;}

  /// Evaluates the gate and the abort for one cycle.
  ///
  /// The state of the gate is kept between the calls: the latch and the
  /// engagement flag are what make the gate a gate.
  EngagementDecision update(const EngagementContext & context);

  /// Clears the latch and the engagement flag: a new mission.
  void reset();

  /// True when the engagement has been aborted.
  bool aborted() const {return abort_latched_;}

  /// True when the gate is latched closed.
  bool latched() const {return gate_latched_;}

  /// True while an engagement is running (the gate has been open at least once).
  bool engaging() const {return engaging_;}

  /// The reason the gate is closed or the abort fired.
  const std::string & reason() const {return reason_;}

private:
  /// What the class of the selected target says about the engagement.
  enum class ClassVerdict
  {
    ENGAGEABLE,   // it is a uav
    WRONG_CLASS,  // it is a person or a vehicle: the gate latches closed
    UNKNOWN,      // the class has not arrived yet: refuse, but do not latch
  };

  /// True when the class is the one the interceptor may engage: a uav.
  static bool isUavClass(int class_id);

  /// True when the class is one of the known TargetClass values. An unknown
  /// class is not a uav, but it may still arrive.
  static bool isKnownClass(int class_id);

  /// The class of the selected target, cross-checked against the track list.
  ///
  /// The class of the selected target comes from /target/state and the class of
  /// the track with the same id comes from /tracks. When they disagree the
  /// track list wins: a class that arrives late, or a track whose class
  /// changes, closes the gate instead of being believed.
  ///
  /// @param context  the situation
  /// @param reason   set to why the class is not acceptable
  /// @return the verdict on the class
  ClassVerdict classIsEngageable(
    const EngagementContext & context, std::string & reason) const;

  /// True when the position is inside the geofence box.
  bool insideGeofence(const Eigen::Vector3d & position) const;

  /// The bystander that aborts the engagement, if there is one.
  ///
  /// A bystander is a ground-class track (a person or a vehicle) that is
  /// neither the engaged target nor the intruder. It aborts when it is within
  /// abort_ground_radius of the interceptor, or within the same radius of the
  /// capture point (the intruder's position): the interceptor would have to
  /// break the safety distance to finish the capture. The distance is measured
  /// in 3D, like the safety distance it protects.
  ///
  /// @param context  the situation
  /// @param reason   set to what triggered the abort
  /// @return true when the engagement has to be aborted
  bool bystanderTooClose(const EngagementContext & context, std::string & reason) const;

  /// Latches the abort, and with it the gate.
  void latchAbort(const std::string & reason);

  EngagementParams params_;

  /// The gate is latched closed for the rest of the mission.
  bool gate_latched_ = false;
  /// The engagement has been aborted.
  bool abort_latched_ = false;
  /// The gate has been open at least once: an engagement is running.
  bool engaging_ = false;
  /// The number of consecutive cycles the target has been confirmed.
  int confirmed_cycles_ = 0;
  /// The track the confirmation counter belongs to.
  int32_t confirmed_track_id_ = -1;
  /// The reason the gate is closed or the abort fired.
  std::string reason_;
};

}  // namespace safety
}  // namespace interceptor
