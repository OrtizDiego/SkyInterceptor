#include <gtest/gtest.h>
#include <Eigen/Dense>

#include <string>
#include <vector>

#include "safety/engagement_gate.hpp"

namespace interceptor
{
namespace safety
{
namespace
{

// TargetClass values, as in common/mission.hpp.
constexpr int kPerson = 0;
constexpr int kCar = 1;
constexpr int kTruck = 2;
constexpr int kBicycle = 3;
constexpr int kUav = 4;

constexpr int32_t kTargetId = 7;

/// The gate with the plan's defaults, except the confirmation counter: most
/// tests want the gate to open as soon as the target is confirmed.
EngagementParams testParams()
{
  EngagementParams params;
  params.min_confirmed_frames = 1;
  params.min_engage_altitude_agl = 10.0;
  params.engage_altitude_band = 5.0;
  params.abort_ground_radius = 15.0;
  params.geofence_x_min = -100.0;
  params.geofence_x_max = 100.0;
  params.geofence_y_min = -100.0;
  params.geofence_y_max = 100.0;
  params.geofence_margin = 1.0;
  return params;
}

/// A track the tracker reports.
KeepOutTarget track(int32_t id, int class_id, double x, double y, double z = 0.0)
{
  KeepOutTarget target;
  target.track_id = id;
  target.class_id = class_id;
  target.position = Eigen::Vector3d(x, y, z);
  return target;
}

/// The situation of an INTERCEPT mission that is under way: armed, with a
/// confirmed uav 20 m ahead and 12 m up, and the interceptor next to it.
EngagementContext intercepting()
{
  EngagementContext context;
  context.intercept_mode = true;
  context.armed = true;
  context.target_track_id = kTargetId;
  context.target_class_id = kUav;
  context.target_confirmed = true;
  context.target_valid = true;
  context.target_position = Eigen::Vector3d(20.0, 0.0, 12.0);
  context.interceptor_position = Eigen::Vector3d(0.0, 0.0, 12.0);
  context.tracks = {track(kTargetId, kUav, 20.0, 0.0, 12.0)};
  return context;
}

// ---------------------------------------------------------------------------
// The class rule: a person is never engaged
// ---------------------------------------------------------------------------

/// A person track is never engaged, whatever the class of the other tracks is.
///
/// The property is over the whole class list: a person, a car, a truck and a
/// bicycle are all refused, whatever else is in the scene. The class of the
/// selected target is checked, and it is checked again against the track list.
TEST(EngagementGateTest, AGroundClassTargetIsNeverEngaged)
{
  const std::vector<int> ground_classes = {kPerson, kCar, kTruck, kBicycle};

  for (int class_id : ground_classes) {
    EngagementGate gate(testParams());
    EngagementContext context = intercepting();
    context.target_class_id = class_id;
    context.tracks = {track(kTargetId, class_id, 20.0, 0.0, 12.0)};

    const EngagementDecision decision = gate.update(context);

    EXPECT_FALSE(decision.engage_allowed)
      << "the gate opened for a target of class " << class_id;
    EXPECT_FALSE(decision.aborted) << "the mission had not started yet";
    EXPECT_FALSE(decision.reason.empty()) << "the gate did not say why";
  }
}

/// The class rule holds even when another track is a uav.
///
/// The selected track is the one that is engaged, so a uav in the scene does
/// not make a person engageable.
TEST(EngagementGateTest, AUavInTheSceneDoesNotMakeAPersonEngageable)
{
  EngagementGate gate(testParams());
  EngagementContext context = intercepting();
  context.target_class_id = kPerson;
  context.tracks = {
    track(kTargetId, kPerson, 20.0, 0.0, 12.0),
    track(8, kUav, 30.0, 5.0, 15.0),
  };

  const EngagementDecision decision = gate.update(context);

  EXPECT_FALSE(decision.engage_allowed);
}

/// The class rule holds when the track list contradicts the selection.
///
/// The selection says uav, the track list says person: the track list is the
/// truth (the class arrived late, or it changed), so the gate closes.
TEST(EngagementGateTest, ATrackListThatSaysPersonClosesTheGate)
{
  EngagementGate gate(testParams());
  EngagementContext context = intercepting();
  context.target_class_id = kUav;
  context.tracks = {track(kTargetId, kPerson, 20.0, 0.0, 12.0)};

  const EngagementDecision decision = gate.update(context);

  EXPECT_FALSE(decision.engage_allowed);
  EXPECT_TRUE(decision.latched) << "a person must latch the gate closed";
}

/// A class violation latches the gate closed for the rest of the mission.
TEST(EngagementGateTest, AClassViolationLatchesTheGate)
{
  EngagementGate gate(testParams());
  EngagementContext context = intercepting();
  context.target_class_id = kPerson;

  const EngagementDecision first = gate.update(context);
  EXPECT_FALSE(first.engage_allowed);
  EXPECT_TRUE(first.latched);

  // The situation is fixed: the selected target is a uav again. The gate stays
  // closed, because the mission has already seen a person.
  context.target_class_id = kUav;
  context.tracks = {track(kTargetId, kUav, 20.0, 0.0, 12.0)};
  const EngagementDecision second = gate.update(context);

  EXPECT_FALSE(second.engage_allowed) << "the gate was re-opened after a class violation";
  EXPECT_TRUE(second.latched);
  EXPECT_FALSE(second.reason.empty()) << "the reason of the latch was lost";
}

/// A class that has not arrived yet is refused, but it does not latch.
TEST(EngagementGateTest, AnUnknownClassIsRefusedButDoesNotLatch)
{
  EngagementGate gate(testParams());
  EngagementContext context = intercepting();
  context.target_class_id = -1;

  const EngagementDecision first = gate.update(context);
  EXPECT_FALSE(first.engage_allowed);
  EXPECT_FALSE(first.latched) << "an unknown class must not end the mission";

  // The class arrives.
  context.target_class_id = kUav;
  const EngagementDecision second = gate.update(context);
  EXPECT_TRUE(second.engage_allowed);
}

/// A uav is engageable: the gate opens.
TEST(EngagementGateTest, AUavIsEngageable)
{
  EngagementGate gate(testParams());
  EngagementContext context = intercepting();

  const EngagementDecision decision = gate.update(context);

  EXPECT_TRUE(decision.engage_allowed);
  EXPECT_FALSE(decision.aborted);
  EXPECT_TRUE(gate.engaging());
}

// ---------------------------------------------------------------------------
// The other gate conditions
// ---------------------------------------------------------------------------

/// The gate is closed until the operator arms the mission.
TEST(EngagementGateTest, TheGateIsClosedUntilTheOperatorArms)
{
  EngagementGate gate(testParams());
  EngagementContext context = intercepting();
  context.armed = false;

  const EngagementDecision decision = gate.update(context);

  EXPECT_FALSE(decision.engage_allowed);
  EXPECT_FALSE(decision.latched) << "an unarmed mission is not a latched refusal";
}

/// The gate is closed while the target is not confirmed for enough frames.
TEST(EngagementGateTest, TheGateNeedsEnoughConfirmedFrames)
{
  EngagementParams params = testParams();
  params.min_confirmed_frames = 10;
  EngagementGate gate(params);
  EngagementContext context = intercepting();

  // The first nine cycles do not open the gate.
  for (int cycle = 0; cycle < 9; ++cycle) {
    const EngagementDecision decision = gate.update(context);
    EXPECT_FALSE(decision.engage_allowed) << "the gate opened after " << cycle + 1 << " frames";
  }
  // The tenth one does.
  const EngagementDecision decision = gate.update(context);
  EXPECT_TRUE(decision.engage_allowed);
}

/// The gate is closed when the intruder is too low.
TEST(EngagementGateTest, TheGateIsClosedWhenTheIntruderIsTooLow)
{
  EngagementGate gate(testParams());
  EngagementContext context = intercepting();
  context.target_position.z() = 5.0;
  context.tracks = {track(kTargetId, kUav, 20.0, 0.0, 5.0)};

  const EngagementDecision decision = gate.update(context);

  EXPECT_FALSE(decision.engage_allowed);
  EXPECT_FALSE(decision.latched) << "a low intruder may take off later";
}

/// The gate is closed when the interceptor is too far below the intruder.
TEST(EngagementGateTest, TheGateIsClosedWhenTheInterceptorIsTooLow)
{
  EngagementGate gate(testParams());
  EngagementContext context = intercepting();
  // The intruder is at 20 m, the interceptor at 10 m: 10 m below the band.
  context.target_position.z() = 20.0;
  context.tracks = {track(kTargetId, kUav, 20.0, 0.0, 20.0)};
  context.interceptor_position.z() = 10.0;

  const EngagementDecision decision = gate.update(context);

  EXPECT_FALSE(decision.engage_allowed);

  // Inside the band it opens again.
  context.interceptor_position.z() = 16.0;
  EngagementGate open_gate(testParams());
  const EngagementDecision open = open_gate.update(context);
  EXPECT_TRUE(open.engage_allowed);
}

/// Once the engagement runs, the band has a hysteresis: the noise of the
/// intruder's altitude does not abort an engagement that began at the edge, but
/// a real drop below the band still does.
TEST(EngagementGateTest, TheBandHasAHysteresisOnceEngaged)
{
  EngagementGate gate(testParams());
  EngagementContext context = intercepting();
  context.target_position.z() = 20.0;
  context.tracks = {track(kTargetId, kUav, 20.0, 0.0, 20.0)};
  context.interceptor_position.z() = 15.0;  // exactly at the edge of the band
  ASSERT_TRUE(gate.update(context).engage_allowed);

  // The intruder's estimate jumps up by the measurement noise.
  context.target_position.z() = 20.5;
  EXPECT_TRUE(gate.update(context).engage_allowed) << "the noise aborted the engagement";
  EXPECT_FALSE(gate.aborted());

  // A real drop: the interceptor is band + hysteresis + 1 m below.
  context.interceptor_position.z() = 20.5 - 5.0 - 2.0 - 1.0;
  const EngagementDecision decision = gate.update(context);
  EXPECT_FALSE(decision.engage_allowed);
  EXPECT_TRUE(decision.aborted) << "a real drop below the band must abort";
}

/// The gate is closed when the intruder leaves the geofence.
TEST(EngagementGateTest, TheGateIsClosedWhenTheIntruderLeavesTheGeofence)
{
  EngagementGate gate(testParams());
  EngagementContext context = intercepting();
  context.target_position.x() = 200.0;
  context.tracks = {track(kTargetId, kUav, 200.0, 0.0, 12.0)};

  const EngagementDecision decision = gate.update(context);

  EXPECT_FALSE(decision.engage_allowed);
  EXPECT_FALSE(decision.latched) << "the intruder may come back";
}

/// The gate is closed when the interceptor leaves the geofence.
TEST(EngagementGateTest, TheGateIsClosedWhenTheInterceptorLeavesTheGeofence)
{
  EngagementGate gate(testParams());
  EngagementContext context = intercepting();
  context.interceptor_position.x() = -200.0;

  const EngagementDecision decision = gate.update(context);

  EXPECT_FALSE(decision.engage_allowed);
}

// ---------------------------------------------------------------------------
// The abort
// ---------------------------------------------------------------------------

/// A bystander inside the abort radius aborts the engagement.
TEST(EngagementGateTest, ABystanderInsideTheAbortRadiusAbortsTheEngagement)
{
  EngagementGate gate(testParams());
  EngagementContext context = intercepting();
  // A person 8 m from the interceptor, which hovers 12 m up: 14.4 m away in
  // 3D, inside the 15 m abort radius.
  context.tracks.push_back(track(9, kPerson, 8.0, 0.0));

  const EngagementDecision decision = gate.update(context);

  EXPECT_TRUE(decision.aborted);
  EXPECT_FALSE(decision.engage_allowed);
  EXPECT_TRUE(decision.latched);
  EXPECT_NE(decision.reason.find("person"), std::string::npos)
    << "the reason does not name the bystander: " << decision.reason;
}

/// A bystander next to the capture point aborts the engagement.
///
/// The capture happens at the intruder's position, so a person next to it
/// would force the interceptor to break the safety distance to finish.
TEST(EngagementGateTest, ABystanderNextToTheCapturePointAbortsTheEngagement)
{
  EngagementGate gate(testParams());
  EngagementContext context = intercepting();
  // The person is 6 m from the intruder (13.4 m away in 3D), far from the
  // interceptor.
  context.tracks.push_back(track(9, kPerson, 26.0, 0.0));

  const EngagementDecision decision = gate.update(context);

  EXPECT_TRUE(decision.aborted);
}

/// A bystander outside the abort radius does not abort the engagement.
TEST(EngagementGateTest, ABystanderOutsideTheAbortRadiusDoesNotAbort)
{
  EngagementGate gate(testParams());
  EngagementContext context = intercepting();
  // 40 m from the interceptor and 20 m from the intruder: outside the radius
  // in 3D as well.
  context.tracks.push_back(track(9, kPerson, 40.0, 0.0));

  const EngagementDecision decision = gate.update(context);

  EXPECT_FALSE(decision.aborted);
  EXPECT_TRUE(decision.engage_allowed);
}

/// The abort is latched: it stays until the mission is reset.
TEST(EngagementGateTest, TheAbortLatches)
{
  EngagementGate gate(testParams());
  EngagementContext context = intercepting();
  context.tracks.push_back(track(9, kPerson, 8.0, 0.0));

  const EngagementDecision first = gate.update(context);
  EXPECT_TRUE(first.aborted);

  // The bystander walks away: the engagement does not resume.
  context.tracks = {track(kTargetId, kUav, 20.0, 0.0, 12.0)};
  const EngagementDecision second = gate.update(context);
  EXPECT_TRUE(second.aborted);
  EXPECT_FALSE(second.engage_allowed);
  EXPECT_TRUE(gate.aborted());

  // A new mission clears it.
  gate.reset();
  EXPECT_FALSE(gate.aborted());
  EXPECT_FALSE(gate.latched());
  const EngagementDecision third = gate.update(context);
  EXPECT_TRUE(third.engage_allowed);
}

/// A gate condition violated during the engagement aborts it.
TEST(EngagementGateTest, AGateConditionViolatedMidEngagementAbortsIt)
{
  EngagementGate gate(testParams());
  EngagementContext context = intercepting();

  // The engagement starts.
  const EngagementDecision first = gate.update(context);
  EXPECT_TRUE(first.engage_allowed);

  // The intruder leaves the geofence in the middle of the engagement.
  context.target_position.x() = 200.0;
  context.tracks = {track(kTargetId, kUav, 200.0, 0.0, 12.0)};
  const EngagementDecision second = gate.update(context);

  EXPECT_TRUE(second.aborted);
  EXPECT_FALSE(second.engage_allowed);
}

/// The target that stops being a uav aborts the engagement.
TEST(EngagementGateTest, ATargetThatStopsBeingAUavAbortsTheEngagement)
{
  EngagementGate gate(testParams());
  EngagementContext context = intercepting();

  const EngagementDecision first = gate.update(context);
  EXPECT_TRUE(first.engage_allowed);

  // The track changes its class in the middle of the engagement.
  context.target_class_id = kPerson;
  context.tracks = {track(kTargetId, kPerson, 20.0, 0.0, 12.0)};
  const EngagementDecision second = gate.update(context);

  EXPECT_TRUE(second.aborted);
  EXPECT_FALSE(second.engage_allowed);
  EXPECT_TRUE(second.latched);
}

/// The e-stop aborts the engagement.
TEST(EngagementGateTest, TheEStopAbortsTheEngagement)
{
  EngagementGate gate(testParams());
  EngagementContext context = intercepting();
  context.estop = true;

  const EngagementDecision decision = gate.update(context);

  EXPECT_TRUE(decision.aborted);
  EXPECT_FALSE(decision.engage_allowed);
  EXPECT_TRUE(decision.latched);
  EXPECT_NE(decision.reason.find("e-stop"), std::string::npos)
    << "the reason does not mention the e-stop: " << decision.reason;
}

/// The gate does not apply in the other modes: FOLLOW is not gated.
TEST(EngagementGateTest, TheGateDoesNotApplyOutsideIntercept)
{
  EngagementGate gate(testParams());
  EngagementContext context = intercepting();
  context.intercept_mode = false;
  context.target_class_id = kPerson;

  const EngagementDecision decision = gate.update(context);

  EXPECT_TRUE(decision.engage_allowed)
    << "the gate closed a FOLLOW mission: " << decision.reason;
}

/// The e-stop does not abort anything outside INTERCEPT: there is no
/// engagement to abort, and the e-stop is the filter's own latch there.
TEST(EngagementGateTest, TheEStopDoesNotAbortOutsideIntercept)
{
  EngagementGate gate(testParams());
  EngagementContext context = intercepting();
  context.intercept_mode = false;
  context.estop = true;

  const EngagementDecision decision = gate.update(context);

  EXPECT_FALSE(decision.aborted);
  EXPECT_TRUE(decision.engage_allowed);
}

/// A person in the scene is not a gate violation, but it is never engaged.
///
/// The bystander is handled by the abort and by the keep-out barrier; the gate
/// is about the track that is engaged.
TEST(EngagementGateTest, APersonInTheSceneIsNotEngagedButDoesNotBlockTheUav)
{
  EngagementGate gate(testParams());
  EngagementContext context = intercepting();
  // A person far away: outside the abort radius.
  context.tracks.push_back(track(9, kPerson, 40.0, 0.0));

  const EngagementDecision decision = gate.update(context);

  EXPECT_TRUE(decision.engage_allowed);
  EXPECT_EQ(decision.reason, "");
}

// ---------------------------------------------------------------------------
// Climbing is not engaging: the take-off
// ---------------------------------------------------------------------------

/// An armed mission whose gate is closed may climb, so the interceptor can
/// reach the altitude band the gate asks for.
TEST(EngagementGateTest, AnArmedMissionMayClimbWhileTheGateIsClosed)
{
  EngagementGate gate(testParams());
  EngagementContext context = intercepting();
  // The interceptor is on the ground, the intruder at 20 m.
  context.target_position.z() = 20.0;
  context.tracks = {track(kTargetId, kUav, 20.0, 0.0, 20.0)};
  context.interceptor_position.z() = 0.0;

  const EngagementDecision decision = gate.update(context);

  EXPECT_FALSE(decision.engage_allowed);
  EXPECT_TRUE(decision.climb_allowed) << "the interceptor could never reach the band";
  EXPECT_FALSE(decision.latched);
}

/// The climb does not need a target: the interceptor takes off before it sees one.
TEST(EngagementGateTest, AnArmedMissionMayClimbBeforeItHasATarget)
{
  EngagementGate gate(testParams());
  EngagementContext context = intercepting();
  context.target_track_id = -1;
  context.tracks.clear();

  const EngagementDecision decision = gate.update(context);

  EXPECT_FALSE(decision.engage_allowed);
  EXPECT_TRUE(decision.climb_allowed);
}

/// An unarmed mission does not climb: the operator has not authorised anything.
TEST(EngagementGateTest, AnUnarmedMissionMayNotClimb)
{
  EngagementGate gate(testParams());
  EngagementContext context = intercepting();
  context.armed = false;

  const EngagementDecision decision = gate.update(context);

  EXPECT_FALSE(decision.engage_allowed);
  EXPECT_FALSE(decision.climb_allowed);
}

/// A latched gate and an abort hold the drone: neither may climb.
TEST(EngagementGateTest, ALatchedGateOrAnAbortMayNotClimb)
{
  EngagementGate latched(testParams());
  EngagementContext context = intercepting();
  context.target_class_id = kPerson;
  latched.update(context);
  context.target_class_id = kUav;
  EXPECT_FALSE(latched.update(context).climb_allowed) << "a latched gate let the drone climb";

  EngagementGate aborted(testParams());
  EngagementContext scene = intercepting();
  aborted.update(scene);
  scene.tracks.push_back(track(9, kPerson, 5.0, 0.0, 0.0));
  const EngagementDecision decision = aborted.update(scene);
  EXPECT_TRUE(decision.aborted);
  EXPECT_FALSE(decision.climb_allowed) << "an aborted engagement let the drone climb";
}

/// The deadlock this rule removes: an interceptor on the ground, an intruder
/// hovering 20 m up and 30 m away, and a planner that flies straight at it.
///
/// The gate and the filter are run together on a first-order vehicle. The
/// interceptor has to take off, climb into the band without moving
/// horizontally, and then the gate has to open.
TEST(EngagementGateTest, TheInterceptorTakesOffIntoTheBandAndTheGateOpens)
{
  EngagementGate gate(testParams());
  SafetyFilterParams filter_params;
  filter_params.drone_radius = 0.0;
  SafetyFilter filter(filter_params);

  const Eigen::Vector3d intruder(30.0, 0.0, 20.0);
  FilterState state;
  state.position = Eigen::Vector3d(0.0, 0.0, 0.0);

  const double dt = 0.02;
  double opened_at = -1.0;
  for (int step = 0; step < 2000; ++step) {
    const double t = step * dt;
    EngagementContext context = intercepting();
    context.target_position = intruder;
    context.tracks = {track(kTargetId, kUav, intruder.x(), intruder.y(), intruder.z())};
    context.interceptor_position = state.position;
    const EngagementDecision decision = gate.update(context);
    if (decision.engage_allowed) {
      opened_at = t;
      break;
    }

    // The planner flies straight at the intruder at up to 8 m/s.
    Eigen::Vector3d command = intruder - state.position;
    if (command.norm() > 8.0) {
      command *= 8.0 / command.norm();
    }
    FilterSetpoint setpoint;
    setpoint.velocity = command;
    const FilterResult result = filter.filter(
      setpoint, state, {}, {}, FilterMode::INTERCEPT, t, 0.0, 0.0, false,
      decision.engage_allowed, "", decision.climb_allowed);

    ASSERT_EQ(result.velocity.head<2>().norm(), 0.0)
      << "the interceptor moved horizontally before the gate opened, at t = " << t;
    state.velocity = result.velocity;
    state.position += result.velocity * dt;
  }

  ASSERT_GE(opened_at, 0.0) << "the gate never opened: the take-off is deadlocked";
  EXPECT_GE(state.position.z(), intruder.z() - testParams().engage_altitude_band - 0.5);
  EXPECT_EQ(state.position.head<2>().norm(), 0.0);
}

}  // namespace
}  // namespace safety
}  // namespace interceptor
