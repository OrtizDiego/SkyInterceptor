#!/usr/bin/env python3
"""
Run a batch of INTERCEPT scenarios and score it against the exit criteria.

The exit criteria of Phase 3 of the plan (section 5):

  * the capture envelope is reached in at least 90 % of the hover runs, 80 % of
    the straight (10 m/s) runs and 60 % of the weave runs;
  * no track that is not a uav is ever engaged;
  * the abort fires in 100 % of the bystander runs.

Every run is a full simulation (intercept_scenario.launch.py): Gazebo, the
ground-truth source, the tracker, the guidance, the safety filter, the
controller and intercept_metrics_node, which writes one row of CSV when the
engagement ends. This script lays out each run, starts it, triggers the
intruder's manoeuvre, collects the rows and scores the batch with
intercept_metrics_report (the same C++ aggregator the unit tests check).

The layout
----------
The pedestrian of the world patrols the square [0, 15] x [0, 15] m. The abort
fires when a person is within abort_ground_radius (15 m, in 3D) of the
interceptor or of the capture point, so the capture scenarios are flown 40 m
and more away from the square, and the bystander scenario puts the intruder
right above it. The abort radius is the configured one in every scenario.

The intruder hovers at the start pose of its path until the interceptor has
climbed into the engagement band (engage_altitude_band below the intruder),
and only then starts its manoeuvre. The take-off is the same in every run, so
the manoeuvre always meets an interceptor that is ready to engage, whatever
the simulator's start-up time was.

The manoeuvre starts with a smooth ramp from the hover (5 s to 10 m/s for the
straight run, 3 s for the weave): with a 2 s ramp (a peak of 8 m/s^2) the
tracker sometimes replaced the intruder's track with a new one mid-engagement,
and a change of the engaged track aborts the engagement by design.

The interceptor flies at 8 m/s and cannot catch a 10 m/s intruder from behind,
so the straight and weave intruders fly at the interceptor (head-on, with a
random lateral offset and heading error): the capture law then backs away along
the intruder's path and lets it close at the speed of its braking curve.

The runs are reproducible: the seed fixes the geometry of the run and the noise
of the ground-truth source.

Usage:
  ros2 run interceptor_drone run_intercept_batch.py --runs-per-scenario 10
  ros2 run interceptor_drone run_intercept_batch.py --scenarios hover,bystander
"""
import argparse
import csv
import io
import os
import random
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

SCENARIOS = ['hover', 'straight', 'weave', 'bystander']

# The engagement band of the gate (safety.engage_altitude_band): the intruder's
# manoeuvre starts when the interceptor is this far below it.
ENGAGE_ALTITUDE_BAND = 5.0


def parse_args(argv):
    """Parse the command line."""
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--runs-per-scenario', type=int, default=5,
                        help='the number of runs of each scenario')
    parser.add_argument('--scenarios', default='all',
                        help='hover, straight, weave, bystander, all or a comma separated list')
    parser.add_argument('--seed-base', type=int, default=1, help='the seed of the first run')
    parser.add_argument('--guidance-law', default='apn', choices=['pn', 'apn'])
    parser.add_argument('--domain-id', type=int, default=78, help='the ROS domain of the runs')
    parser.add_argument('--gazebo-port', type=int, default=11478,
                        help='the port of the Gazebo master of the first run')
    parser.add_argument('--output-dir', default='/tmp/intercept_batch',
                        help='where the runs, the CSV and the report are written')
    parser.add_argument('--sim-timeout', type=float, default=75.0,
                        help='the simulation time a run may take [s]')
    parser.add_argument('--timeout', type=float, default=400.0,
                        help='the wall clock time a run may take [s]')
    parser.add_argument('--retries', type=int, default=2,
                        help='how many times a run that produced nothing is retried')
    parser.add_argument('--dry-run', action='store_true', help='print the layouts only')
    return parser.parse_args(argv)


def scenario_list(names):
    """Return the scenarios of the batch."""
    if names.strip() == 'all':
        return list(SCENARIOS)
    chosen = [name.strip() for name in names.split(',') if name.strip()]
    for name in chosen:
        if name not in SCENARIOS:
            raise SystemExit(f"unknown scenario '{name}', pick from {SCENARIOS}")
    return chosen


def layout(scenario, seed):
    """
    Return the geometry of one run.

    The keys: interceptor (x, y), the intruder's parameter overrides, the
    behaviour it switches to once the interceptor is in the band (None to keep
    hovering) and the altitude that triggers the switch.
    """
    rng = random.Random(f'{scenario}-{seed}')
    target = {'hover_drift': 0.0}
    behaviour = None
    if scenario == 'hover':
        interceptor = (40.0, 50.0)
        target.update(
            start_x=60.0 + rng.uniform(-5.0, 5.0),
            start_y=50.0 + rng.uniform(-10.0, 10.0),
            start_z=12.0 + rng.uniform(0.0, 3.0),
            start_yaw=rng.uniform(-3.14, 3.14))
    elif scenario in ('straight', 'weave'):
        # Head-on: the intruder starts 40 m ahead of the interceptor and flies at
        # it, with a lateral offset and a heading error.
        interceptor = (45.0, 50.0)
        heading = 3.14159265 + rng.uniform(-0.15, 0.15)
        target.update(
            start_x=85.0,
            start_y=50.0 + rng.uniform(-8.0, 8.0),
            start_z=12.0 + rng.uniform(0.0, 2.0),
            start_yaw=heading)
        if scenario == 'straight':
            behaviour = 'straight'
            target.update(
                straight_speed=10.0, straight_direction_yaw=heading,
                straight_duration=0.0, straight_distance=0.0, straight_ramp_time=5.0)
        else:
            behaviour = 'weave'
            target.update(
                weave_speed=6.0, weave_amplitude=2.0, weave_frequency=0.25,
                weave_direction_yaw=heading, weave_ramp_time=3.0)
    else:
        # The bystander: the intruder hovers above the centre of the pedestrian's
        # patrol square, 11 m up, so the pedestrian (whose hip is 1 m up) is within
        # 14.6 m of the capture point wherever it is on the square: inside the 15 m
        # abort radius all the time. An earlier layout moved the intruder up to 2 m
        # off the centre at 12 m, and then a quarter of the patrol was outside the
        # radius: one run in 25 captured with the pedestrian really 15+ m away.
        interceptor = (7.5, -12.0)
        target.update(
            start_x=7.5, start_y=7.5, start_z=11.0,
            start_yaw=rng.uniform(-3.14, 3.14))
    return {
        'interceptor': interceptor,
        'target': target,
        'behaviour': behaviour,
        'trigger_altitude': target['start_z'] - ENGAGE_ALTITUDE_BAND,
    }


def write_target_params(defaults_path, overrides, path):
    """Write the intruder's parameter file of a run: the defaults plus the overrides."""
    import yaml
    with open(defaults_path) as handle:
        document = yaml.safe_load(handle)
    parameters = document['target_drone_behavior_node']['ros__parameters']
    parameters['behaviour'] = 'hover'
    parameters.update(overrides)
    with open(path, 'w') as handle:
        yaml.safe_dump(document, handle)


class ManoeuvreTrigger:
    """
    Starts the intruder's manoeuvre once the interceptor is in the engagement band.

    One trigger serves the whole batch and is armed again for every run: tearing
    an executor down while its thread is still spinning can crash the process.
    """

    def __init__(self):
        """Watch /odom; arm() says what to switch the intruder to, and when."""
        import rclpy
        import rclpy.executors
        from nav_msgs.msg import Odometry
        from rcl_interfaces.srv import SetParameters

        self.behaviour = None
        self.trigger_altitude = 0.0
        self.triggered = False
        self._lock = threading.Lock()
        self.node = rclpy.create_node('intercept_batch_trigger')
        self._client = self.node.create_client(
            SetParameters, '/target_drone_behavior_node/set_parameters')
        self.node.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

    def arm(self, behaviour, trigger_altitude):
        """Switch the intruder to the behaviour when the interceptor reaches the altitude."""
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        from rcl_interfaces.srv import SetParameters
        with self._lock:
            self.behaviour = behaviour
            self.trigger_altitude = trigger_altitude
            self.triggered = False
            self._request = SetParameters.Request(parameters=[Parameter(
                name='behaviour',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_STRING, string_value=behaviour))])

    def disarm(self):
        """Stop triggering: the run is over."""
        with self._lock:
            self.behaviour = None

    def _on_odom(self, msg):
        """Switch the behaviour the first time the interceptor reaches the altitude."""
        with self._lock:
            if self.behaviour is None or self.triggered:
                return
            if msg.pose.pose.position.z < self.trigger_altitude:
                return
            if not self._client.service_is_ready():
                return
            self.triggered = True
            self._client.call_async(self._request)
        print(f'    manoeuvre: {self.behaviour} at '
              f'{msg.pose.pose.position.z:.1f} m', flush=True)

    def close(self):
        """Stop watching, at the end of the batch."""
        self._executor.shutdown()
        self.node.destroy_node()


def stop(process):
    """Bring the launch down with its whole process group, the simulator included."""
    if process.poll() is None:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
        except OSError:
            process.terminate()
        try:
            process.wait(timeout=45)
        except subprocess.TimeoutExpired:
            pass
    # A simulator that outlives its run keeps the Gazebo master port and
    # publishes /clock to the next run.
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
    except OSError:
        pass
    process.wait()


def simulator_died(log_path):
    """Return True when the launch log says that gzserver has died."""
    try:
        text = log_path.read_text(errors='replace')
    except OSError:
        return False
    return any(
        'process has died' in line and '[gzserver' in line for line in text.splitlines())


def read_row(path):
    """Return the row of a run's CSV as a dict, or None when there is none."""
    if not path.exists():
        return None
    rows = list(csv.DictReader(io.StringIO(path.read_text())))
    return rows[-1] if rows else None


def run_one(scenario, seed, args, run_dir, gazebo_port, trigger, attempt=0):
    """Run one scenario and return the CSV line of its row, or None when it failed."""
    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory('interceptor_drone'))
    run_dir.mkdir(parents=True, exist_ok=True)
    geometry = layout(scenario, seed)
    params_path = run_dir / 'target_drone.yaml'
    write_target_params(
        share / 'config' / 'target_drone_params.yaml', geometry['target'], params_path)
    row_path = run_dir / 'run.csv'
    if row_path.exists():
        row_path.unlink()

    ix, iy = geometry['interceptor']
    command = [
        'ros2', 'launch', 'interceptor_drone', 'intercept_scenario.launch.py',
        f'scenario:={scenario}', f'seed:={seed}', f'output_path:={row_path}',
        f'timeout:={args.sim_timeout}', f'guidance_law:={args.guidance_law}',
        f'target_drone_params_file:={params_path}',
        f'interceptor_x:={ix}', f'interceptor_y:={iy}',
        f"target_x:={geometry['target']['start_x']}",
        f"target_y:={geometry['target']['start_y']}",
    ]
    environment = dict(os.environ)
    environment['GAZEBO_MASTER_URI'] = f'http://127.0.0.1:{gazebo_port}'

    print(f'--- {scenario} (seed {seed}) ---', flush=True)
    started = time.time()
    triggered = None
    # The output goes to a file and not to a pipe: a full pipe blocks the launch.
    log_path = run_dir / f'launch_attempt{attempt}.log'
    with log_path.open('w') as log_file:
        process = subprocess.Popen(
            command, env=environment, stdout=log_file, stderr=subprocess.STDOUT,
            start_new_session=True)
    try:
        if geometry['behaviour'] is not None:
            trigger.arm(geometry['behaviour'], geometry['trigger_altitude'])
        # The metrics node writes the row and exits when the engagement ends.
        while time.time() - started < args.timeout:
            if read_row(row_path) is not None or process.poll() is not None:
                break
            if simulator_died(log_path):
                # gzserver crashes now and then (a segfault early in the run).
                # The sim clock stops with it, so the run would only end at the
                # wall clock timeout: give up on it at once and retry.
                break
            time.sleep(1.0)
    finally:
        if geometry['behaviour'] is not None:
            triggered = trigger.triggered
        trigger.disarm()
        stop(process)

    row = read_row(row_path)
    duration = time.time() - started
    if row is None or int(row['sample_count']) == 0:
        # The simulator crashed, or the world did not load: worth another try on a
        # port of its own.
        print(f'    FAILED after {duration:.0f} s: no samples', flush=True)
        if attempt < args.retries:
            return run_one(
                scenario, seed, args, run_dir, gazebo_port + 100, trigger, attempt + 1)
        return None
    if triggered is False:
        print('    note: the manoeuvre was never triggered', flush=True)
    print(
        f"    {duration:.0f} s wall clock: capture {row['capture_confirmed']}, "
        f"envelope {row['envelope_reached']}, closest {float(row['closest_approach_m']):.2f} m, "
        f"abort {row['abort_fired']} {row['abort_reason']}, phase {row['end_phase']}",
        flush=True)
    return row_path.read_text().splitlines()[-1]


def main(argv=None):
    """Run the batch and score it."""
    args = parse_args(argv if argv is not None else sys.argv[1:])
    scenarios = scenario_list(args.scenarios)
    if args.dry_run:
        for scenario in scenarios:
            for index in range(args.runs_per_scenario):
                print(scenario, args.seed_base + index,
                      layout(scenario, args.seed_base + index))
        return 0

    # The domain is set before rclpy starts, so the trigger joins the runs' graph
    # and the batch is kept away from the other sessions in the container.
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    import rclpy
    rclpy.init()
    trigger = ManoeuvreTrigger()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    runs_csv = output_dir / 'runs.csv'
    header = None
    lines = []
    if runs_csv.exists():
        # A batch that was stopped half way is resumed: its rows are kept.
        existing = runs_csv.read_text().splitlines()
        header, lines = existing[0], existing[1:]
    done = {(line.split(',')[0], int(line.split(',')[1])) for line in lines}

    port = args.gazebo_port
    try:
        # The scenarios take turns, so a batch that is stopped half way has runs of
        # every scenario.
        for index in range(args.runs_per_scenario):
            for scenario in scenarios:
                seed = args.seed_base + index
                if (scenario, seed) in done:
                    continue
                run_dir = output_dir / f'{scenario}_seed{seed:03d}'
                line = run_one(scenario, seed, args, run_dir, port, trigger)
                port += 1
                if line is None:
                    continue
                if header is None:
                    header = (run_dir / 'run.csv').read_text().splitlines()[0]
                lines.append(line)
                runs_csv.write_text('\n'.join([header] + lines) + '\n')
    finally:
        trigger.close()
        rclpy.shutdown()

    if not lines:
        print('no run produced a result')
        return 1
    report = subprocess.run(
        ['ros2', 'run', 'interceptor_drone', 'intercept_metrics_report',
         str(runs_csv), str(output_dir / 'summary.csv')],
        capture_output=True, text=True)
    (output_dir / 'report.txt').write_text(report.stdout)
    print(report.stdout)
    print(f'the runs and the report are in {output_dir}')
    return report.returncode


if __name__ == '__main__':
    sys.exit(main())
