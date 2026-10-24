#!/usr/bin/env python3
"""
Run a batch of FOLLOW scenarios and score them against the exit criteria.

The exit criteria of Phase 2 of the plan are stated over 100 randomized runs:

  * the minimum distance to every person or vehicle is at least d_min and to
    every obstacle at least d_obstacle_min, in 100 % of the runs;
  * the target is within +-15 degrees of the boresight at least 95 % of the
    time;
  * the track is reacquired after a 2 s occlusion.

Every run is a full simulation: the launch starts Gazebo, the tracker, the
planner, the safety filter and the controller, and the metrics recorder writes
the numbers of the run into a JSON file. This script runs the batch, collects
the summaries and scores the batch.

The runs are reproducible: the seed of the ground-truth source fixes the noise
and the dropout, and the seed of the scenario shifts the start of the target
along its path. The same seed gives the same run.

Usage:
  ros2 run interceptor_drone run_follow_batch --runs 4 --scenarios walker
  python3 scripts/run_follow_batch.py --runs 20 --scenarios all --output-dir /tmp/batch

Options:
  --runs N            the number of runs (default 4)
  --scenarios LIST    walker, jogger, occlusion, bystander or all (default all).
                      The runs are spread over the scenarios in turn.
  --run-duration S    the length of a run in simulation time (default 60)
  --seed-base N       the seed of the first run (default 1)
  --domain-id N       the ROS domain of the runs (default 77). A domain of its
                      own keeps the batch away from the other sessions.
  --gazebo-port N     the port of the Gazebo master of the runs (default 11377).
                      Every run gets a port of its own, so a batch never collides
                      with another Gazebo in the same container.
  --output-dir PATH   where the summaries and the report are written
  --timeout S         the wall clock time a run may take (default 400)
  --keep-going        keep going when a run fails (the default)
  --dry-run           print the commands without running them
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

SCENARIOS = ['walker', 'jogger', 'occlusion', 'bystander']

# The criteria of the phase, from the plan (section 5, Phase 2).
FRAMING_TOLERANCE_DEG = 15.0
MIN_FRAMING_SCORE = 0.95
REACQUIRE_LOSS_TIME_S = 2.0


def parse_args(argv):
    """Parse the command line."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs', type=int, default=4, help='the number of runs')
    parser.add_argument(
        '--scenarios', default='all',
        help='walker, jogger, occlusion, bystander, all or a comma separated list')
    parser.add_argument('--run-duration', type=float, default=60.0,
                        help='the length of a run in simulation time [s]')
    parser.add_argument('--seed-base', type=int, default=1, help='the seed of the first run')
    parser.add_argument('--domain-id', type=int, default=77, help='the ROS domain of the runs')
    parser.add_argument('--gazebo-port', type=int, default=11377,
                        help='the port of the Gazebo master of the runs')
    parser.add_argument('--output-dir', default='/tmp/follow_batch',
                        help='where the summaries and the report are written')
    parser.add_argument('--timeout', type=float, default=400.0,
                        help='the wall clock time a run may take [s]')
    parser.add_argument('--retries', type=int, default=2,
                        help='how many times a failed run is retried')
    parser.add_argument('--dry-run', action='store_true', help='print the commands only')
    return parser.parse_args(argv)


def scenario_list(names):
    """Return the scenarios of the batch, in the order they are run."""
    if names.strip() == 'all':
        return list(SCENARIOS)
    chosen = [name.strip() for name in names.split(',') if name.strip()]
    for name in chosen:
        if name not in SCENARIOS:
            raise SystemExit(f"unknown scenario '{name}', pick from {SCENARIOS}")
    return chosen


def run_one(scenario, seed, args, run_dir, gazebo_port, attempt=0):
    """Run one scenario and return its summary, or None when it failed."""
    run_dir.mkdir(parents=True, exist_ok=True)
    command = [
        'ros2', 'launch', 'interceptor_drone', 'follow_scenario.launch.py',
        f'follow_scenario:={scenario}',
        f'groundtruth_seed:={seed}',
        f'run_duration_s:={args.run_duration}',
        f'result_dir:={run_dir}',
        'rviz:=false',
        'gui:=false',
    ]
    environment = dict(os.environ)
    environment['ROS_DOMAIN_ID'] = str(args.domain_id)
    # The Gazebo master of this run: a port of its own, so the batch does not
    # collide with another Gazebo in the same container.
    master_uri = f'http://127.0.0.1:{gazebo_port}'
    environment['GAZEBO_MASTER_URI'] = master_uri
    summary_path = run_dir / 'summary.json'

    if args.dry_run:
        print(' '.join(command))
        return None

    print(f'--- {scenario} (seed {seed}) ---', flush=True)
    if summary_path.exists():
        # A summary of an earlier attempt would end this one at once.
        summary_path.unlink()
    started = time.time()
    # The output goes to a file and not to a pipe: the pipe fills up after a few
    # kilobytes and the launch blocks on it.
    log_path = run_dir / 'launch.log'
    with log_path.open('w') as log_file:
        process = subprocess.Popen(
            command, env=environment, stdout=log_file, stderr=subprocess.STDOUT,
            text=True, start_new_session=True)
    output = ''
    try:
        # The recorder writes the summary and stops the launch when the run is
        # over, so the launch ends by itself.
        while time.time() - started < args.timeout:
            if summary_path.exists():
                # The file is written at the very end of the run: give the launch
                # a moment to shut down.
                time.sleep(2.0)
                break
            if process.poll() is not None:
                break
            if simulator_died(log_path):
                # gzserver crashes now and then (a segfault early in the run). The
                # sim clock stops with it, so the run would only end at the wall
                # clock timeout: give up on it at once and retry.
                break
            time.sleep(1.0)
    finally:
        if process.poll() is None:
            # The whole group goes down, the simulator included: a simulator that
            # outlives its run keeps the Gazebo master port and publishes /clock.
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGINT)
            except OSError:
                process.terminate()
            try:
                process.wait(timeout=45)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except OSError:
                    process.kill()
                process.wait()
            # The simulator is in the same process group as the launch, so it goes
            # down with it: a simulator that outlives its run keeps the Gazebo
            # master port and publishes /clock to the next run.
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except OSError:
                pass
        output = log_path.read_text() if log_path.exists() else ''

    duration = time.time() - started
    summary = None
    if summary_path.exists():
        with summary_path.open() as handle:
            summary = json.load(handle)
    if summary is None or summary.get('duration_s', 0.0) < 0.5 * args.run_duration:
        # The run produced nothing, or only a fraction of it: the simulator
        # crashed, or the world did not load. It is worth another try, on a port
        # of its own.
        print(f'    FAILED after {duration:.0f} s: no samples', flush=True)
        if attempt < args.retries:
            print(f'    retrying (attempt {attempt + 2})', flush=True)
            return run_one(
                scenario, seed, args, run_dir, gazebo_port + 100, attempt + 1)
        tail = '\n'.join(output.splitlines()[-15:])
        print(tail, flush=True)
        return None
    print(
        f'    {duration:.0f} s wall clock: framing {summary["framing_score"] * 100:.1f} %, '
        f'min person {summary["min_person_distance_3d"]:.2f} m (3D), '
        f'min obstacle {summary["min_obstacle_distance"]:.2f} m, '
        f'verdict {"PASS" if summary["all_ok"] else "FAIL"}', flush=True)
    return summary


def simulator_died(log_path):
    """Return True when the launch log says that gzserver has died."""
    try:
        text = log_path.read_text(errors='replace')
    except OSError:
        return False
    return any(
        'process has died' in line and '[gzserver' in line for line in text.splitlines())


def load_summary(run_dir, args):
    """Return the summary of a run that is already done, or None."""
    path = run_dir / 'summary.json'
    if not path.exists():
        return None
    try:
        with path.open() as handle:
            summary = json.load(handle)
    except (OSError, ValueError):
        return None
    if summary.get('duration_s', 0.0) < 0.5 * args.run_duration:
        return None
    return summary


def score(summaries, args):
    """Score the batch against the exit criteria."""
    passed = [s for s in summaries if s.get('all_ok')]
    distance_ok = [s for s in summaries if s.get('distance_ok')]
    framing_ok = [s for s in summaries if s.get('framing_ok')]
    reacquire_ok = [s for s in summaries if s.get('reacquire_ok')]

    def worst(key):
        values = [s[key] for s in summaries if key in s and s[key] is not None]
        return min(values) if values else None

    def mean(key):
        values = [s[key] for s in summaries if key in s and s[key] is not None]
        return sum(values) / len(values) if values else None

    long_losses = sum(s.get('long_loss_count', 0) for s in summaries)
    reacquired = sum(s.get('reacquired_count', 0) for s in summaries)
    return {
        'runs': len(summaries),
        'passed': len(passed),
        'distance_ok_runs': len(distance_ok),
        'framing_ok_runs': len(framing_ok),
        'reacquire_ok_runs': len(reacquire_ok),
        'min_person_distance_3d': worst('min_person_distance_3d'),
        'min_person_margin_3d': worst('min_person_margin_3d'),
        'min_person_margin_horizontal': worst('min_person_margin_horizontal'),
        'min_person_margin_horizontal_effective': worst('min_person_margin_horizontal_effective'),
        'min_obstacle_distance': worst('min_obstacle_distance'),
        'min_obstacle_margin': worst('min_obstacle_margin'),
        'framing_score_mean': mean('framing_score'),
        'framing_score_worst': worst('framing_score'),
        'framing_mean_deg': mean('framing_mean_deg'),
        'framing_p95_deg_worst': worst('framing_p95_deg'),
        'long_losses': long_losses,
        'reacquired': reacquired,
        'criteria': {
            'distance_100_percent': len(distance_ok) == len(summaries) and len(summaries) > 0,
            'framing_95_percent': len(framing_ok) == len(summaries) and len(summaries) > 0,
            'reacquire_after_long_loss':
                long_losses == reacquired and (long_losses > 0 or
                                               all(s.get('long_loss_count', 0) == 0
                                                   for s in summaries)),
        },
        'tolerances': {
            'framing_tolerance_deg': FRAMING_TOLERANCE_DEG,
            'min_framing_score': MIN_FRAMING_SCORE,
            'reacquire_loss_time_s': REACQUIRE_LOSS_TIME_S,
        },
    }


def number(value, digits=2, scale=1.0):
    """Format a number of the report, or 'n/a' when there is none."""
    if value is None:
        return 'n/a'
    return f'{value * scale:.{digits}f}'


def report(score, summaries, args):
    """Build the human readable report of the batch."""
    lines = []
    lines.append('=== FOLLOW batch ===')
    lines.append(f"runs: {score['runs']}, passed: {score['passed']}")
    lines.append(
        f"min 3D distance to a person/vehicle: {number(score['min_person_distance_3d'])} m "
        f"(margin to d_min_3d {number(score['min_person_margin_3d'])} m, "
        f"to the horizontal standoff {number(score['min_person_margin_horizontal'])} m, "
        f"to the filter's inflated standoff "
        f"{number(score['min_person_margin_horizontal_effective'])} m)")
    lines.append(
        f"min distance to an obstacle: {number(score['min_obstacle_distance'])} m "
        f"(margin {number(score['min_obstacle_margin'])} m)")
    lines.append(
        f"framing: mean {number(score['framing_score_mean'], 1, 100.0)} %, "
        f"worst {number(score['framing_score_worst'], 1, 100.0)} %, "
        f"mean error {number(score['framing_mean_deg'], 1)} deg")
    lines.append(
        f"long losses: {score['long_losses']}, reacquired: {score['reacquired']}")
    for name, ok in score['criteria'].items():
        lines.append(f"criterion {name}: {'PASS' if ok else 'FAIL'}")
    per_scenario = {}
    for summary in summaries:
        per_scenario.setdefault(summary.get('scenario', '?'), []).append(summary)
    for name, group in sorted(per_scenario.items()):
        frames = [g['framing_score'] for g in group]
        margins = [g['min_person_margin_3d'] for g in group]
        lines.append(
            f"  {name}: {len(group)} runs, framing {min(frames) * 100:.0f}-"
            f"{max(frames) * 100:.0f} %, worst person margin {min(margins):.2f} m, "
            f"passed {sum(1 for g in group if g['all_ok'])}")
    return '\n'.join(lines) + '\n'


def main(argv=None):
    """Run the batch."""
    args = parse_args(argv if argv is not None else sys.argv[1:])
    scenarios = scenario_list(args.scenarios)
    output_dir = Path(args.output_dir)
    # The directory is not deleted: the runs of a previous batch stay there, and
    # the summaries of this batch overwrite the ones of the same name.
    output_dir.mkdir(parents=True, exist_ok=True)

    summaries = []
    for index in range(args.runs):
        scenario = scenarios[index % len(scenarios)]
        seed = args.seed_base + index
        run_dir = output_dir / f'run_{index:03d}_{scenario}_seed{seed}'
        # A batch that was stopped half way is resumed: a run that is done is kept.
        summary = load_summary(run_dir, args)
        if summary is None:
            summary = run_one(scenario, seed, args, run_dir, args.gazebo_port + index)
        if summary is not None:
            summary['run'] = index
            summary['seed'] = seed
            summaries.append(summary)
        # The report is written after every run, so a batch that is stopped
        # half way still leaves the numbers of the runs it did.
        write_report(summaries, args, output_dir, final=False)

    result = score(summaries, args)
    write_report(summaries, args, output_dir, final=True)
    print(report(result, summaries, args))
    print(f"the report is in {output_dir}")
    return 0 if result['passed'] == result['runs'] and result['runs'] > 0 else 1


def write_report(summaries, args, output_dir, final):
    """Write the report of the batch so far."""
    result = score(summaries, args)
    with (output_dir / 'batch_report.json').open('w') as handle:
        json.dump({'score': result, 'runs': summaries}, handle, indent=2)
    text = report(result, summaries, args)
    (output_dir / 'batch_report.txt').write_text(text)
    if final:
        print(text)


if __name__ == '__main__':
    sys.exit(main())
