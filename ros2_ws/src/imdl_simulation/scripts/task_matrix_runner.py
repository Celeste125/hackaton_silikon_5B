#!/usr/bin/env python3
"""Run every official Gazebo mission with its real task solver."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time


OFFICIAL_CASES = tuple(
    f'task{task}_{variant}'
    for task in range(1, 6)
    for variant in ('a', 'b')
)
RESULT_PATTERN = re.compile(r'task_result:\s*(\{.*\})')
READY_MARKER = 'Gazebo ready; starting localization-first Nav2 bringup.'


def parse_case(value: str) -> tuple[str, str]:
    """Return canonical task and variant from ``taskN_a``."""
    match = re.fullmatch(r'task([1-5])_([ab])', value.strip().lower())
    if match is None:
        raise ValueError(
            f'Invalid case {value!r}; expected one of: '
            f'{", ".join(OFFICIAL_CASES)}'
        )
    return f'task{match.group(1)}', match.group(2)


def extract_task_result(log_text: str) -> dict | None:
    """Extract the most recent mission result JSON from a launch log."""
    matches = RESULT_PATTERN.findall(log_text)
    if not matches:
        return None
    try:
        result = json.loads(matches[-1])
    except json.JSONDecodeError:
        return None
    return result if isinstance(result, dict) else None


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding='utf-8', errors='replace')
    except FileNotFoundError:
        return ''


def stop_process_group(process: subprocess.Popen, grace_sec: float = 12.0) -> None:
    """Stop one launch and all children without touching unrelated ROS nodes."""
    if process.poll() is not None:
        return
    for sig, timeout in (
        (signal.SIGINT, grace_sec),
        (signal.SIGTERM, 5.0),
        (signal.SIGKILL, 2.0),
    ):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            continue


def wait_for_text(
    process: subprocess.Popen,
    log_path: Path,
    marker: str,
    timeout_sec: float,
) -> tuple[bool, str]:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        text = read_text(log_path)
        if marker in text:
            return True, ''
        return_code = process.poll()
        if return_code is not None:
            return False, f'process exited early with code {return_code}'
        time.sleep(0.5)
    return False, f'timed out after {timeout_sec:.1f}s waiting for {marker!r}'


def wait_for_result(
    process: subprocess.Popen,
    log_path: Path,
    timeout_sec: float,
) -> tuple[dict | None, str]:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        text = read_text(log_path)
        result = extract_task_result(text)
        if result is not None:
            return result, ''
        return_code = process.poll()
        if return_code is not None:
            return None, f'solver exited early with code {return_code}'
        time.sleep(0.5)
    return None, f'task result timed out after {timeout_sec:.1f}s'


def launch_process(command: list[str], environment: dict, log_path: Path):
    log_stream = log_path.open('w', encoding='utf-8')
    process = subprocess.Popen(
        command,
        env=environment,
        stdout=log_stream,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    return process, log_stream


def run_case(
    case_name: str,
    index: int,
    args,
    run_directory: Path,
) -> dict:
    task, variant = parse_case(case_name)
    case_directory = run_directory / case_name
    case_directory.mkdir(parents=True, exist_ok=True)
    simulation_log = case_directory / 'simulation.log'
    solver_log = case_directory / 'solver.log'

    environment = os.environ.copy()
    environment.update({
        'ROS_DOMAIN_ID': str(args.domain_id),
        'ROS_LOCALHOST_ONLY': '1',
        'ROS_LOG_DIR': str(case_directory / 'ros_logs'),
        'GAZEBO_MASTER_URI': f'http://127.0.0.1:{args.gazebo_port + index}',
    })
    simulation_command = [
        'ros2', 'launch', 'imdl_simulation', 'autonomy.launch.py',
        f'task:={task}',
        f'variant:={variant}',
        'gui:=false',
        'rviz:=false',
        'perception:=true',
        f'readiness_timeout_sec:={args.readiness_timeout}',
    ]
    solver_command = [
        'ros2', 'launch', 'imdl_autonomy', 'run_task.launch.py',
        f'task:={task}',
        f'variant:={variant}',
    ]

    started = time.monotonic()
    simulation = solver = None
    simulation_stream = solver_stream = None
    result = None
    error = ''
    try:
        simulation, simulation_stream = launch_process(
            simulation_command, environment, simulation_log
        )
        ready, error = wait_for_text(
            simulation,
            simulation_log,
            READY_MARKER,
            args.simulation_startup_timeout,
        )
        if not ready:
            return _case_summary(
                case_name, started, result, error, simulation_log, solver_log
            )

        solver, solver_stream = launch_process(
            solver_command, environment, solver_log
        )
        timeout = (
            args.task5_timeout if task == 'task5' else args.task_timeout
        )
        result, error = wait_for_result(solver, solver_log, timeout)
        if result is not None and not result.get('success', False):
            error = (
                f"solver reported failure: {result.get('reason', 'unknown')} - "
                f"{result.get('message', '')}"
            )
    finally:
        if solver is not None:
            stop_process_group(solver)
        if simulation is not None:
            stop_process_group(simulation)
        if solver_stream is not None:
            solver_stream.close()
        if simulation_stream is not None:
            simulation_stream.close()
        time.sleep(args.cleanup_delay)

    return _case_summary(
        case_name, started, result, error, simulation_log, solver_log
    )


def _case_summary(
    case_name: str,
    started: float,
    result: dict | None,
    error: str,
    simulation_log: Path,
    solver_log: Path,
) -> dict:
    success = bool(result and result.get('success')) and not error
    return {
        'case': case_name,
        'success': success,
        'duration_sec': round(time.monotonic() - started, 3),
        'error': error,
        'task_result': result,
        'simulation_log': str(simulation_log),
        'solver_log': str(solver_log),
    }


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Run official IMDL task worlds and solvers end-to-end.'
    )
    parser.add_argument(
        '--domain-id',
        type=int,
        default=99,
        help='Isolated ROS domain used by all matrix processes (default: 99).',
    )
    parser.add_argument(
        '--cases',
        nargs='+',
        default=list(OFFICIAL_CASES),
        help='Subset such as task1_a task3_b; default is all ten.',
    )
    parser.add_argument('--readiness-timeout', type=float, default=120.0)
    parser.add_argument(
        '--simulation-startup-timeout', type=float, default=180.0
    )
    parser.add_argument('--task-timeout', type=float, default=240.0)
    parser.add_argument('--task5-timeout', type=float, default=660.0)
    parser.add_argument('--cleanup-delay', type=float, default=4.0)
    parser.add_argument('--gazebo-port', type=int, default=11490)
    parser.add_argument(
        '--output-dir',
        default='',
        help='Result directory; defaults to /tmp/imdl_task_matrix_<timestamp>.',
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    if shutil.which('ros2') is None:
        print(
            'ros2 is unavailable. Source /opt/ros/humble/setup.bash and '
            'install/setup.bash first.',
            file=sys.stderr,
        )
        return 2
    try:
        cases = [
            f'{task}_{variant}'
            for task, variant in (parse_case(value) for value in args.cases)
        ]
    except ValueError as exc:
        print(exc, file=sys.stderr)
        return 2
    if not 0 <= args.domain_id <= 232:
        print('domain ID must be between 0 and 232', file=sys.stderr)
        return 2

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    run_directory = Path(
        args.output_dir or f'/tmp/imdl_task_matrix_{timestamp}'
    ).expanduser().resolve()
    run_directory.mkdir(parents=True, exist_ok=True)
    print(
        f'Running {len(cases)} cases with ROS_DOMAIN_ID={args.domain_id}, '
        f'ROS_LOCALHOST_ONLY=1'
    )
    print(f'Logs: {run_directory}')

    summaries = []
    for index, case_name in enumerate(cases):
        print(f'[{index + 1}/{len(cases)}] {case_name}: starting', flush=True)
        summary = run_case(case_name, index, args, run_directory)
        summaries.append(summary)
        state = 'PASS' if summary['success'] else 'FAIL'
        print(
            f'[{index + 1}/{len(cases)}] {case_name}: {state} '
            f'({summary["duration_sec"]:.1f}s) {summary["error"]}',
            flush=True,
        )

    report = {
        'domain_id': args.domain_id,
        'localhost_only': True,
        'cases': summaries,
        'passed': sum(item['success'] for item in summaries),
        'failed': sum(not item['success'] for item in summaries),
    }
    report_path = run_directory / 'report.json'
    report_path.write_text(
        json.dumps(report, indent=2, ensure_ascii=True) + '\n',
        encoding='utf-8',
    )
    print(
        f'Result: {report["passed"]}/{len(summaries)} passed; '
        f'report={report_path}'
    )
    return 0 if report['failed'] == 0 else 1


if __name__ == '__main__':
    raise SystemExit(main())
