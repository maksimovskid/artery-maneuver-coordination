#!/usr/bin/env python3

import sys
import argparse
import subprocess
import configparser
import pathlib
import math
import os
import signal

from pathlib import Path
from typing import Optional, Iterable


def positive_timeout(value: str) -> float:
    """Parse a finite, positive wall-clock limit in seconds."""
    seconds = float(value)
    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError("wall-clock timeout must be a finite positive number")
    return seconds


def run_simulation(cmd, *, timeout=None, **kwargs):
    """Run a simulation and reap its process group on timeout (Linux/POSIX)."""
    if timeout is not None:
        timeout = positive_timeout(str(timeout))
    own_group = timeout is not None and os.name == 'posix'
    with subprocess.Popen(cmd, start_new_session=own_group, **kwargs) as process:
        try:
            return process.wait(timeout=timeout)
        except (subprocess.TimeoutExpired, KeyboardInterrupt):
            if own_group:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            else:
                process.kill()
            process.wait()
            raise


def run_artery(
    launch_conf: Path,
    opp_args: Optional[Iterable[str]] = None,
    scenario: Optional[Path] = None,
    runall: bool = False,
    batchsize: Optional[int] = None,
    jobs: Optional[int] = None,
    verbose: bool = False,
    capture_output: bool = False,
    timeout: Optional[float] = None
) -> int:
    
    if opp_args is None:
        opp_args = []
    
    if scenario is None:
        scenario = Path.cwd()

    if launch_conf.is_file():
        config_filename = launch_conf
    elif launch_conf.is_dir():
        config_filename = launch_conf / 'run-artery.ini'
    else:
        raise ValueError('Argument "launch-conf" must be an existing file or directory')

    config_parser = configparser.ConfigParser(default_section='General')
    with open(config_filename) as config_file:
        config_parser.read_file(config_file)

    if (opp_run := config_parser.get('Executables', 'oppRun', fallback=None)) is None:
        raise ValueError('missing oppRun in Executables section')
    if (opp_runall := config_parser.get('Executables', 'oppRunAll', fallback=None)) is None:
        raise ValueError('missing oppRunAll in Executables section')
    if (ned_folders := config_parser.get('Artery', 'nedFolders', fallback=None)) is None:
        raise ValueError('missing nedFolders in Artery section')
    if (libraries := config_parser.get('Artery', 'libraries', fallback=None)) is None:
        raise ValueError('missing libraries in Artery section')

    cmd = []
    if runall:
        cmd.append(opp_runall)
        if batchsize is not None:
            cmd.extend(['-b', opp_args.batch])
        if jobs is not None:
            cmd.extend(['-j', opp_args.jobs])

    cmd.append(opp_run)
    cmd.extend(['-n', ned_folders])
    cmd.extend(libraries.split())
    cmd.extend(opp_args)

    if verbose:
        print('running command: ', ' '.join(cmd))

    stdout, stderr = sys.stdout, sys.stderr
    if capture_output:
        stdout = stderr = subprocess.DEVNULL
    return run_simulation(cmd, cwd=scenario, stderr=stderr, stdout=stdout, timeout=timeout)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--all', action='store_true', help='sets runall mode')
    parser.add_argument('-b', '--batchsize', dest='batch', action='store')
    parser.add_argument('-j', '--jobs', dest='jobs', action='store')
    parser.add_argument('-l', '--launch-conf', action='store', required=True, type=pathlib.Path)
    parser.add_argument('-s', '--scenario', default=pathlib.Path.cwd(), type=pathlib.Path)
    parser.add_argument('-v', '--verbose', action='store_true')
    parser.add_argument("--wall-clock-timeout", type=positive_timeout,
                        help="wall-clock limit in seconds; kills simulation descendants on timeout")
    args, opp_args = parser.parse_known_args()

    # remove '--' from opp_args when used to split run_artery args from opp_run args
    if len(opp_args) > 0 and opp_args[0] == '--':
        opp_args = opp_args[1:]
    
    sys.exit(run_artery(
        args.launch_conf,
        opp_args,
        args.scenario,
        args.all,
        args.batch,
        args.jobs,
        args.verbose,
        timeout=args.wall_clock_timeout
    ))


if __name__ == '__main__':
    try:
        main()
    except subprocess.TimeoutExpired as error:
        print(f"simulation exceeded wall-clock timeout of {error.timeout:g}s", file=sys.stderr)
        sys.exit(124)
    except KeyboardInterrupt:
        print('exited by user')
        sys.exit(130)
