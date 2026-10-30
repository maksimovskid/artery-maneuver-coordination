# Install and verify this research repository

These instructions are for
[Priority-Based Maneuver Coordination in Artery](https://github.com/maksimovskid/artery-maneuver-coordination),
maintained by Daniel Maksimovski. It extends the upstream
[Artery framework](https://github.com/riebl/artery). Use this checkout and its pinned
dependencies; an upstream Artery checkout does not contain these MCM extensions.
Run commands below in Bash on Linux, from the repository root unless stated otherwise.

## Clone with dependencies

```bash
git clone --recursive https://github.com/maksimovskid/artery-maneuver-coordination.git
cd artery-maneuver-coordination
git submodule update --init --recursive
git submodule status --recursive
```

The last command should show initialized submodules at their recorded revisions
(no `-` or `+` prefix). Do not use `git submodule update --remote` for reproduction.
Vanetza comes from the project's `vanetza-maneuver-coordination` fork and provides
its MCM ASN.1 support. INET and Veins supply network/radio integration. The default
build also uses pybind11 for Storyboard; SimuLTE and sim0mqpp serve optional features.
The `scenarios/mt-its2017` submodule supplies an upstream example. Recursive cloning
fetches all of these and any nested submodules. No source files need to be copied
from another checkout. `.gitmodules` records their URLs and Git records their pins.

## Toolchain and dependencies

| Component | Requirement / evidence |
| --- | --- |
| CMake | Minimum **3.20**, enforced by root `CMakeLists.txt`; tested locally with **3.22.1**. |
| C++ compiler | **C++17**, required by root CMake; tested locally with **GCC 11.4.0**. A C compiler and Make or Ninja are also needed. |
| Boost | Root CMake requires **1.69**; the pinned Vanetza requires **1.70**, making **1.70** the effective minimum. Include `date_time`; tested locally with **1.71**. |
| OMNeT++ | Root CMake enforces minimum **5.5.1**. **5.6.2** is used by the local validation setup and specified by `Dockerfile`. Other versions are not validated here. |
| SUMO | **Configured CI baseline: 1.21.0; locally validated: 1.26.0.** Regression verification on 1.21.0 remains pending hosted/container CI. No supported version range is claimed; root CMake enforces no SUMO version minimum. |
| Python | Use **3.10 or later** for the scenario test harness (including its `X \| None` annotations); tested locally with **3.10.12**. Python development headers are needed for Storyboard/pybind11. |
| Vanetza libraries | GeographicLib (optional probe at **1.37**), Crypto++ (**5.6.1**), and OpenSSL (**1.1.1**) are detected by pinned Vanetza CMake. The commands below install GeographicLib and Crypto++; security backends depend on installed libraries/CMake options. |

For a Debian/Ubuntu host, install the development packages below. Package names
can differ between distributions; check the version requirements above.
OMNeT++ and SUMO must be installed separately.

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake ninja-build git pkg-config \
  python3-dev python3-venv libboost-all-dev libgeographiclib-dev libcrypto++-dev
```

Install/build **OMNeT++ 5.6.2** using the [OMNeT++ distribution instructions](https://omnetpp.org), including the
release libraries and `opp_run`/`opp_msgc`. Activate that installation's `setenv`
in the current shell before configuring this repository. This is an external
simulator installation, not an Artery source dependency. Alternatively pass its
installation root as `-DOMNETPP_ROOT=/path/to/omnetpp-5.6.2` to CMake.
Install SUMO using its [installation guide](https://sumo.dlr.de/docs/Installing/index.html)
and ensure `sumo` is on `PATH`; headless tests do not need `sumo-gui`.
Check the active tools:

```bash
command -v opp_run
sumo --version
cmake --version
c++ --version
python3 --version
```

`Dockerfile` and the Debian [devcontainer](devcontainers.md) provide an alternative
dependency environment with OMNeT++ 5.6.2 and SUMO 1.21.0. The Arch devcontainer
specifies SUMO 1.23.1. These are configuration pins, not additional tested-version
claims. CI consumes `ghcr.io/maksimovskid/artery-gh-runner:latest`; its maintainer
builds/rebuilds it using the manually triggered **Prepare GitHub container** workflow
before running CI. The image contains tools, not this project's compiled software.
No container image is built or published by following the local instructions.

## SUMO version reproducibility

**Configured CI baseline: SUMO 1.21.0; locally validated: SUMO 1.26.0.**
Regression verification on 1.21.0 remains pending hosted/container CI. The Dockerfile
and Debian devcontainer explicitly pin 1.21.0; this does not establish a supported
version range or prove that the mutable GHCR `latest` image contains that version.
Confirm the actual executable version inside the CI/container environment before
attributing its regression results to 1.21.0.
For archival reproduction, also record the container image digest and compiler,
library, and Python dependency versions. The base-image tag, OS packages, and
Python requirements are not fully version-locked, so the setup is not a guarantee
of a byte-identical environment on a later date.

The launcher defaults to the `sumo` executable resolved through `PATH`. Select the
intended installation before running tests, check `command -v sumo` and
`sumo --version`, and record the complete version/build output with your repository
revision, scenario configuration, and run/seed. If `SUMO_HOME` is set, it must refer
to that same installation's SUMO data/tools directory; do not retain it from a
different version. A packaged installation need not require setting it manually.
`ARTERY_BUILD_DIR` selects Artery's compiled libraries, not the SUMO executable.

The checked-in validation network has SUMO network-format version 1.16 and a
Netedit 1.19.0 provenance comment; route/type files also contain older generator
comments. Absolute paths inside those XML comments are historical provenance,
not inputs to the simulator. Use the checked-in network, routes, vehicle types,
and trajectory CSV directly: no Netedit regeneration or manually prepared files
are needed for the supplied scenarios.

Static inspection found no explicit requirement for a post-1.21 SUMO feature in
the regression inputs or launcher options. This is not evidence of identical
regression outcomes across versions. Upstream's
[2025 release notes](https://sumo.dlr.de/docs/ChangeLog/Changes_in_2025_releases.html)
document changes to simulation and TraCI behavior after 1.21. Only running the
regressions on the selected version establishes the validation result; the
1.26.0 results must not be described as 1.21.0 CI results.

## Configure and build

Keep the default INET, Veins, environment-model, Storyboard and scenario options
for the supplied validation scenarios. CMake builds their dependencies together;
no manual external-framework builds are needed.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j2
```

## Run coordinated merging

```bash
tools/run_artery.py -l build -s scenarios/artery-maneuver-coordination --wall-clock-timeout 300 -- omnetpp.ini -u Cmdenv -c envmod-19CAVs-merging -r 0 --sim-time-limit=30s --cmdenv-express-mode=false
```

`-s` selects the scenario working directory, so the tracked routes, SUMO network,
and `coordinates_new_map.csv` resolve without machine-specific paths. The
30-second limit is simulated time; the optional 300-second limit is wall-clock time.
Other configurations and scope limitations are described in the [project README](https://github.com/maksimovskid/artery-maneuver-coordination#supported-scenarios-and-animations).

## Verify

Activate a Python virtual environment for release tooling and inherited integration
test dependencies. MCM/CAM simulation assertions themselves use the standard library.

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r tools/ci/requirements.txt
cmake --build build --target mcm_timing_tests cam_policy_tests -j2
ctest --test-dir build -R '^(mcm_timing|cam_policy)$' --output-on-failure
python3 -m unittest discover -s tests/release -v
python3 -m unittest discover -s tests/mcm -p 'test_*.py' -v
python3 -m unittest discover -s tests/cam -p 'test_*.py' -v
```

The focused C++ targets are deliberately excluded from the default build; build
them explicitly before CTest. MCM runs five scenario regressions. CAM runs three
tests (including three periodic-policy subcases). Successful scenario tests remove
temporary output; failures preserve their log path in the assertion message.

Python scenario tests default to `build` relative to the repository root. To use
CI's layout or another configured build, set `ARTERY_BUILD_DIR` (absolute paths also
work). Each simulation defaults to a generous **300-second wall-clock timeout**;
override it for slower hosts. Timeout cleanup kills the simulation process group,
including SUMO descendants on Linux. These limits do not change simulated timing.

```bash
ARTERY_BUILD_DIR=build/Release ARTERY_TEST_TIMEOUT=600 python3 -m unittest discover -s tests/mcm -p 'test_*.py' -v
```

That example requires configuring and building `build/Release` first (substitute
that path in the configure/build commands above). The inherited integration suite
can be run separately; it writes results into ignored scenario result directories:

```bash
PYTHONPATH="$PWD" python3 tools/ci/run.py -l build/run-artery.ini -s scenarios
```

For documentation builds, see [MkDocs instructions](mkdocs.md). For reporting
problems and contributing, see [CONTRIBUTING.md](https://github.com/maksimovskid/artery-maneuver-coordination/blob/master/CONTRIBUTING.md).
