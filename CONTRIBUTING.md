# Contributing

This is research software with a deliberately bounded validation scope. Read the
[installation guide](docs/install.md), [README](README.md), and documented scenario
limitations before proposing changes. Clone recursively, configure with CMake,
and build from the repository root as described in the installation guide.

Report reproducible problems or discuss features in this repository's
[issue tracker](https://github.com/maksimovskid/artery-maneuver-coordination/issues).
Include the commit, OS, compiler, OMNeT++/SUMO versions, configuration, seed/run,
exact command, expected result, and relevant logs. Avoid posting private data.

Submit focused pull requests explaining the problem, resulting behavior, and
validation. Preserve upstream copyright/license notices and follow surrounding
C++/Python style and `.clang-format`. Keep changes within their stated scope;
do not silently change validated scenario semantics, traffic, calibration,
protocol timing, priorities, trajectories, CAM behavior, or DCC/EDCA settings.
Explain intentional scientific changes and update the relevant documentation.

For protocol/runtime changes, add or update tests covering the changed behavior,
including failure paths where relevant. Preserve the service/application boundary.
Before submitting, run the relevant build and focused tests:

```sh
cmake --build build -j2
cmake --build build --target mcm_timing_tests cam_policy_tests -j2
ctest --test-dir build -R '^(mcm_timing|cam_policy)$' --output-on-failure
python3 -m unittest discover -s tests/mcm -p 'test_*.py'
python3 -m unittest discover -s tests/cam -p 'test_*.py'
python3 -m unittest discover -s tests/release -v
git diff --check
```

Activate the Python environment from the installation guide first. For another
build directory, set `ARTERY_BUILD_DIR` for Python scenario tests and adjust the
CMake/CTest commands. CI runs these checks plus inherited integration tests.
Use `git-clang-format --diff` against your base revision to inspect changed C++
formatting; apply formatting to your changes without reformatting unrelated code.
Documentation-only changes generally need link/command checks and, when relevant,
a local MkDocs build rather than complete simulation reruns.
