all:
	@echo Legacy build/run targets in this Makefile are deprecated.
	@echo Build process of external dependencies is handled entirely by CMake now.
	@echo Animation data/render convenience targets remain supported.

inet simulte veins vanetza:
	@echo Makefile target \'$@\' is obsolete. Use CMake.

animation_data_merging:
	python3 tools/animation/capture_scenario.py merging coordinated
	python3 tools/animation/capture_scenario.py merging baseline
	python3 tools/animation/analyze_scenario.py merging

animation_data_lane_change:
	python3 tools/animation/capture_scenario.py lane-change coordinated
	python3 tools/animation/capture_scenario.py lane-change baseline
	python3 tools/animation/analyze_scenario.py lane-change

animation_data_all: animation_data_merging animation_data_lane_change

animation_render_merging:
	python3 tools/animation/render_comparison.py merging

animation_render_merging_closeup:
	python3 tools/animation/render_comparison.py merging --view closeup

animation_render_merging_interaction:
	python3 tools/animation/render_comparison.py merging --view interaction

animation_render_lane_change:
	python3 tools/animation/render_comparison.py lane-change

animation_render_lane_change_closeup:
	python3 tools/animation/render_comparison.py lane-change --view closeup

animation_render_lane_change_interaction:
	python3 tools/animation/render_comparison.py lane-change --view interaction

animation_render_interaction_all:
	python3 tools/animation/render_comparison.py merging --view interaction
	python3 tools/animation/render_comparison.py lane-change --view interaction

animation_render_all:
	python3 tools/animation/render_comparison.py all
	python3 tools/animation/render_comparison.py merging --view closeup
	python3 tools/animation/render_comparison.py lane-change --view closeup
	python3 tools/animation/render_comparison.py merging --view interaction
	python3 tools/animation/render_comparison.py lane-change --view interaction

test_mcm:
	python3 -m unittest discover -s tests/mcm -p "test_*.py"

diagnose_200cav_qos: diagnose_200cav_qos_10s

diagnose_200cav_qos_10s:
	mkdir -p scenarios/artery-maneuver-coordination/results_diagnostic/200cav_qos
	mkdir -p scenarios/artery-maneuver-coordination/results_generated/test_200CAVs
	if [ -x /usr/bin/time ]; then /usr/bin/time -v -o scenarios/artery-maneuver-coordination/results_diagnostic/200cav_qos/runtime.txt tools/run_artery.py -l build -s scenarios/artery-maneuver-coordination -- omnetpp.ini -u Cmdenv -c envmod-200CAVs-qos-diagnostic -r 0 --sim-time-limit=10s --cmdenv-express-mode=false > scenarios/artery-maneuver-coordination/results_diagnostic/200cav_qos/simulation.log 2>&1; else tools/run_artery.py -l build -s scenarios/artery-maneuver-coordination -- omnetpp.ini -u Cmdenv -c envmod-200CAVs-qos-diagnostic -r 0 --sim-time-limit=10s --cmdenv-express-mode=false > scenarios/artery-maneuver-coordination/results_diagnostic/200cav_qos/simulation.log 2>&1; fi
	python3 tools/analyze_high_load_diagnostic.py --runtime-log scenarios/artery-maneuver-coordination/results_diagnostic/200cav_qos/runtime.txt --log scenarios/artery-maneuver-coordination/results_diagnostic/200cav_qos/simulation.log
	@echo Diagnostic CSV files written under scenarios/artery-maneuver-coordination/results_diagnostic/200cav_qos

diagnose_200cav_qos_30s:
	mkdir -p scenarios/artery-maneuver-coordination/results_diagnostic/200cav_qos_30s
	mkdir -p scenarios/artery-maneuver-coordination/results_generated/test_200CAVs
	if [ -x /usr/bin/time ]; then /usr/bin/time -v -o scenarios/artery-maneuver-coordination/results_diagnostic/200cav_qos_30s/runtime.txt tools/run_artery.py -l build -s scenarios/artery-maneuver-coordination -- omnetpp.ini -u Cmdenv -c envmod-200CAVs-qos-diagnostic-30s -r 0 --sim-time-limit=30s --cmdenv-express-mode=false > scenarios/artery-maneuver-coordination/results_diagnostic/200cav_qos_30s/simulation.log 2>&1; else tools/run_artery.py -l build -s scenarios/artery-maneuver-coordination -- omnetpp.ini -u Cmdenv -c envmod-200CAVs-qos-diagnostic-30s -r 0 --sim-time-limit=30s --cmdenv-express-mode=false > scenarios/artery-maneuver-coordination/results_diagnostic/200cav_qos_30s/simulation.log 2>&1; fi
	python3 tools/analyze_high_load_diagnostic.py --input scenarios/artery-maneuver-coordination/results_diagnostic/200cav_qos_30s --output scenarios/artery-maneuver-coordination/results_diagnostic/200cav_qos_30s --config envmod-200CAVs-qos-diagnostic-30s --time-limit 30s --runtime-log scenarios/artery-maneuver-coordination/results_diagnostic/200cav_qos_30s/runtime.txt --log scenarios/artery-maneuver-coordination/results_diagnostic/200cav_qos_30s/simulation.log --compare-with scenarios/artery-maneuver-coordination/results_diagnostic/200cav_qos/diagnostic_summary.csv --comparison-output scenarios/artery-maneuver-coordination/results_diagnostic/diagnostic_10s_vs_30s.csv
	@echo Diagnostic CSV files written under scenarios/artery-maneuver-coordination/results_diagnostic/200cav_qos_30s
	@echo Comparison CSV written to scenarios/artery-maneuver-coordination/results_diagnostic/diagnostic_10s_vs_30s.csv
