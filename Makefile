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
