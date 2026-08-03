"""Regression tests for validated maneuver-coordination scenarios."""

from __future__ import annotations

import unittest

try:
    from .log_assertions import ParsedLog, request_ids
    from .scenario_runner import REPO_ROOT, ScenarioRun, run_scenario
except ImportError:
    from log_assertions import ParsedLog, request_ids
    from scenario_runner import REPO_ROOT, ScenarioRun, run_scenario


class McScenarioTestCase(unittest.TestCase):
    def run_checked_scenario(self, config: str) -> ScenarioRun:
        return run_scenario(config)

    def report_run(self, scenario_run: ScenarioRun) -> None:
        print(
            f"[MCM-TEST] test={self.id().split('.')[-1]} "
            f"config={scenario_run.config} duration_s={scenario_run.duration_s:.2f}",
            flush=True,
        )

    def require_negotiation_subtype(self, log: ParsedLog, subtype: str) -> None:
        log.require("MCM-NEGOTIATION", msg=subtype)

    def forbid_negotiation_subtypes(self, log: ParsedLog, subtypes: tuple[str, ...]) -> None:
        for subtype in subtypes:
            log.forbid("MCM-NEGOTIATION", msg=subtype)

    def require_execute_wire_baseline(
        self,
        log: ParsedLog,
        *,
        station: str,
        request_id: str,
        target1: str,
        target2: str,
        priority: str,
    ) -> None:
        initial_expected = {
            "station": station,
            "subtype": "Execute",
            "kind": "Execution",
            "container": "Execution",
            "requestId": "-1",
            "cooperationId": request_id,
            "target1": target1,
            "target2": target2,
            "priority": priority,
        }
        log.require(
            "MCM-WIRE", direction="queued", origin="initial-execute", **initial_expected
        )
        log.require("MCM-WIRE", direction="sent", **initial_expected)

        repeated_expected = {
            "station": station,
            "subtype": "Execute",
            "kind": "Execution",
            "container": "Execution",
            "requestId": "-1",
            "cooperationId": request_id,
            "target1": target1,
            "target2": target2,
            "priority": priority,
        }
        log.require(
            "MCM-WIRE",
            direction="queued",
            origin="repeated-execute",
            **repeated_expected,
        )
        log.require("MCM-WIRE", direction="sent", **repeated_expected)

    def require_cv_execution_armed(
        self,
        log: ParsedLog,
        *,
        station: str,
        sender: str,
        request_id: str,
        target1: str,
        target2: str,
    ) -> None:
        log.require(
            "MCM-WIRE",
            direction="received",
            event="execute-guard",
            station=station,
            sender=sender,
            subtype="Execute",
            container="Execution",
            cooperationId=request_id,
            target1=target1,
            target2=target2,
            result="pass",
            reason="none",
        )
        armed_fields = {
            "direction": "received",
            "event": "cv-execution-armed",
            "station": station,
            "sender": sender,
            "subtype": "Execute",
            "container": "Execution",
            "cooperationId": request_id,
            "target1": target1,
            "target2": target2,
            "mode": "ManeuverExecutionMode",
        }
        log.require("MCM-WIRE", **armed_fields)
        self.assertEqual(
            log.count("MCM-WIRE", **armed_fields),
            1,
            "repeated Execute must not cause a second CV execution transition",
        )
        log.require(
            "MCM-WIRE",
            direction="received",
            event="execute-guard",
            station=station,
            sender=sender,
            subtype="Execute",
            container="Execution",
            cooperationId=request_id,
            result="reject",
            reason="invalid-cv-state",
        )

    def require_completion_cancel_workaround(self, log: ParsedLog, request_id: str) -> None:
        rv_queued = log.require(
            "MCM-WIRE",
            direction="queued",
            subtype="Cancel",
            kind="Negotiation",
            container="Negotiation",
            origin="execution-completion-workaround",
            role="RV",
            executionState="SendExecute",
            requestId=request_id,
        )
        log.require(
            "MCM-WIRE",
            direction="sent",
            station=rv_queued.fields["station"],
            subtype="Cancel",
            kind="Negotiation",
            container="Negotiation",
            requestId=request_id,
            target1=rv_queued.fields["target1"],
            target2=rv_queued.fields["target2"],
        )
        log.require(
            "MCM-WIRE",
            direction="sent",
            event="execution-completion-workaround-confirmed",
            role="RV",
            station=rv_queued.fields["station"],
            requestId=request_id,
        )

        cv_stations = [rv_queued.fields["target1"]]
        if rv_queued.fields.get("hasTarget2") == "1":
            cv_stations.append(rv_queued.fields["target2"])
        for cv_station in cv_stations:
            cv_queued = log.require(
                "MCM-WIRE",
                direction="queued",
                station=cv_station,
                subtype="Cancel",
                kind="Negotiation",
                container="Negotiation",
                origin="execution-completion-workaround",
                role="CV",
                executionState="SendExecuteCV",
                requestId=request_id,
                target1=rv_queued.fields["station"],
            )
            log.require(
                "MCM-WIRE",
                direction="sent",
                event="execution-completion-workaround-confirmed",
                role="CV",
                station=cv_queued.fields["station"],
                requestId=request_id,
            )

    def assert_no_rv_coordination_failure(self, log: ParsedLog) -> None:
        failures = [
            event
            for event in log.find("MCM-FAILURE")
            if event.fields.get("reason") != "None"
        ]
        self.assertEqual(failures, [], "successful scenario latched an RV failure reason")

    def require_trajectory_semantics(
        self,
        log: ParsedLog,
        *,
        request_id: str,
        cv_stations: tuple[str, ...],
        request_source: str = "initial-request",
    ) -> None:
        requested = log.require(
            "MCM-TRAJECTORY",
            event="rv-requested-trajectory-active",
            requestId=request_id,
            trajectoryRole="requested",
            source=request_source,
        )
        self.assertGreater(int(requested.fields["trajectoryPoints"]), 0)

        retry = log.require(
            "MCM-TRAJECTORY",
            event="rv-request-retry",
            requestId=request_id,
            trajectoryRole="requested",
        )
        self.assertEqual(retry.fields["trajectoryPoints"], requested.fields["trajectoryPoints"])

        negotiated = log.require(
            "MCM-TRAJECTORY",
            event="rv-negotiated-trajectory-established",
            requestId=request_id,
            trajectoryRole="negotiated",
        )
        self.assertEqual(
            negotiated.fields["trajectoryPoints"], requested.fields["trajectoryPoints"]
        )
        retained = log.require(
            "MCM-TRAJECTORY",
            event="repeated-execute-retains-negotiated-trajectory",
            requestId=request_id,
            trajectoryRole="negotiated",
        )
        self.assertEqual(retained.fields["trajectoryPoints"], negotiated.fields["trajectoryPoints"])
        self.assertIn("liveTrajectoryPoints", retained.fields)

        for cv_station in cv_stations:
            cv_established = log.require(
                "MCM-TRAJECTORY",
                event="cv-negotiated-trajectory-established",
                requestId=request_id,
                trajectoryRole="negotiated",
                station=cv_station,
            )
            self.assertGreater(int(cv_established.fields["trajectoryPoints"]), 0)
            log.require(
                "MCM-TRAJECTORY",
                event="cv-negotiated-trajectory-cleared",
                requestId=request_id,
                trajectoryRole="negotiated",
                station=cv_station,
                trajectoryPoints=cv_established.fields["trajectoryPoints"],
            )

        log.require(
            "MCM-TRAJECTORY",
            event="rv-negotiated-trajectory-cleared",
            requestId=request_id,
            trajectoryRole="negotiated",
            trajectoryPoints=negotiated.fields["trajectoryPoints"],
        )


class CoordinatedMergingTest(McScenarioTestCase):
    def test_coordinated_merging_protocol_and_completion(self) -> None:
        with self.run_checked_scenario("envmod-19CAVs-merging") as run:
            log = run.parsed_log

            for subtype in ("Request", "Offer", "Confirm", "Accept"):
                self.require_negotiation_subtype(log, subtype)

            selection = log.require("MCM-MERGE-TARGET", event="selection-summary", rvStation="29")
            self.assertEqual(selection.fields.get("target1"), "169")
            self.assertEqual(selection.fields.get("target2"), "309")

            request = log.require(
                "MCM-NEGOTIATION",
                action="SEND",
                msg="Request",
                localVehicleId="car_ml1_1",
                target1="169",
                target2="309",
            )
            self.assertEqual(request.fields.get("priority"), "MediumPriority")
            request_id = request.fields["requestId"]

            self.require_execute_wire_baseline(
                log,
                station="29",
                request_id=request_id,
                target1="169",
                target2="309",
                priority="MediumPriority",
            )
            self.require_cv_execution_armed(
                log,
                station="169",
                sender="29",
                request_id=request_id,
                target1="169",
                target2="309",
            )
            self.require_cv_execution_armed(
                log,
                station="309",
                sender="29",
                request_id=request_id,
                target1="169",
                target2="309",
            )
            # Temporary baseline characterization: successful completion is
            # still represented as negotiation-container Cancel.
            self.require_completion_cancel_workaround(log, request_id)
            self.require_trajectory_semantics(
                log, request_id=request_id, cv_stations=("169", "309")
            )
            self.assert_no_rv_coordination_failure(log)

            log.require("MCM-GAP-DIAG", phase="execution-start", vehicleId="car_ml1_1")
            log.require("MCM-GAP-DIAG", summary="rv-completion", rvStation="29")

            self.assertIn(
                "McApplication RV station 29 queued completion workaround after passing "
                "negotiated trajectory end",
                log.text,
            )
            self.report_run(run)


class MergingBaselineTest(McScenarioTestCase):
    def test_merging_baseline_suppresses_mcm_negotiation(self) -> None:
        service_xml = (
            REPO_ROOT
            / "scenarios/artery-maneuver-coordination/services-envmod-no-mc.xml"
        ).read_text(encoding="utf-8")
        self.assertIn("artery.application.CaService", service_xml)
        self.assertNotIn("McService", service_xml)

        with self.run_checked_scenario("envmod-19CAVs-merging-baseline") as run:
            log = run.parsed_log

            self.assertNotIn("McService", log.text)
            self.assertIn("CaService", service_xml)
            self.forbid_negotiation_subtypes(
                log,
                ("Request", "Offer", "Confirm", "Accept", "Execute"),
            )
            log.forbid("MCM-MERGE-TARGET")
            log.forbid("MCM-GAP-DIAG")
            self.report_run(run)


class CoordinatedEmergencyLaneChangeTest(McScenarioTestCase):
    def test_coordinated_emergency_lane_change_protocol_and_completion(self) -> None:
        with self.run_checked_scenario("envmod-19CAVs-emergency-lane-change") as run:
            log = run.parsed_log

            log.require(
                "MCM-EMERGENCY",
                event="emergency-brake-trigger",
                vehicleId="car_hl0_Emergency",
            )
            log.require(
                "MCM-EMERGENCY",
                event="sent-emergency-execution-mcm",
                vehicleId="car_hl0_Emergency",
            )
            emergency_abort = log.require(
                "MCM-WIRE",
                direction="sent",
                subtype="Abort",
                kind="Execution",
                container="Execution",
                priority="EmergencyPriority",
            )
            self.assertNotEqual(emergency_abort.fields.get("cooperationId"), "-1")
            self.assertNotEqual(emergency_abort.fields.get("target1"), "0")
            log.require(
                "MCM-LC-TRIGGER",
                event="safety-critical-trigger-armed",
                vehicleId="car_hl1_1",
            )
            log.require(
                "MCM-LC-3VEH",
                event="queued-request",
                vehicleId="car_hl1_1",
                targetCv1="309",
                targetCv2="589",
            )

            request = log.require(
                "MCM-NEGOTIATION",
                action="SEND",
                msg="Request",
                localVehicleId="car_hl1_1",
                priority="HighPriority",
                target1="309",
                target2="589",
            )
            request_id = request.fields["requestId"]
            for subtype in ("Offer", "Confirm", "Accept"):
                self.require_negotiation_subtype(log, subtype)

            log.require("MCM-LC-3VEH", event="queued-execute-after-all-accepts", requestId=request_id)
            self.require_execute_wire_baseline(
                log,
                station="449",
                request_id=request_id,
                target1="309",
                target2="589",
                priority="HighPriority",
            )
            self.require_cv_execution_armed(
                log,
                station="309",
                sender="449",
                request_id=request_id,
                target1="309",
                target2="589",
            )
            self.require_cv_execution_armed(
                log,
                station="589",
                sender="449",
                request_id=request_id,
                target1="309",
                target2="589",
            )
            log.require("MCM-LC-EXEC", event="lane-change-execution-complete", requestId=request_id)
            # Temporary baseline characterization: successful completion is
            # still represented as negotiation-container Cancel.
            self.require_completion_cancel_workaround(log, request_id)
            self.require_trajectory_semantics(
                log, request_id=request_id, cv_stations=("309", "589")
            )
            log.require(
                "MCM-FAILURE",
                station="449",
                role="RV",
                requestId=request_id,
                reason="None",
                source="new-lane-change-request",
            )
            self.assert_no_rv_coordination_failure(log)
            log.forbid("MCM-LC-FAILSAFE")
            self.report_run(run)


class EmergencyBaselineTest(McScenarioTestCase):
    def test_emergency_baseline_preserves_braking_and_suppresses_coordination(self) -> None:
        with self.run_checked_scenario("envmod-19CAVs-emergency-lane-change-baseline") as run:
            log = run.parsed_log

            log.require(
                "MCM-BASELINE",
                event="emergency-mcm-suppressed",
                vehicleId="car_hl0_Emergency",
                reason="emergency-braking-only-baseline",
            )
            log.require(
                "MCM-EMERGENCY",
                event="emergency-brake-trigger",
                vehicleId="car_hl0_Emergency",
            )
            log.forbid("MCM-EMERGENCY", event="sent-emergency-execution-mcm")
            log.forbid("MCM-LC-TRIGGER", event="safety-critical-trigger-armed")
            log.forbid("MCM-LC-3VEH", event="queued-request")
            self.forbid_negotiation_subtypes(
                log,
                ("Request", "Offer", "Confirm", "Accept", "Execute"),
            )
            log.forbid("MCM-CV-CONTROL")
            self.report_run(run)


class SecondRequestSmokeTest(McScenarioTestCase):
    def test_second_request_after_forced_reject(self) -> None:
        with self.run_checked_scenario("envmod-19CAVs-second-request-smoke") as run:
            log = run.parsed_log

            log.require("MCM-SECOND-REQUEST-SMOKE", event="force-first-cv-reject")
            reject = log.require(
                "MCM-NEGOTIATION",
                action="SEND",
                msg="Reject",
                localVehicleId="car_hl2_1",
            )
            first_request_id = reject.fields["requestId"]

            queued = log.require(
                "MCM-LC-STATE",
                event="queued-second-request",
                vehicleId="car_hl1_1",
            )
            ready = log.require(
                "MCM-LC-STATE",
                event="second-request-ready",
                vehicleId="car_hl1_1",
            )
            self.assertEqual(queued.fields.get("previousRequestId"), first_request_id)
            self.assertEqual(ready.fields.get("previousRequestId"), first_request_id)
            self.assertNotEqual(queued.fields.get("requestId"), first_request_id)
            self.assertEqual(queued.fields.get("requestId"), ready.fields.get("requestId"))

            second_request_id = queued.fields["requestId"]
            log.require(
                "MCM-FAILURE",
                station="449",
                role="RV",
                requestId=first_request_id,
                reason="None",
                source="new-lane-change-request",
            )
            self.assert_no_rv_coordination_failure(log)
            first_requested = log.require(
                "MCM-TRAJECTORY",
                event="rv-requested-trajectory-active",
                requestId=first_request_id,
                source="initial-request",
            )
            second_requested = log.require(
                "MCM-TRAJECTORY",
                event="rv-requested-trajectory-active",
                requestId=second_request_id,
                source="second-request",
            )
            self.assertNotEqual(first_request_id, second_request_id)
            self.assertGreater(int(first_requested.fields["trajectoryPoints"]), 0)
            self.assertGreater(int(second_requested.fields["trajectoryPoints"]), 0)
            log.require(
                "MCM-NEGOTIATION",
                action="SEND",
                msg="Request",
                localVehicleId="car_hl1_1",
                requestId=second_request_id,
            )
            log.require(
                "MCM-LC-3VEH",
                event="queued-confirm-after-all-offers",
                requestId=second_request_id,
            )
            log.require(
                "MCM-LC-3VEH",
                event="queued-execute-after-all-accepts",
                requestId=second_request_id,
            )
            self.require_execute_wire_baseline(
                log,
                station="449",
                request_id=second_request_id,
                target1=queued.fields["targetCv1"],
                target2=queued.fields["targetCv2"],
                priority="HighPriority",
            )
            negotiated = log.require(
                "MCM-TRAJECTORY",
                event="rv-negotiated-trajectory-established",
                requestId=second_request_id,
            )
            self.assertEqual(
                negotiated.fields["trajectoryPoints"],
                second_requested.fields["trajectoryPoints"],
            )

            all_request_ids = request_ids(log.find("MCM-NEGOTIATION", msg="Request"))
            self.assertLess(
                all_request_ids.count(int(second_request_id)),
                20,
                "second-request smoke run appears to be stuck in an excessive Request retry loop",
            )
            self.report_run(run)


if __name__ == "__main__":
    unittest.main()
