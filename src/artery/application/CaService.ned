//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package artery.application;

// CAM facilities generation/submission, not access DCC, EDCA/MAC scheduling,
// maneuver negotiation, trajectory planning or MCM priority selection.
// CBR is a unitless [0,1] busy ratio from the selected primary radio channel.
// Boolean flags are unitless and opt-in. Timed adaptive holds are minimum
// recovery holds: one OMNeT++ RNG draw per entry, then now>deadline AND CBR<.50
// to recover. They are neither packet backoffs nor generation/MAC intervals.
// Ending one reduction reason cannot cancel another; baseline is never rewritten.
simple CaService like ItsG5Service
{
    parameters:
        // CaObject event for each successfully decoded/validated incoming CAM,
        // emitted before updating CAM awareness. This is application reception.
        @signal[CamReceived](type=CaObject);
        // CaObject event at immediate CAM generation, just before BTP submission.
        // Historical name: NOT a confirmation of on-air transmission or MAC time.
        @signal[CamSent](type=CaObject);

        // Optional vector modes extract unitless station ID and the packet's
        // generationDeltaTime (milliseconds modulo 65536, not seconds or age).
        // Vector time is the reception/generation event time respectively.
        @statistic[reception](source=CamReceived;record=vector(camStationId)?,vector(camGenerationDeltaTime)?);
        // "transmission" retains its existing name but records CAM generation/
        // submission. Access DCC may subsequently delay or drop the packet.
        @statistic[transmission](source=CamSent;record=vector(camStationId)?,vector(camGenerationDeltaTime)?);

        // Unitless bool, default true; all modes additionally respect the current
        // facilities DP2 interval (clamped by minInterval/maxInterval). Periodic
        // generation requires elapsed>=max(effective period, facilities interval).
        // False bypasses this facilities query, not adaptive 1 Hz or access DCC.
        // LIMERIC/reactive DCC/EDCA remain independent; 1 Hz generation != airtime.
        bool withDccRestriction = default(true);

        // Seconds, default .1 s: existing minimum gate when facilities DCC is
        // disabled; lower bound of its interval clamp otherwise. SameAsCAM with
        // fixedRate=true uses this gate. Adaptive/periodic policy adds its period.
        double minInterval @unit(s) = default(0.1s);
        // Seconds, default 1 s: normal low-dynamics interval and upper bound of
        // the facilities interval clamp. Does not replace a configured/adaptive
        // periodic interval; retained public CAM behavior, not a new adaptive knob.
        double maxInterval @unit(s) = default(1.0s);

        // Unitless bool, default false; existing SameAsCAM baseline option:
        // true bypasses dynamics and generates at minInterval, facilities-limited
        // if enabled. Explicit periodic policies/temporary 1 Hz take precedence.
        bool fixedRate = default(false);

        // Unitless policy name, default SameAsCAM (current immediate dynamics/
        // fixedRate path). Historical compatibility names PeriodicFixed1Hz and
        // PeriodicFixed2Hz select 1 s/.5 s. No legacy 25 ms defer is introduced.
        // Dynamics -> temporary 1 Hz -> Dynamics; Fixed2Hz -> 1 Hz -> Fixed2Hz.
        string triggeringCondition = default("SameAsCAM"); // SameAsCAM, PeriodicFixed1Hz, PeriodicFixed2Hz
        // Unitless bool, default false: master for observed negotiation and
        // Intent. A fresh MCM negotiation container qualifies only when ego is
        // neither ID1 nor present ID2; absent ID2 is safe and expiry is checked.
        // Reference boundaries: .50<=CBR<.65 -> U(.5,1.5 s); CBR>=.65 -> U(1,2 s).
        // No ego-mode requirement; independent of own reduction. Also gates Intent.
        bool newGenCamRules = default(false);
        // Unitless bool, default false; independent of master/CBR/MCM awareness.
        // Ego currentMCSoperatingMode=negotiation selects temporary 1 Hz, with no
        // random hold. Leaving negotiation removes only this reduction reason.
        bool newGenCamRulesMyNegotiation = default(false);
        // Unitless bool, default false; requires newGenCamRules AND this flag,
        // and MCO off. Historical "Intent" means background congestion, not an
        // MCM-intention test. SameAsCAM with no own/observed/background reason:
        // .50<CBR<=.65 -> U(1,2 s); CBR>.65 -> U(1,5 s). Preserve inclusivity.
        // Needs CBR, not an observed negotiation or ego negotiation mode.
        bool newGenCamRulesIntent = default(false);
        // Unitless bool, default false; independent of newGenCamRules. Supersedes
        // Intent rather than drawing/combining both background holds. SameAsCAM
        // with no own/observed/background reason: CBR>.50 -> U(.1,1 s).
        // Needs CBR only; own/observed reasons still coexist with an existing hold.
        bool newGenCamRulesIntent1Hz_MCO = default(false);

        // Unitless bool, default false: optional per-generation cadence log in
        // any mode, never a MAC transmission log or new statistic. Adaptive
        // configuration/reason transitions already log without per-tick spam.
        bool logAdaptiveGeneration = default(false);

        // Degrees, default 4: standard SameAsCAM dynamics heading threshold.
        // Ignored for fixedRate/periodic generation and while temporarily reduced.
        double headingDelta = default(4.0);

        // Metres, default 4 m: standard SameAsCAM displacement threshold;
        // bypassed by fixedRate, explicit periodic policy and temporary reduction.
        double positionDelta @unit(m) = default(4.0m);

        // Metres/second, default .5 m/s: standard SameAsCAM speed-change threshold;
        // bypassed by fixedRate, explicit periodic policy and temporary reduction.
        double speedDelta @unit(mps) = default(0.5mps);

        // Unitless point count, default 23; evaluated per low-frequency container
        // in every mode and capped at 40. Existing encoder fills position deltas
        // as unavailable, not a recorded vehicle trace. Does not change cadence.
        // First CAM includes this container; subsequent inclusion is >=500 ms.
        volatile int pathHistoryLength = default(23);
}
