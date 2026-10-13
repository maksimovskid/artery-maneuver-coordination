"""Presentation-only statuses derived from captured MCM simulation logs."""

import re


def negotiations(data, r):
    """Extract first message sends and execution events from an existing capture."""
    scenario = data.scenario
    root = data.capture_root or r.scenario_output_dir(scenario, 'coordinated').parent
    labels = {k: v[0] for k, v in r.role_map(scenario, data.roles).items()}
    events = data.events['coordinated']
    # Some diagnostic simTime fields cache the previous middleware tick.
    # Use the enclosing OMNeT++ event header for the actual log event time.
    wanted = {e['line'] for e in events}
    event_times = {}
    current = None
    with (root / 'coordinated/simulation.log').open() as stream:
        for number, line in enumerate(stream, 1):
            match = re.match(r'\*\* Event #\d+\s+t=([\d.]+)', line)
            if match:
                current = float(match[1])
            if number in wanted and current is not None:
                event_times[number] = current
    for event in events:
        event['_actual_time'] = event_times.get(event['line'], event.get('simTime'))
    requests = []
    for event in events:
        if (event.get('tag') == 'MCM-NEGOTIATION' and event.get('action') == 'SEND'
                and event.get('msg') == 'Request'
                and event['requestId'] not in [x['request_id'] for x in requests]):
            requests.append({'request_id': event['requestId'], 'rv_id': event['localVehicleId']})
    result = []
    for request in requests[:1 if scenario == "merging" else 2]:
        selected = [e for e in events if e.get('requestId') == request['request_id']]
        first = {}
        for event in selected:
            if (event.get('tag') == 'MCM-NEGOTIATION' and event.get('action') == 'SEND'
                    and event.get('msg') in ('Request', 'Offer', 'Confirm', 'Accept')):
                first.setdefault((event['localVehicleId'], event['msg']), event['_actual_time'])
        rv = request['rv_id']
        cvs = sorted({vid for vid, msg in first if msg == 'Offer'}, key=lambda v: labels[v])
        def find_event(name):
            return next(e['_actual_time'] for e in selected if e.get('event') == name)
        if scenario == 'lane-change':
            execution = find_event('lane-change-execution-start')
            completion = find_event('lane-change-execution-complete')
        else:
            execution = next(e['_actual_time'] for e in events
                             if e.get('tag') == 'MCM-GAP-DIAG'
                             and e.get('phase') == 'execution-start' and e.get('vehicleId') == rv)
            completion = next(t for label, _, t in r.presentation_merge_states(data) if label == labels[rv])
        result.append({**request, 'rv': labels[rv], 'cvs': {labels[v]: v for v in cvs},
                       'request': first[rv, 'Request'], 'confirm': first[rv, 'Confirm'],
                       'offers': {labels[v]: first[v, 'Offer'] for v in cvs},
                       'accepts': {labels[v]: first[v, 'Accept'] for v in cvs},
                       'execution': execution, 'completion': completion,
                       'completion_source': 'MCM-LC-EXEC event' if scenario == 'lane-change' else 'FCD whole-body highway establishment'})
    return result


def participant_status(data, time, r):
    scenario = data.scenario
    if not hasattr(data, "participant_negotiations"):
        data.participant_negotiations = negotiations(data, r)
    records = data.participant_negotiations
    if scenario == "merging":
        first = records[0]
        if not first["request"] <= time < first["completion"]:
            return None
        if time >= first["execution"]:
            return ["RV1 merging"]
        records = records[:1]
    active = next((n for n in reversed(records)
                   if n['request'] <= time < n['completion']), None)
    if active is None and scenario == 'lane-change':
        done = [n for n in records if n['completion'] <= time]
        return [f"{done[-1]['rv']}: Lane change done"] if done else None
    if active is None:
        return None
    phase = ('Lane change' if scenario == 'lane-change' else 'Gap cooperation') if time >= active['execution'] else ('Confirm' if time >= active['confirm'] else 'Request')
    accepted = [cv for cv, t in active['accepts'].items() if t <= time]
    offered = [cv for cv, t in active['offers'].items() if t <= time]
    participants, message = (accepted, 'Accepts') if accepted else (offered, 'Offers')
    # Never claim that a CV has replied before its own first logged send.
    second = ' + '.join(participants) + ': ' + message if participants else 'CVs: awaiting reply'
    return [f"{active['rv']}: {phase}", second]

