import importlib.util
import json
from pathlib import Path
import sys


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'can_trace_audit.py'
SPEC = importlib.util.spec_from_file_location('can_trace_audit', SCRIPT)
can_trace_audit = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = can_trace_audit
SPEC.loader.exec_module(can_trace_audit)


def test_analyze_trace_correlates_command_ack_and_feedback():
    lines = [
        '(1.000000) vcan0 101#0101E803C8000000',
        '(1.004000) vcan0 281#0100000000000000',
        '(1.006000) vcan0 181#0101E8032A000000',
    ]

    frames, parse_errors = can_trace_audit.parse_trace(lines)
    manifest = can_trace_audit.analyze_frames(frames, parse_errors=parse_errors)

    assert manifest['protocol']['commands'] == 1
    assert manifest['protocol']['acknowledged_commands'] == 1
    assert manifest['protocol']['commands_with_feedback'] == 1
    assert manifest['protocol']['protocol_errors'] == []
    assert manifest['observed_latency_ms']['command_to_ack']['p50'] == 4.0
    assert manifest['observed_latency_ms']['command_to_first_feedback']['p50'] == 6.0


def test_parser_accepts_bracket_notation_and_reports_invalid_contract_data():
    frames, parse_errors = can_trace_audit.parse_trace(
        [
            '(2.000000) vcan0 101 [8] 02 01 00 00 C8 00 AA BB',
            '(2.010000) vcan0 281 [8] 02 00 00 00 00 00 00 00',
        ]
    )
    manifest = can_trace_audit.analyze_frames(frames, parse_errors=parse_errors)

    assert parse_errors == []
    assert manifest['protocol']['protocol_errors'] == [
        {
            'line': 1,
            'can_id': '0x101',
            'reason': 'command reserved bytes must be zero',
        }
    ]
    assert manifest['protocol']['unmatched_acks'] == 1


def test_cli_writes_manifest_and_canonical_replay(tmp_path):
    trace = tmp_path / 'capture.log'
    manifest_path = tmp_path / 'manifest.json'
    replay_path = tmp_path / 'replay.log'
    trace.write_text(
        '(1.000000) can0 101#01000000C8000000\n'
        '(1.002000) can0 281#0100000000000000\n',
        encoding='utf-8',
    )

    return_code = can_trace_audit.main(
        [
            '--input',
            str(trace),
            '--output',
            str(manifest_path),
            '--replay-output',
            str(replay_path),
            '--strict',
        ]
    )

    assert return_code == 0
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    assert manifest['source']['name'] == 'capture.log'
    assert len(manifest['source']['sha256']) == 64
    assert replay_path.read_text(encoding='utf-8').splitlines() == [
        '(1.000000) can0 101#01000000C8000000',
        '(1.002000) can0 281#0100000000000000',
    ]
