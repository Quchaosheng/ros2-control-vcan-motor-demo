#!/usr/bin/env python3
"""Audit a candump trace for the vcan_diffbot_demo wire contract.

The tool is intentionally offline: it never opens a CAN socket. It produces a
small, deterministic manifest that can accompany a physical-CAN capture and,
when timestamps are present, a canonical candump log for replay on vcan.
"""

import argparse
from collections import Counter, defaultdict
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import re
import sys
from typing import Iterable, Optional


COMMAND_BASE = 0x100
FEEDBACK_BASE = 0x180
ACK_BASE = 0x280
NODE_IDS = (1, 2)
COMMAND_IDS = {COMMAND_BASE + node_id for node_id in NODE_IDS}
FEEDBACK_IDS = {FEEDBACK_BASE + node_id for node_id in NODE_IDS}
ACK_IDS = {ACK_BASE + node_id for node_id in NODE_IDS}
PROTOCOL_IDS = COMMAND_IDS | FEEDBACK_IDS | ACK_IDS

HASH_FRAME = re.compile(
    r"^\s*(?:\((?P<stamp>[^)]+)\)\s+)?"
    r"(?P<interface>\S+)\s+(?P<can_id>[0-9A-Fa-f]+)#(?P<data>[0-9A-Fa-f]*)\s*$"
)
BRACKET_FRAME = re.compile(
    r"^\s*(?:\((?P<stamp>[^)]+)\)\s+)?"
    r"(?P<interface>\S+)\s+(?P<can_id>[0-9A-Fa-f]+)\s+"
    r"\[(?P<dlc>[0-8])\]\s*(?P<data>(?:[0-9A-Fa-f]{2}\s*)*)$"
)


class TraceParseError(ValueError):
    """Raised when a non-empty candump record cannot be decoded."""


@dataclass(frozen=True)
class TraceFrame:
    line_number: int
    stamp: Optional[float]
    interface: str
    can_id: int
    data: bytes


@dataclass
class CommandRecord:
    frame: TraceFrame
    node_id: int
    sequence: int
    acknowledged: bool = False
    feedback_seen: bool = False


def _parse_stamp(raw_stamp: Optional[str]) -> Optional[float]:
    if raw_stamp is None:
        return None
    try:
        stamp = float(raw_stamp)
    except ValueError as exc:
        raise TraceParseError(f"invalid timestamp {raw_stamp!r}") from exc
    if stamp < 0.0:
        raise TraceParseError(f"timestamp must be non-negative, got {raw_stamp!r}")
    return stamp


def parse_trace_line(line: str, line_number: int) -> Optional[TraceFrame]:
    """Parse one standard candump line in hash or bracket notation."""
    stripped = line.strip()
    if not stripped or stripped.startswith("#"):
        return None

    match = HASH_FRAME.match(line)
    dlc = None
    if match is None:
        match = BRACKET_FRAME.match(line)
        if match is not None:
            dlc = int(match.group("dlc"))
    if match is None:
        raise TraceParseError("unsupported candump record")

    payload = match.group("data").replace(" ", "")
    if len(payload) % 2:
        raise TraceParseError("CAN payload must contain whole bytes")
    try:
        data = bytes.fromhex(payload)
    except ValueError as exc:
        raise TraceParseError("CAN payload is not hexadecimal") from exc
    if dlc is not None and len(data) != dlc:
        raise TraceParseError(
            f"DLC {dlc} does not match {len(data)} payload byte(s)"
        )

    return TraceFrame(
        line_number=line_number,
        stamp=_parse_stamp(match.group("stamp")),
        interface=match.group("interface"),
        can_id=int(match.group("can_id"), 16),
        data=data,
    )


def parse_trace(lines: Iterable[str]):
    frames = []
    parse_errors = []
    for line_number, line in enumerate(lines, start=1):
        try:
            frame = parse_trace_line(line, line_number)
        except TraceParseError as exc:
            parse_errors.append({"line": line_number, "reason": str(exc)})
            continue
        if frame is not None:
            frames.append(frame)
    return frames, parse_errors


def _protocol_error(frame: TraceFrame, reason: str):
    return {
        "line": frame.line_number,
        "can_id": f"0x{frame.can_id:03X}",
        "reason": reason,
    }


def validate_protocol_frame(frame: TraceFrame):
    """Return a protocol error for a relevant malformed frame, if any."""
    if frame.can_id not in PROTOCOL_IDS:
        return None
    if len(frame.data) != 8:
        return _protocol_error(frame, "expected DLC 8")
    if frame.can_id in COMMAND_IDS:
        if frame.data[6:] != b"\x00\x00":
            return _protocol_error(frame, "command reserved bytes must be zero")
        return None
    if frame.can_id in ACK_IDS:
        if frame.data[1] not in (0, 1, 2):
            return _protocol_error(frame, "ACK result is outside the demo contract")
        if any(frame.data[2:]):
            return _protocol_error(frame, "ACK reserved bytes must be zero")
        return None
    if frame.data[1] & ~0x07:
        return _protocol_error(frame, "feedback status uses unsupported bits")
    return None


def _percentile(values, percentile):
    if not values:
        return None
    sorted_values = sorted(values)
    index = round((len(sorted_values) - 1) * percentile / 100.0)
    return round(sorted_values[index], 3)


def _latency_summary(values):
    return {
        "count": len(values),
        "min": _percentile(values, 0),
        "p50": _percentile(values, 50),
        "p95": _percentile(values, 95),
        "p99": _percentile(values, 99),
        "max": _percentile(values, 100),
    }


def _frame_latency_ms(start: TraceFrame, end: TraceFrame):
    if start.stamp is None or end.stamp is None:
        return None
    return (end.stamp - start.stamp) * 1000.0


def _node_id(can_id: int):
    if can_id in COMMAND_IDS:
        return can_id - COMMAND_BASE
    if can_id in FEEDBACK_IDS:
        return can_id - FEEDBACK_BASE
    if can_id in ACK_IDS:
        return can_id - ACK_BASE
    return None


def analyze_frames(frames, parse_errors=None, source_sha256=None, source_name=None):
    """Build a JSON-serializable manifest without exposing raw frame payloads."""
    parse_errors = parse_errors or []
    protocol_errors = []
    commands_by_key = defaultdict(list)
    commands = []
    ack_latency_ms = []
    feedback_latency_ms = []
    timestamp_order_errors = 0
    unmatched_acks = 0
    duplicate_acks = 0
    unmatched_feedback = 0
    ack_results = Counter()

    for frame in frames:
        protocol_error = validate_protocol_frame(frame)
        if protocol_error is not None:
            protocol_errors.append(protocol_error)
            continue

        node_id = _node_id(frame.can_id)
        if frame.can_id in COMMAND_IDS:
            command = CommandRecord(frame, node_id, frame.data[0])
            commands.append(command)
            commands_by_key[(node_id, command.sequence)].append(command)
            continue

        if frame.can_id in ACK_IDS:
            key = (node_id, frame.data[0])
            candidates = commands_by_key.get(key, [])
            command = next(
                (candidate for candidate in candidates if not candidate.acknowledged),
                None,
            )
            if command is None:
                if candidates:
                    duplicate_acks += 1
                else:
                    unmatched_acks += 1
                continue
            command.acknowledged = True
            ack_results[str(frame.data[1])] += 1
            latency = _frame_latency_ms(command.frame, frame)
            if latency is not None:
                if latency < 0:
                    timestamp_order_errors += 1
                else:
                    ack_latency_ms.append(latency)
            continue

        if frame.can_id in FEEDBACK_IDS:
            candidates = commands_by_key.get((node_id, frame.data[0]), [])
            if not candidates:
                unmatched_feedback += 1
                continue
            command = candidates[-1]
            if command.feedback_seen:
                continue
            command.feedback_seen = True
            latency = _frame_latency_ms(command.frame, frame)
            if latency is not None:
                if latency < 0:
                    timestamp_order_errors += 1
                else:
                    feedback_latency_ms.append(latency)

    frame_counts = Counter(f"0x{frame.can_id:03X}" for frame in frames)
    interface_counts = Counter(frame.interface for frame in frames)
    command_count = len(commands)
    return {
        "schema_version": 1,
        "source": {
            "name": source_name,
            "sha256": source_sha256,
        },
        "frames": {
            "total": len(frames),
            "by_can_id": dict(sorted(frame_counts.items())),
            "by_interface": dict(sorted(interface_counts.items())),
        },
        "protocol": {
            "commands": command_count,
            "acknowledged_commands": sum(command.acknowledged for command in commands),
            "commands_with_feedback": sum(
                command.feedback_seen for command in commands
            ),
            "ack_results": dict(sorted(ack_results.items())),
            "unmatched_acks": unmatched_acks,
            "duplicate_acks": duplicate_acks,
            "unmatched_feedback": unmatched_feedback,
            "parse_errors": parse_errors,
            "protocol_errors": protocol_errors,
            "timestamp_order_errors": timestamp_order_errors,
        },
        "observed_latency_ms": {
            "command_to_ack": _latency_summary(ack_latency_ms),
            "command_to_first_feedback": _latency_summary(feedback_latency_ms),
            "meaning": (
                "Observed packet timestamp deltas, not controller execution or "
                "end-to-end motor latency."
            ),
        },
    }


def canonical_candump_line(frame: TraceFrame):
    if frame.stamp is None:
        raise ValueError("replay output requires timestamps on every frame")
    return (
        f"({frame.stamp:.6f}) {frame.interface} "
        f"{frame.can_id:03X}#{frame.data.hex().upper()}"
    )


def write_json(path: Path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def write_replay(path: Path, frames):
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [canonical_candump_line(frame) for frame in frames]
    path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path, help="candump trace file")
    parser.add_argument("--output", required=True, type=Path, help="JSON manifest path")
    parser.add_argument(
        "--replay-output",
        type=Path,
        help="optional canonical candump log; replay it only onto vcan",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="return non-zero when parsing or protocol validation fails",
    )
    args = parser.parse_args(argv)

    try:
        source_bytes = args.input.read_bytes()
    except OSError as exc:
        parser.error(str(exc))
    lines = source_bytes.decode("utf-8", errors="replace").splitlines()
    frames, parse_errors = parse_trace(lines)
    manifest = analyze_frames(
        frames,
        parse_errors=parse_errors,
        source_sha256=hashlib.sha256(source_bytes).hexdigest(),
        source_name=args.input.name,
    )
    write_json(args.output, manifest)

    if args.replay_output is not None:
        try:
            write_replay(args.replay_output, frames)
        except ValueError as exc:
            parser.error(str(exc))

    if args.strict and (
        manifest["protocol"]["parse_errors"]
        or manifest["protocol"]["protocol_errors"]
    ):
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
