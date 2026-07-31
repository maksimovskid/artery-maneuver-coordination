"""Stable log parsing helpers for maneuver-coordination regression tests."""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


TAG_RE = re.compile(r"\[(MCM-[^\]]+)\]")
KEY_VALUE_RE = re.compile(r'([A-Za-z0-9_]+)=("[^"]*"|\S+)')


@dataclass(frozen=True)
class LogEvent:
    tag: str
    fields: dict[str, str]
    line: str
    line_number: int


class ParsedLog:
    def __init__(self, text: str) -> None:
        self.text = text
        self.events = parse_events(text)

    @classmethod
    def from_path(cls, path: Path) -> "ParsedLog":
        return cls(path.read_text(encoding="utf-8", errors="replace"))

    def find(self, tag: str | None = None, **fields: str) -> list[LogEvent]:
        matches: list[LogEvent] = []
        for event in self.events:
            if tag is not None and event.tag != tag:
                continue
            if all(event.fields.get(key) == value for key, value in fields.items()):
                matches.append(event)
        return matches

    def any(self, tag: str | None = None, **fields: str) -> bool:
        return bool(self.find(tag, **fields))

    def count(self, tag: str | None = None, **fields: str) -> int:
        return len(self.find(tag, **fields))

    def require(self, tag: str | None = None, **fields: str) -> LogEvent:
        matches = self.find(tag, **fields)
        if not matches:
            expected = describe_expected(tag, fields)
            raise AssertionError(f"missing expected log event: {expected}")
        return matches[0]

    def forbid(self, tag: str | None = None, **fields: str) -> None:
        matches = self.find(tag, **fields)
        if matches:
            expected = describe_expected(tag, fields)
            first = matches[0]
            raise AssertionError(
                f"forbidden log event present: {expected} at line "
                f"{first.line_number}: {first.line}"
            )


def parse_events(text: str) -> list[LogEvent]:
    events: list[LogEvent] = []
    for line_number, line in enumerate(text.splitlines(), 1):
        match = TAG_RE.search(line)
        if not match:
            continue
        fields = {
            key: value.strip('"')
            for key, value in KEY_VALUE_RE.findall(line)
        }
        events.append(LogEvent(match.group(1), fields, line, line_number))
    return events


def describe_expected(tag: str | None, fields: dict[str, str]) -> str:
    parts: list[str] = []
    if tag is not None:
        parts.append(tag)
    parts.extend(f"{key}={value}" for key, value in fields.items())
    return " ".join(parts) if parts else "any MCM event"


def request_ids(events: Iterable[LogEvent]) -> list[int]:
    ids: list[int] = []
    for event in events:
        value = event.fields.get("requestId")
        if value is None:
            continue
        try:
            ids.append(int(value))
        except ValueError:
            continue
    return ids
