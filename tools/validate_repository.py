#!/usr/bin/env python3
"""Validate repository-local Keil references and documented PB-03 entry points."""

from __future__ import annotations

import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SDK_ROOT = ROOT / "Ai_PB-03F_OPEN-SOURCE"
DRIVE_PATH = re.compile(r"^[A-Za-z]:[\\/]")


def case_insensitive_existing(path: Path) -> bool:
    """Resolve repository paths on both Windows and case-sensitive hosts."""
    try:
        relative = path.resolve().relative_to(ROOT.resolve())
    except ValueError:
        return path.exists()

    current = ROOT.resolve()
    for part in relative.parts:
        direct = current / part
        if direct.exists():
            current = direct
            continue
        if not current.is_dir():
            return False
        match = next(
            (item for item in current.iterdir() if item.name.casefold() == part.casefold()),
            None,
        )
        if match is None:
            return False
        current = match
    return current.exists()


def local_path(project: Path, value: str) -> Path:
    normalized = value.strip().replace("\\", "/")
    return project.parent.joinpath(*normalized.split("/"))


def validate_projects() -> tuple[int, list[str], list[str]]:
    projects = sorted(SDK_ROOT.rglob("*.uvprojx"))
    errors: list[str] = []
    warnings: list[str] = []

    for project in projects:
        display = project.relative_to(ROOT)
        try:
            tree = ET.parse(project)
        except ET.ParseError as exc:
            errors.append(f"{display}: invalid XML: {exc}")
            continue

        for node_name in ("FilePath", "ScatterFile"):
            for node in tree.findall(f".//{node_name}"):
                value = (node.text or "").strip()
                if not value or "$" in value:
                    continue
                if DRIVE_PATH.match(value):
                    errors.append(f"{display}: absolute {node_name}: {value}")
                elif not case_insensitive_existing(local_path(project, value)):
                    errors.append(f"{display}: missing {node_name}: {value}")

        for node in tree.findall(".//IncludePath"):
            for value in (item.strip() for item in (node.text or "").split(";")):
                if not value or "$" in value:
                    continue
                if DRIVE_PATH.match(value):
                    errors.append(f"{display}: absolute IncludePath: {value}")
                elif not case_insensitive_existing(local_path(project, value)):
                    warnings.append(f"{display}: stale IncludePath: {value}")

    if len(projects) != 43:
        errors.append(f"expected 43 Keil projects, found {len(projects)}")
    return len(projects), errors, warnings


def validate_entry_points() -> list[str]:
    base = SDK_ROOT / "Ai-Demo/BASE/bleUart_AT_ADC/Source"
    checks = {
        base / "main.c": r"\bmain\s*\(",
        base / "bleuart_Main.c": r"\bapp_main\s*\(",
        base / "OSAL_bleuart.c": r"\bosalInitTasks\s*\(",
        base / "bleuart.c": r"\bbleuart_ProcessEvent\s*\(",
    }
    errors: list[str] = []
    for path, pattern in checks.items():
        name = path.relative_to(ROOT)
        if not path.is_file():
            errors.append(f"missing entry source: {name}")
        elif not re.search(pattern, path.read_text(encoding="utf-8", errors="replace")):
            errors.append(f"missing documented symbol in {name}: {pattern}")
    return errors


def validate_markdown_pairs() -> tuple[int, list[str]]:
    markdown = [ROOT / "README.md", ROOT / "README.zh.md"]
    docs = ROOT / "docs"
    markdown.extend(sorted(docs.glob("*.md")))
    errors: list[str] = []

    for path in markdown:
        relative = path.relative_to(ROOT)
        if path.name.endswith(".zh.md"):
            peer = path.with_name(path.name.removesuffix(".zh.md") + ".md")
            expected = "[![English]("
        else:
            peer = path.with_name(path.stem + ".zh.md")
            expected = "[![中文]("
        if not peer.is_file():
            errors.append(f"{relative}: missing language peer {peer.relative_to(ROOT)}")
        first_line = path.read_text(encoding="utf-8", errors="replace").splitlines()[:1]
        if not first_line or not first_line[0].startswith(expected):
            errors.append(f"{relative}: missing first-line language badge")
    return len(markdown), errors


def main() -> int:
    project_count, project_errors, project_warnings = validate_projects()
    markdown_count, markdown_errors = validate_markdown_pairs()
    errors = project_errors + validate_entry_points() + markdown_errors

    print(f"Keil projects checked: {project_count}")
    print(f"Markdown files checked: {markdown_count}")
    print("Representative entry chain: main -> app_main -> osalInitTasks -> bleuart_ProcessEvent")
    if project_warnings:
        print(f"Stale include search paths reported: {len(project_warnings)}")
        for warning in project_warnings:
            print(f"WARNING: {warning}")
    if errors:
        print(f"Validation failed with {len(errors)} error(s):", file=sys.stderr)
        for error in errors:
            print(f"- {error}", file=sys.stderr)
        return 1
    print("Repository validation: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
