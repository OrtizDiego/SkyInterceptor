#!/usr/bin/env python3
"""Read and validate ROADMAP.md, the source of truth for plan progress.

Usage:
  roadmap.py [check]   validate the file (exit 1 on errors); this is what CI runs
  roadmap.py ready     tasks that are todo and whose dependencies are all done
  roadmap.py summary   counts per status and per phase
  roadmap.py json      all tasks as JSON (for tools that sync other views)

Only the standard library is used. The file layout is described at the top of
ROADMAP.md: one "### CODE - Title" section per task under a "## Phase N - Name"
heading, with fields written as "- **Name:** value" and continuation lines
indented by two spaces.
"""
import json
import re
import sys
from pathlib import Path

ROADMAP = Path(__file__).resolve().parent.parent / 'ROADMAP.md'
STATUSES = ('todo', 'in_progress', 'review', 'done', 'blocked')
FIELDS = ('Status', 'Depends on', 'PR', 'Branch', 'Scope', 'Notes')
NONE = '—'  # em dash: "no value"

PHASE_RE = re.compile(r'^## Phase (\w+)\s*[–-]\s*(.+?)\s*$')
TASK_RE = re.compile(r'^### ([A-Za-z0-9_.~@+-]+)\s*[–-]\s*(.+?)\s*$')
FIELD_RE = re.compile(r'^- \*\*(.+?):\*\* ?(.*)$')


def parse(text):
    """Return (tasks, errors). Each task is a dict with its fields and phase."""
    tasks, errors = [], []
    phase, task, field = None, None, None
    for number, line in enumerate(text.splitlines(), 1):
        m = PHASE_RE.match(line)
        if m:
            phase, task, field = m.group(1), None, None
            continue
        m = TASK_RE.match(line)
        if m:
            task = {'code': m.group(1), 'title': m.group(2), 'phase': phase,
                    'line': number}
            if phase is None:
                errors.append(f'line {number}: {task["code"]} is outside a phase')
            tasks.append(task)
            field = None
            continue
        if task is None:
            continue
        m = FIELD_RE.match(line)
        if m:
            field = m.group(1)
            if field not in FIELDS:
                errors.append(f'line {number}: unknown field "{field}" in {task["code"]}')
            task[field] = m.group(2).strip()
        elif field and line.startswith('  ') and line.strip():
            task[field] += ' ' + line.strip()
        elif line.strip():
            field = None
    return tasks, errors


def deps_of(task):
    value = task.get('Depends on', NONE)
    return [] if value in (NONE, '') else [d.strip() for d in value.split(',')]


def validate(tasks):
    errors, warnings = [], []
    codes = [t['code'] for t in tasks]
    for code in {c for c in codes if codes.count(c) > 1}:
        errors.append(f'{code}: defined more than once')
    known = set(codes)
    for t in tasks:
        code = t['code']
        for name in FIELDS:
            if name not in t:
                errors.append(f'{code}: missing field "{name}"')
        status = t.get('Status')
        if status is not None and status not in STATUSES:
            errors.append(f'{code}: Status "{status}" is not one of {", ".join(STATUSES)}')
        pr = t.get('PR', NONE)
        if pr != NONE and not pr.isdigit():
            errors.append(f'{code}: PR must be a number or {NONE}, got "{pr}"')
        if status == 'done' and pr == NONE:
            errors.append(f'{code}: a done task needs a PR number')
        for dep in deps_of(t):
            if dep not in known:
                errors.append(f'{code}: depends on unknown task "{dep}"')
            elif dep == code:
                errors.append(f'{code}: depends on itself')
        if status == 'blocked' and t.get('Notes', NONE) == NONE:
            errors.append(f'{code}: a blocked task needs Notes saying what blocks it')
    if not errors:
        by_code = {t['code']: t for t in tasks}
        for t in tasks:
            if t.get('Status') == 'done':
                for dep in deps_of(t):
                    if by_code[dep].get('Status') != 'done':
                        warnings.append(f'{t["code"]} is done but depends on {dep} '
                                        f'({by_code[dep].get("Status")})')
        errors.extend(find_cycles(by_code))
    return errors, warnings


def find_cycles(by_code):
    errors, state = [], {}

    def visit(code, path):
        if state.get(code) == 'done':
            return
        if state.get(code) == 'open':
            errors.append('dependency cycle: ' + ' -> '.join(path + [code]))
            return
        state[code] = 'open'
        for dep in deps_of(by_code[code]):
            visit(dep, path + [code])
        state[code] = 'done'

    for code in by_code:
        visit(code, [])
    return errors


def ready(tasks):
    by_code = {t['code']: t for t in tasks}
    return [t for t in tasks if t.get('Status') == 'todo'
            and all(by_code[d].get('Status') == 'done' for d in deps_of(t))]


def main(argv):
    command = argv[1] if len(argv) > 1 else 'check'
    if not ROADMAP.exists():
        print(f'{ROADMAP} not found', file=sys.stderr)
        return 1
    tasks, errors = parse(ROADMAP.read_text(encoding='utf-8'))
    more_errors, warnings = validate(tasks)
    errors += more_errors
    if not tasks:
        errors.append('no tasks found')

    if command == 'check':
        for w in warnings:
            print(f'warning: {w}')
        for e in errors:
            print(f'error: {e}')
        print(f'{len(tasks)} tasks, {len(errors)} errors, {len(warnings)} warnings')
        return 1 if errors else 0
    if errors:
        for e in errors:
            print(f'error: {e}', file=sys.stderr)
        return 1
    if command == 'ready':
        for t in ready(tasks):
            print(f'{t["code"]}\t{t["title"]}')
        return 0
    if command == 'summary':
        for status in STATUSES:
            names = [t['code'] for t in tasks if t.get('Status') == status]
            print(f'{status:12} {len(names):2}  {" ".join(names)}')
        return 0
    if command == 'json':
        print(json.dumps(tasks, indent=2, ensure_ascii=False))
        return 0
    print(__doc__)
    return 2


if __name__ == '__main__':
    sys.exit(main(sys.argv))
