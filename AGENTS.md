# AGENTS.md

Instructions for any coding agent working in this repository (Claude Code, Codex, Cursor, Copilot, Gemini CLI, ...). Humans: this applies to you too.

## Where things stand

[`ROADMAP.md`](ROADMAP.md) is the single source of truth for what is done and what is still to do. Read it before you start.

```bash
python3 scripts/roadmap.py ready     # tasks you can start now
python3 scripts/roadmap.py summary   # status counts
python3 scripts/roadmap.py check     # validate the file (CI runs this)
```

Pick a task from `ready`, or the one you were given, and read its section in `ROADMAP.md` and in `IMPLEMENTATION_PLAN.md` (§5). Task prompts are in `docs/AGENT_PROMPTS.md`.

## When you finish (or stop)

Edit the task's section in `ROADMAP.md` **in the same PR as your work**: `Status`, `PR`, `Branch`, and `Notes` (what you verified, what is still open, what blocks you). The status describes the state after your PR merges. The rules and status values are at the top of `ROADMAP.md`. Don't mark a task `done` unless its scope is complete and `make build-ws && make test` passes.

## Project rules

Build, test and code-style rules are in [`CLAUDE.md`](CLAUDE.md) (the content is not specific to Claude). Safety distance is the top-level rule: never fix a violation by lowering a threshold.
