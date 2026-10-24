# Task: a persistent data mount for datasets, weights and batch results

**Roadmap:** infrastructure, no roadmap task of its own (it unblocks P3.2 and every batch).
**Branch:** `feat/data-mount`. **Its own PR.**
**Do it first** (right after the P2.4 batch, which is done): it needs the container recreated.

## Why

Everything big has been written to the container's `/tmp`: the UAS dataset
(`/tmp/uas_ds3`), the training runs and weights (`/tmp/uas_train`), every batch
result (`/tmp/p24final`, `/tmp/p35batch`, ...) and the scratch workspaces. When the
container was recreated, the P3.2 dataset and weights were lost and have to be
regenerated. **Rule from now on: big artefacts never go in `/tmp`.**

## What to do

1. In `docker-compose.yml`, add an **absolute** host mount:
   `${HOME}/skyinterceptor-data:/workspace/data:rw`.
   It must not be relative to the compose directory: the compose file is often
   started from a worktree (see the memory note "container src mount follows the
   compose dir"), and a relative path would scatter the data across worktrees.
   Create the host directory if it is missing (document `mkdir -p ~/skyinterceptor-data`).
2. Make `/workspace/data/...` the **default output** of every script that writes
   datasets, weights or batch results, instead of `/tmp`:
   - `src/tools/run_follow_batch.py` (`--output-dir`, now `/tmp/follow_batch`) →
     `/workspace/data/batches/follow/<timestamp>`;
   - `src/tools/run_intercept_batch.py` (now `/tmp/intercept_batch`) →
     `/workspace/data/batches/intercept/<timestamp>`;
   - the P3.2 scripts in PR #32 (`scripts/uas_dataset/`: dataset, training,
     export) → `/workspace/data/uas/...` (coordinate with that PR: rebase it on
     this one, or change its defaults when it lands);
   - `scripts/check.sh` scratch workspaces may stay in `/tmp` (they are build
     scratch, deleted after the run), but say so in a comment.
3. Document it in `DOCKER.md` and `HANDOFF.md`: where the data lives on the host,
   what goes there, and the rule.
4. **Recreate the container from the main checkout** (`/home/diego/code/SkyInterceptor`),
   so `/workspace/interceptor_ws/src` is `main` and not an old worktree:
   `cd /home/diego/code/SkyInterceptor && docker compose up -d --force-recreate`.
   Check with `docker exec interceptor findmnt -T /workspace/interceptor_ws/src -o TARGET,SOURCE`
   and `findmnt -T /workspace/data`.

## Done when

- the mount exists and survives a container recreate (write a file, recreate, read it);
- both batch runners default to `/workspace/data/...` and say so in `--help`;
- `DOCKER.md` and `HANDOFF.md` document it;
- the container's `src` is the main checkout.

## Pitfalls

- If the machine moves to native Ubuntu (see `HANDOFF.md`), do this task **on the
  new machine**: the mount path and the container are recreated there anyway.
- `${HOME}` is the host user's home: compose expands it from the shell that runs
  `docker compose`, so run it as the user, not with `sudo` (or the data lands in
  `/root`).
