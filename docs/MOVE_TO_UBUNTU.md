# Moving SkyInterceptor to a native Ubuntu machine — step by step

This guide takes you from a fresh Ubuntu install to a working container and a running
agent. Do the steps **in order**; each one ends with a **check**. If a check fails,
stop and fix it before going on (the "If it fails" notes say how).

Commands go in a terminal on the **new machine**, unless the step says otherwise.
Copy them one block at a time.

> Written 2026-10-24. Assumes Ubuntu 22.04 or 24.04 desktop, an NVIDIA GPU, and that
> your Linux username is `diego` (step 5 says what to do if it is not).

---

## Step 0 — On the old machine: already done

This was done on 2026-10-24 by the agent; you only need to **copy one file**.

- All code is on GitHub. The P2.4 branch was pushed and merged (PR #31).
- `~/skyinterceptor-transfer.tgz` (about 100 MB) on the old machine contains:
  - `skyinterceptor-data/batches/` — every batch result of the last session (README inside);
  - `skyinterceptor-data/local-git/` — a git bundle of every local branch (some had
    commits that were never pushed) and two untracked notes from the main checkout;
  - `.claude/projects/-home-diego-code-SkyInterceptor/memory/` — the agent's memory.
  - `skyinterceptor-data/MOVE_TO_UBUNTU.md` — this guide.

**Copy `~/skyinterceptor-transfer.tgz` to the new machine**, into your home directory,
with a USB stick, or from the new machine with:

```bash
scp diego@OLD_MACHINE:~/skyinterceptor-transfer.tgz ~/
```

On WSL the file is also visible from Windows at
`\\wsl.localhost\Ubuntu\home\diego\skyinterceptor-transfer.tgz`.

**Check:**

```bash
ls -lh ~/skyinterceptor-transfer.tgz
```

You should see a file of about 100 MB.

---

## Step 1 — The NVIDIA driver

```bash
sudo apt update
```

```bash
sudo ubuntu-drivers install
```

```bash
sudo reboot
```

After the reboot:

```bash
nvidia-smi
```

**Check:** a table with your GPU's name, a driver version, and `CUDA Version: 12.4` or
higher (the container's PyTorch is the CUDA 12.4 build).

**If it fails:**

- `nvidia-smi: command not found` or `couldn't communicate with the NVIDIA driver`:
  Secure Boot may be blocking the driver. Check with `sudo mokutil --sb-state`. If it
  says enabled, either disable Secure Boot in the BIOS/UEFI, or re-run
  `sudo ubuntu-drivers install`, set the password it asks for, and choose
  **Enroll MOK** on the blue screen at the next boot.
- **A laptop with two GPUs** (Intel/AMD + NVIDIA): make the NVIDIA one active, then
  reboot again:

  ```bash
  sudo prime-select nvidia
  ```

---

## Step 2 — Docker and the NVIDIA container toolkit

```bash
curl -fsSL https://get.docker.com | sudo sh
```

```bash
sudo usermod -aG docker $USER
```

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
```

```bash
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

```bash
sudo apt update && sudo apt install -y nvidia-container-toolkit
```

```bash
sudo nvidia-ctk runtime configure --runtime=docker
```

```bash
sudo systemctl restart docker
```

**Log out and log back in** (or reboot), so your user is in the `docker` group.

**Check:**

```bash
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

The same GPU table as in step 1, printed from inside a container.

**If it fails:**

- `permission denied ... docker.sock`: you did not log out and back in. Do it.
- `could not select device driver "" with capabilities: [[gpu]]`: the container
  toolkit is not configured. Re-run the `nvidia-ctk runtime configure` and
  `systemctl restart docker` commands above.

You do **not** need a `docker-compose` wrapper: the Makefile uses `docker compose`
by itself when it is installed (get.docker.com installs it).

```bash
docker compose version
```

**Check:** it prints a version (v2.x).

---

## Step 3 — Tools: git, make, X access, GitHub CLI, Claude Code

```bash
sudo apt install -y git make x11-xserver-utils gh
```

```bash
gh auth login
```

Choose: GitHub.com → HTTPS → Yes (authenticate git) → Login with a web browser.

```bash
curl -fsSL https://claude.ai/install.sh | bash
```

Open a **new terminal** afterwards, so `claude` is on the PATH.

**Your git identity and SSH folder.** The container mounts `~/.ssh` and `~/.gitconfig`
from your home; if they do not exist, Docker creates them as root-owned folders and
git breaks inside the container. Create them now, with your own name and the email you use on GitHub:

```bash
git config --global user.name "Your Name"
```

```bash
git config --global user.email "you@example.com"
```

```bash
mkdir -p ~/.ssh && chmod 700 ~/.ssh
```

**Check:**

```bash
gh auth status && git config --global user.name && ls -ld ~/.ssh ~/.gitconfig && claude --version
```

All four print something, and `~/.ssh` and `~/.gitconfig` belong to you, not root.

---

## Step 4 — Clone the repository

Use **exactly** this path: the agent's memory is filed under the project's path.

```bash
mkdir -p ~/code && git clone https://github.com/OrtizDiego/SkyInterceptor.git ~/code/SkyInterceptor
```

**Check:**

```bash
ls ~/code/SkyInterceptor/docker-compose.native.yml
```

The file exists. (It is in `main` since the PR that prepared this move. If it is
missing, run `git -C ~/code/SkyInterceptor pull`.)

---

## Step 5 — Restore the data and the agent's memory

```bash
tar xzf ~/skyinterceptor-transfer.tgz -C ~
```

**Check:**

```bash
ls ~/skyinterceptor-data/batches ~/.claude/projects/-home-diego-code-SkyInterceptor/memory | head -20
```

You see the batch directories (`p24wb`, `p35batch`, ...) and the memory notes
(`MEMORY.md`, ...).

**If your username is not `diego`:** the memory folder name contains the project path.
Move it to the name for your path (replace `NEWUSER`):

```bash
mv ~/.claude/projects/-home-diego-code-SkyInterceptor ~/.claude/projects/-home-NEWUSER-code-SkyInterceptor
```

**Optional — the branches that were only on the old machine.** Ten local branches had
commits that were never pushed (most are probably already in `main` through squash
merges). They are kept in a bundle; you only need it if a later task asks for one of
them:

```bash
git -C ~/code/SkyInterceptor bundle list-heads ~/skyinterceptor-data/local-git/all-local-branches.bundle
```

To restore one (replace `BRANCH`):

```bash
git -C ~/code/SkyInterceptor fetch ~/skyinterceptor-data/local-git/all-local-branches.bundle BRANCH:BRANCH
```

---

## Step 6 — Build and start the container

Always run `make` from the **main checkout**, so the container mounts the right code:

```bash
cd ~/code/SkyInterceptor
```

```bash
make platform
```

**Check:** it prints `platform: native`, a `docker compose ... -f docker-compose.native.yml`
line, and the data directory. If it says `wsl`, force it with `PLATFORM=native` on every
`make` command (e.g. `make up PLATFORM=native`).

Build the image. **The first build takes 30–60 minutes**; leave it running:

```bash
make build
```

**Check:** the last lines say the image `interceptor-drone:latest` was built, with no
`ERROR`. The full log is in `logs/docker-build.log`.

```bash
make up
```

```bash
make build-ws
```

**Check:** `Summary: 2 packages finished` and no `Failed`.

---

## Step 7 — Verify the setup

```bash
make doctor
```

**Check, line by line:**

| Line | Good |
|---|---|
| host GPU | your GPU's name and driver version |
| `compose:` | `docker compose ok` |
| `data dir:` | `.../skyinterceptor-data ok` |
| `GPU in the container:` | your GPU's name |
| `OpenGL renderer` | names your **NVIDIA** GPU — not `llvmpipe`, not `D3D12` |
| `torch CUDA:` | `True` and your GPU's name |
| `src mounted from:` | `.../code/SkyInterceptor/interceptor_ws/src` (the main checkout, not a worktree) |
| `data mounted from:` | `.../skyinterceptor-data` |

**If it fails:**

- **`OpenGL renderer` says `llvmpipe`:** on a laptop, `sudo prime-select nvidia` and
  reboot. Make sure you are in the desktop session (not over SSH), then `make down`,
  `make up`, and `make doctor` again.
- **`glxinfo found no renderer`:** the container cannot reach your display. Run
  `xhost +local:docker` on the host and `echo $DISPLAY` (should print `:0` or `:1`),
  then `make down` and `make up`.
- **`torch CUDA: False`:** the driver is older than CUDA 12.4 (see step 1), or the
  image was built before the driver was installed — `make build` again.

Then the full test suite (the same checks as CI: a `-Werror` build and all tests,
about 5 minutes):

```bash
make check
```

**Check:** it ends with `All checks passed.` (on a failure it says how many checks failed, and which, above).

And a visual smoke test — the Gazebo window should open with the drone and the park.
**The first start downloads Gazebo's model database: give it 2–3 minutes.**

```bash
make sim
```

Close it with `Ctrl+C` in the terminal.

---

## Step 8 — Start the agent

```bash
cd ~/code/SkyInterceptor && claude
```

Paste this as the first message:

```text
We are now on native Ubuntu (not WSL). Read HANDOFF.md first.
The container was started with `make up`, which uses docker-compose.yml +
docker-compose.native.yml, and mounts persistent data at /workspace/data
(host: ~/skyinterceptor-data). The earlier batch results are in
/workspace/data/batches (README inside). `make doctor` passed.
Next tasks are in HANDOFF.md section 2, in order. Start with the data mount
task (docs/TASKS/data-mount.md): the mount itself is already in the compose
files, what is left is making /workspace/data the default output of the
scripts and documenting it. Then the Gazebo crash task
(docs/TASKS/gazebo-crashes.md): before changing anything, measure the gzserver
crash rate here with the cameras on (about 20 headless runs) — it may already
be gone without WSL's D3D12 rendering.
```

---

## What changed in the repo to prepare this (2026-10-24)

- `docker-compose.yml` is now the **shared base**; the host-specific parts moved to
  overlays: `docker-compose.wsl.yml` (the WSL2 D3D12 rendering: `/dev/dxg`,
  `/usr/lib/wsl`, `/mnt/wslg`, the Wayland/PulseAudio variables) and
  `docker-compose.native.yml` (NVIDIA GLX rendering, PRIME offload for hybrid laptops).
- The `Makefile` picks the overlay by itself (`PLATFORM` from `/proc/version`,
  overridable) and uses `docker compose` when available, else `docker-compose`.
  New targets: `make platform` and `make doctor`.
- The persistent data mount `${HOME}/skyinterceptor-data` → `/workspace/data` is in
  the base compose file, and `make up` creates the host directory.
- On the WSL machine the resolved configuration is the same as before, plus the data
  mount (checked with `docker compose config`). The native variant was validated with
  `docker compose config` but has **not been run** on a native machine yet — this
  guide's checks are the first run.
