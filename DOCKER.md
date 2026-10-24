# Docker Configuration Guide

This document provides detailed information about the simplified Docker setup for the Interceptor Drone System.

## Overview

The project uses a streamlined Docker setup designed for efficient development:

- **Single Stage Dockerfile**: Combines ROS 2 Humble, CUDA, and development tools into one robust image.
- **Unified Service**: A single `interceptor` service in `docker-compose.yml` handles all tasks (dev, test, launch).
- **GPU Support**: NVIDIA CUDA runtime for OpenCV and TensorRT.
- **X11 Forwarding**: GUI applications (Gazebo, RViz) work seamlessly.
- **Volume Mounts**: Source code on host is mounted to the container for live editing.
- **Persistent Build Cache**: Named Docker volumes persist build artifacts, making recompilation fast.

## Dockerfile

The Dockerfile starts from `nvidia/cuda:11.8.0-devel-ubuntu22.04` and installs:
- ROS 2 Humble Desktop
- Gazebo 11
- OpenCV & Eigen
- Python dependencies (YOLOv8, TensorRT support)
- Development tools (GDB, Valgrind, Linter, Formatter)

## Docker Compose

The compose setup is a **shared base plus one overlay per kind of host**:

| File | What |
|---|---|
| `docker-compose.yml` | the base: the `interceptor` service, `privileged: true` and `runtime: nvidia`, `network_mode: host`, the source, build, SSH/Git and data mounts |
| `docker-compose.wsl.yml` | WSL2 (Windows): GPU rendering through Mesa's D3D12 driver (`/dev/dxg`, `/usr/lib/wsl`, `/mnt/wslg`, the WSLg variables) |
| `docker-compose.native.yml` | native Linux: GPU rendering through the NVIDIA GLX driver the container toolkit injects, with PRIME offload for hybrid laptops |

**Use `make`, not `docker compose` directly.** The Makefile picks the overlay from
`/proc/version` (`PLATFORM=wsl` or `native`; override it with e.g.
`make up PLATFORM=native`) and uses the `docker compose` plugin when it is installed,
else the old `docker-compose`. `make platform` prints what it picked. By hand:

```bash
docker compose -f docker-compose.yml -f docker-compose.native.yml up -d
```

### Persistent data: `/workspace/data`

`${HOME}/skyinterceptor-data` on the host is mounted at `/workspace/data` in the
container. It is an absolute path, so it is the same whichever checkout or worktree
started the container, and it survives a container recreate; `make up` creates it.
**Rule: datasets, weights and batch results go in `/workspace/data`, never in the
container's `/tmp`** — the container's `/tmp` is lost when it is recreated (that is
how the P3.2 dataset and weights were lost). Scratch build workspaces may stay in
`/tmp`.

### Start it from the main checkout

The source mount (`./interceptor_ws/src`) is relative to the directory compose is
started from. Start the container from the main checkout, not from a worktree, or it
builds the worktree's code. `make doctor` shows where `src` is mounted from.

### Moving to a native Ubuntu machine

Step by step, with a check after each step: [`docs/MOVE_TO_UBUNTU.md`](docs/MOVE_TO_UBUNTU.md).

## GPU Rendering on WSL2

`runtime: nvidia` gives the container **compute** access to the GPU (CUDA,
TensorRT, `nvidia-smi`, GPU OpenCV). It does **not**, by itself, give Gazebo or
RViz hardware **OpenGL rendering** on WSL2. On WSL2 the classic NVIDIA GLX path
is not used; hardware OpenGL is provided by **Mesa's D3D12 (Gallium) driver**,
which reaches the GPU through the WSL driver libraries in `/usr/lib/wsl/lib` and
the `/dev/dxg` device. Without those, Gazebo silently falls back to CPU software
rendering (`llvmpipe`), which is very slow.

To enable GPU rendering, the WSL overlay (`docker-compose.wsl.yml`):
- mounts `/usr/lib/wsl:ro` (provides `libd3d12.so`, `libdxcore.so`, …) and adds
  it to `LD_LIBRARY_PATH`;
- mounts `/mnt/wslg` for the WSLg X11/Wayland display sockets;
- passes the `/dev/dxg` device through to the container;
- sets `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` so the D3D12 driver targets the
  NVIDIA GPU;
- uses a current Mesa (from the `kisak/kisak-mesa` PPA in the Dockerfile) that
  has reliable D3D12 support.

**Verify it is working.** After `make up`, the container banner prints an
`OpenGL Renderer:` line. Or run inside the container:

```bash
glxinfo -B | grep -E "Device|OpenGL renderer"
```

You want the renderer to name your NVIDIA GPU (e.g. `D3D12 (NVIDIA ...)`). If it
says `llvmpipe` or `softpipe`, rendering is still on the CPU.

> **Note:** These changes require rebuilding the image (`make build`) and
> recreating the container (`make down && make up`) so the new mounts, device,
> and Mesa packages take effect. WSL2 GPU support also requires an up-to-date
> NVIDIA Windows driver and WSLg (default on recent Windows 10/11).

## GPU Rendering on native Linux

On a native Linux host none of the WSL2 machinery exists. With the NVIDIA driver on the
host and the NVIDIA container toolkit configured for Docker, `runtime: nvidia` and
`NVIDIA_DRIVER_CAPABILITIES=all` make the toolkit inject the driver's GLX/EGL libraries
into the container, and OpenGL renders on the GPU directly. The native overlay
(`docker-compose.native.yml`) only adds `__NV_PRIME_RENDER_OFFLOAD=1` and
`__GLX_VENDOR_LIBRARY_NAME=nvidia`, so a hybrid-graphics laptop renders on the NVIDIA
GPU rather than the integrated one (harmless on a desktop). The entrypoint's WSL
tweaks only apply when `/usr/lib/wsl/lib` exists, so they stay out of the way.

`make doctor` should then show an `OpenGL renderer` naming the NVIDIA GPU (not
`llvmpipe`, not `D3D12`). The full setup, step by step:
[`docs/MOVE_TO_UBUNTU.md`](docs/MOVE_TO_UBUNTU.md).

## Makefile Commands

The `Makefile` simplifies interaction with the Docker container:

| Command | Description |
|---------|-------------|
| `make build` | Build the Docker image |
| `make up` | Start the container in the background |
| `make down` | Stop and remove the container |
| `make shell` | Open a new bash shell inside the running container |
| `make build-ws` | Build the ROS 2 workspace inside the container |
| `make sim` | Launch the Gazebo simulation |
| `make full` | Launch the full interceptor system |
| `make test` | Run unit tests and linters |
| `make clean` | Remove build, install, and log artifacts |
| `make status` | Show container status |

## Development Workflow

All `make` targets are run **from the host**; they call `docker compose exec` internally (with the overlay for the host, see above). `make doctor` checks the host and the container in one go: the GPU, the OpenGL renderer, PyTorch's CUDA, and the source and data mounts.

1.  **Start the environment**: `make up`
2.  **Build the code**: `make build-ws`
3.  **Run the system**: `make full`
4.  **Iterate**: Edit code on your host; it's instantly reflected in the container. Re-run `make build-ws` to compile.
5.  **Debug interactively**: `make shell` opens a bash shell inside the container.

## Troubleshooting

### GPU Not Detected
Ensure the NVIDIA Container Toolkit is installed on your host and run:
`docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi`

### Gazebo Slow / Using CPU Instead of GPU (WSL2)
This means OpenGL is rendering on the CPU (`llvmpipe`). See the
[GPU Rendering on WSL2](#gpu-rendering-on-wsl2) section above. Checklist:
- Rebuild and recreate: `make build && make down && make up`.
- Confirm `/dev/dxg`, `/usr/lib/wsl`, and `/mnt/wslg` exist on the WSL host.
- Inside the container, run `glxinfo -B | grep Device` — it should name your
  NVIDIA GPU, not `llvmpipe`.
- Update your NVIDIA Windows driver if the GPU still isn't picked up.

### GUI Applications Not Displaying
The `make up` command automatically runs `xhost +local:docker`. If displays still don't work, ensure your `DISPLAY` environment variable is set correctly on the host.

### Network Issues
Since we use `network_mode: host`, ensure there are no port conflicts with other ROS 2 instances or local services.