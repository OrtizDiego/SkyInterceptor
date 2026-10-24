# Task: P3.2 — the UAS detector (dataset, training, export, end to end)

**Roadmap:** `ROADMAP.md` → `### P3.2 – UAS detector`.
**Branch:** `feat/p3-2-uas-detector` (draft **PR #32**). **Do it after** the data mount
and the Gazebo crash tasks.

## Where it stands

PR #32 (draft) has the scripts (`scripts/uas_dataset/`: `generate_dataset.py`,
`train_uav_detector.py`, `export_uav_model.py`, `dataset_config.yaml`),
`config/uav_detector_params.yaml`, and the detector/launch changes. It builds and
its tests pass. **The dataset and the smoke-training weights are gone**: they were in
the container's `/tmp` and were lost when it was recreated. The one training run
that existed crashed in validation (`DataLoader worker ... exited unexpectedly`) and
its metrics were all zero: nothing was ever trained.

## What to do

1. **Rebase PR #32 on main** (it is far behind: #31, #33, #37-#41 landed since).
2. **Regenerate the dataset into `/workspace/data/uas/`** (the data mount task) with
   a **fixed seed**. Commit only the config, the seed and `manifest.csv` — never the
   images or the weights (`.gitignore` already ignores `*.pt`; add the dataset
   directory if needed). The manifest's `reason` column labels the negative frames
   *as negatives* (`outside_frame`, ...); keep that.
3. **Train on the GPU and say which device was actually used** (print
   `torch.cuda.is_available()` and the device name in the training log, and put it
   in the PR). On the WSL2 host the image's torch (cu130) needs a newer driver than
   the host had; a compatible build was side-installed at `/opt/torch124`
   (`PYTHONPATH=/opt/torch124`). On a native Ubuntu host with a current driver this
   may not be needed. Fix the DataLoader crash (`workers=0`/`2`, or the shared-memory
   size of the container: `shm_size` in docker-compose).
4. **Evaluate on the held-out split** (precision, recall, mAP50, mAP50-95), and
   report the numbers, not just "it trained".
5. **Export ONNX** (TensorRT if it works on the host), and wire the model path and
   the `uav` class into the detector config.
6. **Prove it end to end** with `perception_source:=vision`: the detector publishes a
   `uav` detection, the 3D localizer turns it into a 3D detection, and the tracker
   confirms a `uav` track (logs or a recorded topic in the PR).
7. **Update PR #32** and the P3.2 roadmap section (runs, device, metrics, what is
   still open).

## Pitfalls

- The stereo cameras must be **on** for this task (the Gazebo crash task adds an
  `enable_cameras` argument that the ground-truth runs turn off).
- The vision depth map runs at ~6 Hz (SGBM + WLS): a sparse detection rate can stop
  the tracker from confirming. Check the confirmation (3-of-5) against the rate.
- Container NumPy 2 breaks the Python `cv_bridge`; the detector decodes the image
  with NumPy directly (see the memory note). Keep it that way.
