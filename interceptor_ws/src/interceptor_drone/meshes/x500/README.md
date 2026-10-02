# Holybro X500 V2 meshes

Visual meshes for `urdf/interceptor_quadrotor.urdf.xacro`, taken from the PX4
`x500_base` model in [PX4/PX4-gazebo-models](https://github.com/PX4/PX4-gazebo-models)
(commit `a15af9628536914ff7201c992fce5e3cb5d70db9`), BSD-3-Clause, see `LICENSE`.

| File | Upstream | Changes |
|---|---|---|
| `x500_frame.dae` | `NXP-HGD-CF.dae` | Landing-gear plastic, flight controller and metal hardware decimated, Blender light removed, coordinates rounded to 1e-5 m (22 MB → 8 MB). Regenerate with `scripts/reduce_x500_mesh.py`. |
| `CF.png` | `CF.png` | Carbon-fibre texture, downscaled to 512×512. |
| `5010Base.dae`, `5010Bell.dae` | same | Blender lights and cameras removed. |
| `1345_prop_ccw.stl`, `1345_prop_cw.stl` | same | None. |

Placement (frame yaw π, z offset 0.025 m, motors at ±0.174 m, prop scale
11/13) follows the upstream `model.sdf`.
