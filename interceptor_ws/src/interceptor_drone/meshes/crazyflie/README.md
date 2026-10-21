# Bitcraze Crazyflie 2.1 meshes

Visual meshes for `urdf/target_drone.urdf.xacro`, taken from the official
`crazyflie_description` package in
[bitcraze/crazyflie-simulation](https://github.com/bitcraze/crazyflie-simulation)
(branch `main`), **MIT licence**, see `LICENSE`.

| File | Upstream | Changes |
|---|---|---|
| `collada_files/cf2_assembly.dae` | same | None |
| `collada_files/ccw_prop.dae` | same | None |
| `collada_files/cw_prop.dae` | same | None |

The meshes are in metres already, so no scaling is needed. The `cf2_assembly.dae`
body mesh has its origin at the body centre; the inertial origin sits 17.4 mm
above it, as in the upstream URDF.

This is the intruder of the INTERCEPT scenario: a 92 mm, 27 g quadrotor, which
makes it visibly a *small* drone next to the 500 mm interceptor it is going after.
