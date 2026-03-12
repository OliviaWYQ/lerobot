# Isaac Sim Import Notes (URDF v1)

## 1. Files
- URDF: `household_mobile_manipulator_v1.urdf`
- BOM: `../BOM_v1_household_robot.md`
- Converter: `../tools/convert_urdf_to_usd.py`

## 2. Current URDF Scope
- Includes:
  - differential base (`left_wheel_joint`, `right_wheel_joint`)
  - lift prismatic joint (`lift_z_joint`, 0~0.30m)
  - stewart base/top scaffolds with updated geometry
  - outer passive support ring rollers (anti-tip ring)
  - top body scaffold and arm/camera mount links
- Placeholder:
  - Stewart is represented as a simplified scaffold, not true closed-loop mechanics.

## 3. Important Limitation
- A true Stewart platform is a closed kinematic loop.
- URDF cannot represent closed loops directly.
- In Isaac Sim, implement the Stewart mechanism using USD articulation/physics constraints after import.

## 4. Suggested Isaac Workflow
1. Import URDF as base articulation scaffold.
2. Create Stewart lower/upper plates and 6 actuators in USD.
3. Add constraints (or D6 joints) to realize closed-loop behavior.
4. Bind high-level commands:
   - `body/height_cmd` -> `lift_z_joint`
   - `body/attitude_cmd` -> Stewart constraint targets
5. Attach real left/right arm USD/URDF under `left_arm_mount_link` and `right_arm_mount_link`.

### Quick conversion command
Run this with Isaac Sim python:
```bash
cd /home/nvidia/Desktop/lerobot-main
./python.sh design/isaac_sim/tools/convert_urdf_to_usd.py \
  --urdf design/isaac_sim/urdf/household_mobile_manipulator_v1.urdf \
  --usd design/isaac_sim/usd/household_mobile_manipulator_v1.usd \
  --headless
```

## 5. Parameters to Tune in Isaac
- Lift:
  - effort/velocity limits
  - damping/friction
- Base:
  - wheel friction, drive gains, max torque
- Stewart:
  - actuator stiffness, damping, force limits
  - motion envelope and safety bounds

## 6. Next Integration Step
- Replace placeholder arm mounts with real dual-arm URDF/USD.
- Add contact geometry for the platform and payload validation.
- Add IMU + camera sensors and verify observation topics for policy pipelines.
- Replace Stewart placeholder joint with true 6-actuator closed-loop USD constraints.
