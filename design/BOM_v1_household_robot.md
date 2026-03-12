# BOM v1 - Household Robot (Wheels + Lift + Stewart + Dual Arms)

## 1. System Targets (from requirements)
- Runtime: 8h continuous
- Environment: indoor flat ground, small thresholds
- Payload target: 1-3kg per arm typical, 5kg short peak
- Height modulation: equivalent to human household operation range via lift + posture

## 2. BOM Table
| Subsystem | Item | Qty | Key Specs (Target) | Candidate / Notes |
|---|---:|---:|---|---|
| Base | Differential drive wheel motor + driver | 2 | Existing xlerobot kit | Reuse (no new purchase in incremental plan) |
| Base | Passive caster / support wheel | 1-2 | Low rolling resistance | Depends on frame geometry |
| Base | Wheel + tire set | 2 | Indoor, low noise, high grip | 6-10 inch typical |
| Base | Stewart outer support ring rollers | 8-12 | Passive low-friction rollers, ground clearance 5-12mm | Anti-tip ring around Stewart base (safety contact) |
| Base | Base IMU | 1 | 6-axis or 9-axis, 100Hz+ | For posture compensation and safety |
| Base | Main frame | 1 | Aluminum profile / plate | Keep CG low |
| Lift | Electric linear lift column | 1 | Stroke 250-350mm, position feedback | Main Z channel (coarse) |
| Lift | Lift motor driver/controller | 1 | BTS7960/IBT-2 (low-cost) or RoboClaw (upgrade) | V1 low-cost uses BTS7960 |
| Lift | Upper mounting plate | 1 | Fits Stewart base plate | Rigid, anti-twist |
| Stewart | Stewart platform kit | 1 | 6-DOF, small stroke high bandwidth, base dia 320-360mm | Based on downloaded Stewart-Platform repo |
| Stewart | 6 actuators + drivers | 6 | V1: BTS7960 + Teensy PID; V2: RoboClaw/CAN servo | V1 focuses on low cost and fast bring-up |
| Stewart | Top payload plate | 1 | Top dia 280-330mm, mount dual-arm base and sensors | Include cable path |
| Arms | Left manipulator arm | 1 | Existing xlerobot arm | Reuse |
| Arms | Right manipulator arm | 1 | Existing xlerobot arm | Reuse |
| Arms | Gripper (left/right) | 2 | Existing end effectors | Reuse |
| Vision | Head camera | 1 | RGB or RGBD | Stable viewpoint for policy |
| Vision | Wrist camera (left/right) | 2 | RGB | Optional for task-specific training |
| Compute | Main compute (PC/Jetson) | 1 | Isaac Sim typically on PC with NVIDIA GPU | Use Jetson for deployment, PC for simulation/training |
| Power | Battery pack | 1 | Sized by measured average power * 8h * 1.2 | Estimate after subsystem bench tests |
| Power | BMS | 1 | Over-current/over-temp protection | Mandatory |
| Power | DC-DC rails | N | 24V/12V/5V as needed | Separate noisy motor rail from compute rail |
| Safety | E-stop button | 1+ | Hardware latching | Cut motor power path |
| Safety | Fuses / breakers | N | By subsystem current | Mandatory |
| Safety | Limit switches (lift + Stewart) | N | End-stop protection | Hardware interlock preferred |

## 3. Sizing Checklist
- [ ] Base total mass and CG estimated
- [ ] Lift static and dynamic load margin >= 1.5x
- [ ] Stewart actuator force margin >= 2x peak expected load
- [ ] Stewart geometry frozen: `Rb=140~160mm`, `Rp=120~145mm`, top plate dia <= 360mm
- [ ] Outer support ring verified: contact only at large tilt/impact, not continuous drag
- [ ] Battery Wh verified by measured average power profile
- [ ] Thermal profile checked for 8h run (motors, drivers, compute)

## 4. Interfaces to Freeze Before Purchasing
- Mechanical interfaces:
  - Base <-> Lift bolt pattern
  - Lift <-> Stewart base plate
  - Stewart top plate <-> dual-arm mount
- Electrical interfaces:
  - Power rail voltages and max currents
  - Emergency stop wiring topology
- Software interfaces:
  - `body/height_cmd`
  - `body/attitude_cmd`
  - `base/vel_cmd`
  - `arm/*_cmd`

## 5. Procurement Notes
- Buy one extra actuator/driver as spare for Stewart subsystem.
- Prioritize components with encoder feedback and documented control protocol.
- Require CAD drawings before purchase for stack-up verification.
- Do not oversize Stewart top plate beyond the base support polygon unless base track width is increased.
- Low-cost V1 recommendation: `Teensy 4.1 + BTS7960` first, then upgrade to RoboClaw/servo only if needed.
