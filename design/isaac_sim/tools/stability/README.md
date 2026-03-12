# Stability Calculator

## Quick Start
```bash
cd /home/nvidia/Desktop/lerobot-main/design/isaac_sim/tools/stability
python3 stability_calc.py --config sample_config.json
```

## What It Computes
- total mass and COM `(x,y,z)`
- static tip angle (worst support axis)
- static margin vs operating tilt requirement
- dynamic lateral acceleration limit (in `g`)
- suggested base speed caps for candidate turn radii

## How To Use With Your Robot
1. Replace `mass_items` with your measured/estimated masses and COM positions.
2. Set `support_polygon.half_width_m / half_length_m` from real wheel/support-ring geometry.
3. Adjust `requirements` for your safety target.
4. Re-run and check `PASS/FAIL`.
