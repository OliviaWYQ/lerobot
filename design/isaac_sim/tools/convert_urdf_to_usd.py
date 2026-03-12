#!/usr/bin/env python3
"""
Convert URDF to USD for Isaac Sim.

Run with Isaac Sim python, for example:
  ./python.sh design/isaac_sim/tools/convert_urdf_to_usd.py \
    --urdf design/isaac_sim/urdf/household_mobile_manipulator_v1.urdf \
    --usd design/isaac_sim/usd/household_mobile_manipulator_v1.usd \
    --headless
"""

from __future__ import annotations

import argparse
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="URDF to USD converter for Isaac Sim")
    parser.add_argument("--urdf", type=Path, required=True, help="Input URDF path")
    parser.add_argument("--usd", type=Path, required=True, help="Output USD path")
    parser.add_argument("--prim-path", type=str, default="/World/Robot", help="Target robot prim path")
    parser.add_argument("--headless", action="store_true", help="Run Isaac Sim headless")
    parser.add_argument("--fix-base", action="store_true", help="Import with fixed base")
    parser.add_argument(
        "--merge-fixed-joints",
        action="store_true",
        help="Merge fixed joints during import (default: False)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.usd.parent.mkdir(parents=True, exist_ok=True)

    # Isaac Sim modules are only available in Isaac's Python runtime.
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": args.headless})

    import omni.kit.commands
    import omni.usd

    urdf_path = str(args.urdf.resolve())
    usd_path = str(args.usd.resolve())

    ok, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    if not ok:
        raise RuntimeError("Failed to create URDF import config.")

    import_config.set_make_default_prim(True)
    import_config.set_merge_fixed_joints(args.merge_fixed_joints)
    import_config.set_fix_base(args.fix_base)
    import_config.set_create_physics_scene(True)
    import_config.set_make_instanceable(False)

    status, imported_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=import_config,
        dest_path=usd_path,
    )
    if not status:
        raise RuntimeError(f"Failed to import URDF: {urdf_path}")

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No USD stage available after URDF import.")

    # If importer created a different root, move/alias is left to downstream workflows.
    # We keep this script minimal and deterministic: import + save to destination.
    omni.usd.get_context().save_as_stage(usd_path)
    print(f"[OK] URDF imported: {urdf_path}")
    print(f"[OK] USD saved   : {usd_path}")
    print(f"[INFO] Imported prim path: {imported_path}")
    print(f"[INFO] Requested prim path: {args.prim_path}")

    simulation_app.close()


if __name__ == "__main__":
    main()
