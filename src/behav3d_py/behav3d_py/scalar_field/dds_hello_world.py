#!/usr/bin/env python3
"""Minimal 3DP-DDS bead simulation.

This script is intentionally independent from lib_scalar. It only exercises
the external dds package: bead primitives, dense simulation, mesh extraction,
file export, and the native dds PyVistaQt workbench.
"""

from __future__ import annotations

import ctypes
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

from dds import BeadProfile, DepositionMetadata, Domain, LineDeposit, Simulator
from dds.cli import run_cli
from dds.geometry import mesh_surface_area, mesh_volume_estimate, write_mesh


DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parent / "output" / "dds_hello_world"
QT_LIB_DIR_RELATIVE = Path("external/qt-libs/root/usr/lib/x86_64-linux-gnu")


@dataclass
class Args:
    """Run a small 3DP-DDS bead simulation and optionally open the dds workbench."""

    output_dir: Path = DEFAULT_OUTPUT_DIR
    view: bool = True
    view_mode: Literal["surface", "occupancy", "density"] = "surface"
    threshold: float = 0.5
    voxel_mm: float = 2.0
    bead_width_mm: float = 16.0
    bead_height_mm: float = 12.0
    length_mm: float = 80.0
    lane_spacing_mm: float = 18.0


def mm(value: float) -> float:
    """Convert millimetres to metres."""

    return 1e-3 * float(value)


def can_load_xcb_cursor() -> bool:
    """Return True when Qt's xcb cursor dependency is loadable."""

    try:
        ctypes.CDLL("libxcb-cursor.so.0")
        return True
    except OSError:
        return False


def find_local_qt_lib_dir() -> Path | None:
    """Find the local no-sudo extraction of libxcb-cursor0, if present."""

    for parent in Path(__file__).resolve().parents:
        candidate = parent / QT_LIB_DIR_RELATIVE
        if (candidate / "libxcb-cursor.so.0").exists():
            return candidate
    return None


def ensure_qt_xcb_cursor_for_view(view: bool) -> None:
    """Relaunch with local Qt xcb libs when the system package is missing."""

    if not view or can_load_xcb_cursor():
        return

    qt_lib_dir = find_local_qt_lib_dir()
    if qt_lib_dir is None:
        print(
            "Qt visualization needs libxcb-cursor0 on this X11 session.\n"
            "Install it with: sudo apt-get install libxcb-cursor0",
            file=sys.stderr,
        )
        return

    env = os.environ.copy()
    current = env.get("LD_LIBRARY_PATH", "")
    lib_dir_str = str(qt_lib_dir)
    if lib_dir_str in [part for part in current.split(":") if part]:
        return

    env["LD_LIBRARY_PATH"] = lib_dir_str if not current else f"{lib_dir_str}:{current}"
    env["BEHAV3D_DDS_QT_LOCAL_LIBS"] = "1"
    os.execvpe(sys.executable, [sys.executable, *sys.argv], env)


def build_demo_deposits(args: Args) -> list[LineDeposit]:
    """Create a tiny two-layer toolpath using top-referenced bead targets."""

    width = mm(args.bead_width_mm)
    height = mm(args.bead_height_mm)
    length = mm(args.length_mm)
    lane_spacing = mm(args.lane_spacing_mm)

    profile = BeadProfile(width=width, height=height)
    layer0 = DepositionMetadata(
        layer_id=0,
        user_data={"material_id": "demo-clay", "tool_id": "T0", "role": "base"},
    )
    layer1 = DepositionMetadata(
        layer_id=1,
        user_data={"material_id": "demo-clay", "tool_id": "T0", "role": "top"},
    )

    z0 = height
    z1 = 2.0 * height
    y0 = 0.0
    y1 = lane_spacing
    y_mid = 0.5 * lane_spacing

    return [
        LineDeposit(start=(0.0, y0, z0), end=(length, y0, z0), profile=profile, metadata=layer0),
        LineDeposit(start=(0.0, y1, z0), end=(length, y1, z0), profile=profile, metadata=layer0),
        LineDeposit(
            start=(0.15 * length, y_mid, z1),
            end=(0.85 * length, y_mid, z1),
            profile=profile,
            metadata=layer1,
        ),
    ]


def run_simulation(args: Args):
    """Build the simulator, add deposits incrementally, and return a result."""

    if args.voxel_mm <= 0.0:
        raise ValueError(f"voxel_mm must be > 0, got {args.voxel_mm}")
    if args.bead_width_mm <= 0.0:
        raise ValueError(f"bead_width_mm must be > 0, got {args.bead_width_mm}")
    if args.bead_height_mm <= 0.0:
        raise ValueError(f"bead_height_mm must be > 0, got {args.bead_height_mm}")

    deposits = build_demo_deposits(args)
    domain = Domain.from_deposits(deposits, voxel_size=mm(args.voxel_mm), padding="auto")

    simulator = Simulator(domain)
    for deposit in deposits:
        simulator.add_deposit(deposit)

    return simulator.result(compositions=("max", "coverage"), threshold=args.threshold)


def write_outputs(args: Args, result) -> None:
    """Write simple outputs for headless inspection."""

    args.output_dir.mkdir(parents=True, exist_ok=True)
    surface = result.analysis.surface_mesh(threshold=args.threshold)
    surface_path = write_mesh(args.output_dir / "surface.ply", surface)
    checkpoint_path = result.checkpoint(args.output_dir / "checkpoint.npz")
    bundle_paths = result.save(
        args.output_dir / "bundle",
        metadata={
            "source": "dds_hello_world",
            "threshold": float(args.threshold),
            "voxel_mm": float(args.voxel_mm),
            "bead_width_mm": float(args.bead_width_mm),
            "bead_height_mm": float(args.bead_height_mm),
        },
    )

    occupancy = result.analysis.occupancy(threshold=args.threshold)
    density_max = result.field("max")
    coverage = result.field("coverage")
    dep_index = result.analysis.deposition_index_field()
    layer_ids = sorted(
        {
            deposit.metadata.layer_id
            for deposit in result.deposits
            if deposit.metadata.layer_id is not None
        }
    )

    print("3DP-DDS hello world")
    print(f"dds surface: {surface_path}")
    print(f"dds checkpoint: {checkpoint_path}")
    print(f"bundle files: {', '.join(sorted(bundle_paths))}")
    print(f"grid_shape: {result.domain.grid_shape}")
    print(f"voxel_size_m: {result.domain.voxel_size}")
    print(f"deposits: {len(result.deposits)}")
    print(f"layers: {tuple(layer_ids)}")
    print(f"occupied_voxels: {int(occupancy.sum())}")
    print(
        f"density_max_range: "
        f"({float(density_max.min()):.3f}, {float(density_max.max()):.3f})"
    )
    print(f"coverage_max: {float(coverage.max()):.3f}")
    print(f"deposition_index_max: {int(dep_index.max())}")
    print(f"surface_vertices: {surface.n_vertices}")
    print(f"surface_faces: {surface.n_faces}")
    if not surface.is_empty:
        print(f"surface_area_m2: {float(mesh_surface_area(surface)):.6f}")
        print(f"volume_estimate_m3: {float(mesh_volume_estimate(surface)):.9f}")


def main(args: Args) -> None:
    ensure_qt_xcb_cursor_for_view(args.view)

    result = run_simulation(args)
    write_outputs(args, result)

    if args.view:
        import dds.viz

        workbench = dds.viz.show(result, view_mode=args.view_mode, off_screen=False)
        workbench.app.exec()


if __name__ == "__main__":
    run_cli(Args, main)
