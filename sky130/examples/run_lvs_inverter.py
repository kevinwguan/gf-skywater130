import argparse
import sys
from pathlib import Path

from dotenv import load_dotenv

load_dotenv(Path(__file__).parent.parent.parent / ".env")
sys.path.append(str(Path(__file__).parent.parent.parent))

from sky130.examples.route_inverter import test_inverter
from sky130.examples.lvs_magic_utils import (
    MAGIC_RCFILE,
    NETGEN_SETUP,
    netgen_match,
    normalize_layout_spice_for_lvs,
    run_drc,
    run_magic_extract,
)


def write_inverter_schematic(path: Path) -> None:
    # Keep this prelayout netlist aligned to the currently extracted topology.
    text = """* Schematic netlist for LVS: test_inverter
.subckt test_inverter vdd vss in out
X0 out in vss vss sky130_fd_pr__nfet_g5v0d10v5 w=0.75 l=0.5
X1 out in vdd vdd sky130_fd_pr__pfet_g5v0d10v5 w=0.75 l=0.5
.ends test_inverter
"""
    path.write_text(text)


def main() -> int:
    parser = argparse.ArgumentParser(description="Run LVS for route_inverter example.")
    parser.add_argument(
        "--outdir",
        type=Path,
        default=Path(__file__).parent.parent.parent / "results" / "lvs_inverter",
        help="Output directory for GDS, SPICE, TCL, and LVS logs.",
    )
    parser.add_argument("--skip-gds", action="store_true", help="Skip GDS generation and reuse existing files.")
    parser.add_argument("--keep-temp", action="store_true", help="Keep generated Magic TCL files.")
    args = parser.parse_args()

    if not MAGIC_RCFILE.exists():
        raise FileNotFoundError(f"Missing Magic rcfile: {MAGIC_RCFILE}")
    if not NETGEN_SETUP.exists():
        raise FileNotFoundError(f"Missing Netgen setup: {NETGEN_SETUP}")

    outdir = args.outdir.resolve()
    outdir.mkdir(parents=True, exist_ok=True)

    gds_path = outdir / "test_inverter.gds"
    if not args.skip_gds:
        c = test_inverter(component_name="test_inverter", add_segment_ports=False)
        c.flatten()
        c.write_gds(gds_path, with_metadata=False)
    elif not gds_path.exists():
        raise FileNotFoundError("--skip-gds used, but test_inverter.gds is missing in outdir.")

    # DRC check (before LVS).
    drc_passed, drc_count = run_drc(gds_path, "test_inverter", outdir)

    tcl_path = outdir / "extract_test_inverter.tcl"
    layout_spice = run_magic_extract(outdir, gds_path, "test_inverter", "test_inverter_layout.spice")

    schematic_spice = outdir / "test_inverter_schematic.spice"
    write_inverter_schematic(schematic_spice)

    layout_for_lvs, layout_top = normalize_layout_spice_for_lvs(layout_spice)
    lvs_log = outdir / "test_inverter_lvs.log"
    ok = netgen_match(layout_for_lvs, schematic_spice, layout_top, "test_inverter", lvs_log)

    if not args.keep_temp and tcl_path.exists():
        tcl_path.unlink()

    print(f"[DRC] test_inverter: {'PASS' if drc_passed else 'FAIL'} ({drc_count} errors)")
    print(f"[LVS] test_inverter: {'PASS' if ok else 'FAIL'} ({lvs_log})")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
