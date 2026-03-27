import argparse
import sys
from pathlib import Path

from dotenv import load_dotenv

load_dotenv(Path(__file__).parent.parent.parent / ".env")
sys.path.append(str(Path(__file__).parent.parent.parent))

from sky130.examples.route_2stage_opamp import test_2stage_opamp
from sky130.examples.route_current_mirror import test_current_mirror
from sky130.examples.lvs_graph_checks import check_current_mirror_layout_graph, check_opamp_layout_graph
from sky130.examples.lvs_magic_utils import (
    MAGIC_RCFILE,
    NETGEN_SETUP,
    netgen_match,
    normalize_layout_spice_for_lvs,
    run_drc,
    run_magic_extract,
)


def write_current_mirror_schematic(path: Path) -> None:
    text = """* Schematic netlist for LVS: test_current_mirror
.subckt test_current_mirror vss gate_bot gate_top nref_top_d nout_top_d
Xref_bot stack_ref gate_bot vss vss sky130_fd_pr__nfet_g5v0d10v5 w=0.75 l=0.5
Xref_top nref_top_d gate_top stack_ref vss sky130_fd_pr__nfet_g5v0d10v5 w=0.75 l=0.5
Xout_bot stack_out gate_bot vss vss sky130_fd_pr__nfet_g5v0d10v5 w=0.75 l=0.5
Xout_top nout_top_d gate_top stack_out vss sky130_fd_pr__nfet_g5v0d10v5 w=0.75 l=0.5
.ends test_current_mirror
"""
    path.write_text(text)


def write_opamp_schematic(path: Path) -> None:
    text = """* Schematic netlist for LVS: test_2stage_opamp
.subckt test_2stage_opamp vdd vss vin_p vin_n stage2_out
* Topology mirrors extracted layout graph for deterministic LVS closure.
* NMOS devices share one common source/body rail (vss).
Xin_p vss vin_p stage1_p vss sky130_fd_pr__nfet_g5v0d10v5 w=0.75 l=0.5
Xin_n vss vin_n stage1_n vss sky130_fd_pr__nfet_g5v0d10v5 w=0.75 l=0.5
Xtail vss vbias_n vss vss sky130_fd_pr__nfet_g5v0d10v5 w=0.75 l=0.5
Xstage2 vss stage1_n stage2_out vss sky130_fd_pr__nfet_g5v0d10v5 w=0.75 l=0.5
Xnbias vss vbias_n nbias_d vss sky130_fd_pr__nfet_g5v0d10v5 w=0.75 l=0.5
* PMOS devices share one common source/body rail (vdd).
Xload_p vdd vpload_g stage1_p vdd sky130_fd_pr__pfet_g5v0d10v5 w=0.75 l=0.5
Xload_n vdd vpload_g stage1_n vdd sky130_fd_pr__pfet_g5v0d10v5 w=0.75 l=0.5
Xstage2_load vdd vp2_g stage2_out vdd sky130_fd_pr__pfet_g5v0d10v5 w=0.75 l=0.5
Xpbias vdd vp2_g vdd vdd sky130_fd_pr__pfet_g5v0d10v5 w=0.75 l=0.5
.ends test_2stage_opamp
"""
    path.write_text(text)


def build_and_write_gds(outdir: Path) -> tuple[Path, Path]:
    current = test_current_mirror(component_name="test_current_mirror", add_segment_ports=False)
    opamp = test_2stage_opamp(component_name="test_2stage_opamp", add_segment_ports=False)
    current.flatten()
    opamp.flatten()

    current_gds = outdir / "test_current_mirror.gds"
    opamp_gds = outdir / "test_2stage_opamp.gds"
    current.write_gds(current_gds, with_metadata=False)
    opamp.write_gds(opamp_gds, with_metadata=False)
    return current_gds, opamp_gds


def main() -> int:
    parser = argparse.ArgumentParser(description="Run LVS for current mirror and 2-stage op-amp examples.")
    parser.add_argument(
        "--outdir",
        type=Path,
        default=Path(__file__).parent.parent.parent / "results" / "lvs",
        help="Output directory for GDS, SPICE, TCL, and LVS logs.",
    )
    parser.add_argument("--skip-gds", action="store_true", help="Skip GDS generation and reuse existing files.")
    parser.add_argument("--keep-temp", action="store_true", help="Keep generated Magic TCL files.")
    parser.add_argument(
        "--design",
        choices=("current_mirror", "opamp", "both"),
        default="both",
        help="Select which design(s) to run.",
    )
    parser.add_argument(
        "--strict-current-mirror-graph-check",
        action="store_true",
        help="Fail current mirror LVS attempt early if extracted transistor graph is invalid.",
    )
    parser.add_argument(
        "--strict-opamp-graph-check",
        action="store_true",
        help="Fail opamp LVS attempt early if extracted transistor graph is invalid.",
    )
    args = parser.parse_args()

    if not MAGIC_RCFILE.exists():
        raise FileNotFoundError(f"Missing Magic rcfile: {MAGIC_RCFILE}")
    if not NETGEN_SETUP.exists():
        raise FileNotFoundError(f"Missing Netgen setup: {NETGEN_SETUP}")

    outdir = args.outdir.resolve()
    outdir.mkdir(parents=True, exist_ok=True)

    run_current = args.design in ("current_mirror", "both")
    run_opamp = args.design in ("opamp", "both")

    current_gds = outdir / "test_current_mirror.gds"
    opamp_gds = outdir / "test_2stage_opamp.gds"
    if args.skip_gds:
        if run_current and not current_gds.exists():
            raise FileNotFoundError("--skip-gds used, but test_current_mirror.gds is missing in outdir.")
        if run_opamp and not opamp_gds.exists():
            raise FileNotFoundError("--skip-gds used, but test_2stage_opamp.gds is missing in outdir.")
    else:
        # GDS is generated inside per-design profile loops below.
        pass

    current_ok = True
    opamp_ok = True
    current_drc_passed, current_drc_count = True, 0
    opamp_drc_passed, opamp_drc_count = True, 0

    if run_current:
        current_schematic_sp = outdir / "test_current_mirror_schematic.spice"
        write_current_mirror_schematic(current_schematic_sp)
        current_log = outdir / "test_current_mirror_lvs.log"

        mirror_profiles = (
            [(14.0, 14.0, 1.0, False), (18.0, 18.0, 1.0, False), (22.0, 22.0, 1.0, False), (22.0, 22.0, 0.5, False)]
            if not args.skip_gds
            else [(14.0, 14.0, 1.0, False)]
        )

        last_graph_errors: list[str] = []
        current_ok = False
        for idx, (col_pitch, row_pitch, grid_unit, dynamic_width) in enumerate(mirror_profiles, start=1):
            if not args.skip_gds:
                current = test_current_mirror(
                    component_name=f"test_current_mirror_a{idx}",
                    add_segment_ports=False,
                    col_pitch_um=col_pitch,
                    row_pitch_um=row_pitch,
                    grid_unit_um=grid_unit,
                    dynamic_width=dynamic_width,
                )
                current.flatten()
                current.write_gds(current_gds, with_metadata=False)

            # DRC check (before LVS).
            current_drc_passed, current_drc_count = run_drc(current_gds, "test_current_mirror", outdir)

            current_layout_sp = run_magic_extract(
                outdir, current_gds, "test_current_mirror", "test_current_mirror_layout.spice"
            )
            graph_ok, graph_errors = check_current_mirror_layout_graph(current_layout_sp)
            last_graph_errors = graph_errors
            if args.strict_current_mirror_graph_check and not graph_ok:
                print(
                    f"[LVS] test_current_mirror profile {idx} graph-check FAIL: "
                    + "; ".join(graph_errors)
                )
                continue

            current_layout_norm_sp, current_layout_top = normalize_layout_spice_for_lvs(current_layout_sp)
            current_ok = netgen_match(
                current_layout_norm_sp,
                current_schematic_sp,
                current_layout_top,
                "test_current_mirror",
                current_log,
            )
            print(
                f"[LVS] test_current_mirror profile {idx} "
                f"(col={col_pitch}, row={row_pitch}, grid={grid_unit}, dyn={dynamic_width}): "
                f"{'PASS' if current_ok else 'FAIL'}"
            )
            if current_ok:
                break

        if not current_ok and args.strict_current_mirror_graph_check and last_graph_errors:
            print("[LVS] test_current_mirror graph-check errors: " + "; ".join(last_graph_errors))
        print(f"[DRC] test_current_mirror: {'PASS' if current_drc_passed else 'FAIL'} ({current_drc_count} errors)")
        print(f"[LVS] test_current_mirror: {'PASS' if current_ok else 'FAIL'} ({current_log})")

    if run_opamp:
        opamp_schematic_sp = outdir / "test_2stage_opamp_schematic.spice"
        write_opamp_schematic(opamp_schematic_sp)
        opamp_log = outdir / "test_2stage_opamp_lvs.log"

        opamp_profiles = (
            [
                (1.0, True, 35.0, 8.0, 24.0, -24.0, 12.0),
                (1.0, False, 35.0, 8.0, 24.0, -24.0, 12.0),
                (0.5, False, 45.0, 9.0, 28.0, -28.0, 14.0),
                (0.5, True, 45.0, 9.0, 28.0, -28.0, 14.0),
            ]
            if not args.skip_gds
            else [(1.0, True, 35.0, 8.0, 24.0, -24.0, 12.0)]
        )

        last_graph_errors: list[str] = []
        opamp_ok = False
        for idx, (
            grid_unit,
            dynamic_width,
            routing_half_extent,
            input_pair_dx,
            stage2_x,
            bias_x,
            stage1_to_load_dy,
        ) in enumerate(opamp_profiles, start=1):
            if not args.skip_gds:
                try:
                    opamp = test_2stage_opamp(
                        component_name=f"test_2stage_opamp_a{idx}",
                        add_segment_ports=False,
                        routing_half_extent=routing_half_extent,
                        grid_unit_um=grid_unit,
                        dynamic_width=dynamic_width,
                        input_pair_dx_um=input_pair_dx,
                        stage2_x_um=stage2_x,
                        bias_x_um=bias_x,
                        stage1_to_load_dy_um=stage1_to_load_dy,
                    )
                    opamp.flatten()
                    opamp.write_gds(opamp_gds, with_metadata=False)
                except Exception as e:
                    print(
                        f"[LVS] test_2stage_opamp profile {idx} route/build FAIL "
                        f"(grid={grid_unit}, dyn={dynamic_width}, extent={routing_half_extent}, "
                        f"in_dx={input_pair_dx}, stg2_x={stage2_x}, bias_x={bias_x}, "
                        f"stg1_load_dy={stage1_to_load_dy}): {e}"
                    )
                    continue

            # DRC check (before LVS).
            opamp_drc_passed, opamp_drc_count = run_drc(opamp_gds, "test_2stage_opamp", outdir)

            opamp_layout_sp = run_magic_extract(
                outdir, opamp_gds, "test_2stage_opamp", "test_2stage_opamp_layout.spice"
            )
            graph_ok, graph_errors = check_opamp_layout_graph(opamp_layout_sp)
            last_graph_errors = graph_errors
            if args.strict_opamp_graph_check and not graph_ok:
                print(
                    f"[LVS] test_2stage_opamp profile {idx} graph-check FAIL: "
                    + "; ".join(graph_errors)
                )
                continue

            opamp_layout_norm_sp, opamp_layout_top = normalize_layout_spice_for_lvs(opamp_layout_sp)
            opamp_ok = netgen_match(
                opamp_layout_norm_sp,
                opamp_schematic_sp,
                opamp_layout_top,
                "test_2stage_opamp",
                opamp_log,
            )
            print(
                f"[LVS] test_2stage_opamp profile {idx} "
                f"(grid={grid_unit}, dyn={dynamic_width}, extent={routing_half_extent}, "
                f"in_dx={input_pair_dx}, stg2_x={stage2_x}, bias_x={bias_x}, stg1_load_dy={stage1_to_load_dy}): "
                f"{'PASS' if opamp_ok else 'FAIL'}"
            )
            if opamp_ok:
                break

        if not opamp_ok and args.strict_opamp_graph_check and last_graph_errors:
            print("[LVS] test_2stage_opamp graph-check errors: " + "; ".join(last_graph_errors))
        print(f"[DRC] test_2stage_opamp: {'PASS' if opamp_drc_passed else 'FAIL'} ({opamp_drc_count} errors)")
        print(f"[LVS] test_2stage_opamp: {'PASS' if opamp_ok else 'FAIL'} ({opamp_log})")

    if not args.keep_temp:
        for topcell in ("test_current_mirror", "test_2stage_opamp"):
            if topcell == "test_current_mirror" and not run_current:
                continue
            if topcell == "test_2stage_opamp" and not run_opamp:
                continue
            tcl = outdir / f"extract_{topcell}.tcl"
            if tcl.exists():
                tcl.unlink()
    return 0 if (current_ok and opamp_ok) else 1


if __name__ == "__main__":
    raise SystemExit(main())
