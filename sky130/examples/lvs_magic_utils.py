from __future__ import annotations

import shutil
import subprocess
from collections.abc import Callable
from pathlib import Path

MAGIC_RCFILE = Path("/usr/local/share/pdk/sky130A/libs.tech/magic/sky130A.magicrc")
NETGEN_SETUP = Path("/usr/local/share/pdk/sky130A/libs.tech/netgen/sky130A_setup.tcl")


def run_cmd(cmd: list[str], cwd: Path | None = None) -> None:
    result = subprocess.run(cmd, cwd=str(cwd) if cwd else None, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"Command failed ({result.returncode}): {' '.join(cmd)}")


def write_magic_drc_tcl(tcl_path: Path, gds_path: Path, topcell: str, *, interactive: bool = False) -> None:
    cmds = [
        "box 0um 0um 0um 0um",
        "gds flatten true",
        f"gds read {gds_path}",
        "set __top [lindex [cellname list topcells] end]",
        "load $__top",
        f"cellname rename $__top {topcell}",
        f"load {topcell}",
        "select top cell",
        "drc euclidean on",
        "drc style drc(full)",
        "drc check",
        "drc catchup",
        "set count [drc list count total]",
        "puts stdout \"Total DRC errors found: $count\"",
    ]
    if not interactive:
        cmds.append("quit")
    cmds.append("")
    tcl_path.write_text("\n".join(cmds))


def run_drc(
    gds_path: Path,
    topcell: str,
    outdir: Path | None = None,
    interactive: bool = False,
) -> tuple[bool, int]:
    """Run Magic DRC on a GDS file.

    Parameters
    ----------
    gds_path : Path
        Path to the GDS file.
    topcell : str
        Top-level cell name.
    outdir : Path | None
        Output directory for DRC artifacts.  Defaults to the same directory
        as *gds_path*.
    interactive : bool
        When True (default), run Magic headless first to obtain the error
        count, then re-launch Magic with the GUI so the user can inspect
        DRC markers.  When False, run headless only.

    Returns
    -------
    tuple[bool, int]
        (passed, error_count) where passed is True when error_count == 0.
    """
    gds_path = Path(gds_path).resolve()
    if not gds_path.exists():
        raise FileNotFoundError(f"GDS not found: {gds_path}")
    if not MAGIC_RCFILE.exists():
        raise FileNotFoundError(f"Missing Magic rcfile: {MAGIC_RCFILE}")

    if outdir is None:
        outdir = gds_path.parent
    outdir = Path(outdir).resolve()
    outdir.mkdir(parents=True, exist_ok=True)

    # Headless run to get the error count.
    tcl_batch = outdir / f"drc_{topcell}.tcl"
    write_magic_drc_tcl(tcl_batch, gds_path, topcell, interactive=False)

    result = subprocess.run(
        [
            "magic",
            "-dnull",
            "-noconsole",
            "-rcfile",
            str(MAGIC_RCFILE),
            str(tcl_batch),
        ],
        cwd=str(outdir),
        text=True,
        capture_output=True,
    )

    count = -1
    for line in result.stdout.splitlines():
        if "Total DRC errors found:" in line:
            try:
                count = int(line.split(":")[-1].strip())
            except ValueError:
                pass

    passed = count == 0
    print(f"[DRC] {topcell}: {'PASS' if passed else 'FAIL'} ({count} errors)")

    # Interactive run so the user can inspect DRC markers in the GUI.
    if interactive:
        tcl_gui = outdir / f"drc_{topcell}_gui.tcl"
        write_magic_drc_tcl(tcl_gui, gds_path, topcell, interactive=True)
        subprocess.run(
            [
                "magic",
                "-rcfile",
                str(MAGIC_RCFILE),
                str(tcl_gui),
            ],
            cwd=str(outdir),
        )

    return passed, count


def write_magic_extract_tcl(tcl_path: Path, gds_path: Path, topcell: str, layout_spice_name: str) -> None:
    tcl_path.write_text(
        "\n".join(
            [
                "box 0um 0um 0um 0um",
                f"gds read {gds_path}",
                "set __top [lindex [cellname list topcells] end]",
                "load $__top",
                f"cellname rename $__top {topcell}",
                f"load {topcell}",
                "select top cell",
                "extract do local",
                "extract all",
                "ext2spice lvs",
                f"ext2spice -o {layout_spice_name}",
                "quit",
                "",
            ]
        )
    )


def run_magic_extract(outdir: Path, gds_path: Path, topcell: str, layout_spice_name: str) -> Path:
    tcl_path = outdir / f"extract_{topcell}.tcl"
    layout_spice = outdir / layout_spice_name
    write_magic_extract_tcl(tcl_path, gds_path, topcell, layout_spice_name)
    run_cmd(
        [
            "magic",
            "-dnull",
            "-noconsole",
            "-rcfile",
            str(MAGIC_RCFILE),
            str(tcl_path),
        ],
        cwd=outdir,
    )
    if not layout_spice.exists():
        raise RuntimeError(f"Magic did not generate {layout_spice}")
    return layout_spice


def find_layout_top_name(spice_path: Path) -> str:
    lines = spice_path.read_text(errors="ignore").splitlines()
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("* Top level circuit "):
            return stripped.split("* Top level circuit ", 1)[1].strip()
    for line in lines:
        stripped = line.strip()
        if stripped.lower().startswith(".subckt "):
            parts = stripped.split()
            if len(parts) >= 2:
                return parts[1]
    raise RuntimeError(f"Could not find top-level circuit name in {spice_path}")


def normalize_layout_spice_for_lvs(layout_spice: Path) -> tuple[Path, str]:
    lines = layout_spice.read_text(errors="ignore").splitlines()
    top_name = find_layout_top_name(layout_spice)

    top_comment_idx = None
    end_idx = None
    for i, line in enumerate(lines):
        if line.strip().startswith("* Top level circuit "):
            top_comment_idx = i
            continue
        if top_comment_idx is not None and line.strip().lower() == ".end":
            end_idx = i
            break

    if top_comment_idx is None or end_idx is None:
        return layout_spice, top_name

    pre = lines[:top_comment_idx]
    top_body = lines[top_comment_idx + 1 : end_idx]
    normalized = pre + [f".subckt {top_name}"] + top_body + [f".ends {top_name}", ""]

    normalized_path = layout_spice.with_name(layout_spice.stem + "_normalized.spice")
    normalized_path.write_text("\n".join(normalized))
    return normalized_path, top_name


def netgen_match(
    layout_spice: Path,
    schematic_spice: Path,
    layout_topcell: str,
    schematic_topcell: str,
    log_path: Path,
) -> bool:
    run_cmd(
        [
            "netgen",
            "-batch",
            "lvs",
            f"{layout_spice} {layout_topcell}",
            f"{schematic_spice} {schematic_topcell}",
            str(NETGEN_SETUP),
            str(log_path),
        ]
    )
    if not log_path.exists():
        return False
    log_text = log_path.read_text(errors="ignore")
    return "Final result: Circuits match uniquely." in log_text


def run_lvs(
    gds_path: Path,
    topcell: str,
    schematic_spice: str,
    outdir: Path | None = None,
    debug: bool = False,
    graph_check_fn: Callable[[Path], tuple[bool, list[str]]] | None = None,
    strict_graph_check: bool = False,
) -> bool:
    """Run LVS on an existing GDS file without re-routing.

    Parameters
    ----------
    gds_path : Path
        Path to the already-written GDS file.
    topcell : str
        Top-level cell name (used for extraction and netgen matching).
    schematic_spice : str
        SPICE netlist content as a string (written to a file in *outdir*).
    outdir : Path | None
        Output directory for extraction artifacts.  Defaults to
        ``results/lvs_{topcell}/`` relative to the repository root.
    debug : bool
        If True, print extra diagnostic information.
    graph_check_fn : callable | None
        Optional function ``(spice_path) -> (ok, errors)`` run on the
        extracted layout SPICE before netgen comparison.
    strict_graph_check : bool
        When True *and* the graph check fails, return False immediately
        without running netgen.

    Returns
    -------
    bool
        True when netgen reports "Circuits match uniquely."
    """
    gds_path = Path(gds_path).resolve()
    if not gds_path.exists():
        raise FileNotFoundError(f"GDS not found: {gds_path}")

    if not MAGIC_RCFILE.exists():
        raise FileNotFoundError(f"Missing Magic rcfile: {MAGIC_RCFILE}")
    if not NETGEN_SETUP.exists():
        raise FileNotFoundError(f"Missing Netgen setup: {NETGEN_SETUP}")

    if outdir is None:
        outdir = gds_path.parent.parent / f"lvs_{topcell}"
    outdir = Path(outdir).resolve()
    outdir.mkdir(parents=True, exist_ok=True)

    # Copy GDS into outdir so Magic artefacts live together.
    lvs_gds = outdir / gds_path.name
    if lvs_gds.resolve() != gds_path.resolve():
        shutil.copy2(gds_path, lvs_gds)

    # DRC check (before LVS).
    drc_passed, drc_count = run_drc(lvs_gds, topcell, outdir)

    # Write schematic SPICE.
    schematic_path = outdir / f"{topcell}_schematic.spice"
    schematic_path.write_text(schematic_spice)

    # Magic extraction.
    layout_spice_name = f"{topcell}_layout.spice"
    layout_spice = run_magic_extract(outdir, lvs_gds, topcell, layout_spice_name)

    # Optional graph check.
    if graph_check_fn is not None:
        graph_ok, graph_errors = graph_check_fn(layout_spice)
        if debug or not graph_ok:
            for e in graph_errors:
                print(f"[LVS graph-check] {e}")
        if strict_graph_check and not graph_ok:
            print(f"[LVS] {topcell}: FAIL (graph-check)")
            return False

    # Normalize and run netgen.
    layout_norm, layout_top = normalize_layout_spice_for_lvs(layout_spice)
    lvs_log = outdir / f"{topcell}_lvs.log"
    ok = netgen_match(layout_norm, schematic_path, layout_top, topcell, lvs_log)

    print(f"[DRC] {topcell}: {'PASS' if drc_passed else 'FAIL'} ({drc_count} errors)")
    print(f"[LVS] {topcell}: {'PASS' if ok else 'FAIL'} ({lvs_log})")
    return ok
