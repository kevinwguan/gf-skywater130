"""
Stress-test generator for DoRoutes routing engine.

Auto-generates 10 analog circuits beyond the existing 6 examples,
each with both manual-placement and auto-placement variants.

Usage:
    python stress_test.py                        # All circuits (manual placement)
    python stress_test.py --placed               # Also generate auto-placed variants
    python stress_test.py --circuit nand2        # Single circuit
    python stress_test.py --circuit nand2 latch  # Multiple circuits
    python stress_test.py --no-show              # Skip KLayout visualization
    python stress_test.py --out-dir results/stress
"""

import argparse
import sys
import time
import traceback
from dataclasses import dataclass, field
from pathlib import Path

from dotenv import load_dotenv

load_dotenv(Path(__file__).parent.parent.parent / ".env")
sys.path.append(str(Path(__file__).parent.parent.parent))

from gdsfactory.component import Component
import sky130  # noqa: F401  — activates PDK
from gdsfactory.pdk import get_active_pdk
from gdsfactory.add_pins import add_instance_label

from doroutes.placement import place_auto
from doroutes.multilayer import (
    PinLabelSpec,
    RouteNetSpec,
    label_unrouted_pins,
    route_nets_deterministic_copy,
)
from sky130.routing_utils import SKY130_CONFIG


def _plot_light(c: Component, title: str = ""):
    """Render component to a matplotlib figure with a white background."""
    from io import BytesIO
    import pathlib

    import klayout.lay as lay
    import kfactory as kf
    import matplotlib.pyplot as plt
    from gdsfactory.pdk import get_layer_views
    from gdsfactory.config import GDSDIR_TEMP

    c.insert_vinsts()
    lyp_path = GDSDIR_TEMP / "layer_properties.lyp"
    layer_views = get_layer_views()
    if isinstance(layer_views, str | pathlib.Path):
        lyp_path = pathlib.Path(layer_views)
    else:
        layer_views.to_lyp(filepath=lyp_path)

    lv = lay.LayoutView()
    cvi = lv.create_layout(True)
    lv.active_cellview_index = cvi
    cv = lv.cellview(cvi)
    layout = cv.layout()
    layout.assign(kf.kcl.layout)
    cv.cell = layout.cell(c.name)
    lv.max_hier()
    lv.load_layer_props(str(lyp_path))
    lv.add_missing_layers()
    lv.zoom_fit()
    lv.set_config("text-visible", "false")
    lv.set_config("grid-show-ruler", "false")
    lv.set_config("background-color", "#ffffff")

    pb = lv.get_pixels_with_options(width=2400, height=1800)
    png_data = pb.to_png_data()
    with BytesIO(png_data) as f:
        img_array = plt.imread(f)
    dpi = 150
    fig_w = img_array.shape[1] / dpi
    fig_h = img_array.shape[0] / dpi + (0.4 if title else 0)
    fig, ax = plt.subplots(figsize=(fig_w, fig_h), dpi=dpi)
    ax.imshow(img_array)
    ax.axis("off")
    if title:
        ax.set_title(title, fontsize=14, fontweight="bold", pad=10)
    fig.tight_layout(pad=0.3)
    return fig


# ---------------------------------------------------------------------------
# Shorthand constants
# ---------------------------------------------------------------------------
NFET = "sky130_fd_pr__nfet_g5v0d10v5"
PFET = "sky130_fd_pr__pfet_g5v0d10v5"
W, L = 0.75, 0.5
D, G, S, B = "DRAIN", "GATE", "SOURCE", "BODY"


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------

@dataclass
class DeviceDef:
    name: str
    dev_type: str        # "nmos_5v" or "pmos_5v"
    x: float             # manual-placement x (um)
    y: float             # manual-placement y (um)
    mirror: bool = False
    group: str = ""


@dataclass
class NetDef:
    name: str
    start: tuple         # (inst_name, PORT_SUFFIX)
    stop: tuple          # (inst_name, PORT_SUFFIX)
    prefix: str
    top_level: bool = False


@dataclass
class UnroutedDef:
    pin_name: str
    inst: str
    port_suffix: str


@dataclass
class CircuitSpec:
    name: str
    devices: list
    nets: list
    unrouted: list
    spice_pins: str       # ".subckt ... <these pins>"
    spice_body: str       # device lines
    extent: float = 20.0

    def spice(self, placed: bool = False) -> str:
        sfx = "_placed" if placed else ""
        n = f"test_{self.name}{sfx}"
        return f"* Schematic: {n}\n.subckt {n} {self.spice_pins}\n{self.spice_body}.ends {n}\n"


def _n(inst, d, g, s, b):
    return f"X{inst} {d} {g} {s} {b} {NFET} w={W} l={L}\n"


def _p(inst, d, g, s, b):
    return f"X{inst} {d} {g} {s} {b} {PFET} w={W} l={L}\n"


# ---------------------------------------------------------------------------
# 1. NAND2  —  2 PMOS (parallel) + 2 NMOS (series)
# ---------------------------------------------------------------------------
CIRCUITS: dict[str, CircuitSpec] = {}

CIRCUITS["nand2"] = CircuitSpec(
    name="nand2",
    devices=[
        DeviceDef("pa", "pmos_5v", -8, 14, mirror=True, group="nwell"),
        DeviceDef("pb", "pmos_5v",  8, 22, mirror=True, group="nwell"),
        DeviceDef("na", "nmos_5v", -4,  0, group="pwell"),
        DeviceDef("nb", "nmos_5v",  4, -12, group="pwell"),
    ],
    nets=[
        NetDef("out",         ("pa", D), ("na", D), "out",  top_level=True),
        NetDef("out_join",    ("pb", D), ("pa", D), "out"),
        NetDef("in_a",        ("pa", G), ("na", G), "in_a", top_level=True),
        NetDef("in_b",        ("pb", G), ("nb", G), "in_b", top_level=True),
        NetDef("mid",         ("na", S), ("nb", D), "mid"),
        NetDef("vdd_a",       ("pa", S), ("pa", B), "vdd",  top_level=True),
        NetDef("vdd_b",       ("pb", S), ("pb", B), "vdd"),
        NetDef("vdd_join",    ("pa", S), ("pb", S), "vdd"),
        NetDef("vss",         ("nb", S), ("nb", B), "vss",  top_level=True),
        NetDef("vss_body_a",  ("na", B), ("nb", S), "vss"),
    ],
    unrouted=[],
    spice_pins="vdd vss in_a in_b out",
    spice_body=(
        _p("pa", "out", "in_a", "vdd", "vdd")
        + _p("pb", "out", "in_b", "vdd", "vdd")
        + _n("na", "out", "in_a", "mid", "vss")
        + _n("nb", "mid", "in_b", "vss", "vss")
    ),
    extent=22.0,
)

# ---------------------------------------------------------------------------
# 2. NOR2  —  2 PMOS (series) + 2 NMOS (parallel)
# ---------------------------------------------------------------------------
CIRCUITS["nor2"] = CircuitSpec(
    name="nor2",
    devices=[
        DeviceDef("pa", "pmos_5v",  0, 16, mirror=True, group="nwell"),
        DeviceDef("pb", "pmos_5v",  0,  8, mirror=True, group="nwell"),
        DeviceDef("na", "nmos_5v", -4, -2, group="pwell"),
        DeviceDef("nb", "nmos_5v",  6, -2, group="pwell"),
    ],
    nets=[
        NetDef("out_np",     ("pb", D), ("na", D), "out",  top_level=True),
        NetDef("out_join_n", ("na", D), ("nb", D), "out"),
        NetDef("in_a",       ("pa", G), ("na", G), "in_a", top_level=True),
        NetDef("in_b",       ("pb", G), ("nb", G), "in_b", top_level=True),
        NetDef("mid_p",      ("pa", D), ("pb", S), "mid"),
        NetDef("vdd",        ("pa", S), ("pa", B), "vdd",  top_level=True),
        NetDef("vdd_body_b", ("pb", B), ("pa", S), "vdd"),
        NetDef("vss_a",      ("na", S), ("na", B), "vss",  top_level=True),
        NetDef("vss_b",      ("nb", S), ("nb", B), "vss"),
        NetDef("vss_join",   ("na", S), ("nb", S), "vss"),
    ],
    unrouted=[],
    spice_pins="vdd vss in_a in_b out",
    spice_body=(
        _p("pa", "mid", "in_a", "vdd", "vdd")
        + _p("pb", "out", "in_b", "mid", "vdd")
        + _n("na", "out", "in_a", "vss", "vss")
        + _n("nb", "out", "in_b", "vss", "vss")
    ),
    extent=22.0,
)

# ---------------------------------------------------------------------------
# 3. Buffer  —  two cascaded inverters (4 devices)
# ---------------------------------------------------------------------------
CIRCUITS["buffer"] = CircuitSpec(
    name="buffer",
    devices=[
        DeviceDef("p1", "pmos_5v",  0, 8, mirror=True, group="nwell"),
        DeviceDef("n1", "nmos_5v",  0, 0, group="pwell"),
        DeviceDef("p2", "pmos_5v", 12, 8, mirror=True, group="nwell"),
        DeviceDef("n2", "nmos_5v", 12, 0, group="pwell"),
    ],
    nets=[
        # inverter 1 output → mid node
        NetDef("mid_inv1",   ("p1", D), ("n1", D), "mid"),
        NetDef("mid_to_p2g", ("n1", D), ("p2", G), "mid"),
        NetDef("mid_to_n2g", ("p2", G), ("n2", G), "mid"),
        # inverter 2 output
        NetDef("out",        ("p2", D), ("n2", D), "out", top_level=True),
        # input
        NetDef("in_p1",      ("p1", G), ("n1", G), "in",  top_level=True),
        # supplies
        NetDef("vdd_1",      ("p1", S), ("p1", B), "vdd", top_level=True),
        NetDef("vdd_2",      ("p2", S), ("p2", B), "vdd"),
        NetDef("vdd_join",   ("p1", S), ("p2", S), "vdd"),
        NetDef("vss_1",      ("n1", S), ("n1", B), "vss", top_level=True),
        NetDef("vss_2",      ("n2", S), ("n2", B), "vss"),
        NetDef("vss_join",   ("n1", S), ("n2", S), "vss"),
    ],
    unrouted=[],
    spice_pins="vdd vss in out",
    spice_body=(
        _p("p1", "mid", "in", "vdd", "vdd")
        + _n("n1", "mid", "in", "vss", "vss")
        + _p("p2", "out", "mid", "vdd", "vdd")
        + _n("n2", "out", "mid", "vss", "vss")
    ),
    extent=22.0,
)

# ---------------------------------------------------------------------------
# 4. Differential pair with active load  —  3 NMOS + 2 PMOS
# ---------------------------------------------------------------------------
CIRCUITS["diffpair"] = CircuitSpec(
    name="diffpair",
    devices=[
        DeviceDef("loadp", "pmos_5v", -12, 26, mirror=True, group="nwell"),
        DeviceDef("loadn", "pmos_5v",  12, 34, mirror=True, group="nwell"),
        DeviceDef("inp",   "nmos_5v", -12,  6, group="pwell_dp"),
        DeviceDef("inn",   "nmos_5v",  12,  6, group="pwell_dp"),
        DeviceDef("tail",  "nmos_5v",   0, -12, group="pwell_tail"),
    ],
    nets=[
        # tail current source
        NetDef("tail_to_inp", ("tail", D), ("inp", S), "tail"),
        NetDef("tail_join",   ("inp",  S), ("inn", S), "tail"),
        # output nodes
        NetDef("outn",        ("inp",  D), ("loadp", D), "outn"),
        NetDef("outp",        ("inn",  D), ("loadn", D), "outp", top_level=True),
        # diode + mirror gate
        NetDef("diode",       ("loadp", D), ("loadp", G), "outn"),
        NetDef("mirror_g",    ("loadp", G), ("loadn", G), "outn"),
        # supplies
        NetDef("vdd_lp",      ("loadp", S), ("loadp", B), "vdd", top_level=True),
        NetDef("vdd_ln",      ("loadn", S), ("loadn", B), "vdd"),
        NetDef("vdd_join",    ("loadp", S), ("loadn", S), "vdd"),
        NetDef("vss_tail",    ("tail",  S), ("tail",  B), "vss", top_level=True),
        NetDef("vss_body_inp",("inp",   B), ("tail",  S), "vss"),
        NetDef("vss_body_inn",("inn",   B), ("tail",  S), "vss"),
    ],
    unrouted=[
        UnroutedDef("vbias", "tail", G),
        UnroutedDef("vin_p", "inp",  G),
        UnroutedDef("vin_n", "inn",  G),
    ],
    spice_pins="vdd vss vbias vin_p vin_n outp",
    spice_body=(
        _n("tail", "tail_node", "vbias", "vss", "vss")
        + _n("inp", "outn", "vin_p", "tail_node", "vss")
        + _n("inn", "outp", "vin_n", "tail_node", "vss")
        + _p("loadp", "outn", "outn", "vdd", "vdd")
        + _p("loadn", "outp", "outn", "vdd", "vdd")
    ),
    extent=30.0,
)

# ---------------------------------------------------------------------------
# 5. Common-source amplifier  —  1 NMOS + 1 PMOS active load
# ---------------------------------------------------------------------------
CIRCUITS["cs_amp"] = CircuitSpec(
    name="cs_amp",
    devices=[
        DeviceDef("pload", "pmos_5v", 0, 10, mirror=True, group="nwell"),
        DeviceDef("nin",   "nmos_5v", 0,  0, group="pwell"),
    ],
    nets=[
        NetDef("out", ("nin", D), ("pload", D), "out", top_level=True),
        NetDef("vdd", ("pload", S), ("pload", B), "vdd", top_level=True),
        NetDef("vss", ("nin",   S), ("nin",   B), "vss", top_level=True),
    ],
    unrouted=[
        UnroutedDef("in",    "nin",   G),
        UnroutedDef("vbias", "pload", G),
    ],
    spice_pins="vdd vss in out vbias",
    spice_body=(
        _n("nin", "out", "in", "vss", "vss")
        + _p("pload", "out", "vbias", "vdd", "vdd")
    ),
    extent=18.0,
)

# ---------------------------------------------------------------------------
# 6. Telescopic cascode  —  2 NMOS + 2 PMOS stacked vertically
# ---------------------------------------------------------------------------
CIRCUITS["telescopic"] = CircuitSpec(
    name="telescopic",
    devices=[
        DeviceDef("ptop",  "pmos_5v", 0, 30, mirror=True, group="nwell"),
        DeviceDef("pcas",  "pmos_5v", 0, 20, mirror=True, group="nwell"),
        DeviceDef("ncas",  "nmos_5v", 0,  6, group="pwell"),
        DeviceDef("nbot",  "nmos_5v", 0, -6, group="pwell"),
    ],
    nets=[
        NetDef("mid_n",        ("nbot", D), ("ncas", S), "mid_n"),
        NetDef("out",          ("ncas", D), ("pcas", D), "out", top_level=True),
        NetDef("mid_p",        ("pcas", S), ("ptop", D), "mid_p"),
        NetDef("vdd",          ("ptop", S), ("ptop", B), "vdd", top_level=True),
        NetDef("vdd_body_cas", ("pcas", B), ("ptop", S), "vdd"),
        NetDef("vss",          ("nbot", S), ("nbot", B), "vss", top_level=True),
        NetDef("vss_body_cas", ("ncas", B), ("nbot", S), "vss"),
    ],
    unrouted=[
        UnroutedDef("vin",    "nbot", G),
        UnroutedDef("vcas_n", "ncas", G),
        UnroutedDef("vcas_p", "pcas", G),
        UnroutedDef("vbias",  "ptop", G),
    ],
    spice_pins="vdd vss vin vcas_n vcas_p vbias out",
    spice_body=(
        _n("nbot", "mid_n", "vin", "vss", "vss")
        + _n("ncas", "out", "vcas_n", "mid_n", "vss")
        + _p("pcas", "out", "vcas_p", "mid_p", "vdd")
        + _p("ptop", "mid_p", "vbias", "vdd", "vdd")
    ),
    extent=25.0,
)

# ---------------------------------------------------------------------------
# 7. Transmission gate  —  1 NMOS + 1 PMOS in parallel
# ---------------------------------------------------------------------------
CIRCUITS["tgate"] = CircuitSpec(
    name="tgate",
    devices=[
        DeviceDef("ppass", "pmos_5v", 0, 8, mirror=True, group="nwell"),
        DeviceDef("npass", "nmos_5v", 0, 0, group="pwell"),
    ],
    nets=[
        NetDef("out_join", ("npass", D), ("ppass", D), "out", top_level=True),
        NetDef("in_join",  ("npass", S), ("ppass", S), "in",  top_level=True),
    ],
    unrouted=[
        UnroutedDef("en",   "npass", G),
        UnroutedDef("en_b", "ppass", G),
        UnroutedDef("vss",  "npass", B),
        UnroutedDef("vdd",  "ppass", B),
    ],
    spice_pins="vss vdd en en_b in out",
    spice_body=(
        _n("npass", "out", "en", "in", "vss")
        + _p("ppass", "out", "en_b", "in", "vdd")
    ),
    extent=18.0,
)

# ---------------------------------------------------------------------------
# 8. 1:2 current mirror  —  3 NMOS in a row
# ---------------------------------------------------------------------------
CIRCUITS["mirror_1to2"] = CircuitSpec(
    name="mirror_1to2",
    devices=[
        DeviceDef("nref",  "nmos_5v",  0, 0, group="pwell"),
        DeviceDef("nout1", "nmos_5v", 10, 0, group="pwell"),
        DeviceDef("nout2", "nmos_5v", 20, 0, group="pwell"),
    ],
    nets=[
        # diode + gate bus
        NetDef("diode",      ("nref",  D), ("nref",  G), "gate", top_level=True),
        NetDef("gate_to_o1", ("nref",  G), ("nout1", G), "gate"),
        NetDef("gate_to_o2", ("nout1", G), ("nout2", G), "gate"),
        # VSS
        NetDef("vss_ref",    ("nref",  S), ("nref",  B), "vss", top_level=True),
        NetDef("vss_o1",     ("nout1", S), ("nout1", B), "vss"),
        NetDef("vss_o2",     ("nout2", S), ("nout2", B), "vss"),
        NetDef("vss_join_1", ("nref",  S), ("nout1", S), "vss"),
        NetDef("vss_join_2", ("nout1", S), ("nout2", S), "vss"),
    ],
    unrouted=[
        UnroutedDef("iout1", "nout1", D),
        UnroutedDef("iout2", "nout2", D),
    ],
    spice_pins="vss gate iout1 iout2",
    spice_body=(
        _n("nref",  "gate",  "gate",  "vss", "vss")
        + _n("nout1", "iout1", "gate", "vss", "vss")
        + _n("nout2", "iout2", "gate", "vss", "vss")
    ),
    extent=25.0,
)

# ---------------------------------------------------------------------------
# 9. Cross-coupled latch  —  2 PMOS + 2 NMOS (routes must cross!)
# ---------------------------------------------------------------------------
CIRCUITS["latch"] = CircuitSpec(
    name="latch",
    devices=[
        DeviceDef("pa", "pmos_5v", -12, 16, mirror=True, group="nwell"),
        DeviceDef("pb", "pmos_5v",  12, 22, mirror=True, group="nwell"),
        DeviceDef("na", "nmos_5v", -12,  0, group="pwell"),
        DeviceDef("nb", "nmos_5v",  12,  6, group="pwell"),
    ],
    nets=[
        # qa and qb nodes
        NetDef("qa_join",   ("pa", D), ("na", D), "qa",  top_level=True),
        NetDef("qb_join",   ("pb", D), ("nb", D), "qb",  top_level=True),
        # cross-coupling  (these must physically cross each other)
        NetDef("cross_a",   ("na", D), ("pb", G), "qa"),   # qa -> gate of pb
        NetDef("cross_b",   ("nb", D), ("pa", G), "qb"),   # qb -> gate of pa
        # supplies
        NetDef("vdd_a",     ("pa", S), ("pa", B), "vdd",  top_level=True),
        NetDef("vdd_b",     ("pb", S), ("pb", B), "vdd"),
        NetDef("vdd_join",  ("pa", S), ("pb", S), "vdd"),
        NetDef("vss_a",     ("na", S), ("na", B), "vss",  top_level=True),
        NetDef("vss_b",     ("nb", S), ("nb", B), "vss"),
        NetDef("vss_join",  ("na", S), ("nb", S), "vss"),
    ],
    unrouted=[
        UnroutedDef("in",   "na", G),
        UnroutedDef("in_b", "nb", G),
    ],
    spice_pins="vdd vss in in_b qa qb",
    spice_body=(
        _p("pa", "qa", "qb", "vdd", "vdd")
        + _p("pb", "qb", "qa", "vdd", "vdd")
        + _n("na", "qa", "in", "vss", "vss")
        + _n("nb", "qb", "in_b", "vss", "vss")
    ),
    extent=25.0,
)

# ---------------------------------------------------------------------------
# 10. Source follower  —  2 NMOS (input + current-source bias)
# ---------------------------------------------------------------------------
CIRCUITS["source_follower"] = CircuitSpec(
    name="source_follower",
    devices=[
        DeviceDef("nin",   "nmos_5v", 6, 14, group="pwell"),
        DeviceDef("nbias", "nmos_5v", 0,  0, group="pwell"),
    ],
    nets=[
        NetDef("out", ("nin", S), ("nbias", D), "out", top_level=True),
        NetDef("vss",       ("nbias", S), ("nbias", B), "vss", top_level=True),
        NetDef("vss_body",  ("nin",   B), ("nbias", S), "vss"),
    ],
    unrouted=[
        UnroutedDef("in",    "nin",   G),
        UnroutedDef("vbias", "nbias", G),
        UnroutedDef("vdd",   "nin",   D),
    ],
    spice_pins="vdd vss in out vbias",
    spice_body=(
        _n("nin",   "vdd", "in", "out", "vss")
        + _n("nbias", "out", "vbias", "vss", "vss")
    ),
    extent=18.0,
)


# ---------------------------------------------------------------------------
# Generic circuit builder
# ---------------------------------------------------------------------------

def build_circuit(spec: CircuitSpec, placed: bool = False) -> tuple[Component, list]:
    """Build a circuit from *spec*, returning (component, route_results).

    When *placed* is True, positions are chosen by the DoRoutes placement
    engine; otherwise the manual coordinates in each DeviceDef are used.
    """
    pdk = get_active_pdk()
    sfx = "_placed" if placed else ""
    c = Component(f"test_{spec.name}{sfx}")

    # ---- 1. Create device instances ----
    refs: dict[str, object] = {}
    for d in spec.devices:
        ref = c.add_ref(
            pdk.get_component(d.dev_type, instance_name=d.name),
            name=d.name,
        )
        refs[d.name] = ref

    # ---- 2. Build RouteNetSpec list (pre-placement port refs) ----
    def _port(inst, suffix):
        return refs[inst].ports[f"{inst}_{suffix}"]

    all_nets = [
        RouteNetSpec(
            name=n.name,
            start=_port(n.start[0], n.start[1]),
            stop=_port(n.stop[0], n.stop[1]),
            port_name_prefix=n.prefix,
            is_top_level_pin=n.top_level,
        )
        for n in spec.nets
    ]

    # ---- 3. Placement ----
    report = None
    if placed:
        inv_dbu = 1.0 / c.kcl.dbu
        nd = len(spec.devices)
        bbox_w = max(30.0, nd * 12.0)
        bbox_h = max(30.0, nd * 12.0)
        bbox_dbu = (
            int(bbox_h * inv_dbu),          # north
            int(bbox_w / 2 * inv_dbu),      # east
            0,                               # south
            int(-bbox_w / 2 * inv_dbu),     # west
        )
        report = place_auto(
            c,
            instances=[{"name": d.name, "group": d.group} for d in spec.devices],
            nets=all_nets,
            mode="tiling",
            objective={
                "hpwl_weight": 1.0,
                "congestion_weight": 0.8,
                "density_weight": 0.5,
                "displacement_weight": 0.1,
                "grouping_weight": 0.5,
            },
            constraints={
                "min_spacing": int(4.0 * inv_dbu),
                "density_target": 0.5,
                "bbox": bbox_dbu,
            },
            iterations=50,
        )
        for d in spec.devices:
            if d.mirror:
                refs[d.name].mirror_y(refs[d.name].dcenter[1])
    else:
        for d in spec.devices:
            refs[d.name].move((d.x, d.y))
            if d.mirror:
                refs[d.name].mirror_y(refs[d.name].dcenter[1])

    # ---- 4. Add ports (AFTER all movement) ----
    for d in spec.devices:
        c.add_ports(refs[d.name])

    # ---- 5. Refresh nets after placement/movement ----
    all_nets = [
        RouteNetSpec(
            name=net.name,
            start=c.ports[net.start.name],
            stop=c.ports[net.stop.name],
            port_name_prefix=net.port_name_prefix,
            is_top_level_pin=net.is_top_level_pin,
        )
        for net in all_nets
    ]

    # ---- 6. Routing area marker ----
    # Compute extent from device bounding boxes + margin
    margin = 12.0
    ext = spec.extent
    for d in spec.devices:
        try:
            bb = refs[d.name].dbbox()
            ext = max(ext,
                      abs(bb.left) + margin, abs(bb.right) + margin,
                      abs(bb.bottom) + margin, abs(bb.top) + margin)
        except Exception:
            pass

    c.add_polygon(
        [(-ext, -ext), (ext, -ext), (ext, ext), (-ext, ext)],
        layer=(235, 4),
    )

    # ---- 7. Route ----
    route_kw = dict(
        config=SKY130_CONFIG,
        grid_unit=1.0,
        dynamic_width=False,
        layers_to_avoid=[(68, 20), (69, 20)],
        add_segment_ports=True,
        require_all=False,
        deterministic=True,
        clearance=0.14,
        clearance_ladder=(0.14,),
    )
    if report is not None:
        route_kw["placement_report"] = report

    c, routed_map = route_nets_deterministic_copy(c, nets=all_nets, **route_kw)
    # routed_map is a dict {net_name: [ports]} for successfully routed nets
    routed_names = set(routed_map.keys()) if isinstance(routed_map, dict) else set()

    # ---- 8. Label unrouted pins ----
    if spec.unrouted:
        pin_specs = []
        for u in spec.unrouted:
            pname = f"{u.inst}_{u.port_suffix}"
            try:
                port = c.insts[u.inst].ports[pname]
                pin_specs.append(PinLabelSpec(pin_name=u.pin_name, port=port))
            except (KeyError, AttributeError):
                pass
        if pin_specs:
            label_unrouted_pins(c, pin_specs, config=SKY130_CONFIG)

    # ---- 9. Instance labels ----
    for d in spec.devices:
        try:
            add_instance_label(c, c.insts[d.name], instance_name=d.name)
        except (KeyError, AttributeError):
            pass

    return c, routed_names


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

def run_one(
    spec: CircuitSpec,
    placed: bool,
    out_dir: Path,
    skip_lvs: bool = True,
    show: bool = True,
) -> dict:
    """Build one circuit variant, write GDS, show in KLayout, return result summary."""
    variant = "placed" if placed else "manual"
    tag = f"{spec.name} ({variant})"
    t0 = time.time()
    try:
        c, routed_names = build_circuit(spec, placed=placed)
        n_nets = len(spec.nets)
        n_ok = len(routed_names)
        elapsed = time.time() - t0

        c.flatten()
        c.remove_layers(layers=[(235, 4)])
        sfx = "_placed" if placed else ""
        gds_name = f"test_{spec.name}{sfx}.gds"
        gds_path = out_dir / gds_name
        c.write_gds(str(gds_path), with_metadata=False)

        # Save layout image (light mode)
        sub = "placed" if placed else "manual"
        img_dir = out_dir / "images" / sub
        img_dir.mkdir(parents=True, exist_ok=True)
        img_name = f"test_{spec.name}.png"
        pretty = spec.name.replace("_", " ").title()
        title = f"{pretty} ({'Auto-Placed' if placed else 'Manual'})"
        try:
            import matplotlib
            matplotlib.use("Agg")
            fig = _plot_light(c, title=title)
            if fig is not None:
                fig.savefig(str(img_dir / img_name), dpi=200, bbox_inches="tight")
                import matplotlib.pyplot as plt
                plt.close(fig)
        except Exception as e:
            print(f"    Image save error: {e}")

        # Show in KLayout so the user can inspect each design
        if show:
            c.show()

        status = "OK" if n_ok == n_nets else "PARTIAL"
        print(f"  {tag:30s}  {status:7s}  {n_ok}/{n_nets} nets   {elapsed:.1f}s   -> {gds_name}")

        if not skip_lvs and n_ok == n_nets:
            try:
                from sky130.examples.lvs_magic_utils import run_lvs

                cell_name = f"test_{spec.name}{sfx}"
                run_lvs(gds_path, cell_name, spec.spice(placed=placed))
            except Exception as e:
                print(f"    LVS error: {e}")

        return {"name": spec.name, "variant": variant, "status": status,
                "routed": n_ok, "total": n_nets, "time": elapsed}
    except Exception:
        elapsed = time.time() - t0
        print(f"  {tag:30s}  FAILED   {elapsed:.1f}s")
        traceback.print_exc()
        return {"name": spec.name, "variant": variant, "status": "FAILED",
                "routed": 0, "total": len(spec.nets), "time": elapsed}


def main():
    parser = argparse.ArgumentParser(description="DoRoutes routing stress test.")
    parser.add_argument("--circuit", nargs="*", default=None,
                        help="Circuit name(s) to run. Default: all.")
    parser.add_argument("--placed", action="store_true",
                        help="Also generate auto-placed variants alongside manual.")
    parser.add_argument("--out-dir", default="./results/stress",
                        help="Output directory for GDS files.")
    parser.add_argument("--skip-lvs", action="store_true", default=True,
                        help="Skip LVS (default: skip).")
    parser.add_argument("--run-lvs", action="store_true",
                        help="Run LVS on fully-routed circuits.")
    parser.add_argument("--no-show", action="store_true",
                        help="Skip c.show() KLayout visualization.")
    args = parser.parse_args()

    if args.run_lvs:
        args.skip_lvs = False

    show = not args.no_show

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    names = args.circuit if args.circuit else list(CIRCUITS.keys())

    label = f"{len(names)} circuits"
    if args.placed:
        label += " (manual + placed)"

    print(f"=== DoRoutes Stress Test ({label}) ===")
    print(f"Output: {out_dir.resolve()}\n")

    all_results = []
    for name in names:
        if name not in CIRCUITS:
            print(f"  Unknown circuit: {name}  (available: {', '.join(CIRCUITS)})")
            continue
        spec = CIRCUITS[name]
        all_results.append(run_one(spec, placed=False, out_dir=out_dir, skip_lvs=args.skip_lvs, show=show))
        if args.placed:
            all_results.append(run_one(spec, placed=True, out_dir=out_dir, skip_lvs=args.skip_lvs, show=show))

    # Summary
    print(f"\n{'='*60}")
    ok = sum(1 for r in all_results if r["status"] == "OK")
    partial = sum(1 for r in all_results if r["status"] == "PARTIAL")
    failed = sum(1 for r in all_results if r["status"] == "FAILED")
    total_nets = sum(r["total"] for r in all_results)
    routed_nets = sum(r["routed"] for r in all_results)
    total_time = sum(r["time"] for r in all_results)
    print(f"Circuits:  {ok} OK  |  {partial} PARTIAL  |  {failed} FAILED")
    print(f"Nets:      {routed_nets}/{total_nets} routed")
    print(f"Time:      {total_time:.1f}s total")


if __name__ == "__main__":
    main()
