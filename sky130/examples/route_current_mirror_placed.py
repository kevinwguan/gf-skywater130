"""Current mirror example with automatic placement via DoRoutes.

Same circuit and routing as ``route_current_mirror.py`` but instead of
hard-coded grid offsets, the four NMOS positions are chosen by the
DoRoutes placement engine (``doroutes.placement.place_auto``).
After placement the nets are routed using ``doroutes.multilayer``.
"""

import argparse
import sys
from pathlib import Path

# Load environment variables from .env file (must be before doroutes import)
from dotenv import load_dotenv
load_dotenv(Path(__file__).parent.parent.parent / ".env")

sys.path.append(str(Path(__file__).parent.parent.parent))

from gdsfactory.component import Component
import sky130
from gdsfactory.pdk import get_active_pdk
from gdsfactory.add_pins import add_instance_label

from doroutes.placement import place_auto
from doroutes.multilayer import PinLabelSpec, RouteNetSpec, label_unrouted_pins, route_multilayer_3d, route_nets_deterministic_copy
from doroutes.multilayer.engine_multinet import _precompute_port_geometries
from sky130.routing_utils import SKY130_CONFIG


CURRENT_MIRROR_SCHEMATIC = """\
* Schematic netlist for LVS: test_current_mirror_placed
.subckt test_current_mirror_placed vss gate_bot gate_top nref_top_d nout_top_d
Xref_bot stack_ref gate_bot vss vss sky130_fd_pr__nfet_g5v0d10v5 w=0.75 l=0.5
Xref_top nref_top_d gate_top stack_ref vss sky130_fd_pr__nfet_g5v0d10v5 w=0.75 l=0.5
Xout_bot stack_out gate_bot vss vss sky130_fd_pr__nfet_g5v0d10v5 w=0.75 l=0.5
Xout_top nout_top_d gate_top stack_out vss sky130_fd_pr__nfet_g5v0d10v5 w=0.75 l=0.5
.ends test_current_mirror_placed
"""


# ---------------------------------------------------------------------------
# Main current mirror function
# ---------------------------------------------------------------------------

def test_current_mirror_placed(
    component_name: str = "test_current_mirror_placed",
    add_segment_ports: bool = True,
    routing_half_extent: float = 30.0,
    grid_unit_um: float = 1.0,
    dynamic_width: bool = True,
) -> Component:
    pdk = get_active_pdk()
    c = Component(component_name)

    # 4T NMOS cascode mirror — all instances start at origin
    nmos_ref_bot = c.add_ref(
        pdk.get_component("nmos_5v", instance_name="nmos_ref_bot"), name="nmos_ref_bot"
    )
    nmos_ref_top = c.add_ref(
        pdk.get_component("nmos_5v", instance_name="nmos_ref_top"), name="nmos_ref_top"
    )
    nmos_out_bot = c.add_ref(
        pdk.get_component("nmos_5v", instance_name="nmos_out_bot"), name="nmos_out_bot"
    )
    nmos_out_top = c.add_ref(
        pdk.get_component("nmos_5v", instance_name="nmos_out_top"), name="nmos_out_top"
    )

    # ---- Define all nets ONCE (used for both placement and routing) ----
    all_nets = [
        RouteNetSpec(
            name="stack_ref",
            start=nmos_ref_bot.ports["nmos_ref_bot_DRAIN"],
            stop=nmos_ref_top.ports["nmos_ref_top_SOURCE"],
            port_name_prefix="stack_ref",
        ),
        RouteNetSpec(
            name="stack_out",
            start=nmos_out_bot.ports["nmos_out_bot_DRAIN"],
            stop=nmos_out_top.ports["nmos_out_top_SOURCE"],
            port_name_prefix="stack_out",
        ),
        RouteNetSpec(
            name="gate_bot",
            start=nmos_ref_bot.ports["nmos_ref_bot_GATE"],
            stop=nmos_out_bot.ports["nmos_out_bot_GATE"],
            port_name_prefix="gate_bot",
            is_top_level_pin=True,
        ),
        RouteNetSpec(
            name="gate_top",
            start=nmos_ref_top.ports["nmos_ref_top_GATE"],
            stop=nmos_out_top.ports["nmos_out_top_GATE"],
            port_name_prefix="gate_top",
            is_top_level_pin=True,
        ),
        RouteNetSpec(
            name="vss_src_join",
            start=nmos_ref_bot.ports["nmos_ref_bot_SOURCE"],
            stop=nmos_out_bot.ports["nmos_out_bot_SOURCE"],
            port_name_prefix="vss",
            is_top_level_pin=True,
        ),
        RouteNetSpec(
            name="vss_body_ref_bot",
            start=nmos_ref_bot.ports["nmos_ref_bot_SOURCE"],
            stop=nmos_ref_bot.ports["nmos_ref_bot_BODY"],
            port_name_prefix="vss",
            is_top_level_pin=True,
        ),
        RouteNetSpec(
            name="vss_body_out_bot",
            start=nmos_out_bot.ports["nmos_out_bot_SOURCE"],
            stop=nmos_out_bot.ports["nmos_out_bot_BODY"],
            port_name_prefix="vss",
            is_top_level_pin=True,
        ),
        RouteNetSpec(
            name="vss_body_ref_top",
            start=nmos_ref_bot.ports["nmos_ref_bot_SOURCE"],
            stop=nmos_ref_top.ports["nmos_ref_top_BODY"],
            port_name_prefix="vss",
            is_top_level_pin=True,
        ),
        RouteNetSpec(
            name="vss_body_out_top",
            start=nmos_out_bot.ports["nmos_out_bot_SOURCE"],
            stop=nmos_out_top.ports["nmos_out_top_BODY"],
            port_name_prefix="vss",
            is_top_level_pin=True,
        ),
    ]

    # ---- AUTOMATIC PLACEMENT via DoRoutes ----
    print("Placing 4T cascode current mirror with DoRoutes placement engine...")

    inv_dbu = 1.0 / c.kcl.dbu

    # Placement bbox: narrow to force 2-column topology, tall for vertical stacking
    bb = nmos_ref_bot.dbbox()
    dev_w = bb.right - bb.left
    dev_h = bb.top - bb.bottom
    bbox_w = dev_w * 2 + 20.0   # ~2 devices wide + routing room
    bbox_h = dev_h * 2 + 35.0   # ~2 devices tall + generous vertical room
    bbox_dbu = (
        int(bbox_h * inv_dbu),          # north
        int(bbox_w / 2 * inv_dbu),      # east
        0,                               # south
        int(-bbox_w / 2 * inv_dbu),     # west
    )

    report = place_auto(
        c,
        instances=[
            {"name": "nmos_ref_bot", "group": "ref_branch"},
            {"name": "nmos_ref_top", "group": "ref_branch"},
            {"name": "nmos_out_bot", "group": "out_branch"},
            {"name": "nmos_out_top", "group": "out_branch"},
        ],
        nets=all_nets,
        mode="tiling",
        objective={
            "hpwl_weight": 1.0,
            "congestion_weight": 0.8,
            "density_weight": 0.5,
            "displacement_weight": 0.1,
            "grouping_weight": 1.0,
        },
        constraints={
            "min_spacing": int(10.0 * inv_dbu),
            "density_target": 0.4,
            "bbox": bbox_dbu,
        },
        iterations=60,
    )
    print(f"[PLACE] mode={report['mode_used']}  legalized={report['legalized']}  hpwl={report['hpwl']:.0f} dbu")
    for name, (x, y, _orient) in report["placements"].items():
        print(f"[PLACE]   {name:15s} -> ({x / inv_dbu:+.2f}, {y / inv_dbu:+.2f}) um")

    # Important: add ports ONLY after finished moving instances
    c.add_ports(nmos_ref_bot)
    c.add_ports(nmos_ref_top)
    c.add_ports(nmos_out_bot)
    c.add_ports(nmos_out_top)

    # Refresh nets with post-placement port positions
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

    # Pre-compute port geometries on pristine component before any routing.
    geom_cache = _precompute_port_geometries(c, all_nets, SKY130_CONFIG, 0.14)

    # Add routing area marker
    routing_area_layer = (235, 4)
    c.add_polygon(
        [
            (-routing_half_extent, -routing_half_extent),
            (routing_half_extent, -routing_half_extent),
            (routing_half_extent, routing_half_extent),
            (-routing_half_extent, routing_half_extent),
        ],
        layer=routing_area_layer,
    )

    layers_to_avoid = [(68, 20), (69, 20)]

    print("Routing 4T cascode current mirror with 3D multi-layer A* (M1=H, M2=V with via transitions)...")

    # Route all nets batch-wise.
    c, _ = route_nets_deterministic_copy(
        c,
        nets=all_nets,
        config=SKY130_CONFIG,
        grid_unit=grid_unit_um,
        dynamic_width=dynamic_width,
        layers_to_avoid=layers_to_avoid,
        add_segment_ports=add_segment_ports,
        require_all=False,
        deterministic=True,
        placement_report=report,
        clearance=0.14,
        clearance_ladder=(0.14,),
        geom_cache=geom_cache,
    )

    # Label unrouted top-level pins (single-port, no routing needed)
    label_unrouted_pins(c, [
        PinLabelSpec(pin_name="nref_top_d", port=c.insts["nmos_ref_top"].ports["nmos_ref_top_DRAIN"]),
        PinLabelSpec(pin_name="nout_top_d", port=c.insts["nmos_out_top"].ports["nmos_out_top_DRAIN"]),
    ], config=SKY130_CONFIG, debug=True)

    # Reacquire instances after routing attempts to avoid stale reference handles.
    add_instance_label(c, c.insts["nmos_ref_bot"], instance_name="nmos_ref_bot")
    add_instance_label(c, c.insts["nmos_ref_top"], instance_name="nmos_ref_top")
    add_instance_label(c, c.insts["nmos_out_bot"], instance_name="nmos_out_bot")
    add_instance_label(c, c.insts["nmos_out_top"], instance_name="nmos_out_top")

    return c


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Route current mirror example with automatic placement (headless by default)."
    )
    parser.add_argument(
        "--skip-lvs",
        action="store_true",
        help="Skip LVS run after writing GDS.",
    )
    parser.add_argument(
        "--out-gds",
        default="./results/test_current_mirror_placed.gds",
        help="Output GDS path.",
    )
    args = parser.parse_args()

    c = test_current_mirror_placed(
        component_name="test_current_mirror_placed_inst",
        add_segment_ports=True,
        routing_half_extent=25.0,
        grid_unit_um=1.0,
        dynamic_width=True,
    )
    c.flatten()
    c.remove_layers(layers=[(235, 4)])
    c.pprint_ports()
    c.show()
    gds_path = Path(args.out_gds)
    gds_path.parent.mkdir(parents=True, exist_ok=True)
    c.write_gds(str(gds_path), with_metadata=False)
    print(f"[OK] Wrote {gds_path}")
    # Save layout image
    try:
        from sky130.examples.stress_test import _plot_light
        import matplotlib
        matplotlib.use("Agg")
        img_dir = gds_path.parent / "images"
        img_dir.mkdir(parents=True, exist_ok=True)
        fig = _plot_light(c)
        if fig is not None:
            img_path = img_dir / (gds_path.stem + ".png")
            fig.savefig(str(img_path), dpi=200, bbox_inches="tight")
            import matplotlib.pyplot as plt
            plt.close(fig)
            print(f"[OK] Saved image {img_path}")
    except Exception as e:
        print(f"[WARN] Image save error: {e}")
    if args.skip_lvs:
        print("[INFO] Skipping LVS (--skip-lvs)")
    else:
        from sky130.examples.lvs_graph_checks import check_current_mirror_layout_graph
        from sky130.examples.lvs_magic_utils import run_lvs

        run_lvs(
            gds_path,
            "test_current_mirror_placed",
            CURRENT_MIRROR_SCHEMATIC,
            graph_check_fn=check_current_mirror_layout_graph,
        )
