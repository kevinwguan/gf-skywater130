"""Inverter example with automatic placement via DoRoutes.

Same circuit and routing as ``route_inverter.py`` but instead of
hard-coded ``instance.move()`` offsets, the PMOS and NMOS positions are
chosen by the DoRoutes placement engine (``doroutes.placement.place_auto``).
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
from doroutes.multilayer import RouteNetSpec, route_nets_deterministic_copy
from sky130.routing_utils import SKY130_CONFIG


INVERTER_SCHEMATIC = """\
* Schematic netlist for LVS: test_inverter_placed
.subckt test_inverter_placed vdd vss in out
X0 out in vss vss sky130_fd_pr__nfet_g5v0d10v5 w=0.75 l=0.5
X1 out in vdd vdd sky130_fd_pr__pfet_g5v0d10v5 w=0.75 l=0.5
.ends test_inverter_placed
"""


# ---------------------------------------------------------------------------
# Main inverter function
# ---------------------------------------------------------------------------

def test_inverter_placed(
    component_name: str = "test_inverter_placed",
    add_segment_ports: bool = True,
    routing_half_extent: float = 15.0,
    grid_unit_um: float = 1.0,
    mirror_pmos: bool = True,
) -> Component:
    pdk = get_active_pdk()
    c = Component(component_name)

    # Create instances — both start at origin (no manual move!)
    pmos = c.add_ref(pdk.get_component("pmos_5v", instance_name="pmos"), name="pmos")
    nmos = c.add_ref(pdk.get_component("nmos_5v", instance_name="nmos"), name="nmos")

    # ---- Define all nets ONCE (used for both placement and routing) ----
    all_nets = [
        RouteNetSpec(
            name="out",
            start=pmos.ports["pmos_DRAIN"],
            stop=nmos.ports["nmos_DRAIN"],
            port_name_prefix="out",
            is_top_level_pin=True,
        ),
        RouteNetSpec(
            name="in",
            start=pmos.ports["pmos_GATE"],
            stop=nmos.ports["nmos_GATE"],
            port_name_prefix="in",
            is_top_level_pin=True,
        ),
        RouteNetSpec(
            name="vdd",
            start=pmos.ports["pmos_SOURCE"],
            stop=pmos.ports["pmos_BODY"],
            port_name_prefix="vdd",
            is_top_level_pin=True,
        ),
        RouteNetSpec(
            name="vss",
            start=nmos.ports["nmos_SOURCE"],
            stop=nmos.ports["nmos_BODY"],
            port_name_prefix="vss",
            is_top_level_pin=True,
        ),
    ]

    # ---- AUTOMATIC PLACEMENT via DoRoutes ----
    print("Placing PMOS/NMOS with DoRoutes placement engine...")

    inv_dbu = 1.0 / c.kcl.dbu
    # Compute bbox from device sizes
    pbb = pmos.dbbox()
    nbb = nmos.dbbox()
    max_w = max(pbb.right - pbb.left, nbb.right - nbb.left)
    total_h = (pbb.top - pbb.bottom) + (nbb.top - nbb.bottom)
    bbox_half_w = max_w / 2 + 1.0
    bbox_h = total_h + 8.0 + 0.5
    bbox_dbu = (
        int(bbox_h * inv_dbu),
        int(bbox_half_w * inv_dbu),
        0,
        int(-bbox_half_w * inv_dbu),
    )

    report = place_auto(
        c,
        instances=[
            {"name": "pmos", "group": "nwell"},
            {"name": "nmos", "group": "pwell"},
        ],
        nets=all_nets,
        mode="tiling",
        objective={
            "hpwl_weight": 1.0,
            "congestion_weight": 0.5,
            "density_weight": 0.3,
            "displacement_weight": 0.1,
            "grouping_weight": 0.5,
        },
        constraints={
            "min_spacing": int(2.0 * inv_dbu),
            "density_target": 0.7,
            "bbox": bbox_dbu,
        },
        iterations=40,
    )
    print(f"[PLACE] mode={report['mode_used']}  legalized={report['legalized']}  hpwl={report['hpwl']:.0f} dbu")
    for name, (x, y, _orient) in report["placements"].items():
        print(f"[PLACE]   {name:5s} -> ({x / inv_dbu:+.2f}, {y / inv_dbu:+.2f}) um")

    # Mirror PMOS (same as original example) after engine placement
    if mirror_pmos:
        pmos.mirror_y(pmos.dcenter[1])

    # Important: add ports ONLY after finished moving instances
    c.add_ports(pmos)
    c.add_ports(nmos)

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

    # Add routing area markers to ensure grid extends beyond the obstacle
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

    # Layers to avoid - ALL metal on these layers including device metal
    layers_to_avoid = [(68, 20), (69, 20)]

    print("Routing with 3D multi-layer A* (M1=H, M2=V with via transitions)...")

    c, _ = route_nets_deterministic_copy(
        c,
        nets=all_nets,
        config=SKY130_CONFIG,
        grid_unit=grid_unit_um,
        dynamic_width=True,
        layers_to_avoid=layers_to_avoid,
        add_segment_ports=add_segment_ports,
        require_all=True,
        deterministic=True,
        placement_report=report,
    )

    # Reacquire instances after routing attempts to avoid stale reference handles.
    add_instance_label(c, c.insts["pmos"], instance_name="pmos")
    add_instance_label(c, c.insts["nmos"], instance_name="nmos")

    return c


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Route inverter example with automatic placement (headless by default)."
    )
    parser.add_argument(
        "--skip-lvs",
        action="store_true",
        help="Skip LVS run after writing GDS.",
    )
    parser.add_argument(
        "--out-gds",
        default="./results/test_inverter_placed.gds",
        help="Output GDS path.",
    )
    args = parser.parse_args()

    c = test_inverter_placed(
        component_name="test_inverter_placed_inst",
        add_segment_ports=True,
        mirror_pmos=True,
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
        from sky130.examples.lvs_magic_utils import run_lvs

        run_lvs(gds_path, "test_inverter_placed", INVERTER_SCHEMATIC)
