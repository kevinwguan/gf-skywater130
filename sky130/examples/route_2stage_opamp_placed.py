"""2-stage op-amp example with automatic placement via DoRoutes.

Same circuit and routing as ``route_2stage_opamp.py`` but instead of
hard-coded floorplan offsets, the nine device positions are chosen by
the DoRoutes placement engine (``doroutes.placement.place_auto``).
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


OPAMP_SCHEMATIC = """\
* Schematic netlist for LVS: test_2stage_opamp_placed
.subckt test_2stage_opamp_placed vdd vss vin_p vin_n stage2_out
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
.ends test_2stage_opamp_placed
"""


# ---------------------------------------------------------------------------
# Main opamp function
# ---------------------------------------------------------------------------

def test_2stage_opamp_placed(
    component_name: str = "test_2stage_opamp_placed",
    add_segment_ports: bool = True,
    routing_half_extent: float = 40.0,
    grid_unit_um: float = 1.0,
    dynamic_width: bool = True,
) -> Component:
    pdk = get_active_pdk()
    c = Component(component_name)

    # Stage 1: NMOS differential pair + PMOS active loads + NMOS tail source.
    nmos_in_p = c.add_ref(pdk.get_component("nmos_5v", instance_name="nmos_in_p"), name="nmos_in_p")
    nmos_in_n = c.add_ref(pdk.get_component("nmos_5v", instance_name="nmos_in_n"), name="nmos_in_n")
    pmos_load_p = c.add_ref(pdk.get_component("pmos_5v", instance_name="pmos_load_p"), name="pmos_load_p")
    pmos_load_n = c.add_ref(pdk.get_component("pmos_5v", instance_name="pmos_load_n"), name="pmos_load_n")
    nmos_tail = c.add_ref(pdk.get_component("nmos_5v", instance_name="nmos_tail"), name="nmos_tail")

    # Stage 2: common-source gain stage + PMOS load.
    nmos_stage2 = c.add_ref(pdk.get_component("nmos_5v", instance_name="nmos_stage2"), name="nmos_stage2")
    pmos_stage2_load = c.add_ref(
        pdk.get_component("pmos_5v", instance_name="pmos_stage2_load"), name="pmos_stage2_load"
    )

    # Bias helper devices.
    nmos_bias_ref = c.add_ref(pdk.get_component("nmos_5v", instance_name="nmos_bias_ref"), name="nmos_bias_ref")
    pmos_bias_ref = c.add_ref(pdk.get_component("pmos_5v", instance_name="pmos_bias_ref"), name="pmos_bias_ref")

    # ---- Define all nets ONCE (used for both placement and routing) ----
    # Critical nets are pre-routed individually with fixed-width fallback.
    critical_nets = [
        RouteNetSpec(
            name="vss_join_core",
            start=nmos_tail.ports["nmos_tail_SOURCE"],
            stop=nmos_stage2.ports["nmos_stage2_SOURCE"],
            port_name_prefix="vss",
            is_top_level_pin=True,
        ),
        RouteNetSpec(
            name="tail_to_inp",
            start=nmos_tail.ports["nmos_tail_DRAIN"],
            stop=nmos_in_p.ports["nmos_in_p_SOURCE"],
            port_name_prefix="tail_to_inp",
        ),
        RouteNetSpec(
            name="tail_to_inn",
            start=nmos_tail.ports["nmos_tail_DRAIN"],
            stop=nmos_in_n.ports["nmos_in_n_SOURCE"],
            port_name_prefix="tail_to_inn",
        ),
        RouteNetSpec(
            name="v1_to_stage2",
            start=nmos_in_n.ports["nmos_in_n_DRAIN"],
            stop=nmos_stage2.ports["nmos_stage2_GATE"],
            port_name_prefix="v1_to_stage2",
        ),
        RouteNetSpec(
            name="tail_bias",
            start=nmos_tail.ports["nmos_tail_GATE"],
            stop=nmos_bias_ref.ports["nmos_bias_ref_GATE"],
            port_name_prefix="tail_bias",
        ),
        RouteNetSpec(
            name="pbias_d_to_vdd",
            start=pmos_bias_ref.ports["pmos_bias_ref_DRAIN"],
            stop=pmos_bias_ref.ports["pmos_bias_ref_SOURCE"],
            port_name_prefix="vdd",
            is_top_level_pin=True,
        ),
    ]

    # Remaining nets routed batch-wise.
    remaining_nets = [
        RouteNetSpec(
            name="vss_join_bias",
            start=nmos_tail.ports["nmos_tail_SOURCE"],
            stop=nmos_bias_ref.ports["nmos_bias_ref_SOURCE"],
            port_name_prefix="vss",
            is_top_level_pin=True,
        ),
        RouteNetSpec(
            name="vss_body_tail",
            start=nmos_tail.ports["nmos_tail_SOURCE"],
            stop=nmos_tail.ports["nmos_tail_BODY"],
            port_name_prefix="vss",
            is_top_level_pin=True,
        ),
        RouteNetSpec(
            name="vss_body_stage2",
            start=nmos_stage2.ports["nmos_stage2_SOURCE"],
            stop=nmos_stage2.ports["nmos_stage2_BODY"],
            port_name_prefix="vss",
            is_top_level_pin=True,
        ),
        RouteNetSpec(
            name="vss_body_in_p",
            start=nmos_in_p.ports["nmos_in_p_SOURCE"],
            stop=nmos_in_p.ports["nmos_in_p_BODY"],
            port_name_prefix="vss",
            is_top_level_pin=True,
        ),
        RouteNetSpec(
            name="vss_body_in_n",
            start=nmos_in_n.ports["nmos_in_n_SOURCE"],
            stop=nmos_in_n.ports["nmos_in_n_BODY"],
            port_name_prefix="vss",
            is_top_level_pin=True,
        ),
        RouteNetSpec(
            name="vss_body_bias_ref",
            start=nmos_bias_ref.ports["nmos_bias_ref_SOURCE"],
            stop=nmos_bias_ref.ports["nmos_bias_ref_BODY"],
            port_name_prefix="vss",
            is_top_level_pin=True,
        ),
        RouteNetSpec(
            name="vdd_join_stage2_to_bias",
            start=pmos_stage2_load.ports["pmos_stage2_load_SOURCE"],
            stop=pmos_bias_ref.ports["pmos_bias_ref_SOURCE"],
            port_name_prefix="vdd",
            is_top_level_pin=True,
        ),
        RouteNetSpec(
            name="vdd_join_loads",
            start=pmos_load_p.ports["pmos_load_p_SOURCE"],
            stop=pmos_load_n.ports["pmos_load_n_SOURCE"],
            port_name_prefix="vdd",
            is_top_level_pin=True,
        ),
        RouteNetSpec(
            name="vdd_join_loads_to_stage2",
            start=pmos_load_p.ports["pmos_load_p_SOURCE"],
            stop=pmos_stage2_load.ports["pmos_stage2_load_SOURCE"],
            port_name_prefix="vdd",
            is_top_level_pin=True,
        ),
        RouteNetSpec(
            name="vdd_body_stage2_load",
            start=pmos_stage2_load.ports["pmos_stage2_load_SOURCE"],
            stop=pmos_stage2_load.ports["pmos_stage2_load_BODY"],
            port_name_prefix="vdd",
            is_top_level_pin=True,
        ),
        RouteNetSpec(
            name="vdd_body_load_p",
            start=pmos_load_p.ports["pmos_load_p_SOURCE"],
            stop=pmos_load_p.ports["pmos_load_p_BODY"],
            port_name_prefix="vdd",
            is_top_level_pin=True,
        ),
        RouteNetSpec(
            name="vdd_body_load_n",
            start=pmos_load_n.ports["pmos_load_n_SOURCE"],
            stop=pmos_load_n.ports["pmos_load_n_BODY"],
            port_name_prefix="vdd",
            is_top_level_pin=True,
        ),
        RouteNetSpec(
            name="vdd_body_bias_ref",
            start=pmos_bias_ref.ports["pmos_bias_ref_SOURCE"],
            stop=pmos_bias_ref.ports["pmos_bias_ref_BODY"],
            port_name_prefix="vdd",
            is_top_level_pin=True,
        ),
        RouteNetSpec(
            name="stage1_node_p",
            start=nmos_in_p.ports["nmos_in_p_DRAIN"],
            stop=pmos_load_p.ports["pmos_load_p_DRAIN"],
            port_name_prefix="stage1_node_p",
        ),
        RouteNetSpec(
            name="stage1_node_n",
            start=nmos_in_n.ports["nmos_in_n_DRAIN"],
            stop=pmos_load_n.ports["pmos_load_n_DRAIN"],
            port_name_prefix="stage1_node_n",
        ),
        RouteNetSpec(
            name="load_bias_pair",
            start=pmos_load_p.ports["pmos_load_p_GATE"],
            stop=pmos_load_n.ports["pmos_load_n_GATE"],
            port_name_prefix="load_bias_pair",
        ),
        RouteNetSpec(
            name="stage2_out",
            start=nmos_stage2.ports["nmos_stage2_DRAIN"],
            stop=pmos_stage2_load.ports["pmos_stage2_load_DRAIN"],
            port_name_prefix="stage2_out",
            is_top_level_pin=True,
        ),
        RouteNetSpec(
            name="stage2_load_bias",
            start=pmos_stage2_load.ports["pmos_stage2_load_GATE"],
            stop=pmos_bias_ref.ports["pmos_bias_ref_GATE"],
            port_name_prefix="stage2_load_bias",
        ),
    ]

    # Combined net list for placement — sees ALL inter-instance connections.
    all_nets = critical_nets + remaining_nets

    # ---- AUTOMATIC PLACEMENT via DoRoutes ----
    print("Placing 2-stage CMOS op-amp with DoRoutes placement engine...")

    inv_dbu = 1.0 / c.kcl.dbu

    # Compute bbox from device sizes — generous room for 9 devices
    bb = nmos_in_p.dbbox()
    dev_w = bb.right - bb.left
    dev_h = bb.top - bb.bottom
    bbox_w = dev_w * 6 + 40.0
    bbox_h = dev_h * 4 + 40.0
    bbox_dbu = (
        int(bbox_h * inv_dbu),
        int(bbox_w / 2 * inv_dbu),
        int(-bbox_h / 3 * inv_dbu),
        int(-bbox_w / 2 * inv_dbu),
    )

    report = place_auto(
        c,
        instances=[
            {"name": "nmos_in_p",         "group": "pwell"},
            {"name": "nmos_in_n",         "group": "pwell"},
            {"name": "pmos_load_p",       "group": "nwell"},
            {"name": "pmos_load_n",       "group": "nwell"},
            {"name": "nmos_tail",         "group": "pwell"},
            {"name": "nmos_stage2",       "group": "pwell"},
            {"name": "pmos_stage2_load",  "group": "nwell"},
            {"name": "nmos_bias_ref",     "group": "pwell"},
            {"name": "pmos_bias_ref",     "group": "nwell"},
        ],
        nets=all_nets,
        mode="tiling",
        objective={
            "hpwl_weight": 1.0,
            "congestion_weight": 0.5,
            "density_weight": 0.3,
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
        print(f"[PLACE]   {name:20s} -> ({x / inv_dbu:+.2f}, {y / inv_dbu:+.2f}) um")

    # Important: add ports ONLY after finished moving instances
    for inst in [nmos_in_p, nmos_in_n, pmos_load_p, pmos_load_n, nmos_tail,
                 nmos_stage2, pmos_stage2_load, nmos_bias_ref, pmos_bias_ref]:
        c.add_ports(inst)

    # Refresh nets with post-placement port positions
    critical_nets = [
        RouteNetSpec(
            name=net.name,
            start=c.ports[net.start.name],
            stop=c.ports[net.stop.name],
            port_name_prefix=net.port_name_prefix,
            is_top_level_pin=net.is_top_level_pin,
        )
        for net in critical_nets
    ]
    remaining_nets = [
        RouteNetSpec(
            name=net.name,
            start=c.ports[net.start.name],
            stop=c.ports[net.stop.name],
            port_name_prefix=net.port_name_prefix,
            is_top_level_pin=net.is_top_level_pin,
        )
        for net in remaining_nets
    ]
    all_nets = critical_nets + remaining_nets

    # Pre-compute port geometries on the pristine component BEFORE any routing
    # modifies the layout.  This prevents KLayout Region extraction from merging
    # route metal with port polygons, corrupting geometry for later nets.
    geom_cache = _precompute_port_geometries(c, all_nets, SKY130_CONFIG, 0.14)

    # Routing extent marker.
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

    print("Routing 2-stage CMOS op-amp core with 3D multi-layer A* (M1=H, M2=V with via transitions)...")

    # Pre-route known hard nets with dynamic width sizing.
    dbu = c.kcl.dbu
    for net in critical_nets:
        before_inst = len(c.insts)
        cached_start = geom_cache.get((int(net.start.dcenter[0] / dbu), int(net.start.dcenter[1] / dbu)))
        cached_stop = geom_cache.get((int(net.stop.dcenter[0] / dbu), int(net.stop.dcenter[1] / dbu)))
        route_multilayer_3d(
            c,
            start=net.start,
            stop=net.stop,
            config=SKY130_CONFIG,
            grid_unit=grid_unit_um,
            width=0.14,
            dynamic_width=True,
            layers_to_avoid=layers_to_avoid,
            add_segment_ports=net.is_top_level_pin,
            port_name_prefix=net.port_name_prefix,
            deterministic=True,
            clearance_ladder=(0.28, 0.14),
            cached_start_geom=cached_start,
            cached_stop_geom=cached_stop,
        )
        if len(c.insts) <= before_inst:
            # Wide endpoints may not fit in tight placement; retry fixed-width.
            route_multilayer_3d(
                c,
                start=net.start,
                stop=net.stop,
                config=SKY130_CONFIG,
                grid_unit=grid_unit_um,
                width=0.14,
                dynamic_width=False,
                layers_to_avoid=layers_to_avoid,
                add_segment_ports=net.is_top_level_pin,
                port_name_prefix=net.port_name_prefix,
                deterministic=True,
                clearance_ladder=(0.28, 0.14),
                cached_start_geom=cached_start,
                cached_stop_geom=cached_stop,
            )
        if len(c.insts) <= before_inst:
            print(f"[OPAMP] WARNING: Critical pre-route failed for net '{net.name}'")

    # Route remaining nets batch-wise.
    c, _ = route_nets_deterministic_copy(
        c,
        nets=remaining_nets,
        config=SKY130_CONFIG,
        grid_unit=grid_unit_um,
        width=0.14,
        dynamic_width=dynamic_width,
        layers_to_avoid=layers_to_avoid,
        add_segment_ports=add_segment_ports,
        require_all=False,
        deterministic=True,
        placement_report=report,
        geom_cache=geom_cache,
        clearance_ladder=(0.28, 0.14),
    )

    # Label unrouted top-level pins (single-port, no routing needed)
    label_unrouted_pins(c, [
        PinLabelSpec(pin_name="vin_p", port=c.insts["nmos_in_p"].ports["nmos_in_p_GATE"]),
        PinLabelSpec(pin_name="vin_n", port=c.insts["nmos_in_n"].ports["nmos_in_n_GATE"]),
    ], config=SKY130_CONFIG, debug=True)

    add_instance_label(c, c.insts["nmos_in_p"], instance_name="nmos_in_p")
    add_instance_label(c, c.insts["nmos_in_n"], instance_name="nmos_in_n")
    add_instance_label(c, c.insts["pmos_load_p"], instance_name="pmos_load_p")
    add_instance_label(c, c.insts["pmos_load_n"], instance_name="pmos_load_n")
    add_instance_label(c, c.insts["nmos_tail"], instance_name="nmos_tail")
    add_instance_label(c, c.insts["nmos_stage2"], instance_name="nmos_stage2")
    add_instance_label(c, c.insts["pmos_stage2_load"], instance_name="pmos_stage2_load")
    add_instance_label(c, c.insts["nmos_bias_ref"], instance_name="nmos_bias_ref")
    add_instance_label(c, c.insts["pmos_bias_ref"], instance_name="pmos_bias_ref")

    return c


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Route 2-stage opamp example with automatic placement (headless by default)."
    )
    parser.add_argument(
        "--skip-lvs",
        action="store_true",
        help="Skip LVS run after writing GDS.",
    )
    parser.add_argument(
        "--out-gds",
        default="./results/test_2stage_opamp_placed.gds",
        help="Output GDS path.",
    )
    args = parser.parse_args()

    c = test_2stage_opamp_placed(
        component_name="test_2stage_opamp_placed_inst",
        add_segment_ports=True,
        routing_half_extent=40.0,
        grid_unit_um=1.0,
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
        from sky130.examples.lvs_graph_checks import check_opamp_layout_graph
        from sky130.examples.lvs_magic_utils import run_lvs

        run_lvs(
            gds_path,
            "test_2stage_opamp_placed",
            OPAMP_SCHEMATIC,
            graph_check_fn=check_opamp_layout_graph,
        )
