"""Test NMOS 5V with square waypoints (d, g, s, b) connected to device ports."""

import sys
from pathlib import Path

# Load environment variables from .env file (must be before doroutes import)
from dotenv import load_dotenv
load_dotenv(Path(__file__).parent.parent.parent / ".env")

sys.path.append(str(Path(__file__).parent.parent.parent))

from gdsfactory.component import Component
import sky130
from gdsfactory.pdk import get_active_pdk
import gdsfactory as gf
from sky130.examples.lvs_magic_utils import run_magic_extract
from sky130.routing_utils import route_multilayer_3d


def test_nmos_5v() -> Component:
    """Create NMOS 5V test with D, G, S, B waypoints."""
    pdk = get_active_pdk()
    c = Component("test_nmos_5v")

    # Instantiate NMOS 5V
    nmos = c.add_ref(pdk.get_component('nmos_5v', instance_name='nmos'), name='nmos')
    c.add_ports(nmos)

    # Get device port centers for waypoint placement
    drain_center = c.ports['nmos_DRAIN'].dcenter
    gate_center = c.ports['nmos_GATE'].dcenter
    source_center = c.ports['nmos_SOURCE'].dcenter
    body_center = c.ports['nmos_BODY'].dcenter

    # M1 layer for waypoints
    m1_layer = (68, 20)
    m2_layer = (69, 20)
    wp_size = 0.5

    # Place square waypoints offset from device ports
    # d (drain) - offset to the left
    wp_d = c.add_ref(sky130.pcells.waypoint(width=wp_size, layer=m2_layer, instance_name='wp_d'), name='wp_d')
    wp_d.dmove((drain_center[0] - 5, drain_center[1] - 5))

    wp_size = 0.3
    # g (gate) - offset above
    wp_g = c.add_ref(sky130.pcells.waypoint(width=wp_size, layer=m1_layer, instance_name='wp_g'), name='wp_g')
    wp_g.dmove((gate_center[0], gate_center[1] + 5))


    wp_size = 0.5
    # s (source) - offset to the right
    wp_s = c.add_ref(sky130.pcells.waypoint(width=wp_size, layer=m1_layer, instance_name='wp_s'), name='wp_s')
    wp_s.dmove((source_center[0] + 5, source_center[1] - 5))

    # b (body) - offset far right
    wp_b = c.add_ref(sky130.pcells.waypoint(width=wp_size, layer=m1_layer, instance_name='wp_b'), name='wp_b')
    wp_b.dmove((body_center[0] + 5, body_center[1] + 5))

    c.add_ports(wp_d)
    c.add_ports(wp_g)
    c.add_ports(wp_s)
    c.add_ports(wp_b)

    # Add routing area markers to ensure grid extends beyond the obstacle
    routing_area_layer = (235, 4)  # Dummy layer for grid extent
    c.add_polygon([(-50, -20), (110, -20), (110, 50), (-50, 50)], layer=routing_area_layer)

    # Routing parameters
    grid_unit_um = 1.0
    wire_width = 0.25
    layers_to_avoid = [(68, 20), (69, 20)]

    # Route DRAIN -> d waypoint
    route_multilayer_3d(
        c,
        start=nmos.ports['nmos_DRAIN'],
        stop=wp_d.ports['wp_d_waypoint'],
        grid_unit=grid_unit_um,
        width=wire_width,
        layers_to_avoid=layers_to_avoid,
        add_segment_ports=True,
        port_name_prefix="d"
    )

    # Route GATE -> g waypoint
    route_multilayer_3d(
        c,
        start=nmos.ports['nmos_GATE'],
        stop=wp_g.ports['wp_g_waypoint'],
        grid_unit=grid_unit_um,
        width=wire_width,
        layers_to_avoid=layers_to_avoid,
        add_segment_ports=True,
        port_name_prefix="g"
    )

    # Route SOURCE -> s waypoint
    route_multilayer_3d(
        c,
        start=nmos.ports['nmos_SOURCE'],
        stop=wp_s.ports['wp_s_waypoint'],
        grid_unit=grid_unit_um,
        width=wire_width,
        layers_to_avoid=layers_to_avoid,
        add_segment_ports=True,
        port_name_prefix="s"
    )

    # Route BODY -> b waypoint
    route_multilayer_3d(
        c,
        start=nmos.ports['nmos_BODY'],
        stop=wp_b.ports['wp_b_waypoint'],
        grid_unit=grid_unit_um,
        width=wire_width,
        layers_to_avoid=layers_to_avoid,
        add_segment_ports=True,
        port_name_prefix="b"
    )

    return c


if __name__ == "__main__":
    c = test_nmos_5v()
    c.flatten()
    c.pprint_ports()
    c.show()
    input("Press Enter to continue...")
    outdir = Path(__file__).parent.parent.parent / "results"
    outdir.mkdir(parents=True, exist_ok=True)
    gds_path = outdir / "test_nmos_5v.gds"
    c.write_gds(gds_path, with_metadata=False)
    sp_path = run_magic_extract(outdir, gds_path, "test_nmos_5v", "test_nmos_5v.sp")
    print(f"[OK] Magic extracted SPICE: {sp_path}")
