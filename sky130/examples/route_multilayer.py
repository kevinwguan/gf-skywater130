
import subprocess
import sys
from pathlib import Path

# Load environment variables
from dotenv import load_dotenv
load_dotenv(Path(__file__).parent.parent.parent / ".env")

sys.path.append(str(Path(__file__).parent.parent.parent))

from gdsfactory.component import Component
import sky130
from gdsfactory.pdk import get_active_pdk
import gdsfactory as gf
from sky130.routing_utils import route_hierarchical_astar, SKY130_CONFIG

def test_multilayer_routing() -> Component:
    pdk = get_active_pdk()
    c = Component("test_multilayer_routing")

    # Create instances
    instance1 = c.add_ref(pdk.get_component('nmos_5v'), name='instance1')
    instance2 = c.add_ref(pdk.get_component('nmos_5v'), name='instance2')
    c.add_ports(instance1)
    c.add_ports(instance2)
    
    # Place: diagonal to force bends
    instance1.move((0, 0))
    instance2.move((50, 40))

    # Add obstruction to force a non-trivial path
    obstruction_layer = (68, 20) # Met1
    obstruction = c.add_polygon([(20, 0), (30, 0), (30, 50), (20, 50)], layer=obstruction_layer)
    
    # Route
    route_hierarchical_astar(
        c,
        start=instance1.ports['DRAIN'],
        stop=instance2.ports['SOURCE'],
        config=SKY130_CONFIG,
        global_grid_unit=2.0,
        detail_grid_unit=0.25,
        width=0.5,
        layers_to_avoid=[obstruction_layer]
    )

    return c

if __name__ == "__main__":
    c = test_multilayer_routing()
    c.show()
    c.write_gds("./test_multilayer.gds")
    print("Test complete. GDS written to test_multilayer.gds")
