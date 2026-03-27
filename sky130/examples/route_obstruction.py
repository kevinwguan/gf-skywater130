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
import gdsfactory as gf
from sky130.examples.lvs_magic_utils import run_magic_extract
from sky130.routing_utils import route_hierarchical_astar, SKY130_CONFIG


def draw_astar_grid(c, layers, grid_unit_um, bend_radius_um, debug_layer=(235, 4)):
    """Draw the A* grid overlay for debugging.
    
    Args:
        c: Component to add grid reference to
        layers: Layers to check for obstructions (list of tuples)
        grid_unit_um: The grid discretization unit in um
        bend_radius_um: The bend radius in um
        debug_layer: Layer to draw grid on
    
    Returns:
        The grid component reference added to c
    """
    from kfactory import kdb
    
    # Conversion factor: 1 um = 1000 dbu (assuming 1nm dbu)
    dbu_per_um = 1000
    
    kc = c.kcl.kcells[c.name]
    bbox = kc.bbox()
    
    # Convert um to dbu for kdb operations
    grid_unit_dbu = int(grid_unit_um * dbu_per_um)
    padding_dbu = int(2 * bend_radius_um * dbu_per_um)
    
    # Bounding box in dbu with padding
    top_dbu = bbox.top + padding_dbu
    right_dbu = bbox.right + padding_dbu
    bottom_dbu = bbox.bottom - padding_dbu
    left_dbu = bbox.left - padding_dbu
    
    # Collect obstruction polygons into a kdb.Region
    obstruction_region = kdb.Region()
    for layer in layers:
        layer_idx = kc.kcl.layer(*layer)
        obstruction_region += kdb.Region(kc.begin_shapes_rec(layer_idx))
    
    # Create a new component for the grid overlay
    grid_component = gf.Component("astar_grid_overlay")
    
    # Draw grid cells (iterate in dbu, position in um)
    x_dbu = left_dbu
    while x_dbu <= right_dbu:
        y_dbu = bottom_dbu
        while y_dbu <= top_dbu:
            # Create cell box for intersection check (in dbu)
            half_dbu = grid_unit_dbu // 2
            cell_box = kdb.Box(x_dbu - half_dbu, y_dbu - half_dbu, 
                               x_dbu + half_dbu, y_dbu + half_dbu)
            cell_region = kdb.Region(cell_box)
            
            # Check intersection with obstructions
            intersection = cell_region & obstruction_region
            
            # Convert position to um for gdsfactory
            x_um = x_dbu / dbu_per_um
            y_um = y_dbu / dbu_per_um
            
            if not intersection.is_empty():
                # Obstructed cell - draw filled square (size in um)
                obs_size_um = grid_unit_um / 2
                obs_cell = gf.components.regular_polygon(side_length=obs_size_um, sides=4, layer=debug_layer)
                tmp = grid_component.add_ref(obs_cell)
                tmp.dcenter = (x_um, y_um)
            else:
                # Free cell - draw small dot (size in um)
                free_size_um = 0.1  # 50nm = 0.05um
                free_cell = gf.components.regular_polygon(side_length=free_size_um, sides=4, layer=debug_layer)
                tmp = grid_component.add_ref(free_cell)
                tmp.dcenter = (x_um, y_um)
            
            y_dbu += grid_unit_dbu
        x_dbu += grid_unit_dbu
    
    # Add grid as a reference to the original component
    return c.add_ref(grid_component)


def test_obstruction() -> Component:
    pdk = get_active_pdk()
    c = Component("test_obstruction")

    # Create instances
    instance1 = c.add_ref(pdk.get_component('nmos_5v'), name='instance1')
    instance2 = c.add_ref(pdk.get_component('nmos_5v'), name='instance2')
    c.add_ports(instance1)
    c.add_ports(instance2)
    
    # Place instances
    instance1.move((0, 0))
    instance2.move((100, 0))

    instance3 = c.add_ref(pdk.get_component('nmos_5v'), name='instance3')
    instance3.move((20, 40))
    c.add_ports(instance3)

    # Create an obstruction that partially blocks the direct route
    # Use Metal1 layer - ALL existing metal on routing layers must be avoided
    obstruction_layer = (68, 20)  # Metal1
    
    # Position the obstacle to block the direct horizontal path between instance1 and instance2
    obstruction_m1 = c.add_polygon([(40, 0), (60, 0), (60, 15), (40, 15)], layer=obstruction_layer)
    obstruction_m2 = c.add_polygon([(40, 0), (60, 0), (60, 15), (40, 15)], layer=(69, 20))  # Metal2
    
    # Add routing area markers to ensure grid extends beyond the obstacle
    routing_area_layer = (235, 4)  # Dummy layer for grid extent
    c.add_polygon([(-5, -20), (110, -20), (110, 50), (-5, 50)], layer=routing_area_layer)

    # Grid resolution for A* pathfinding
    grid_unit_um = 1.0    # 1um grid resolution
    wire_width = 0.25     # Standard wire width
    
    # Layers to avoid - ALL metal on these layers including device metal
    # The router must route around everything on both M1 and M2
    layers_to_avoid = [obstruction_layer, (69, 20)]

    # Draw the A* grid overlay for debugging
    #draw_astar_grid(c, layers_to_avoid, grid_unit_um, 5.0)
    # c.show()
    print("Routing with hierarchical strategy (global 2um + detail 0.25um)...")

    # Import hierarchical router
    from sky130.routing_utils import route_hierarchical

    # Route 1: instance1 DRAIN -> instance2 SOURCE
    # Using hierarchical A* with global routing then detail refinement
    route_hierarchical(
        c,
        start=instance1.ports['DRAIN'],
        stop=instance2.ports['SOURCE'],
        global_grid_unit=1.0,    # Coarse grid for fast global routing
        detail_grid_unit=0.05,   # Fine grid for obstacle avoidance
        width=wire_width,
        layers_to_avoid=layers_to_avoid,
    )

    c.show()
    input()

    # Route 2: instance2 DRAIN -> instance3 SOURCE
    route_hierarchical(
        c,
        start=instance2.ports['DRAIN'],
        stop=instance3.ports['SOURCE'],
        global_grid_unit=1.0,
        detail_grid_unit=0.05,
        width=wire_width,
        layers_to_avoid=layers_to_avoid,
    )

    # c.show()
    c.show()
    input()

    # Route 3: instance3 DRAIN -> instance1 SOURCE
    route_hierarchical(
        c,
        start=instance3.ports['DRAIN'],
        stop=instance1.ports['SOURCE'],
        global_grid_unit=1.0,
        detail_grid_unit=0.05,
        width=wire_width,
        layers_to_avoid=layers_to_avoid,
    )

    # c.show()

    """
    # Route gates using multilayer strategy too
    layers_to_avoid_m2 = [(69, 20)]
    
    # Route 4: instance2 GATE -> instance3 GATE
    route_hierarchical_astar(
        c,
        start=instance2.ports['GATE'],
        stop=instance3.ports['GATE'],
        config=SKY130_CONFIG,
        global_grid_unit=grid_unit_um,
        width=wire_width,
        layers_to_avoid=layers_to_avoid_m2,
    )

    # Route 5: instance2 GATE -> instance1 GATE
    route_hierarchical_astar(
        c,
        start=instance2.ports['GATE'],
        stop=instance1.ports['GATE'],
        config=SKY130_CONFIG,
        global_grid_unit=grid_unit_um,
        width=wire_width,
        layers_to_avoid=layers_to_avoid_m2,
    )
    # c.show()
    """

    # Add instance labels
    add_instance_label(c, instance1, instance_name='instance1')
    add_instance_label(c, instance2, instance_name='instance2')

    return c

if __name__ == "__main__":
    c = test_obstruction()
    c.pprint_ports()
    c.show()
    c.flatten()
    outdir = Path(__file__).parent.parent.parent / "results"
    outdir.mkdir(parents=True, exist_ok=True)
    gds_path = outdir / "test_obstruction.gds"
    c.write_gds(gds_path, with_metadata=False)
    sp_path = run_magic_extract(outdir, gds_path, "test_obstruction", "test_obstruction.sp")
    print(f"[OK] Magic extracted SPICE: {sp_path}")
    
