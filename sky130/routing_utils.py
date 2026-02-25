"""Thin Sky130 adapter around doroutes.multilayer — preserves backward-compatible API."""

from doroutes.multilayer import (
    MetalLayerSpec,
    PortGeometry,
    RoutingConfig,
    RouteNetSpec,
    route_hierarchical as _route_hierarchical,
    route_hierarchical_astar as _route_hierarchical_astar,
    route_multilayer_3d as _route_multilayer_3d,
    route_nets_deterministic as _route_nets_deterministic,
    route_nets_deterministic_copy as _route_nets_deterministic_copy,
)
from sky130.pcells.vias import via_m1_m2

# ---------------------------------------------------------------------------
# Sky130 PDK configuration
# ---------------------------------------------------------------------------
SKY130_CONFIG = RoutingConfig(
    metal_layers=(
        MetalLayerSpec(
            layer_tuple=(68, 20),
            preferred_direction="h",
            min_width=0.14,
            min_via_pad=0.29,
            below_cut_layer=(67, 44),
        ),
        MetalLayerSpec(
            layer_tuple=(69, 20),
            preferred_direction="v",
            min_width=0.14,
            min_via_pad=0.29,
            below_cut_layer=(68, 44),
        ),
    ),
    via_factory=via_m1_m2,
    via_metal_enclosure_add=0.07,
)

# Backward-compatible constants
LAYER_M1 = SKY130_CONFIG.layer_m1  # (68, 20)
LAYER_M2 = SKY130_CONFIG.layer_m2  # (69, 20)


# ---------------------------------------------------------------------------
# Public routing functions — inject SKY130_CONFIG by default
# ---------------------------------------------------------------------------
def route_multilayer_3d(c, start, stop, config=None, **kw):
    return _route_multilayer_3d(c, start, stop, config=config or SKY130_CONFIG, **kw)


def route_hierarchical_astar(c, start, stop, config=None, **kw):
    return _route_hierarchical_astar(c, start, stop, config=config or SKY130_CONFIG, **kw)


def route_hierarchical(c, start, stop, config=None, **kw):
    return _route_hierarchical(c, start, stop, config=config or SKY130_CONFIG, **kw)


def route_nets_deterministic(c, nets, config=None, **kw):
    return _route_nets_deterministic(c, nets, config=config or SKY130_CONFIG, **kw)


def route_nets_deterministic_copy(c, nets, config=None, **kw):
    return _route_nets_deterministic_copy(c, nets, config=config or SKY130_CONFIG, **kw)
