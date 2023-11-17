from __future__ import annotations
from typing import TYPE_CHECKING, List

if TYPE_CHECKING:
    from py_bridge_designer.bridge import Bridge
    from py_bridge_designer.Members import CrossSection


def calculate_cost(bridge: Bridge) -> int:
    used: List[CrossSection] = []
    mtl_cost = 0.00
    for member in bridge.members:
        xs = member.cross_section
        mtl_cost += xs.material.cost[xs.section] * \
            bridge.parameters.shapes[xs.section][xs.size].area * \
            member.length * xs.material.density

        # Look for this cross-section in the list of those already used
        found = sum([1 if not xs.is_equal(used[i])
                    else 0 for i in range(len(used))])
        if found == len(used):
            # Didn't find this cross-section.  Add it.
            used.append(xs)

    product_cost = len(used) * bridge.parameters.ordering_fee
    connection_cost = bridge.n_joints * bridge.parameters.connection_cost
    total_cost = 2 * (mtl_cost + connection_cost) + \
        product_cost + bridge.load_scenario.desc.site_cost

    total_cost = round(total_cost)
    return total_cost
