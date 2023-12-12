"""
draw.py

Py Bridge Designer
ported from C by Nathan Hartzler

Implementation reference:
https://sourceforge.net/p/wpbdc/rails/ci/master/tree/vendor/gems/WPBDC/ext/WPBDC/bridge_sketch.c
by Gene K. Ressler

"""
from __future__ import annotations
from typing import TYPE_CHECKING, Tuple
import cv2
import numpy as np
from py_bridge_designer.scenario import CABLE_SUPPORT_LEFT, CABLE_SUPPORT_BOTH, CABLE_ANCHORAGE_X_OFFSET
from py_bridge_designer.analysis import FailMode

if TYPE_CHECKING:
    from py_bridge_designer.bridge import Bridge

# Number of bridge designer grids to use as a margin around the image edge.
SKETCH_MARGIN = 10

# Number of pixels in a single bridge designer grid.
SKETCH_GRID_SIZE = 16

MAX_NOFORCE_R, MAX_NOFORCE_G, MAX_NOFORCE_B = 192, 192, 192
MAX_TENSION_R, MAX_TENSION_G, MAX_TENSION_B = 0, 0, 255
MAX_COMPRESSION_R, MAX_COMPRESSION_G, MAX_COMPRESSION_B = 255, 0, 0


def _set_intensity(intensity: float) -> float:
    if intensity < 0:
        return 0.0
    elif intensity > 1:
        return 1.0
    else:
        return intensity


def _set_color(intensity: float, max_type: str) -> Tuple[int, int, int]:
    _intensity = _set_intensity(intensity)

    if max_type == 'tension':
        max_r, max_g, max_b = MAX_TENSION_R, MAX_TENSION_G, MAX_TENSION_B
    else:
        max_r, max_g, max_b = MAX_COMPRESSION_R, MAX_COMPRESSION_G, MAX_COMPRESSION_B

    r = int(max_r * _intensity + MAX_NOFORCE_R * (1.0 - _intensity))
    g = int(max_g * _intensity + MAX_NOFORCE_G * (1.0 - _intensity))
    b = int(max_b * _intensity + MAX_NOFORCE_B * (1.0 - _intensity))

    return r, g, b


def _orient_x_pixel(x_org_pixel: int, grid_pixels: float, position: float) -> int:
    return x_org_pixel + int(position * grid_pixels)


def _orient_y_pixel(y_org_pixel: int, grid_pixels: float, under_grids: int, position: float) -> int:
    return y_org_pixel + int((position + under_grids) * grid_pixels)


def draw_bridge(bridge: Bridge, width=850, height=700, line_thickness=2):
    """
    Returns:
        NDArray[unint8] with size (height, width, 3)
    """
    image = np.zeros(shape=(height, width, 3), dtype=np.uint8)

    # Setup the variables
    bridge_width_grids = bridge.load_scenario.num_length_grids
    bridge_height_grids = bridge.load_scenario.over_grids + \
        bridge.load_scenario.under_grids

    bridge_left_anchor_margin = CABLE_ANCHORAGE_X_OFFSET if bridge.load_scenario.support_type == CABLE_SUPPORT_LEFT else 0
    bridge_right_anchor_margin = CABLE_ANCHORAGE_X_OFFSET if bridge.load_scenario.support_type == CABLE_SUPPORT_BOTH else 0

    # Convert the bridge coordinates to the image coordinates
    x_pixels = width / (bridge_width_grids + bridge_left_anchor_margin +
                        bridge_right_anchor_margin + 2 * SKETCH_MARGIN)
    y_pixels = height / (bridge_height_grids + 2 * SKETCH_MARGIN)
    grid_pixels = min(x_pixels, y_pixels)

    x_org_pixel = int((width - (bridge_width_grids - bridge_left_anchor_margin +
                                bridge_right_anchor_margin) * grid_pixels) / 2.0)
    y_org_pixel = int((height - bridge_height_grids * grid_pixels) / 2.0)

    def sketch_x(x: float):
        return _orient_x_pixel(x_org_pixel, grid_pixels, x)

    def sketch_y(y: float):
        return _orient_y_pixel(
            y_org_pixel, grid_pixels, bridge.load_scenario.under_grids, y)

    # Draw the build area
    cv2.rectangle(
        image,
        pt1=(sketch_x(0), sketch_y(-bridge.load_scenario.under_grids)),
        pt2=(sketch_x(bridge.load_scenario.num_length_grids),
             sketch_y(bridge.load_scenario.over_grids)),
        color=(127, 127, 127),
        thickness=line_thickness // 2)

    # Draw the prescribed joints
    for j in bridge.load_scenario.prescribed_joints:
        cv2.circle(image,
                   center=(sketch_x(j.x), sketch_y(j.y)),
                   radius=line_thickness + 3,
                   color=(127, 127, 127),
                   thickness=line_thickness - 1)

    for member in bridge.members:
        j1 = member.start_joint
        j2 = member.end_joint
        r, g, b = 255, 255, 255

        if bridge.analysis is not None:
            if bridge.analysis.member_strength[member.number].compressive_fail_mode == FailMode.FailModeSlenderness:
                r, g, b = 255, 0, 255
            else:
                compressive = bridge.analysis.member_strength[member.number].compressive
                compression_intensity = bridge.analysis.max_forces[
                    member.number].compression / compressive if compressive > 0 else 0

                tensile = bridge.analysis.member_strength[member.number].tensile
                tension_intensity = bridge.analysis.max_forces[member.number].tension / \
                    tensile if tensile > 0 else 0

                if compression_intensity > tension_intensity:
                    r, g, b = _set_color(compression_intensity, 'compression')
                else:
                    r, g, b = _set_color(tension_intensity, 'tension')
        # Draw joints
        cv2.circle(image,
                   center=(sketch_x(j1.x), sketch_y(j1.y)),
                   radius=line_thickness + 3,
                   color=(r, g, b),
                   thickness=line_thickness - 1)
        cv2.circle(image,
                   center=(sketch_x(j2.x), sketch_y(j2.y)),
                   radius=line_thickness + 3,
                   color=(r, g, b),
                   thickness=line_thickness - 1)
        # Draw member line
        cv2.line(image,
                 pt1=(sketch_x(j1.x), sketch_y(j1.y)),
                 pt2=(sketch_x(j2.x), sketch_y(j2.y)),
                 color=(r, g, b),
                 thickness=line_thickness)

    image = cv2.rotate(image, rotateCode=cv2.ROTATE_180)
    return image
