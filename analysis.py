from __future__ import annotations
from enum import Enum
from typing import TYPE_CHECKING, List, Tuple
from py_bridge_designer.scenario import CABLE_ANCHORAGE_X_OFFSET, ARCH_SUPPORT, CABLE_SUPPORT_LEFT, CABLE_SUPPORT_BOTH, INTERMEDIATE_SUPPORT, HIGH_PIER


if TYPE_CHECKING:
    from py_bridge_designer.bridge import Bridge


def _fold_max(max: float, x: float) -> float:
    return x if x > max else max


def _square(x: float) -> float:
    return x * x


class AnalysisError(Enum):
    NoAnalysisError = 0
    AnalysisBadPivot = 2


class FailMode(Enum):
    FailModeNone = 0
    FailModeBuckles = 1
    FailModeYields = 2
    FailModeSlenderness = 3


class MemberStrength():
    def __init__(self,
                 compressive: float,
                 tensile: float,
                 compressive_fail_mode: FailMode,
                 tensile_fail_mode: FailMode):
        self.compressive = compressive
        self.tensile = tensile
        self.compressive_fail_mode = compressive_fail_mode
        self.tensile_fail_mode = tensile_fail_mode


class MaxForces():
    def __init__(self,
                 compression: float,
                 tension: float):
        self.compression = compression
        self.tension = tension


class Analysis():
    def __init__(self, bridge: Bridge, test_print=False):
        self._bridge = bridge
        self.error = AnalysisError.NoAnalysisError
        self.test_print = test_print

        # Fill restaint vectors with False values
        self.x_restraints = [False for _ in range(bridge.n_joints + 1)]
        self.y_restraints = [False for _ in range(bridge.n_joints + 1)]

        # Initialize stiffness matrix
        self.n_equations = self._bridge.n_joints * 2
        self.stiffness = [
            [0.0 for _ in range(self.n_equations)] for _ in range(self.n_equations)
        ]

    def _apply_restraints(self):
        n_loaded_joints = self._bridge.load_scenario.n_loaded_joints

        # ======================
        # Assume simple support
        # ======================

        # Left side...
        self.x_restraints[1] = True
        self.y_restraints[1] = True
        # Right side...
        self.y_restraints[n_loaded_joints] = True

        # Set up index to point to joint after last loaded joint
        joint_index = n_loaded_joints + 1
        support_type = self._bridge.load_scenario.support_type

        # ===============================
        # Adjust for intermediate support
        # ===============================
        if support_type == INTERMEDIATE_SUPPORT:
            if self._bridge.load_scenario.support_type == HIGH_PIER:
                intermed_joint_num = self._bridge.load_scenario.intermediate_support_joint_no
                self.x_restraints[intermed_joint_num] = True
                self.y_restraints[intermed_joint_num] = True
                self.x_restraints[1] = False
            else:
                self.x_restraints[joint_index] = True
                self.y_restraints[joint_index] = True
                joint_index += 1

        # ==================
        # Adjust for arches
        # ==================
        if support_type == ARCH_SUPPORT:
            # Set x and y restraints for both arch supports
            for _ in range(2):
                self.x_restraints[joint_index] = True
                self.y_restraints[joint_index] = True
                joint_index += 1
            # Undo simple support
            # Left side...
            self.x_restraints[1] = False
            self.y_restraints[1] = False
            # Right side...
            self.y_restraints[n_loaded_joints] = False

        # ==========================
        # Adjust for cable supports
        # ==========================
        if support_type == CABLE_SUPPORT_LEFT or support_type == CABLE_SUPPORT_BOTH:
            self.x_restraints[joint_index] = True
            self.y_restraints[joint_index] = True
            joint_index += 1
        if support_type == CABLE_SUPPORT_BOTH:
            self.x_restraints[joint_index] = True
            self.y_restraints[joint_index] = True

    def _get_stiffness(self, i: int, j: int):
        return self.stiffness[i - 1][j - 1]

    def _set_stiffness(self, i: int, j: int, x: float):
        self.stiffness[i - 1][j - 1] = x

    def _increment_stiffness(self, i: int, j: int, x: float):
        prev_x = self._get_stiffness(i, j)
        self._set_stiffness(i, j, prev_x + x)

    def _decrement_stiffness(self, i: int, j: int, x: float):
        prev_x = self._get_stiffness(i, j)
        self._set_stiffness(i, j, prev_x - x)

    def _apply_initial_stiffness(self):
        for member in self._bridge.members:
            j1 = member.start_joint.number
            j2 = member.end_joint.number
            xs = member.cross_section
            ae_over_l = self._bridge.parameters.shapes[xs.section][xs.size].area * \
                xs.material.E / member.length
            xx = ae_over_l * _square(member.cos_x)
            yy = ae_over_l * _square(member.cos_y)
            xy = ae_over_l * member.cos_x * member.cos_y

            j12m1 = 2 * j1 - 1
            j12 = 2 * j1
            j22m1 = 2 * j2 - 1
            j22 = 2 * j2

            self._increment_stiffness(j12m1, j12m1, xx)
            self._increment_stiffness(j12m1, j12, xy)
            self._decrement_stiffness(j12m1, j22m1, xx)
            self._decrement_stiffness(j12m1, j22, xy)
            self._increment_stiffness(j12, j12m1, xy)
            self._increment_stiffness(j12, j12, yy)
            self._decrement_stiffness(j12, j22m1, xy)
            self._decrement_stiffness(j12, j22, yy)
            self._decrement_stiffness(j22m1, j12m1, xx)
            self._decrement_stiffness(j22m1, j12, xy)
            self._increment_stiffness(j22m1, j22m1, xx)
            self._increment_stiffness(j22m1, j22, xy)
            self._decrement_stiffness(j22, j12m1, xy)
            self._decrement_stiffness(j22, j12, yy)
            self._increment_stiffness(j22, j22m1, xy)
            self._increment_stiffness(j22, j22, yy)

    def _apply_support_restraints(self):
        ...

    def get_results(self) -> Tuple[bool, float]:
        self._apply_restraints()
        self._apply_initial_stiffness()
        if self.test_print:
            print("stiffness matrix:", self.stiffness)
        cost = 20000
        return False, cost
