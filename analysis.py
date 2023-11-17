from __future__ import annotations
from enum import Enum
from math import sqrt
from typing import TYPE_CHECKING, List, Tuple
from scenario import ARCH_SUPPORT, CABLE_SUPPORT_LEFT, CABLE_SUPPORT_BOTH, INTERMEDIATE_SUPPORT, HIGH_PIER
from cost import calculate_cost


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

        # Initialize the dispacement matrix
        self.displacement = [
            [0.0 for _ in range(self._bridge.n_load_instances)] for _ in range(self.n_equations)
        ]

        # Initialize the member_force matrix
        self.member_force = [
            [0.0 for _ in range(self._bridge.n_load_instances)] for _ in range(self._bridge.n_members)
        ]

        # Initialize member_strength vector
        self.member_strength: List[MemberStrength] = [
            MemberStrength(0.0, 0.0, FailMode.FailModeNone, FailMode.FailModeNone) for _ in range(self._bridge.n_members + 1)
        ]

        # Initialize max_forces vector
        self.max_forces: List[MaxForces] = [
            MaxForces(0.0, 0.0) for _ in range(self._bridge.n_members + 1)]

        self.n_compressive_failures = 0
        self.n_tensile_failures = 0

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

    def _divide_stiffness(self, i: int, j: int, x: float):
        prev_x = self._get_stiffness(i, j)
        self._set_stiffness(i, j, prev_x / x)

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
        for point_load in self._bridge.load_instances:
            for joint_index in range(1, self._bridge.n_joints + 1):
                if self.x_restraints[joint_index]:
                    i2m1 = 2 * joint_index - 1
                    for j in range(1, self.n_equations + 1):
                        self._set_stiffness(i2m1, j, 0.0)
                        self._set_stiffness(j, i2m1, 0.0)
                    self._set_stiffness(i2m1, i2m1, 1.0)
                    point_load[i2m1] = 0.0
                if self.y_restraints[joint_index]:
                    i2 = 2 * joint_index
                    for j in range(1, self.n_equations + 1):
                        self._set_stiffness(i2, j, 0.0)
                        self._set_stiffness(j, i2, 0.0)
                    self._set_stiffness(i2, i2, 1.0)
                    point_load[i2] = 0.0

    def _invert(self) -> bool:
        for equation_index in range(1, self.n_equations + 1):
            pivot = self._get_stiffness(equation_index, equation_index)
            if abs(pivot) < 0.99:
                self.error = AnalysisError.AnalysisBadPivot
                return False

            pivr = 1.0 / pivot
            for k in range(1, self.n_equations + 1):
                self._divide_stiffness(equation_index, k, pivot)

            for k in range(1, self.n_equations + 1):
                if k != equation_index:
                    pivot = self._get_stiffness(k, equation_index)
                    for j in range(1, self.n_equations + 1):
                        decrement_by = self._get_stiffness(
                            equation_index, j) * pivot
                        self._decrement_stiffness(k, j, decrement_by)
                    self._set_stiffness(k, equation_index, -pivot * pivr)
            self._set_stiffness(equation_index, equation_index, pivr)
        return True

    def _get_displacement(self, i: int, j: int):
        return self.displacement[i - 1][j - 1]

    def _set_displacement(self, i: int, j: int, x: float):
        self.displacement[i - 1][j - 1] = x

    def _get_member_force(self, i: int, j: int):
        return self.member_force[i - 1][j - 1]

    def _set_member_force(self, i: int, j: int, x: float):
        self.member_force[i - 1][j - 1] = x

    def _compute_end_forces(self, load_instance_index: int):
        for member in self._bridge.members:
            xs = member.cross_section
            ae_over_l = self._bridge.parameters.shapes[xs.section][xs.size].area * \
                xs.material.E / member.length
            j1 = member.start_joint.number
            j2 = member.end_joint.number
            end_force = ae_over_l * \
                ((member.cos_x * (self._get_displacement((2 * j2 - 1), load_instance_index) - self._get_displacement((2 * j1 - 1), load_instance_index))
                  ) + (member.cos_y * (self._get_displacement((2 * j2), load_instance_index) - self._get_displacement((2 * j1), load_instance_index))))
            self._set_member_force(
                member.number, load_instance_index, end_force)

    def _compute_joint_displacements(self):
        for load_instance_index in range(1, self._bridge.n_load_instances + 1):
            for i in range(1, self.n_equations + 1):
                temp = 0.0
                for j in range(1, self.n_equations + 1):
                    temp += self._get_stiffness(i, j) * \
                        self._bridge.load_instances[load_instance_index][j]
                self._set_displacement(i, load_instance_index, temp)

            if self.test_print:
                print("Joint displacements for Load Case", load_instance_index)
                print("Jnt #     /\\X         /\\Y")
                print("----- ----------- -----------")
                for i in range(1, self._bridge.n_joints + 1):
                    d1 = self._get_displacement(2 * i - 1, load_instance_index)
                    d2 = self._get_displacement(2 * i, load_instance_index)
                    print("%5d %11.5lf %11.5lf" % (i, d1, d2))

            self._compute_end_forces(load_instance_index)

    def _compute_member_strengths(self):
        for member in self._bridge.members:
            xs = member.cross_section
            compressive_fail_mode = FailMode.FailModeNone
            compressive = 0.0
            tensile_fail_mode = FailMode.FailModeNone
            tensile = 0.0
            Fy = xs.material.Fy
            params = self._bridge.parameters
            area = params.shapes[xs.section][xs.size].area
            moment = params.shapes[xs.section][xs.size].moment
            radius_of_gyration = sqrt(moment / area) if area > 0 else 0
            slenderness = member.length / radius_of_gyration if radius_of_gyration > 0 else 0
            support_type = self._bridge.load_scenario.support_type

            # if the bridge has cable support joints or else slenderness is not excessive
            if support_type == CABLE_SUPPORT_LEFT or support_type == CABLE_SUPPORT_BOTH or slenderness < params.slenderness_limit:
                # Calculate lambda point
                lam = _square(member.length) * Fy * area / \
                    (9.8696044 * xs.material.E * moment)
                if lam <= 2.25:
                    compressive_fail_mode = FailMode.FailModeYields
                    compressive = params.compression_resistance_factor * \
                        pow(0.66, lam) * Fy * area
                else:
                    compressive_fail_mode = FailMode.FailModeBuckles
                    compressive = params.compression_resistance_factor * 0.88 * Fy * area / lam

                tensile_fail_mode = FailMode.FailModeYields
                tensile = params.tension_resistance_factor * Fy * area
            else:
                compressive_fail_mode = FailMode.FailModeSlenderness
                tensile_fail_mode = FailMode.FailModeSlenderness
                compressive = 0.0
                tensile = 0.0

            # Append stats to member strength vector
            self.member_strength[member.number] = MemberStrength(
                compressive, tensile, compressive_fail_mode, tensile_fail_mode)

    def _summarize_results(self):
        for member in self._bridge.members:
            max_compression = 0.0
            max_tension = 0.0
            for load_instance_index in range(1, self._bridge.n_load_instances + 1):
                member_force = self._get_member_force(
                    member.number, load_instance_index)
                if member_force < 0:
                    max_compression = _fold_max(max_compression, -member_force)
                else:
                    max_tension = _fold_max(max_tension, member_force)
            self.max_forces[member.number] = MaxForces(
                max_compression, max_tension)

            ms = self.member_strength[member.number]

            if ms.compressive_fail_mode != FailMode.FailModeSlenderness and max_compression < ms.compressive:
                self.member_strength[member.number].compressive_fail_mode = FailMode.FailModeNone
            else:
                self.n_compressive_failures += 1

            if ms.tensile_fail_mode != FailMode.FailModeSlenderness and max_tension < ms.tensile:
                self.member_strength[member.number].tensile_fail_mode = FailMode.FailModeNone
            else:
                self.n_tensile_failures += 1

    def get_results(self) -> Tuple[bool, float]:
        cost = calculate_cost(self._bridge)
        self._apply_restraints()
        self._apply_initial_stiffness()
        self._apply_support_restraints()
        valid = self._invert()
        if not valid:
            return False, cost

        self._compute_joint_displacements()
        self._compute_member_strengths()
        self._summarize_results()

        if self.error != AnalysisError.NoAnalysisError or self.n_compressive_failures > 0 or self.n_tensile_failures > 0:
            return False, cost

        return True, cost
