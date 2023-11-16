from typing import List, Tuple
from enum import Enum
from py_bridge_designer.members import Joint, CrossSection, Member
from py_bridge_designer.scenario import LoadScenario
from py_bridge_designer.parameters import Params
from py_bridge_designer.analysis import Analysis


class BridgeError(Enum):
    BridgeNoError = 0
    BridgeAtMaxJoints = 1
    BridgeJointOutOfBounds = 2


class Bridge():
    def __init__(self, load_scenario_index=0):
        self.error = BridgeError.BridgeNoError
        self.load_scenario = LoadScenario(load_scenario_index)

        # Fill contents of bridge joints from its load scenario
        self.n_joints = self.load_scenario.n_prescribed_joints
        self.joints = []  # type: List[Joint]
        self.joint_coords = dict()
        for j in range(self.n_joints):
            joint = self.load_scenario.prescribed_joints[j]
            self.joints.append(joint)
            self.joint_coords[(joint.x, joint.y)] = joint

        self.parameters = Params()
        self.n_members = 0
        self.members = []  # type: List[Member]
        self.member_coords = dict()
        self.max_y = 32
        self.min_y = -96
        self.matrix_x = 242
        self.matrix_y = 130
        self.max_joints = 130  # it is best if max joints and maxtrix_y are the same value
        self.max_material_types = 3
        self.max_section_types = 2
        self.max_section_size = 33
        self.n_load_instances = 0

    # ===========================================
    # Joints and Members Functions
    # ===========================================

    def _add_joint(self, coords: Tuple[int, int]) -> Tuple[bool, BridgeError]:
        """Adds a joint to the Bridge. 
        If either the 'start' or 'end' joints are out of bounds, the joint will be rejected.

        Args:
            coords: Tuple[int] in the shape (x,y)
        Returns:
            bool True if joint added, False if joint was rejected
        """
        x = coords[0]
        y = coords[1]

        # Check if the bridge has reached the maximum amount of joints
        if self.n_joints == self.max_joints:
            # joint rejected
            return False, BridgeError.BridgeAtMaxJoints

        # Check if joint coordinates are outside of bounds of the bridge's load scenario
        # check x
        if x > self.load_scenario.max_x or x < self.load_scenario.min_x:
            # Make sure x is not a cable anchor
            if self.load_scenario.cable_anchors_x is not None and x not in self.load_scenario.cable_anchors_x:
                # joint rejected
                return False, BridgeError.BridgeJointOutOfBounds

        # check y
        if y > self.max_y or y < self.min_y:
            # joint rejected
            return False, BridgeError.BridgeJointOutOfBounds

        # Add the joint
        self.n_joints += 1
        joint = Joint(number=self.n_joints, x=x, y=y)
        self.joints.append(joint)
        self.joint_coords[coords] = joint
        return True, BridgeError.BridgeNoError  # joint added

    def add_member(self,
                   start_x: int,
                   start_y: int,
                   end_x: int,
                   end_y: int,
                   material_index: int,
                   section_index: int,
                   size: int) -> Tuple[bool, BridgeError]:
        """Adds a member to the Bridge. 
        If either the 'start' or 'end' joints do not exist, it will add them.

        Args:
            start_x: int for the starting joint x connection
            start_y: int for the starting joint y connection
            end_x: int for the ending joint x connection
            end_y: int for the ending joint y connection
            material_index: int between 0 and 2, the index of the member material type
            section_index: int either 0 (Bar) or 1 (Tube)
            size: int between 0 and 32
        Returns:
            bool True if member added, False if member was rejected
        """
        # Check if joints already exists
        if not (start_x, start_y) in self.joint_coords:
            joint_accepted, bridge_error = self._add_joint(
                coords=(start_x, start_y))
            if not joint_accepted:
                # member rejected because of joint
                return False, bridge_error

        if not (end_x, end_y) in self.joint_coords:
            joint_accepted, bridge_error = self._add_joint(
                coords=(end_x, end_y))
            if not joint_accepted:
                # member rejected because of joint
                return False, bridge_error

        # Get joints
        start_joint: Joint = self.joint_coords[(start_x, start_y)]
        end_joint: Joint = self.joint_coords[(end_x, end_y)]

        # Set Cross Section
        cross_section = CrossSection(
            parameters=self.parameters,
            material_index=material_index,
            section=section_index,
            size=size
        )

        # Add the member
        self.n_members += 1
        member = Member(
            number=self.n_members,
            start_joint=start_joint,
            end_joint=end_joint,
            cross_section=cross_section,
            grid_size=self.load_scenario.grid_size
        )
        self.members.append(member)
        self.member_coords[start_joint.number] = end_joint.number
        self.member_coords[end_joint.number] = start_joint.number

        return True, BridgeError.BridgeNoError  # member added

    # ===========================================
    # Observation Functions
    # ===========================================

    def _zeros(self) -> List[List[int]]:
        zeros = []
        for _ in range(self.matrix_y):
            row = []
            for _ in range(self.matrix_x):
                row.append(0)
            zeros.append(row)
        return zeros

    def get_state(self) -> List[List[List[int]]]:
        """Get the current state of the bridge as an 3D adjacency tensor
        Returns:
            An adjacency tensor in the shape (2, self.matrix_y, self.matrix_x) with values {0,1}
            1st matrix representing the joint coordinates
                - 1 if a joint exists at that (x,y) position and 0 otherwise
            2nd matrix representing the member connections
                - 1 if a member exists between that joint and 0 otherwise
                - Technically, only the shape (self.matrix_y, self.matrix_y) is considered and the rest is padded
        """

        coord_matrix = self._zeros()  # initialize the 1st matrix to all zeros
        match_count = 0
        for y in range(self.matrix_y):
            for x in range(self.matrix_x):
                # Convert adjacency index values to coordinate values
                x_coord = x - 32
                y_coord = y - 96
                if (x_coord, y_coord) in self.joint_coords:
                    # Use adjacency index values
                    coord_matrix[y][x] = 1

        member_matrix = self._zeros()
        for i in range(self.max_joints):
            joint_number = i + 1
            if joint_number in self.member_coords:
                # Adjust the 1-based joint_numbers for the 0-based matrix index
                start_joint = joint_number - 1
                end_joint = self.member_coords[joint_number] - 1
                # Add the adjacency numbers
                member_matrix[start_joint][end_joint] = 1
                member_matrix[end_joint][start_joint] = 1

        return [coord_matrix, member_matrix]

    # ===========================================
    # Analysis Functions
    # ===========================================

    def _setup_load_instances(self):
        self.n_equations = self.n_joints * 2
        self.n_load_instances = self.load_scenario.n_loaded_joints + 1
        self.load_instances: List[List[float]] = []
        for _ in range(self.n_load_instances + 1):
            point_load: List[float] = []
            for _ in range(self.n_equations + 1):
                point_load.append(0.0)
            self.load_instances.append(point_load)

    def _apply_self_weights(self):
        for member in self.members:
            dead_load: float = self.parameters.dead_load_factor * \
                self.parameters.shapes[member.cross_section.section][member.cross_section.size].area * \
                member.length * member.cross_section.material.density * \
                9.8066 / 2.0 / 1000.0

            force_point_1 = 2 * member.start_joint.number
            force_point_2 = 2 * member.end_joint.number

            for point_load in self.load_instances:
                point_load[force_point_1] -= dead_load
                point_load[force_point_2] -= dead_load

    def _apply_dead_load(self):
        load_scenario = self.load_scenario
        load_case = self.parameters.load_cases[load_scenario.load_case]

        for joint in self.joints:
            force_point = 2 * joint.number
            for point_load in self.load_instances:
                load = load_case.point_dead_load
                # Account for the ends of the bridge deck
                if (joint.number == 1 or joint.number == load_scenario.n_loaded_joints):
                    load /= 2  # divide load by 2
                point_load[force_point] -= load

    def _apply_live_load(self):
        load_case = self.parameters.load_cases[self.load_scenario.load_case]
        for i in range(2, self.n_load_instances):
            point_load = self.load_instances[i]
            force_point_front = 2 * i
            force_point_rear = force_point_front - 2
            point_load[force_point_front] -= self.parameters.live_load_factor * \
                load_case.front_axle_load
            point_load[force_point_rear] -= self.parameters.live_load_factor * \
                load_case.rear_axle_load

    def _apply_loads(self, test_print=False):
        self._setup_load_instances()
        self._apply_self_weights()
        self._apply_dead_load()
        self._apply_live_load()

        if test_print:
            for i in range(1, self.n_load_instances + 1):
                point_load = self.load_instances[i]
                print("Point Loads Load Case", i)
                print("Jnt #      X           Y")
                print("----- ----------- -----------")
                for joint in self.joints:
                    print("%5d %11.5f %11.5f" % (
                        joint.number, point_load[2 * joint.number - 1], point_load[2 * joint.number]))

    def analyze(self, test_print=False) -> Tuple[bool, float]:
        self._apply_loads(test_print)
        analysis = Analysis(bridge=self, test_print=test_print)
        valid, cost = analysis.get_results()
        return valid, cost
