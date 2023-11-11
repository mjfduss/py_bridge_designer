from typing import List, Tuple
from enum import Enum
from py_bridge_designer.members import Joint, CrossSection, Member
from py_bridge_designer.scenario import LoadScenario
from py_bridge_designer.parameters import Params


class BridgeError(Enum):
    BridgeNoError = 1
    BridgeTooManyElements = 2
    BridgeTooFewJoints = 3
    BridgeWrongPrescribedJoints = 4
    BridgeTooFewMembers = 5
    BridgeDupJoints = 6
    BridgeDupMembers = 7
    BridgeJointOnMember = 8


class Bridge():
    def __init__(self, load_scenario_index=0):
        self.error = BridgeError.BridgeNoError
        self.load_scenario = LoadScenario(load_scenario_index)

        # Fill contents of bridge joints and members from its load scenario
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
        self.max_y = 32
        self.min_y = -96

    def add_joint(self, coords: Tuple[int, int]) -> bool:
        """Adds a joint to the Bridge. 
        If either the 'start' or 'end' joints are out of bounds, the joint will be rejected.

        Args:
            coords: Tuple[int] in the shape (x,y)
        Returns:
            bool True if joint added, False if joint was rejected
        """
        x = coords[0]
        y = coords[1]

        # Check if joint coordinates are outside of bounds of the bridge's load scenario
        # check x
        if x > self.load_scenario.max_x or x < self.load_scenario.min_x:
            # Make sure x is not a cable anchor
            if self.load_scenario.cable_anchors_x is not None and x not in self.load_scenario.cable_anchors_x:
                return False  # joint rejected
        # check y
        if y > self.max_y or y < self.min_y:
            return False  # joint rejected

        # Add the joint
        self.n_joints += 1
        joint = Joint(number=self.n_joints, x=x, y=y)
        self.joints.append(joint)
        self.joint_coords[coords] = joint
        return True  # joint added

    def add_member(self,
                   start_x: int,
                   start_y: int,
                   end_x: int,
                   end_y: int,
                   material_index: int,
                   section_index: int,
                   size: int) -> bool:
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
            joint_accepted = self.add_joint(coords=(start_x, start_y))
            if not joint_accepted:
                return False  # member rejected
        if not (end_x, end_y) in self.joint_coords:
            joint_accepted = self.add_joint(coords=(end_x, end_y))
            if not joint_accepted:
                return False  # member rejected

        # Get joints
        start_joint = self.joint_coords[(start_x, start_y)]
        end_joint = self.joint_coords[(end_x, end_y)]

        # TODO: ?CLAMP DOWN INPUT for material_index, section_index, and size?

        # Set Cross Section
        cross_section = CrossSection(
            parameters=self.parameters,
            material_index=material_index,
            section=section_index,
            size=size
        )

        self.n_members += 1
        member = Member(
            number=self.n_members,
            start_joint=start_joint,
            end_joint=end_joint,
            cross_section=cross_section,
            grid_size=self.load_scenario.grid_size
        )
        self.members.append(member)

        return True  # member added
