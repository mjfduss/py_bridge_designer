from typing import List, Tuple
from enum import Enum
from py_bridge_designer.members import Joint, CrossSection, Member
from py_bridge_designer.scenario import LoadScenario
from py_bridge_designer.parameters import Params, get_section

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
    def __init__(self, load_scenario_index = 0):
        self.error = BridgeError.BridgeNoError
        self.load_scenario = LoadScenario(load_scenario_index)
        
        # Fill contents of bridge joints and members from its load scenario
        self.n_joints = self.load_scenario.n_prescribed_joints
        self.joints = [] # type: List[Joint]
        self.joint_coords = dict()
        for j in range(self.n_joints):
            joint = self.load_scenario.prescribed_joints[j]
            self.joints.append(joint)
            self.joint_coords[(joint.x, joint.y)] = joint
        
        
        self.parameters = Params()
        self.n_members = 0
        self.members = [] # type: List[Member]

        
    def add_joint(self, coords: Tuple[int, int]) -> Joint:
        self.n_joints += 1
        joint = Joint(number=self.n_joints, x=coords[0], y=coords[1])
        self.joints.append(joint)
        self.joint_coords[coords] = joint
        return Joint
    
    def add_member(self, 
                   start: Tuple[int, int], 
                   end: Tuple[int, int], 
                   material_index: int, 
                   section_index: int, 
                   size: int):
        """Adds a member to the Bridge. 
        If either the 'start' or 'end' joints do not exist, it will add them.
        
        Args:
            start: (x,y) tuple for the starting joint connection
            end: (x,y) tuple for the ending joint connection
            material_index: int between 0 and 2, the index of the member material type
            section_index: int either 0 (Bar) or 1 (Tube)
            size: int between 0 and 32
        """
        if not start in self.joint_coords:
            self.add_joint(coords=start)
        if not end in self.joint_coords:
            self.add_joint(coords=end)
            
        start_joint = self.joint_coords[start]
        end_joint = self.joint_coords[end]
        cross_section = CrossSection(
                parameters=self.parameters,
                material_index=material_index,
                section= get_section(section_index),
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
    
        
            
            
