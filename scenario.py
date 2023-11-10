from typing import List, Tuple
from enum import Enum
from py_bridge_designer.scenario_descriptions import scenario_descriptions_table
from py_bridge_designer.members import Joint

CABLE_ANCHORAGE_X_OFFSET = 32

class SupportTypes(Enum):
    ARCH_SUPPORT = 1
    CABLE_SUPPORT_LEFT = 2
    CABLE_SUPPORT_RIGHT = 4
    INTERMEDIATE_SUPPORT = 8
    HI_NOT_LO = 16

class ScenarioDescriptor():
    def __init__(self, index: int):
        if index < 0 or index > 391:
            index = 0
        description = scenario_descriptions_table[index] # type: Tuple(str, str, float)   
        self.index = index
        self.id = description[0]
        self.number = description[1]
        self.site_cost = description[2]
        
class LoadScenario():
    def __init__(self, load_scenario_index: int):
        # ===================
        # Get Scenario Description
        # ===================
        
        desc: type[ScenarioDescriptor] = ScenarioDescriptor(load_scenario_index)
        
        # ===================
        # Setup Support Types
        # ===================

        self.support_type = 0
        # digit 10 => (0 = low pier, 1 = high pier)
        if (desc.id[9] > '0'):
            self.support_type = SupportTypes.HI_NOT_LO
        
        # digit 9 => panel point at which pier is located. (0 = no pier).
        self.intermediate_support_joint_no = int(desc.id[8])
        if (self.intermediate_support_joint_no > 0):
            self.support_type = SupportTypes.INTERMEDIATE_SUPPORT
        # digit 8 => (0 = simple, 1 = arch, 2 = cable left, 3 = cable both)
        if (desc.id[7] == '1'):
            self.support_type = SupportTypes.ARCH_SUPPORT
        elif (desc.id[7] == '2'):
            self.support_type = SupportTypes.CABLE_SUPPORT_LEFT
        elif (desc.id[7] == '3'):
            self.support_type = SupportTypes.CABLE_SUPPORT_RIGHT
        
        
        # ===================
        # Setup Other Values
        # ===================

        # digits 6 and 7 => meters under span
        self.under_meters = int(desc.id[5])
        # digits 4 and 5 => meters over span
        self.over_meters = int(desc.id[3])
        # digits 2 and 3 => number of bridge panels
        self.n_panels = int(desc.id[1])
        # digit 1 is the load case, 1-based
        self.load_case = int(desc.id[0]) - 1 # -1 correction for 0-based load_case table
        
        self.grid_size = 0.25
        panel_size = 16
        self.over_grids = self.over_meters * 4
        self.under_grids = self.under_meters * 4
        
        self.num_length_grids = self.n_panels * panel_size
        self.n_loaded_joints = self.n_panels + 1
        
        # Loaded joints are prescribed.
        n_prescribed_joints = self.n_loaded_joints

        # ================================
        # Update Support Types and Joints
        # ================================
        
        # Add one prescribed joint for the intermediate support, if any.
        if((self.support_type == SupportTypes.INTERMEDIATE_SUPPORT) and (self.support_type != SupportTypes.HI_NOT_LO)):
            n_prescribed_joints += 1
        
        # Another two for the arch base, if we have an arch.
        if (self.support_type == SupportTypes.ARCH_SUPPORT):
            n_prescribed_joints += 2
        
        # And another one or two for cable anchorages if they're present.
        if (self.support_type == SupportTypes.CABLE_SUPPORT_LEFT):
            n_prescribed_joints += 1
        if (self.support_type == SupportTypes.CABLE_SUPPORT_RIGHT):
            n_prescribed_joints += 1
        
        # ================================
        # Fill prescribed_joints Vector
        # ================================
        self.prescribed_joints = [] # type: List[Joint]
        joint_index = 0
        x = 0
        y = 0
        for _ in range(self.n_loaded_joints):
            self.prescribed_joints.append(
                Joint(number = joint_index + 1, 
                      x = x,
                      y = y)
                )
            x += panel_size
            joint_index +=1
        
        # Loop leaves joint_index pointing at next joint.  Add the low intermediate support, if any.
        if ((self.support_type == SupportTypes.INTERMEDIATE_SUPPORT) and (self.support_type != SupportTypes.HI_NOT_LO)):
            self.prescribed_joints.append(
                Joint(number= joint_index + 1,
                      x = (self.intermediate_support_joint_no - 1) * panel_size,
                      y = -self.under_grids)
            )
            joint_index += 1
        
        # Add the arch base supports, if any
        if (self.support_type == SupportTypes.ARCH_SUPPORT):
            self.prescribed_joints.append(
                Joint(number= joint_index + 1,
                      x = 0,
                      y = -self.under_grids)
            )
            joint_index += 1
            
            self.prescribed_joints.append(
                Joint(number= joint_index + 1,
                      x = self.prescribed_joints[self.n_loaded_joints - 1].x,
                      y = -self.under_grids)
            )
            joint_index += 1
        
        # Add the cable anchorages, if any.
        if (self.support_type == SupportTypes.CABLE_SUPPORT_LEFT):
            self.prescribed_joints.append(
                Joint(number= joint_index + 1,
                      x = -CABLE_ANCHORAGE_X_OFFSET,
                      y = 0)
            )
            joint_index += 1
        if (self.support_type == SupportTypes.CABLE_SUPPORT_RIGHT):
            self.prescribed_joints.append(
                Joint(number= joint_index + 1,
                      x = self.prescribed_joints[self.n_loaded_joints - 1].x + CABLE_ANCHORAGE_X_OFFSET,
                      y = 0)
            )
            joint_index += 1
        self.n_prescribed_joints = n_prescribed_joints