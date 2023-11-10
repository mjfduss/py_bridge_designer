from math import sqrt
from py_bridge_designer.parameters import Params, Section

def _world_coords(grid, grid_size):
    """Converts grid coordinates to physical world coordinates"""
    return grid * grid_size
    
class Joint():
    def __init__(self, number: int, x: int, y: int):
        self.number = number
        self.x = x
        self.y = y

class CrossSection():
    def __init__(self, 
                 parameters: Params, 
                 material_index: int, 
                 section: Section, 
                 size: int):
        self.material = parameters.materials[material_index] # Between 0 and 2
        self.section = section # either Bar or Tube
        self.size = size # Between 0 and 32

class Member():
    def __init__(self, 
        number: int, 
        start_joint: Joint, 
        end_joint: Joint, 
        cross_section: CrossSection, 
        grid_size: float):
        
        self.number = number
        self.start_joint = start_joint 
        self.end_joint = end_joint 
        self.cross_section = cross_section
        self.compression = 0.0
        self.tension = 0.0
        
        # ======================
        # Setup Member Geometry
        # ======================
        start = self.start_joint
        end = self.end_joint
        dx = _world_coords(end.x - start.x, grid_size)
        dy = _world_coords(end.y - start.y, grid_size)
        len = sqrt(dx * dx + dy * dy)
        self.length = len
        self.cos_x = dx / len
        self.cos_y = dy / len
