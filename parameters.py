from typing import List
from enum import Enum

N_AVAILIBLE_MATERIALS = 3   # Number of available materials
N_SECTIONS = 2              # Number of available sections

# available sections themselves (must match NSections above)
class Section(Enum):
    Bar = 0
    Tube = 1

def get_section(index: int):
    if index == 0:
        return Section.Bar
    elif index == 1:
        return Section.Tube
   
N_BAR_SIZES = 33
N_TUBE_SIZES = 33
MAX_N_SIZES = 33
constant_n_sizes = {0:N_BAR_SIZES, 1: N_TUBE_SIZES}

# From Steve, 15 Nov 2004. Last year these were computed.
_width_tbl = [
    30,35,40,45,50,55,60,65,70,75,80,				# 0 to 10
    90,100,110,120,130,140,150,160,170,180,190,200, # 11 to 22
    220,240,260,280,300,                            # 23 to 27
    320,340,360,400,500                             # 28 to 32
]

def n_sizes(section: Section) -> int:
    if section < N_SECTIONS:
        return constant_n_sizes[section]
    else:
        return 0

def _sqr(x: float):
    """Square a float"""
    return x * x

def _p4(x: float):
    """Compute fourth power of a float"""
    return _sqr(_sqr(x))



class Material():
    def __init__(self, 
                 name: str, 
                 short_name: str, 
                 E: float,
                 Fy: float,
                 density: float,
                 cost: List[float]) -> None:
        self.name = name                # Name as string.
        self.short_name = short_name    # Name as a short string (for tables)
        self.E = E                      # Modulus of elasticity
        self.Fy = Fy                    # Yield strength
        self.density = density          # Density in Kg/m^3
        self.cost = cost                # Cost in dollars per cm^3 by section [Bar, Tube]
    
class Shape():
    def __init__(self, name: str, width: float, area: float, moment: float) -> None:
        self.name = name        # Name as string
        self.width = width      # Width dimension
        self.area = area        # Cross-sectional area, m^3
        self.moment = moment    # First moment
        
    
DEAD_LOAD_FACTOR = 1.35
N_LOAD_CASES = 4

class LoadCase():
    def __init__(self, 
                 name: str, 
                 point_dead_load: float, 
                 front_axle_load: float,
                 rear_axle_load: float) -> None:
        self.name = name                        # Name as string
        self.point_dead_load = point_dead_load  # Dead load due to roadway
        self.front_axle_load = front_axle_load  # Live load due to front axle
        self.rear_axle_load = rear_axle_load    # Live load due to rear axle

class Params():
    def __init__(self) -> None:
        self.slenderness_limit = 300.0              # The least illegal slenderness value.
        self.dead_load_factor = DEAD_LOAD_FACTOR    # Dead load safety factor
        self.live_load_factor = 1.75 * 1.33         # Live load safety factor
        self.compression_resistance_factor = 0.90   # Compression loading safety factor
        self.tension_resistance_factor = 0.95       # Tension loading safety factor
        
        # ===================
        # Setup Load Cases
        # ===================
        self.load_cases = []                        # type: List[LoadCase]
        self.load_cases.append(
            LoadCase(name="Case A (Heavy Deck, Light Truck)",
                     point_dead_load=DEAD_LOAD_FACTOR * 120.265 + 33.097,
                     front_axle_load=44.0,
                     rear_axle_load=181.0)
        )
        self.load_cases.append(
            LoadCase(name="Case B (Heavy Deck, Heavy Truck)",
                     point_dead_load=DEAD_LOAD_FACTOR * 120.265 + 33.097,
                     front_axle_load=124.0,
                     rear_axle_load=124.0)
        )
        self.load_cases.append(
            LoadCase(name="Case C (Light Deck, Light Truck)",
                     point_dead_load=DEAD_LOAD_FACTOR * 82.608 + 33.097,
                     front_axle_load=44.0,
                     rear_axle_load=181.0)
        )
        self.load_cases.append(
            LoadCase(name="Case D (Light Deck, Heavy Truck)",
                     point_dead_load=DEAD_LOAD_FACTOR * 82.608 + 33.097,
                     front_axle_load=124.0,
                     rear_axle_load=124.0)
        )
        self.connection_cost = 400.0                # Cost of connecting members at a joint
        self.ordering_fee = 1000.0                  # Fee per cross-section used
        
        # ===================
        # Setup Materials
        # ===================
        self.materials = []                         # Descriptions of available materials
        self.materials.append(
            Material(name="Carbon Steel (A36)",
                     short_name="CS",
                     E=200000000.0,
                     Fy=250000.0,
                     density=7850.0,
                     cost=[4.30,6.30])
        )
        self.materials.append(
            Material(name="High Strength Steel (A572)",
                     short_name="HSS",
                     E=200000000.0,
                     Fy=345000.0,
                     density=7850.0,
                     cost=[5.60,7.00])
        )
        self.materials.append(
            Material(name="Quenched & Tempered Steel",
                     short_name="QTS",
                     E=200000000.0,
                     Fy=485000.0,
                     density=7850.0,
                     cost=[6.00,7.70])
        )
        self.n_sizes = [N_BAR_SIZES, N_TUBE_SIZES]  # Number of sizes available by section.
       
        # ===================
        # Setup Shapes
        # ===================
        self.shapes = [] # type: List[List[Shape]]
        for section_index in range(N_SECTIONS):
            sections = []
            for size_index in range(self.n_sizes[section_index]):
                width = _width_tbl[size_index]
                
                if section_index == 0: # Section.Bar
                    sections.append(
                        Shape(name=f"{width}x{width}",
                              width=width,
                              area=_sqr(width) * 1e-6,
                              moment=_p4(width) / 12 * 1e-12)
                    )
                elif section_index == 1: # Section.Tube
                    tube_thickness = width / 20
                    if (tube_thickness < 2):
                        tube_thickness = 2
                    sections.append(
                        Shape(name=f"{width}x{width}x{tube_thickness}",
                              width=width,
                              area=( _sqr(width) - _sqr(width - 2 * tube_thickness) ) * 1e-6,
                              moment=( _p4(width) - _p4(width -  2 * tube_thickness) ) / 12 * 1e-12)
                    )                      
            self.shapes.append(sections)
        

