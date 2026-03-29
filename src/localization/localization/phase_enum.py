from enum import Enum

class Phase(Enum):
    INIT = 1
    EXPLORE = 2           
    OBSTACLE = 3          
    RESCAN = 4            
    RECOVERY = 5          
    DONE = 6              
