import krpc  
import time 
import numpy as np
import math  
from math import *
from coord_rotations import *
from PIDtools import PositionPID
from numpy.linalg import norm
import keyboard
import copy

class rocket_control(object):
    def __init__(self, vessel):
        self.name = vessel.name
        self.flight_surface = self.vessel.flight(vessel.surface_reference_frame)
    def slow_descend_controll(self,):
        pass
    def auto_hight_maintainence(self,):
        pass

