import numpy as np
from enum import Enum

from FabrikJoint import FabrikJoint


class ConstraintCoordinateSystem(Enum):
    LOCAL = 1
    GLOBAL = 2

class FabrikJoint2D(FabrikJoint):
    MIN_2D_CONSTRAINT_ANGLE_DEGS = 0.0
    MAX_2D_CONSTRAINT_ANGLE_DEGS = 180.0


    def __init__(self, clockwise_constraint_degs=MAX_2D_CONSTRAINT_ANGLE_DEGS,
                 anticlockwise_constraint_degs=MAX_2D_CONSTRAINT_ANGLE_DEGS,
                 constraint_coord_system=ConstraintCoordinateSystem.LOCAL):
        super().__init__()
        self.m_anticlockwise_constraint_degs = None
        self.m_clockwise_constraint_degs = None
        self.set_clockwise_constraint_degs(clockwise_constraint_degs)
        self.set_anticlockwise_constraint_degs(anticlockwise_constraint_degs)
        self.constraint_coordinate_system = constraint_coord_system

    def set(self, source_joint):
        self.set_clockwise_constraint_degs(source_joint.m_clockwise_constraint_degs)
        self.set_anticlockwise_constraint_degs(source_joint.m_anticlockwise_constraint_degs)
        self.constraint_coordinate_system = source_joint.constraint_coordinate_system

    def set_clockwise_constraint_degs(self, angle_degs):
        self.m_clockwise_constraint_degs = max(min(angle_degs, self.MAX_2D_CONSTRAINT_ANGLE_DEGS), self.MIN_2D_CONSTRAINT_ANGLE_DEGS)

    def set_anticlockwise_constraint_degs(self, angle_degs):
        self.m_anticlockwise_constraint_degs = max(min(angle_degs, self.MAX_2D_CONSTRAINT_ANGLE_DEGS), self.MIN_2D_CONSTRAINT_ANGLE_DEGS)

    def get_clockwise_constraint_degs(self):
        return self.m_clockwise_constraint_degs

    def get_anticlockwise_constraint_degs(self):
        return self.m_anticlockwise_constraint_degs

    def get_constraint_coordinate_system(self):
        return self.constraint_coordinate_system

    def set_constraint_coordinate_system(self, coord_system):
        self.constraint_coordinate_system = coord_system

    def __hash__(self):
        return hash((self.m_clockwise_constraint_degs, self.m_anticlockwise_constraint_degs))

    def __eq__(self, other):
        if isinstance(other, FabrikJoint2D):
            return (self.m_clockwise_constraint_degs == other.m_clockwise_constraint_degs and
                    self.m_anticlockwise_constraint_degs == other.m_anticlockwise_constraint_degs)
        return False