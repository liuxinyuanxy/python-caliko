import numpy as np

from FabrikJoint2D import FabrikJoint2D
from utils.Colour4f import Colour4f
from utils.Vec2f import Vec2f

class FabrikBone2D:
    def __init__(self, start_location=None, end_location=None, direction_uv=None, length=None, cw_constraint_degs=None, acw_constraint_degs=None, colour=None):
        self.mJoint = FabrikJoint2D()
        self.mStartLocation = Vec2f()
        self.mEndLocation = Vec2f()
        self.mName = ""
        self.mLength = 0.0
        self.mGlobalConstraintUV = Vec2f(1.0, 0.0)
        self.mColour = Colour4f()
        self.mLineWidth = 1.0

        if start_location and end_location:
            self.mStartLocation.set(start_location)
            self.mEndLocation.set(end_location)
            self.set_length(Vec2f.distance_between(start_location, end_location))
        elif start_location and direction_uv and length:
            self.mStartLocation.set(start_location)
            self.mEndLocation.set(start_location.plus(direction_uv.normalised().times(length)))
            self.set_length(length)
            if cw_constraint_degs is not None and acw_constraint_degs is not None:
                self.set_clockwise_constraint_degs(cw_constraint_degs)
                self.set_anticlockwise_constraint_degs(acw_constraint_degs)
            if colour:
                self.mColour.set(colour)

    def length(self):
        return self.mLength

    def get_colour(self):
        return self.mColour

    def set_colour(self, colour):
        self.mColour.set(colour)

    def get_line_width(self):
        return self.mLineWidth

    def get_start_location(self):
        return self.mStartLocation

    def get_start_location_as_array(self):
        return [self.mStartLocation.x, self.mStartLocation.y]

    def get_end_location(self):
        return self.mEndLocation

    def get_end_location_as_array(self):
        return [self.mEndLocation.x, self.mEndLocation.y]

    def set_joint(self, joint):
        self.mJoint.set(joint)

    def get_joint(self):
        return self.mJoint

    def set_clockwise_constraint_degs(self, angle_degs):
        self.mJoint.set_clockwise_constraint_degs(angle_degs)

    def get_clockwise_constraint_degs(self):
        return self.mJoint.get_clockwise_constraint_degs()

    def set_anticlockwise_constraint_degs(self, angle_degs):
        self.mJoint.set_anticlockwise_constraint_degs(angle_degs)

    def get_anticlockwise_constraint_degs(self):
        return self.mJoint.get_anticlockwise_constraint_degs()

    def get_direction_uv(self):
        return Vec2f.get_direction_uv(self.mStartLocation, self.mEndLocation)

    def get_global_constraint_uv(self):
        return self.mGlobalConstraintUV

    def set_global_constraint_uv(self, v):
        self.mGlobalConstraintUV = v

    def set_line_width(self, line_width):
        self.mLineWidth = np.clip(line_width, 1.0, 64.0)

    def set_name(self, name):
        self.mName = name[:100] if name else ""

    def get_name(self):
        return self.mName

    def get_joint_constraint_coordinate_system(self):
        return self.mJoint.get_constraint_coordinate_system()

    def set_joint_constraint_coordinate_system(self, coord_system):
        self.mJoint.set_constraint_coordinate_system(coord_system)

    def __str__(self):
        sb = [f"Start joint location : {self.mStartLocation}\n", f"End   joint location : {self.mEndLocation}\n",
              f"Bone direction       : {Vec2f.get_direction_uv(self.mStartLocation, self.mEndLocation)}\n",
              f"Bone length          : {self.mLength}\n", str(self.mJoint)]
        return ''.join(sb)

    def set_start_location(self, location):
        self.mStartLocation.set(location)

    def set_end_location(self, location):
        self.mEndLocation.set(location)

    def set_length(self, length):
        if length >= 0.0:
            self.mLength = length
        else:
            raise ValueError("Bone length must be a positive value.")

