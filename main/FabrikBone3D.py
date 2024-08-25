from enum import Enum

from FabrikJoint3D import FabrikJoint3D
from utils.Colour4f import Colour4f
from utils.Vec3f import Vec3f


class BoneConnectionPoint(Enum):
    START = 1
    END = 2

class FabrikBone3D:
    MIN_LINE_WIDTH = 1.0
    MAX_LINE_WIDTH = 64.0

    def __init__(self, start_location=None, end_location=None, direction_uv=None, length=None, name=None, colour=None):
        self.mJoint = FabrikJoint3D()
        self.mStartLocation = Vec3f()
        self.mEndLocation = Vec3f()
        self.mName = ""
        self.mLength = 0.0
        self.mColour = Colour4f()
        self.mLineWidth = 1.0
        self.mBoneConnectionPoint = BoneConnectionPoint.END

        if start_location and end_location:
            self.mStartLocation.set(start_location)
            self.mEndLocation.set(end_location)
            self.set_length(Vec3f.distance_between(start_location, end_location))
            if name:
                self.set_name(name)
        elif start_location and direction_uv and length:
            self.mStartLocation.set(start_location)
            self.mEndLocation.set(start_location.plus(direction_uv.normalised().times(length)))
            self.set_length(length)
            if name:
                self.set_name(name)
            if colour:
                self.set_colour(colour)

    def length(self):
        return self.mLength

    def live_length(self):
        return Vec3f.distance_between(self.mStartLocation, self.mEndLocation)

    def set_bone_connection_point(self, bcp):
        self.mBoneConnectionPoint = bcp

    def get_bone_connection_point(self):
        return self.mBoneConnectionPoint

    def get_colour(self):
        return self.mColour

    def set_colour(self, colour):
        self.mColour.set(colour)

    def get_line_width(self):
        return self.mLineWidth

    def get_start_location(self):
        return self.mStartLocation

    def get_start_location_as_array(self):
        return [self.mStartLocation.x, self.mStartLocation.y, self.mStartLocation.z]

    def get_end_location(self):
        return self.mEndLocation

    def get_end_location_as_array(self):
        return [self.mEndLocation.x, self.mEndLocation.y, self.mEndLocation.z]

    def set_joint(self, joint):
        self.mJoint.set(joint)

    def get_joint(self):
        return self.mJoint

    def get_joint_type(self):
        return self.mJoint.get_joint_type()

    def set_hinge_joint_clockwise_constraint_degs(self, angle_degs):
        self.mJoint.set_hinge_joint_clockwise_constraint_degs(angle_degs)

    def get_hinge_joint_clockwise_constraint_degs(self):
        return self.mJoint.get_hinge_clockwise_constraint_degs()

    def set_hinge_joint_anticlockwise_constraint_degs(self, angle_degs):
        self.mJoint.set_hinge_joint_anticlockwise_constraint_degs(angle_degs)

    def get_hinge_joint_anticlockwise_constraint_degs(self):
        return self.mJoint.get_hinge_anticlockwise_constraint_degs()

    def set_ball_joint_constraint_degs(self, angle_degs):
        self.mJoint.set_ball_joint_constraint_degs(angle_degs)

    def get_ball_joint_constraint_degs(self):
        return self.mJoint.get_ball_joint_constraint_degs()

    def get_direction_uv(self):
        return Vec3f.get_direction_uv(self.mStartLocation, self.mEndLocation)

    def get_global_pitch_degs(self):
        return self.get_direction_uv().get_global_pitch_degs()

    def get_global_yaw_degs(self):
        return self.get_direction_uv().get_global_yaw_degs()

    def set_line_width(self, line_width):
        if self.MIN_LINE_WIDTH <= line_width <= self.MAX_LINE_WIDTH:
            self.mLineWidth = line_width
        else:
            raise ValueError(f"Line width must be between {self.MIN_LINE_WIDTH} and {self.MAX_LINE_WIDTH} inclusive.")

    def set_name(self, name):
        self.mName = name[:100] if name else ""

    def get_name(self):
        return self.mName

    def __str__(self):
        return f"Start joint location : {self.mStartLocation}\nEnd   joint location : {self.mEndLocation}\nBone length          : {self.mLength}\nColour               : {self.mColour}"

    def set_start_location(self, location):
        self.mStartLocation.set(location)

    def set_end_location(self, location):
        self.mEndLocation.set(location)

    def set_length(self, length):
        if length > 0.0:
            self.mLength = length
        else:
            raise ValueError("Bone length must be a positive value.")
