import numpy as np
from enum import Enum
import copy

from FabrikJoint import FabrikJoint
from utils.Utils import Utils
from utils.Vec3f import Vec3f


class JointType(Enum):
    BALL = 1
    GLOBAL_HINGE = 2
    LOCAL_HINGE = 3

class FabrikJoint3D:
    MIN_CONSTRAINT_ANGLE_DEGS = 0.0
    MAX_CONSTRAINT_ANGLE_DEGS = 180.0
    NEW_LINE = "\n"

    def __init__(self, source=None):
        self.m_rotor_constraint_degs = self.MAX_CONSTRAINT_ANGLE_DEGS
        self.m_hinge_clockwise_constraint_degs = self.MAX_CONSTRAINT_ANGLE_DEGS
        self.m_hinge_anticlockwise_constraint_degs = self.MAX_CONSTRAINT_ANGLE_DEGS
        self.m_rotation_axis_uv = Vec3f()
        self.m_reference_axis_uv = Vec3f()
        self.m_joint_type = JointType.BALL

        if source is not None:
            self.set(source)

    def clone(self, source):
        return FabrikJoint3D(source)

    def set(self, source):
        self.m_joint_type = source.m_joint_type
        self.m_rotor_constraint_degs = source.m_rotor_constraint_degs
        self.m_hinge_clockwise_constraint_degs = source.m_hinge_clockwise_constraint_degs
        self.m_hinge_anticlockwise_constraint_degs = source.m_hinge_anticlockwise_constraint_degs
        self.m_rotation_axis_uv.set(source.m_rotation_axis_uv)
        self.m_reference_axis_uv.set(source.m_reference_axis_uv)

    def set_as_ball_joint(self, constraint_angle_degs):
        self.validate_constraint_angle_degs(constraint_angle_degs)
        self.m_rotor_constraint_degs = constraint_angle_degs
        self.m_joint_type = JointType.BALL

    def set_hinge(self, joint_type, rotation_axis, clockwise_constraint_degs, anticlockwise_constraint_degs, reference_axis):
        if not Utils.approximately_equals(Vec3f.dot(rotation_axis, reference_axis), 0.0, 0.01):
            angle_degs = Vec3f.get_angle_between_degs(rotation_axis, reference_axis)
            raise ValueError(f"The reference axis must be in the plane of the hinge rotation axis - angle between them is currently: {angle_degs}")

        self.validate_constraint_angle_degs(clockwise_constraint_degs)
        self.validate_constraint_angle_degs(anticlockwise_constraint_degs)
        self.validate_axis(rotation_axis)
        self.validate_axis(reference_axis)

        self.m_hinge_clockwise_constraint_degs = clockwise_constraint_degs
        self.m_hinge_anticlockwise_constraint_degs = anticlockwise_constraint_degs
        self.m_joint_type = joint_type
        self.m_rotation_axis_uv.set(rotation_axis.normalised())
        self.m_reference_axis_uv.set(reference_axis.normalised())

    def set_as_global_hinge(self, global_rotation_axis, cw_constraint_degs, acw_constraint_degs, global_reference_axis):
        self.set_hinge(JointType.GLOBAL_HINGE, global_rotation_axis, cw_constraint_degs, acw_constraint_degs, global_reference_axis)

    def set_as_local_hinge(self, local_rotation_axis, cw_constraint_degs, acw_constraint_degs, local_reference_axis):
        self.set_hinge(JointType.LOCAL_HINGE, local_rotation_axis, cw_constraint_degs, acw_constraint_degs, local_reference_axis)

    def get_hinge_clockwise_constraint_degs(self):
        if self.m_joint_type != JointType.BALL:
            return self.m_hinge_clockwise_constraint_degs
        else:
            raise RuntimeError("Joint type is JointType.BALL - it does not have hinge constraint angles.")

    def get_hinge_anticlockwise_constraint_degs(self):
        if self.m_joint_type != JointType.BALL:
            return self.m_hinge_anticlockwise_constraint_degs
        else:
            raise RuntimeError("Joint type is JointType.BALL - it does not have hinge constraint angles.")

    def set_ball_joint_constraint_degs(self, angle_degs):
        self.validate_constraint_angle_degs(angle_degs)
        if self.m_joint_type == JointType.BALL:
            self.m_rotor_constraint_degs = angle_degs
        else:
            raise RuntimeError(f"This joint is of type: {self.m_joint_type} - only joints of type JointType.BALL have a ball joint constraint angle.")

    def get_ball_joint_constraint_degs(self):
        if self.m_joint_type == JointType.BALL:
            return self.m_rotor_constraint_degs
        else:
            raise RuntimeError("This joint is not of type JointType.BALL - it does not have a ball joint constraint angle.")

    def set_hinge_joint_clockwise_constraint_degs(self, angle_degs):
        self.validate_constraint_angle_degs(angle_degs)
        if self.m_joint_type != JointType.BALL:
            self.m_hinge_clockwise_constraint_degs = angle_degs
        else:
            raise RuntimeError("Joint type is JointType.BALL - it does not have hinge constraint angles.")

    def set_hinge_joint_anticlockwise_constraint_degs(self, angle_degs):
        self.validate_constraint_angle_degs(angle_degs)
        if self.m_joint_type != JointType.BALL:
            self.m_hinge_anticlockwise_constraint_degs = angle_degs
        else:
            raise RuntimeError("Joint type is JointType.BALL - it does not have hinge constraint angles.")

    def set_hinge_rotation_axis(self, axis):
        self.validate_axis(axis)
        if self.m_joint_type != JointType.BALL:
            self.m_rotation_axis_uv.set(axis.normalised())
        else:
            raise RuntimeError("Joint type is JointType.BALL - it does not have a hinge rotation axis.")

    def get_hinge_reference_axis(self):
        if self.m_joint_type != JointType.BALL:
            return self.m_reference_axis_uv
        else:
            raise RuntimeError("Joint type is JointType.BALL - it does not have a hinge reference axis.")

    def set_hinge_reference_axis(self, reference_axis):
        self.validate_axis(reference_axis)
        if self.m_joint_type != JointType.BALL:
            self.m_reference_axis_uv.set(reference_axis.normalised())
        else:
            raise RuntimeError("Joint type is JointType.BALL - it does not have a hinge reference axis.")

    def get_hinge_rotation_axis(self):
        if self.m_joint_type != JointType.BALL:
            return self.m_rotation_axis_uv
        else:
            raise RuntimeError("Joint type is JointType.BALL - it does not have a hinge rotation axis.")

    def get_joint_type(self):
        return self.m_joint_type

    def __str__(self):
        sb = []
        if self.m_joint_type == JointType.BALL:
            sb.append(f"Joint type: Ball{self.NEW_LINE}")
            sb.append(f"Constraint angle: {self.m_rotor_constraint_degs}{self.NEW_LINE}")
        elif self.m_joint_type in [JointType.GLOBAL_HINGE, JointType.LOCAL_HINGE]:
            if self.m_joint_type == JointType.GLOBAL_HINGE:
                sb.append(f"Joint type                    : Global hinge{self.NEW_LINE}")
            else:
                sb.append(f"Joint type                    : Local hinge{self.NEW_LINE}")
            sb.append(f"Rotation axis                 : {self.m_rotation_axis_uv}{self.NEW_LINE}")
            sb.append(f"Reference axis                : {self.m_reference_axis_uv}{self.NEW_LINE}")
            sb.append(f"Anticlockwise constraint angle: {self.m_hinge_clockwise_constraint_degs}{self.NEW_LINE}")
            sb.append(f"Clockwise constraint angle    : {self.m_hinge_clockwise_constraint_degs}{self.NEW_LINE}")
        return ''.join(sb)

    @staticmethod
    def validate_constraint_angle_degs(angle_degs):
        if angle_degs < FabrikJoint3D.MIN_CONSTRAINT_ANGLE_DEGS or angle_degs > FabrikJoint3D.MAX_CONSTRAINT_ANGLE_DEGS:
            raise ValueError(f"Constraint angles must be within the range {FabrikJoint3D.MIN_CONSTRAINT_ANGLE_DEGS} to {FabrikJoint3D.MAX_CONSTRAINT_ANGLE_DEGS} inclusive.")

    @staticmethod
    def validate_axis(axis):
        if axis.length() <= 0.0:
            raise ValueError("Provided axis is illegal - it has a magnitude of zero.")

    def __hash__(self):
        return hash((self.m_rotor_constraint_degs, self.m_hinge_clockwise_constraint_degs, self.m_hinge_anticlockwise_constraint_degs,
                     self.m_rotation_axis_uv, self.m_reference_axis_uv, self.m_joint_type))

    def __eq__(self, other):
        if isinstance(other, FabrikJoint3D):
            return (Utils.approximately_equals(self.m_rotor_constraint_degs, other.m_rotor_constraint_degs) and
                    Utils.approximately_equals(self.m_hinge_clockwise_constraint_degs, other.m_hinge_clockwise_constraint_degs) and
                    Utils.approximately_equals(self.m_hinge_anticlockwise_constraint_degs, other.m_hinge_anticlockwise_constraint_degs) and
                    self.m_rotation_axis_uv == other.m_rotation_axis_uv and
                    self.m_reference_axis_uv == other.m_reference_axis_uv and
                    self.m_joint_type == other.m_joint_type)