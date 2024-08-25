import numpy as np

from utils.Vec3f import Vec3f


class Mat3f:
    DEGS_TO_RADS = np.pi / 180.0

    def __init__(self, value=0.0, m00=None, m01=None, m02=None, m10=None, m11=None, m12=None, m20=None, m21=None, m22=None):
        if m00 is not None:
            self.m = np.array([
                [m00, m01, m02],
                [m10, m11, m12],
                [m20, m21, m22]
            ])
        elif value != 0.0:
            self.m = np.eye(3) * value
        else:
            self.m = np.zeros((3, 3))

    def zero(self):
        self.m = np.zeros((3, 3))

    def set_identity(self):
        self.m = np.eye(3)

    @staticmethod
    def transpose(m):
        return Mat3f(m00=m.m[0, 0], m01=m.m[1, 0], m02=m.m[2, 0],
                     m10=m.m[0, 1], m11=m.m[1, 1], m12=m.m[2, 1],
                     m20=m.m[0, 2], m21=m.m[1, 2], m22=m.m[2, 2])

    @staticmethod
    def create_rotation_matrix(reference_direction):
        z_axis = reference_direction.normalised()
        if abs(reference_direction.y) > 0.9999:
            x_axis = Vec3f(1.0, 0.0, 0.0)
            y_axis = Vec3f.cross(x_axis, z_axis).normalised()
        else:
            x_axis = Vec3f.cross(reference_direction, Vec3f(0.0, 1.0, 0.0)).normalised()
            y_axis = Vec3f.cross(x_axis, z_axis).normalised()
        return Mat3f(m00=x_axis.x, m01=x_axis.y, m02=x_axis.z,
                     m10=y_axis.x, m11=y_axis.y, m12=y_axis.z,
                     m20=z_axis.x, m21=z_axis.y, m22=z_axis.z)

    def is_orthogonal(self):
        x_cross_y_dot = Vec3f.dot(self.get_x_basis(), self.get_y_basis())
        x_cross_z_dot = Vec3f.dot(self.get_x_basis(), self.get_z_basis())
        y_cross_z_dot = Vec3f.dot(self.get_y_basis(), self.get_z_basis())
        return np.isclose(x_cross_y_dot, 0.0, atol=0.01) and np.isclose(x_cross_z_dot, 0.0, atol=0.01) and np.isclose(y_cross_z_dot, 0.0, atol=0.01)

    def times(self, other):
        if isinstance(other, Mat3f):
            return Mat3f(m00=np.dot(self.m[0], other.m[:, 0]), m01=np.dot(self.m[0], other.m[:, 1]), m02=np.dot(self.m[0], other.m[:, 2]),
                         m10=np.dot(self.m[1], other.m[:, 0]), m11=np.dot(self.m[1], other.m[:, 1]), m12=np.dot(self.m[1], other.m[:, 2]),
                         m20=np.dot(self.m[2], other.m[:, 0]), m21=np.dot(self.m[2], other.m[:, 1]), m22=np.dot(self.m[2], other.m[:, 2]))
        elif isinstance(other, Vec3f):
            return Vec3f(*np.dot(self.m, [other.x, other.y, other.z]))

    def determinant(self):
        return np.linalg.det(self.m)

    @staticmethod
    def inverse(m):
        inv_m = np.linalg.inv(m.m)
        return Mat3f(m00=inv_m[0, 0], m01=inv_m[0, 1], m02=inv_m[0, 2],
                     m10=inv_m[1, 0], m11=inv_m[1, 1], m12=inv_m[1, 2],
                     m20=inv_m[2, 0], m21=inv_m[2, 1], m22=inv_m[2, 2])

    def rotate_rads(self, rotation_axis, angle_rads):
        sin = np.sin(angle_rads)
        cos = np.cos(angle_rads)
        one_minus_cos = 1.0 - cos
        xy = rotation_axis.x * rotation_axis.y
        yz = rotation_axis.y * rotation_axis.z
        xz = rotation_axis.x * rotation_axis.z
        xs = rotation_axis.x * sin
        ys = rotation_axis.y * sin
        zs = rotation_axis.z * sin
        f00 = rotation_axis.x * rotation_axis.x * one_minus_cos + cos
        f01 = xy * one_minus_cos + zs
        f02 = xz * one_minus_cos - ys
        f10 = xy * one_minus_cos - zs
        f11 = rotation_axis.y * rotation_axis.y * one_minus_cos + cos
        f12 = yz * one_minus_cos + xs
        f20 = xz * one_minus_cos + ys
        f21 = yz * one_minus_cos - xs
        f22 = rotation_axis.z * rotation_axis.z * one_minus_cos + cos
        rotation_matrix = np.array([
            [f00, f01, f02],
            [f10, f11, f12],
            [f20, f21, f22]
        ])
        result = np.dot(self.m, rotation_matrix)
        return Mat3f(m00=result[0, 0], m01=result[0, 1], m02=result[0, 2],
                     m10=result[1, 0], m11=result[1, 1], m12=result[1, 2],
                     m20=result[2, 0], m21=result[2, 1], m22=result[2, 2])

    def rotate_degs(self, angle_degs, local_axis):
        return self.rotate_rads(local_axis, angle_degs * Mat3f.DEGS_TO_RADS)

    def set_x_basis(self, v):
        self.m[0, :] = [v.x, v.y, v.z]

    def get_x_basis(self):
        return Vec3f(*self.m[0, :])

    def set_y_basis(self, v):
        self.m[1, :] = [v.x, v.y, v.z]

    def get_y_basis(self):
        return Vec3f(*self.m[1, :])

    def set_z_basis(self, v):
        self.m[2, :] = [v.x, v.y, v.z]

    def get_z_basis(self):
        return Vec3f(*self.m[2, :])

    def to_array(self):
        return self.m.flatten()

    def __str__(self):
        return f"X Axis: {self.m[0, 0]:.3f}, {self.m[0, 1]:.3f}, {self.m[0, 2]:.3f}\n" \
               f"Y Axis: {self.m[1, 0]:.3f}, {self.m[1, 1]:.3f}, {self.m[1, 2]:.3f}\n" \
               f"Z Axis: {self.m[2, 0]:.3f}, {self.m[2, 1]:.3f}, {self.m[2, 2]:.3f}"