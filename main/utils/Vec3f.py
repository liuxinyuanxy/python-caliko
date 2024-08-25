from __future__ import annotations
import numpy as np
from enum import Enum
from typing import List, Optional

class Vec3f:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def set(self, other):
        self.x = other.x
        self.y = other.y
        self.z = other.z

    DEGS_TO_RADS = np.pi / 180.0
    RADS_TO_DEGS = 180.0 / np.pi

    def approximately_equals(self, v, tolerance):
        if tolerance < 0.0:
            raise ValueError("Equality threshold must be greater than or equal to 0.0f")
        return abs(self.x - v.x) < tolerance and abs(self.y - v.y) < tolerance and abs(self.z - v.z) < tolerance

    def zero(self):
        self.x = self.y = self.z = 0.0

    def negate(self):
        self.x = -self.x
        self.y = -self.y
        self.z = -self.z
        return self

    def to_array(self):
        return [self.x, self.y, self.z]

    def negated(self):
        return Vec3f(-self.x, -self.y, -self.z)

    def normalise(self):
        magnitude = self.length()
        if magnitude > 0.0:
            self.x /= magnitude
            self.y /= magnitude
            self.z /= magnitude
        return self

    def normalised(self):
        return Vec3f(self.x, self.y, self.z).normalise()

    def length(self):
        return np.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def abs(self):
        return Vec3f(abs(self.x), abs(self.y), abs(self.z))

    def plus(self, v):
        return Vec3f(self.x + v.x, self.y + v.y, self.z + v.z)

    def minus(self, v):
        return Vec3f(self.x - v.x, self.y - v.y, self.z - v.z)

    def times(self, value):
        return Vec3f(self.x * value, self.y * value, self.z * value)

    def divided_by(self, value):
        return Vec3f(self.x / value, self.y / value, self.z / value)

    def dot(self, v):
        return self.x * v.x + self.y * v.y + self.z * v.z

    def cross(self, v):
        return Vec3f(self.y * v.z - self.z * v.y, self.z * v.x - self.x * v.z, self.x * v.y - self.y * v.x)

    def distance_between(self, v):
        dx = v.x - self.x
        dy = v.y - self.y
        dz = v.z - self.z
        return np.sqrt(dx * dx + dy * dy + dz * dz)

    def manhattan_distance_between(self, v):
        return abs(v.x - self.x) + abs(v.y - self.y) + abs(v.z - self.z)

    def within_manhattan_distance(self, v, distance):
        return abs(v.x - self.x) <= distance and abs(v.y - self.y) <= distance and abs(v.z - self.z) <= distance

    def gen_perpendicular_vector_quick(self):
        if abs(self.y) < 0.99:
            return Vec3f(-self.z, 0.0, self.x).normalise()
        else:
            return Vec3f(0.0, self.z, -self.y).normalise()

    def gen_perpendicular_vector_hm(self):
        a = self.abs()
        if a.x <= a.y and a.x <= a.z:
            return Vec3f(0.0, -self.z, self.y).normalise()
        elif a.y <= a.x and a.y <= a.z:
            return Vec3f(-self.z, 0.0, self.x).normalise()
        else:
            return Vec3f(-self.y, self.x, 0.0).normalise()

    def gen_perpendicular_vector_frisvad(self):
        if self.z < -0.9999999:
            return Vec3f(0.0, -1.0, 0.0)
        a = 1.0 / (1.0 + self.z)
        return Vec3f(1.0 - self.x * self.x * a, -self.x * self.y * a, -self.x).normalise()

    def get_uv_between(self, v):
        return v.minus(self).normalise()

    def get_angle_between_rads(self, v):
        return np.arccos(self.normalised().dot(v.normalised()))

    def get_angle_between_degs(self, v):
        return self.get_angle_between_rads(v) * Vec3f.RADS_TO_DEGS

    def get_signed_angle_between_degs(self, v, normal):
        unsigned_angle = self.get_angle_between_degs(v)
        sign = np.sign(self.cross(v).dot(normal))
        return unsigned_angle * sign

    def get_angle_limited_unit_vector_degs(self, baseline, angle_limit_degs):
        angle_between_vectors_degs = self.get_angle_between_degs(baseline)
        if angle_between_vectors_degs > angle_limit_degs:
            correction_axis = self.cross(baseline).normalise()
            return baseline.rotate_about_axis_degs(angle_limit_degs, correction_axis).normalise()
        else:
            return self.normalise()

    def get_global_pitch_degs(self):
        x_projected = self.project_onto_plane(Vec3f(1.0, 0.0, 0.0))
        pitch = Vec3f(0.0, 0.0, 1.0).negated().get_angle_between_degs(x_projected)
        return -pitch if x_projected.y < 0.0 else pitch

    def get_global_yaw_degs(self):
        y_projected = self.project_onto_plane(Vec3f(0.0, 1.0, 0.0))
        yaw = Vec3f(0.0, 0.0, 1.0).negated().get_angle_between_degs(y_projected)
        return -yaw if y_projected.x < 0.0 else yaw

    def rotate_x_rads(self, angle_rads):
        cos_theta = np.cos(angle_rads)
        sin_theta = np.sin(angle_rads)
        return Vec3f(self.x, self.y * cos_theta - self.z * sin_theta, self.y * sin_theta + self.z * cos_theta)

    def rotate_x_degs(self, angle_degs):
        return self.rotate_x_rads(angle_degs * Vec3f.DEGS_TO_RADS)

    def rotate_y_rads(self, angle_rads):
        cos_theta = np.cos(angle_rads)
        sin_theta = np.sin(angle_rads)
        return Vec3f(self.z * sin_theta + self.x * cos_theta, self.y, self.z * cos_theta - self.x * sin_theta)

    def rotate_y_degs(self, angle_degs):
        return self.rotate_y_rads(angle_degs * Vec3f.DEGS_TO_RADS)

    def rotate_z_rads(self, angle_rads):
        cos_theta = np.cos(angle_rads)
        sin_theta = np.sin(angle_rads)
        return Vec3f(self.x * cos_theta - self.y * sin_theta, self.x * sin_theta + self.y * cos_theta, self.z)

    def rotate_z_degs(self, angle_degs):
        return self.rotate_z_rads(angle_degs * Vec3f.DEGS_TO_RADS)

    def rotate_about_axis_rads(self, angle_rads, rotation_axis):
        sin_theta = np.sin(angle_rads)
        cos_theta = np.cos(angle_rads)
        one_minus_cos_theta = 1.0 - cos_theta
        xy_one = rotation_axis.x * rotation_axis.y * one_minus_cos_theta
        xz_one = rotation_axis.x * rotation_axis.z * one_minus_cos_theta
        yz_one = rotation_axis.y * rotation_axis.z * one_minus_cos_theta
        rotation_matrix = np.array([
            [rotation_axis.x * rotation_axis.x * one_minus_cos_theta + cos_theta, xy_one + rotation_axis.z * sin_theta, xz_one - rotation_axis.y * sin_theta],
            [xy_one - rotation_axis.z * sin_theta, rotation_axis.y * rotation_axis.y * one_minus_cos_theta + cos_theta, yz_one + rotation_axis.x * sin_theta],
            [xz_one + rotation_axis.y * sin_theta, yz_one - rotation_axis.x * sin_theta, rotation_axis.z * rotation_axis.z * one_minus_cos_theta + cos_theta]
        ])
        return Vec3f(*np.dot(rotation_matrix, [self.x, self.y, self.z]))

    def rotate_about_axis_degs(self, angle_degs, rotation_axis):
        return self.rotate_about_axis_rads(angle_degs * Vec3f.DEGS_TO_RADS, rotation_axis)

    def project_onto_plane(self, plane_normal):
        if plane_normal.length() == 0.0:
            raise ValueError("Plane normal cannot be a zero vector.")
        b = self.normalised()
        n = plane_normal.normalised()
        return b.minus(n.times(b.dot(plane_normal))).normalise()

    def randomise(self, min_val, max_val):
        self.x = np.random.uniform(min_val, max_val)
        self.y = np.random.uniform(min_val, max_val)
        self.z = np.random.uniform(min_val, max_val)

    @staticmethod
    def get_direction_uv(a, b):
        return b.minus(a).normalise()

    # 	public static boolean perpendicular(Vec3f a, Vec3f b)
    # 	{
    # 		return Utils.approximatelyEquals( Vec3f.dotProduct(a, b), 0.0f, 0.01f ) ? true : false;
    # 	}
    @staticmethod
    def perpendicular(a, b):
        return np.isclose(a.dot(b), 0.0, atol=0.01)

    def __str__(self):
        return f"x: {self.x:.3f}, y: {self.y:.3f}, z: {self.z:.3f}"

    def __eq__(self, other):
        if isinstance(other, Vec3f):
            return np.isclose(self.x, other.x) and np.isclose(self.y, other.y) and np.isclose(self.z, other.z)
        return False

    def __hash__(self):
        return hash((self.x, self.y, self.z))

X_AXIS = Vec3f(1.0, 0.0, 0.0)
Y_AXIS = Vec3f(0.0, 1.0, 0.0)
Z_AXIS = Vec3f(0.0, 0.0, 1.0)