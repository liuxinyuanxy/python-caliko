import numpy as np

from utils.Mat3f import Mat3f
from utils.Vec3f import Vec3f


class Mat4f:
    DEGS_TO_RADS = np.pi / 180.0

    def __init__(self, value=0.0, source=None, rotation_matrix=None, origin=None):
        if source is not None:
            if len(source) != 16:
                raise ValueError("Source array must contain precisely 16 floats.")
            self.m = np.array(source).reshape(4, 4)
        elif rotation_matrix is not None and origin is not None:
            self.m = np.eye(4)
            self.m[:3, :3] = rotation_matrix.m
            self.m[:3, 3] = origin.x, origin.y, origin.z
        elif value != 0.0:
            self.m = np.eye(4) * value
        else:
            self.m = np.zeros((4, 4))

    def zero(self):
        self.m = np.zeros((4, 4))

    def set_identity(self):
        self.m = np.eye(4)

    def set_from_array(self, source):
        if len(source) != 16:
            raise ValueError("Source array must contain precisely 16 floats.")
        self.m = np.array(source).reshape(4, 4)

    def get_x_basis(self):
        return Vec3f(*self.m[0, :3])

    def get_x_basis_array(self):
        return self.m[0, :3]

    def set_x_basis(self, v):
        if not np.isclose(v.length(), 1.0, atol=0.0001):
            v = v.normalised()
        self.m[0, :3] = v.x, v.y, v.z

    def set_x_basis_from_array(self, f):
        if len(f) != 3:
            return
        v = Vec3f(*f)
        if not np.isclose(v.length(), 1.0, atol=0.0001):
            v = v.normalised()
        self.m[0, :3] = v.x, v.y, v.z

    def get_y_basis(self):
        return Vec3f(*self.m[1, :3])

    def get_y_basis_array(self):
        return self.m[1, :3]

    def set_y_basis(self, v):
        if not np.isclose(v.length(), 1.0, atol=0.0001):
            v = v.normalised()
        self.m[1, :3] = v.x, v.y, v.z

    def set_y_basis_from_array(self, f):
        if len(f) != 3:
            return
        v = Vec3f(*f)
        if not np.isclose(v.length(), 1.0, atol=0.0001):
            v = v.normalised()
        self.m[1, :3] = v.x, v.y, v.z

    def get_z_basis(self):
        return Vec3f(*self.m[2, :3])

    def get_z_basis_array(self):
        return self.m[2, :3]

    def set_z_basis(self, v):
        if not np.isclose(v.length(), 1.0, atol=0.0001):
            v = v.normalised()
        self.m[2, :3] = v.x, v.y, v.z

    def set_z_basis_from_array(self, f):
        if len(f) != 3:
            return
        v = Vec3f(*f)
        if not np.isclose(v.length(), 1.0, atol=0.0001):
            v = v.normalised()
        self.m[2, :3] = v.x, v.y, v.z

    def get_origin(self):
        return Vec3f(*self.m[3, :3])

    def set_origin(self, v):
        self.m[3, :3] = v.x, v.y, v.z
        self.m[3, 3] = 1.0

    def is_orthogonal(self):
        x_axis = self.m[0, :3]
        y_axis = self.m[1, :3]
        z_axis = self.m[2, :3]
        x_dot_y = np.dot(x_axis, y_axis)
        x_dot_z = np.dot(x_axis, z_axis)
        y_dot_z = np.dot(y_axis, z_axis)
        return np.isclose(x_dot_y, 0.0, atol=0.01) and np.isclose(x_dot_z, 0.0, atol=0.01) and np.isclose(y_dot_z, 0.0, atol=0.01)

    def transpose(self):
        self.m = self.m.T
        return self

    def transposed(self):
        return Mat4f(source=self.m.T.flatten())

    def times(self, other):
        if isinstance(other, Mat4f):
            return Mat4f(source=(self.m @ other.m).flatten())
        elif isinstance(other, Vec3f):
            return Vec3f(*(self.m @ np.append(other.to_array(), 1.0)))

    def transform_point(self, v):
        return Vec3f(*(self.m @ np.append(v.to_array(), 1.0))[:3])

    def transform_direction(self, v):
        return Vec3f(*(self.m[:3, :3] @ v.to_array()))

    @staticmethod
    def inverse(m):
        try:
            inv_m = np.linalg.inv(m.m)
            return Mat4f(source=inv_m.flatten())
        except np.linalg.LinAlgError:
            raise ValueError("Cannot invert a matrix with a determinant of zero.")

    def determinant(self):
        return np.linalg.det(self.m)

    def translate(self, v):
        translation_matrix = np.eye(4)
        translation_matrix[:3, 3] = v.to_array()
        self.m = self.m @ translation_matrix
        return self

    def translate_xyz(self, x, y, z):
        return self.translate(Vec3f(x, y, z))

    def rotate_about_local_axis_rads(self, angle_rads, local_axis):
        rotation_matrix = self.create_rotation_matrix_rads(angle_rads, local_axis)
        self.m = self.m @ rotation_matrix
        return self

    def rotate_about_local_axis_degs(self, angle_degs, local_axis):
        return self.rotate_about_local_axis_rads(angle_degs * Mat4f.DEGS_TO_RADS, local_axis)

    def rotate_about_world_axis_rads(self, angle_rads, world_axis):
        rotation_matrix = self.create_rotation_matrix_rads(angle_rads, world_axis)
        self.m[:3, :3] = rotation_matrix[:3, :3] @ self.m[:3, :3]
        return self

    @staticmethod
    def rotate_about_world_axis_rads_static(matrix, angle_rads, world_axis):
        rotation_matrix = Mat4f.create_rotation_matrix_rads(angle_rads, world_axis)
        result = Mat4f(source=matrix.m.flatten())
        result.m[:3, :3] = rotation_matrix[:3, :3] @ result.m[:3, :3]
        return result

    @staticmethod
    def rotate_about_world_axis_degs(matrix, angle_degs, world_axis):
        return Mat4f.rotate_about_world_axis_rads(matrix, angle_degs * Mat4f.DEGS_TO_RADS, world_axis)

    @staticmethod
    def rotate_matrix_about_local_axis_rads(matrix, angle_rads, local_axis):
        world_space_axis = matrix.transform_direction(local_axis)
        return Mat4f.rotate_about_world_axis_rads(matrix, angle_rads, world_space_axis)

    @staticmethod
    def rotate_matrix_about_local_axis_degs(matrix, angle_degs, local_axis):
        return Mat4f.rotate_matrix_about_local_axis_rads(matrix, angle_degs * Mat4f.DEGS_TO_RADS, local_axis)

    def to_mat3f(self):
        return Mat3f(value=0, m00=self.m[0, 0], m01=self.m[0, 1], m02=self.m[0, 2],
                     m10=self.m[1, 0], m11=self.m[1, 1], m12=self.m[1, 2],
                     m20=self.m[2, 0], m21=self.m[2, 1], m22=self.m[2, 2])

    def to_array(self):
        return self.m.flatten()

    def __str__(self):
        return f"X-axis: {self.m[0, :4]}\nY-axis: {self.m[1, :4]}\nZ-axis: {self.m[2, :4]}\nOrigin: {self.m[3, :4]}"

    @staticmethod
    def create_orthographic_projection_matrix(left, right, top, bottom, near, far):
        if right - left == 0.0:
            raise ValueError("(right - left) cannot be zero.")
        if top - bottom == 0.0:
            raise ValueError("(top - bottom) cannot be zero.")
        if far - near == 0.0:
            raise ValueError("(far - near) cannot be zero.")

        m = np.zeros((4, 4))
        m[0, 0] = 2.0 / (right - left)
        m[1, 1] = 2.0 / (top - bottom)
        m[2, 2] = -2.0 / (far - near)
        m[3, 0] = -(right + left) / (right - left)
        m[3, 1] = -(top + bottom) / (top - bottom)
        m[3, 2] = -(far + near) / (far - near)
        m[3, 3] = 1.0
        return Mat4f(source=m.flatten())

    @staticmethod
    def create_perspective_projection_matrix_6args(left, right, top, bottom, near, far):
        p = np.zeros((4, 4))
        p[0, 0] = (2.0 * near) / (right - left)
        p[1, 1] = (2.0 * near) / (top - bottom)
        p[2, 0] = (right + left) / (right - left)
        p[2, 1] = (top + bottom) / (top - bottom)
        p[2, 2] = -(far + near) / (far - near)
        p[2, 3] = -1.0
        p[3, 2] = (-2.0 * far * near) / (far - near)
        return Mat4f(source=p.flatten())

    @staticmethod
    def create_perspective_projection_matrix_4args(vert_fov_degs, aspect_ratio, z_near, z_far):
        if aspect_ratio < 0.0:
            raise ValueError("Aspect ratio cannot be negative.")
        if z_near <= 0.0 or z_far <= 0.0:
            raise ValueError("The values of zNear and zFar must be positive.")
        if z_near >= z_far:
            raise ValueError("zNear must be less than than zFar.")
        if vert_fov_degs < 1.0 or vert_fov_degs > 179.0:
            raise ValueError("Vertical FoV must be within 1 and 179 degrees inclusive.")

        frustum_length = z_far - z_near
        half_vert_fov_rads = (vert_fov_degs / 2.0) * Mat4f.DEGS_TO_RADS
        cotangent = 1.0 / np.tan(half_vert_fov_rads)

        p = np.zeros((4, 4))
        p[0, 0] = cotangent / aspect_ratio
        p[1, 1] = cotangent
        p[2, 2] = -(z_far + z_near) / frustum_length
        p[2, 3] = -1.0
        p[3, 2] = (-2.0 * z_near * z_far) / frustum_length
        return Mat4f(source=p.flatten())

    @staticmethod
    def create_rotation_matrix_rads(angle_rads, axis):
        cos = np.cos(angle_rads)
        sin = np.sin(angle_rads)
        one_minus_cos = 1.0 - cos
        x, y, z = axis.normalised().to_array()
        rotation_matrix = np.array([
            [cos + x * x * one_minus_cos, x * y * one_minus_cos - z * sin, x * z * one_minus_cos + y * sin, 0.0],
            [y * x * one_minus_cos + z * sin, cos + y * y * one_minus_cos, y * z * one_minus_cos - x * sin, 0.0],
            [z * x * one_minus_cos - y * sin, z * y * one_minus_cos + x * sin, cos + z * z * one_minus_cos, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])
        return rotation_matrix
