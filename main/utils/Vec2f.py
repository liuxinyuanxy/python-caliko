import numpy as np

class Vec2f:
    DEGS_TO_RADS = np.pi / 180.0
    RADS_TO_DEGS = 180.0 / np.pi

    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y

    def approximately_equals(self, v, tolerance):
        if tolerance < 0.0:
            raise ValueError("Equality threshold must be greater than or equal to 0.0f")
        return abs(self.x - v.x) < tolerance and abs(self.y - v.y) < tolerance

    def plus(self, v):
        return Vec2f(self.x + v.x, self.y + v.y)

    def minus(self, v):
        return Vec2f(self.x - v.x, self.y - v.y)

    def times(self, value):
        return Vec2f(self.x * value, self.y * value)

    def divided_by(self, value):
        return Vec2f(self.x / value, self.y / value)

    def negated(self):
        return Vec2f(-self.x, -self.y)

    def set(self, source):
        self.x = source.x
        self.y = source.y

    def set_xy(self, x, y):
        self.x = x
        self.y = y

    def length(self):
        return np.sqrt(self.x * self.x + self.y * self.y)

    def normalise(self):
        magnitude = self.length()
        if magnitude > 0.0:
            self.x /= magnitude
            self.y /= magnitude
        return self

    @staticmethod
    def normalised(source):
        return Vec2f(source.x, source.y).normalise()

    @staticmethod
    def get_direction_uv(a, b):
        return b.minus(a).normalise()

    @staticmethod
    def distance_between(v1, v2):
        return np.sqrt((v2.x - v1.x) ** 2 + (v2.y - v1.y) ** 2)

    @staticmethod
    def dot(v1, v2):
        return v1.x * v2.x + v1.y * v2.y

    def dot_self(self, v):
        return self.x * v.x + self.y * v.y

    @staticmethod
    def get_unsigned_angle_between_vectors_degs(v1, v2):
        return np.arccos(Vec2f.normalised(v1).dot_self(Vec2f.normalised(v2))) * Vec2f.RADS_TO_DEGS

    @staticmethod
    def zcross(u, v):
        p = u.x * v.y - v.x * u.y
        if p > 0.0:
            return 1
        elif p < 0.0:
            return -1
        return 0

    def get_signed_angle_degs_to(self, other_vector):
        this_vector_uv = Vec2f.normalised(self)
        other_vector_uv = Vec2f.normalised(other_vector)
        unsigned_angle_degs = np.arccos(this_vector_uv.dot_self(other_vector_uv)) * Vec2f.RADS_TO_DEGS
        if Vec2f.zcross(this_vector_uv, other_vector_uv) == 1:
            return unsigned_angle_degs
        else:
            return -unsigned_angle_degs

    @staticmethod
    def get_constrained_uv(direction_uv, baseline_uv, clockwise_constraint_degs, anti_clockwise_constraint_degs):
        signed_angle_degs = baseline_uv.get_signed_angle_degs_to(direction_uv)
        if signed_angle_degs > anti_clockwise_constraint_degs:
            return Vec2f.rotate_degs(baseline_uv, anti_clockwise_constraint_degs)
        if signed_angle_degs < -clockwise_constraint_degs:
            return Vec2f.rotate_degs(baseline_uv, -clockwise_constraint_degs)
        return direction_uv

    def rotate_rads(self, angle_rads):
        cos_theta = np.cos(angle_rads)
        sin_theta = np.sin(angle_rads)
        rotated_vector = Vec2f(self.x * cos_theta - self.y * sin_theta, self.x * sin_theta + self.y * cos_theta)
        self.x = rotated_vector.x
        self.y = rotated_vector.y
        return self

    @staticmethod
    def rotate_degs(source, angle_degs):
        angle_rads = angle_degs * Vec2f.DEGS_TO_RADS
        cos_theta = np.cos(angle_rads)
        sin_theta = np.sin(angle_rads)
        return Vec2f(source.x * cos_theta - source.y * sin_theta, source.x * sin_theta + source.y * cos_theta)

    def __str__(self):
        return f"{self.x:.3f}, {self.y:.3f}"

    def __eq__(self, other):
        if isinstance(other, Vec2f):
            return np.isclose(self.x, other.x) and np.isclose(self.y, other.y)
        return False

    def __hash__(self):
        return hash((self.x, self.y))