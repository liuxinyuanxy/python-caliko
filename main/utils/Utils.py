import numpy as np
import random

from utils.Colour4f import Colour4f


class Utils:
    DEGS_TO_RADS = np.pi / 180.0
    RADS_TO_DEGS = 180.0 / np.pi
    df = "{:.3f}"
    NEW_LINE = "\n"

    RED = Colour4f(1.0, 0.0, 0.0, 1.0)
    GREEN = Colour4f(0.0, 1.0, 0.0, 1.0)
    BLUE = Colour4f(0.0, 0.0, 1.0, 1.0)
    MID_RED = Colour4f(0.6, 0.0, 0.0, 1.0)
    MID_GREEN = Colour4f(0.0, 0.6, 0.0, 1.0)
    MID_BLUE = Colour4f(0.0, 0.0, 0.6, 1.0)
    BLACK = Colour4f(0.0, 0.0, 0.0, 1.0)
    GREY = Colour4f(0.5, 0.5, 0.5, 1.0)
    WHITE = Colour4f(1.0, 1.0, 1.0, 1.0)
    YELLOW = Colour4f(1.0, 1.0, 0.0, 1.0)
    CYAN = Colour4f(0.0, 1.0, 1.0, 1.0)
    MAGENTA = Colour4f(1.0, 0.0, 1.0, 1.0)

    random = random.Random()
    MAX_NAME_LENGTH = 100

    @staticmethod
    def set_random_seed(seed_value):
        Utils.random.seed(seed_value)

    @staticmethod
    def rand_range_float(min_val, max_val):
        return Utils.random.random() * (max_val - min_val) + min_val

    @staticmethod
    def rand_range_int(min_val, max_val):
        return Utils.random.randint(min_val, max_val - 1)

    @staticmethod
    def cot(angle_rads):
        return 1.0 / np.tan(angle_rads)

    @staticmethod
    def radians_to_degrees(angle_rads):
        return angle_rads * Utils.RADS_TO_DEGS

    @staticmethod
    def degrees_to_radians(angle_degs):
        return angle_degs * Utils.DEGS_TO_RADS

    @staticmethod
    def create_float_buffer(num_floats):
        return np.zeros(num_floats, dtype=np.float32)

    @staticmethod
    def sign(value):
        return 1.0 if value >= 0.0 else -1.0

    @staticmethod
    def set_seed(seed):
        Utils.random.seed(seed)

    @staticmethod
    def validate_direction_uv(direction_uv):
        if np.linalg.norm(direction_uv) <= 0.0:
            raise ValueError("Vec2f/Vec3f direction unit vector cannot be zero.")

    @staticmethod
    def validate_length(length):
        if length < 0.0:
            raise ValueError("Length must be greater than or equal to zero.")

    @staticmethod
    def convert_range(orig_value, orig_min, orig_max, new_min, new_max):
        orig_range = orig_max - orig_min
        new_range = new_max - new_min

        if -0.000001 < orig_range < 0.000001:
            return (new_min + new_max) / 2.0
        else:
            return ((orig_value - orig_min) * new_range / orig_range) + new_min

    @staticmethod
    def approximately_equals(a, b, tolerance = 0.0001):
        return abs(a - b) <= tolerance

    @staticmethod
    def validate_line_width(line_width):
        if line_width < 1.0 or line_width > 32.0:
            raise ValueError("Line widths must be within the range 1.0 to 32.0 - but only 1.0 is guaranteed to be supported.")

    @staticmethod
    def get_validated_name(name):
        return name[:Utils.MAX_NAME_LENGTH] if len(name) >= Utils.MAX_NAME_LENGTH else name