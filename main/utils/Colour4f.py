import numpy as np
import random

class Colour4f:
    MIN_COMPONENT_VALUE = 0.0
    MAX_COMPONENT_VALUE = 1.0

    def __init__(self, *args):
        if len(args) == 0:
            self.r = self.g = self.b = self.a = 1.0
        elif len(args) == 1 and isinstance(args[0], Colour4f):
            source = args[0]
            self.r = source.r
            self.g = source.g
            self.b = source.b
            self.a = source.a
        elif len(args) == 1 and isinstance(args[0], list):
            source_values = args[0]
            if len(source_values) == 4:
                self.r = self.clamp(source_values[0])
                self.g = self.clamp(source_values[1])
                self.b = self.clamp(source_values[2])
                self.a = self.clamp(source_values[3])
            else:
                raise ValueError("Colour source array size must be precisely 4 elements.")
        elif len(args) == 4:
            red, green, blue, alpha = args
            self.r = self.clamp(red)
            self.g = self.clamp(green)
            self.b = self.clamp(blue)
            self.a = self.clamp(alpha)
        else:
            raise ValueError("Invalid arguments")

    def set(self, source):
        if isinstance(source, Colour4f):
            self.r = self.clamp(source.r)
            self.g = self.clamp(source.g)
            self.b = self.clamp(source.b)
            self.a = self.clamp(source.a)
        elif isinstance(source, tuple) and len(source) == 4:
            red, green, blue, alpha = source
            self.r = self.clamp(red)
            self.g = self.clamp(green)
            self.b = self.clamp(blue)
            self.a = self.clamp(alpha)
        else:
            raise ValueError("Invalid source")

    def add_rgb(self, red, green, blue):
        self.r = self.clamp(self.r + red)
        self.g = self.clamp(self.g + green)
        self.b = self.clamp(self.b + blue)
        return self

    def subtract_rgb(self, red, green, blue):
        self.r = self.clamp(self.r - red)
        self.g = self.clamp(self.g - green)
        self.b = self.clamp(self.b - blue)
        return self

    def lighten(self, amount):
        return self.add_rgb(amount, amount, amount)

    def darken(self, amount):
        return self.subtract_rgb(amount, amount, amount)

    def to_array(self):
        return [self.r, self.g, self.b, self.a]

    def __str__(self):
        return f"Red: {self.r}, Green: {self.g}, Blue: {self.b}, Alpha: {self.a}"

    @staticmethod
    def random_opaque_colour():
        return Colour4f(random.random(), random.random(), random.random(), 1.0)

    @staticmethod
    def clamp(component_value):
        return min(max(component_value, Colour4f.MIN_COMPONENT_VALUE), Colour4f.MAX_COMPONENT_VALUE)

    def __hash__(self):
        return hash((self.r, self.g, self.b, self.a))

    def __eq__(self, other):
        if isinstance(other, Colour4f):
            return self.r == other.r and self.g == other.g and self.b == other.b and self.a == other.a
        return False