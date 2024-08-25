from enum import Enum

from FabrikBone2D import FabrikBone2D
from FabrikJoint2D import ConstraintCoordinateSystem
from utils.Colour4f import Colour4f
from utils.Vec2f import Vec2f


class BaseboneConstraintType2D(Enum):
    NONE = 1
    GLOBAL_ABSOLUTE = 2
    LOCAL_RELATIVE = 3
    LOCAL_ABSOLUTE = 4


class BoneConnectionPoint(Enum):
    START = 1
    END = 2

class FabrikChain2D:

    def __init__(self, name=None):
        self.m_chain = []
        self.m_name = name
        self.m_solve_distance_threshold = 1.0
        self.m_max_iteration_attempts = 15
        self.m_min_iteration_change = 0.01
        self.m_chain_length = 0.0
        self.m_base_location = Vec2f()
        self.m_fixed_base_mode = True
        self.m_basebone_constraint_type = BaseboneConstraintType2D.NONE
        self.m_bone_connection_point = BoneConnectionPoint.END
        self.m_basebone_constraint_uv = Vec2f()
        self.m_basebone_relative_constraint_uv = Vec2f()
        self.m_last_target_location = Vec2f(float('inf'), float('inf'))
        self.m_last_base_location = Vec2f(float('inf'), float('inf'))
        self.m_embedded_target = Vec2f()
        self.m_use_embedded_target = False
        self.m_current_solve_distance = float('inf')
        self.m_connected_chain_number = -1
        self.m_connected_bone_number = -1

    def add_bone(self, bone):
        self.m_chain.append(bone)
        if len(self.m_chain) == 1:
            self.m_base_location = bone.get_start_location()
            self.m_basebone_constraint_uv = bone.get_direction_uv()
        self.update_chain_length()

    def add_consecutive_constrained_bone(self, direction_uv, length, clockwise_degs, anticlockwise_degs, colour=None):
        if not self.m_chain:
            raise RuntimeError("You cannot add the base bone to a chain using this method as it does not provide a start location.")
        prev_bone_end = self.m_chain[-1].get_end_location()
        new_bone = FabrikBone2D(prev_bone_end, direction_uv, length, clockwise_degs, anticlockwise_degs, colour or Colour4f())
        self.add_bone(new_bone)

    def add_consecutive_bone_uvl(self, direction_uv, length):
        self.add_consecutive_constrained_bone(direction_uv, length, 180.0, 180.0, Colour4f())

    def add_consecutive_bone(self, bone):
        direction_uv = bone.get_direction_uv()
        length = bone.length()
        if not self.m_chain:
            raise RuntimeError("You cannot add the base bone to a chain using this method as it does not provide a start location.")
        prev_bone_end = self.m_chain[-1].get_end_location()
        bone.set_start_location(prev_bone_end)
        bone.set_end_location(prev_bone_end.plus(direction_uv.times(length)))
        self.add_bone(bone)

    def get_basebone_constraint_type(self):
        return self.m_basebone_constraint_type

    def get_basebone_constraint_uv(self):
        return self.m_basebone_constraint_uv

    def get_base_location(self):
        if not self.m_chain:
            raise RuntimeError("Cannot get base location as there are zero bones in the chain.")
        return self.m_chain[0].get_start_location()

    def get_bone(self, bone_number):
        return self.m_chain[bone_number]

    def get_bone_connection_point(self):
        return self.m_bone_connection_point

    def get_chain(self):
        return self.m_chain

    def get_chain_length(self):
        return self.m_chain_length

    def get_connected_bone_number(self):
        return self.m_connected_bone_number

    def get_connected_chain_number(self):
        return self.m_connected_chain_number

    def get_effector_location(self):
        if not self.m_chain:
            raise RuntimeError("Cannot get effector location as there are zero bones in the chain.")
        return self.m_chain[-1].get_end_location()

    def get_embedded_target_mode(self):
        return self.m_use_embedded_target

    def get_embedded_target(self):
        return self.m_embedded_target

    def get_last_target_location(self):
        return self.m_last_target_location

    def get_name(self):
        return self.m_name

    def get_num_bones(self):
        return len(self.m_chain)

    def remove_bone(self, bone_number):
        if bone_number < len(self.m_chain):
            self.m_chain.pop(bone_number)
            self.update_chain_length()
        else:
            raise ValueError(f"Bone {bone_number} does not exist in this chain.")

    def set_basebone_constraint_type(self, type):
        self.m_basebone_constraint_type = type

    def set_basebone_constraint_uv(self, constraint_uv):
        self.m_basebone_constraint_uv = constraint_uv.normalised()

    def set_base_location(self, base_location):
        self.m_base_location = base_location

    def set_bone_connection_point(self, bone_connection_point):
        self.m_bone_connection_point = bone_connection_point

    def set_chain(self, chain):
        self.m_chain = chain

    def set_colour(self, colour):
        for bone in self.m_chain:
            bone.set_colour(colour)

    def set_connected_bone_number(self, bone_number):
        self.m_connected_bone_number = bone_number

    def set_connected_chain_number(self, chain_number):
        self.m_connected_chain_number = chain_number

    def set_fixed_base_mode(self, value):
        if not value and self.m_connected_chain_number != -1:
            raise RuntimeError("This chain is connected to another chain so must remain in fixed base mode.")
        if self.m_basebone_constraint_type == BaseboneConstraintType2D.GLOBAL_ABSOLUTE and not value:
            raise RuntimeError("Cannot set a non-fixed base mode when the chain's constraint type is BaseBoneConstraintType2D.GLOBAL_ABSOLUTE.")
        self.m_fixed_base_mode = value

    def set_max_iteration_attempts(self, max_iterations):
        if max_iterations < 1:
            raise ValueError("The maximum number of attempts to solve this IK chain must be at least 1.")
        self.m_max_iteration_attempts = max_iterations

    def set_min_iteration_change(self, min_iteration_change):
        if min_iteration_change < 0.0:
            raise ValueError("The minimum iteration change value must be more than or equal to zero.")
        self.m_min_iteration_change = min_iteration_change

    def set_name(self, name):
        self.m_name = name[:100] if name else None

    def set_solve_distance_threshold(self, solve_distance):
        if solve_distance < 0.0:
            raise ValueError("The solve distance threshold must be greater than or equal to zero.")
        self.m_solve_distance_threshold = solve_distance

    def solve_ik(self, target):
        for loop in range(len(self.m_chain) - 1, -1, -1):
            this_bone = self.m_chain[loop]
            bone_length = this_bone.length()
            if loop != len(self.m_chain) - 1:
                outer_bone = self.m_chain[loop + 1]
                outer_bone_outer_to_inner_uv = outer_bone.get_direction_uv().negated()
                this_bone_outer_to_inner_uv = this_bone.get_direction_uv().negated()
                clockwise_constraint_degs = outer_bone.get_joint().get_clockwise_constraint_degs()
                anticlockwise_constraint_degs = outer_bone.get_joint().get_anticlockwise_constraint_degs()
                constrained_uv = Vec2f.get_constrained_uv(this_bone_outer_to_inner_uv, outer_bone_outer_to_inner_uv, clockwise_constraint_degs, anticlockwise_constraint_degs)
                new_start_location = this_bone.get_end_location().plus(constrained_uv.times(bone_length))
                this_bone.set_start_location(new_start_location)
                if loop > 0:
                    self.m_chain[loop - 1].set_end_location(new_start_location)
            else:
                this_bone.set_end_location(target)
                this_bone_outer_to_inner_uv = this_bone.get_direction_uv().negated()
                if loop > 0:
                    inner_bone_outer_to_inner_uv = self.m_chain[loop - 1].get_direction_uv().negated()
                    clockwise_constraint_degs = this_bone.get_joint().get_clockwise_constraint_degs()
                    anticlockwise_constraint_degs = this_bone.get_joint().get_anticlockwise_constraint_degs()
                    if this_bone.get_joint().get_constraint_coordinate_system() == ConstraintCoordinateSystem.LOCAL:
                        constrained_uv = Vec2f.get_constrained_uv(this_bone_outer_to_inner_uv, inner_bone_outer_to_inner_uv, clockwise_constraint_degs, anticlockwise_constraint_degs)
                    else:
                        constrained_uv = Vec2f.get_constrained_uv(this_bone_outer_to_inner_uv, this_bone.get_global_constraint_uv().negated(), clockwise_constraint_degs, anticlockwise_constraint_degs)
                else:
                    if this_bone.get_joint_constraint_coordinate_system() == ConstraintCoordinateSystem.LOCAL:
                        constrained_uv = this_bone_outer_to_inner_uv
                    else:
                        constrained_uv = Vec2f.get_constrained_uv(this_bone_outer_to_inner_uv, this_bone.get_global_constraint_uv().negated(), this_bone.get_clockwise_constraint_degs(), this_bone.get_anticlockwise_constraint_degs())
                new_start_location = this_bone.get_end_location().plus(constrained_uv.times(bone_length))
                this_bone.set_start_location(new_start_location)
                if loop > 0:
                    self.m_chain[loop - 1].set_end_location(new_start_location)

        for loop in range(len(self.m_chain)):
            bone_length = self.m_chain[loop].length()
            this_bone = self.m_chain[loop]
            if loop != 0:
                previous_bone = self.m_chain[loop - 1]
                this_bone_inner_to_outer_uv = this_bone.get_direction_uv()
                prev_bone_inner_to_outer_uv = previous_bone.get_direction_uv()
                clockwise_constraint_degs = this_bone.get_joint().get_clockwise_constraint_degs()
                anticlockwise_constraint_degs = this_bone.get_joint().get_anticlockwise_constraint_degs()
                if this_bone.get_joint_constraint_coordinate_system() == ConstraintCoordinateSystem.LOCAL:
                    constrained_uv = Vec2f.get_constrained_uv(this_bone_inner_to_outer_uv, prev_bone_inner_to_outer_uv, clockwise_constraint_degs, anticlockwise_constraint_degs)
                else:
                    constrained_uv = Vec2f.get_constrained_uv(this_bone_inner_to_outer_uv, this_bone.get_global_constraint_uv(), clockwise_constraint_degs, anticlockwise_constraint_degs)
                new_end_location = this_bone.get_start_location().plus(constrained_uv.times(bone_length))
                this_bone.set_end_location(new_end_location)
                if loop < len(self.m_chain) - 1:
                    self.m_chain[loop + 1].set_start_location(new_end_location)
            else:
                if self.m_fixed_base_mode:
                    self.m_chain[0].set_start_location(self.m_base_location)
                else:
                    bone_zero_uv = self.m_chain[0].get_direction_uv()
                    bone_zero_end_location = self.m_chain[0].get_end_location()
                    new_bone_zero_start_location = bone_zero_end_location.minus(bone_zero_uv.times(bone_length))
                    self.m_chain[0].set_start_location(new_bone_zero_start_location)
                if self.m_basebone_constraint_type == BaseboneConstraintType2D.NONE:
                    this_bone_inner_to_outer_uv = this_bone.get_direction_uv()
                    new_end_location = this_bone.get_start_location().plus(this_bone_inner_to_outer_uv.times(bone_length))
                    self.m_chain[0].set_end_location(new_end_location)
                    if len(self.m_chain) > 1:
                        self.m_chain[1].set_start_location(new_end_location)
                else:
                    this_bone_inner_to_outer_uv = this_bone.get_direction_uv()
                    clockwise_constraint_degs = this_bone.get_joint().get_clockwise_constraint_degs()
                    anticlockwise_constraint_degs = this_bone.get_joint().get_anticlockwise_constraint_degs()
                    if self.m_basebone_constraint_type == BaseboneConstraintType2D.LOCAL_ABSOLUTE:
                        constrained_uv = Vec2f.get_constrained_uv(this_bone_inner_to_outer_uv, self.m_basebone_relative_constraint_uv, clockwise_constraint_degs, anticlockwise_constraint_degs)
                    else:
                        constrained_uv = Vec2f.get_constrained_uv(this_bone_inner_to_outer_uv, self.m_basebone_constraint_uv, clockwise_constraint_degs, anticlockwise_constraint_degs)
                    new_end_location = self.m_chain[loop].get_start_location().plus(constrained_uv.times(bone_length))
                    self.m_chain[loop].set_end_location(new_end_location)
                    if loop < len(self.m_chain) - 1:
                        self.m_chain[loop + 1].set_start_location(new_end_location)

        self.m_last_target_location = target
        current_effector_location = self.m_chain[-1].get_end_location()
        return Vec2f.distance_between(current_effector_location, target)

    def set_embedded_target_mode(self, value):
        self.m_use_embedded_target = value

    def __str__(self):
        result = f"----- FabrikChain2D: {self.m_name} -----\n"
        result += f"Number of bones: {len(self.m_chain)}\n"
        result += f"Fixed base mode: {'Yes' if self.m_fixed_base_mode else 'No'}\n"
        result += f"Base location: {self.get_base_location()}"
        return result

    def update_chain_length(self):
        self.m_chain_length = sum(bone.length() for bone in self.m_chain)

    def update_embedded_target(self, new_embedded_target):
        if self.m_use_embedded_target:
            self.m_embedded_target = new_embedded_target
        else:
            raise RuntimeError("This chain does not have embedded targets enabled - enable with setEmbeddedTargetMode(true).")

    def update_embedded_target_xy(self, x, y):
        if self.m_use_embedded_target:
            self.m_embedded_target = Vec2f(x, y)
        else:
            raise RuntimeError("This chain does not have embedded targets enabled - enable with setEmbeddedTargetMode(true).")

    def solve_for_embedded_target(self):
        if self.m_use_embedded_target:
            return self.solve_for_target(self.m_embedded_target)
        else:
            raise RuntimeError("This chain does not have embedded targets enabled - enable with setEmbeddedTargetMode(true).")

    def solve_for_target(self, new_target):
        if self.m_last_target_location.approximately_equals(new_target, 0.001) and self.m_last_base_location.approximately_equals(self.m_base_location, 0.001):
            return self.m_current_solve_distance

        starting_distance = float('inf')
        starting_solution = None
        if self.m_last_base_location.approximately_equals(self.m_base_location, 0.001):
            starting_distance = Vec2f.distance_between(self.m_chain[-1].get_end_location(), new_target)
            starting_solution = self.clone_chain_vector()

        best_solution = []
        best_solve_distance = float('inf')
        last_pass_solve_distance = float('inf')

        for loop in range(self.m_max_iteration_attempts):
            solve_distance = self.solve_ik(new_target)
            if solve_distance < best_solve_distance:
                best_solve_distance = solve_distance
                best_solution = self.clone_chain_vector()
                if solve_distance <= self.m_solve_distance_threshold:
                    break
            else:
                if abs(solve_distance - last_pass_solve_distance) < self.m_min_iteration_change:
                    break
            last_pass_solve_distance = solve_distance

        if best_solve_distance < starting_distance:
            self.m_current_solve_distance = best_solve_distance
            self.m_chain = best_solution
        else:
            self.m_current_solve_distance = starting_distance
            self.m_chain = starting_solution

        self.m_last_base_location = self.m_base_location
        self.m_last_target_location = new_target

        return self.m_current_solve_distance

    def get_basebone_relative_constraint_uv(self):
        return self.m_basebone_relative_constraint_uv

    def set_basebone_relative_constraint_uv(self, constraint_uv):
        self.m_basebone_relative_constraint_uv = constraint_uv

    def get_max_iteration_attempts(self):
        return self.m_max_iteration_attempts

    def get_min_iteration_change(self):
        return self.m_min_iteration_change

    def get_solve_distance_threshold(self):
        return self.m_solve_distance_threshold

    def clone_chain_vector(self):
        cloned_chain = [FabrikBone2D(bone.get_start_location(), bone.get_direction_uv(), bone.length(), bone.get_joint().get_clockwise_constraint_degs(), bone.get_joint().get_anticlockwise_constraint_degs(), bone.colour) for bone in self.m_chain]
        return cloned_chain