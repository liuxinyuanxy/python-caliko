from enum import Enum

from FabrikBone3D import FabrikBone3D
from FabrikJoint3D import FabrikJoint3D, JointType
from utils.Colour4f import Colour4f
from utils.Mat3f import Mat3f
from utils.Vec3f import Vec3f


class BaseboneConstraintType3D(Enum):
    NONE = 0
    GLOBAL_ROTOR = 1
    LOCAL_ROTOR = 2
    GLOBAL_HINGE = 3
    LOCAL_HINGE = 4


class FabrikChain3D:
    def __init__(self, name=None, source=None):
        if source:
            self.m_chain = source.clone_ik_chain()
            self.m_fixed_base_location = source.get_base_location()
            self.m_last_target_location = source.get_last_target_location()
            self.m_last_base_location = source.m_last_base_location
            self.m_embedded_target = source.get_embedded_target()
            if source.m_basebone_constraint_type != BaseboneConstraintType3D.NONE:
                self.m_basebone_constraint_uv = source.get_basebone_constraint_uv()
                self.m_basebone_relative_constraint_uv = source.get_basebone_relative_constraint_uv()
            self.m_chain_length = source.get_chain_length()
            self.m_current_solve_distance = source.m_current_solve_distance
            self.m_connected_chain_number = source.m_connected_chain_number
            self.m_connected_bone_number = source.m_connected_bone_number
            self.m_basebone_constraint_type = source.m_basebone_constraint_type
            self.m_name = source.get_name()
            self.m_constraint_line_width = source.m_constraint_line_width
            self.m_use_embedded_target = source.get_embedded_target_mode()
        self.m_chain = []
        self.m_name = name
        self.m_solve_distance_threshold = 1.0
        self.m_max_iteration_attempts = 20
        self.m_min_iteration_change = 0.01
        self.m_chain_length = 0.0
        self.m_fixed_base_location = Vec3f()
        self.m_fixed_base_mode = True
        self.m_basebone_constraint_type = BaseboneConstraintType3D.NONE
        self.m_basebone_constraint_uv = Vec3f()
        self.m_basebone_relative_constraint_uv = Vec3f()
        self.m_basebone_relative_reference_constraint_uv = Vec3f()
        self.m_last_target_location = Vec3f(float('inf'), float('inf'), float('inf'))
        self.m_constraint_line_width = 2.0
        self.m_last_base_location = Vec3f(float('inf'), float('inf'), float('inf'))
        self.m_current_solve_distance = float('inf')
        self.m_connected_chain_number = -1
        self.m_connected_bone_number = -1
        self.m_embedded_target = Vec3f()
        self.m_use_embedded_target = False

    def add_bone(self, bone):
        self.m_chain.append(bone)
        if len(self.m_chain) == 1:
            self.m_fixed_base_location.set(bone.get_start_location())
            self.m_basebone_constraint_uv = bone.get_direction_uv()
        self.update_chain_length()

    def add_consecutive_bone(self, direction_uv, length, colour=None):
        if not self.m_chain:
            raise RuntimeError("You cannot add the basebone as a consecutive bone as it does not provide a start location. Use the add_bone() method instead.")
        prev_bone_end = self.m_chain[-1].get_end_location()
        new_bone = FabrikBone3D(start_location=prev_bone_end,direction_uv=direction_uv.normalised(),length=length,colour=colour if colour else Colour4f())
        self.add_bone(new_bone)

    def add_consecutive_hinged_bone(self, direction_uv, length, joint_type, hinge_rotation_axis, clockwise_degs, anticlockwise_degs, hinge_reference_axis, colour=None):
        if not self.m_chain:
            raise RuntimeError("You must add a basebone before adding a consectutive bone.")
        direction_uv.normalise()
        hinge_rotation_axis.normalise()
        prev_bone_end = self.m_chain[-1].get_end_location()
        bone = FabrikBone3D(start_location=prev_bone_end,direction_uv=direction_uv,length=length)
        bone.set_colour(colour if colour else Colour4f())
        joint = FabrikJoint3D()
        if joint_type == JointType.GLOBAL_HINGE:
            joint.set_hinge(JointType.GLOBAL_HINGE, hinge_rotation_axis, clockwise_degs, anticlockwise_degs, hinge_reference_axis)
        elif joint_type == JointType.LOCAL_HINGE:
            joint.set_hinge(JointType.LOCAL_HINGE, hinge_rotation_axis, clockwise_degs, anticlockwise_degs, hinge_reference_axis)
        else:
            raise ValueError("Hinge joint types may be only JointType.GLOBAL_HINGE or JointType.LOCAL_HINGE.")
        bone.set_joint(joint)
        self.add_bone(bone)

    def get_basebone_relative_constraint_uv(self):
        return self.m_basebone_relative_constraint_uv

    def get_basebone_constraint_type(self):
        return self.m_basebone_constraint_type

    def set_constraint_line_width(self, line_width):
        if 1.0 <= line_width <= 32.0:
            self.m_constraint_line_width = line_width
        else:
            raise ValueError("Valid values are 1.0f to 32.0f inclusive.")

    def get_basebone_constraint_uv(self):
        if self.m_basebone_constraint_type != BaseboneConstraintType3D.NONE:
            return self.m_basebone_constraint_uv
        else:
            raise RuntimeError("Cannot return the basebone constraint when the basebone constraint type is NONE.")

    def get_base_location(self):
        return self.m_chain[0].get_start_location()

    def get_bone(self, bone_number):
        return self.m_chain[bone_number]

    def get_chain(self):
        return self.m_chain

    def get_chain_length(self):
        return self.m_chain_length

    def get_connected_bone_number(self):
        return self.m_connected_bone_number

    def get_connected_chain_number(self):
        return self.m_connected_chain_number

    def get_effector_location(self):
        return self.m_chain[-1].get_end_location()

    def get_embedded_target_mode(self):
        return self.m_use_embedded_target

    def get_embedded_target(self):
        return self.m_embedded_target

    def get_last_target_location(self):
        return self.m_last_target_location

    def get_live_chain_length(self):
        length = 0.0
        for bone in self.m_chain:
            length += bone.live_length()
        return length

    def get_name(self):
        return self.m_name

    def get_num_bones(self):
        return len(self.m_chain)

    def remove_bone(self, bone_number):
        if bone_number < len(self.m_chain):
            self.m_chain.pop(bone_number)
            self.update_chain_length()
        else:
            raise ValueError("Bone does not exist to be removed from the chain. Bones are zero indexed.")

    def set_basebone_relative_constraint_uv(self, constraint_uv):
        self.m_basebone_relative_constraint_uv = constraint_uv

    def set_basebone_relative_reference_constraint_uv(self, constraint_uv):
        self.m_basebone_relative_reference_constraint_uv = constraint_uv

    def get_basebone_relative_reference_constraint_uv(self):
        return self.m_basebone_relative_reference_constraint_uv

    def set_embedded_target_mode(self, value):
        self.m_use_embedded_target = value

    def set_rotor_basebone_constraint(self, rotor_type, constraint_axis, angle_degs):
        if not self.m_chain:
            raise RuntimeError("Chain must contain a basebone before we can specify the basebone constraint type.")
        if constraint_axis.length() <= 0.0:
            raise ValueError("Constraint axis cannot be zero.")
        if angle_degs < 0.0:
            angle_degs = 0.0
        if angle_degs > 180.0:
            angle_degs = 180.0
        if rotor_type not in [BaseboneConstraintType3D.GLOBAL_ROTOR, BaseboneConstraintType3D.LOCAL_ROTOR]:
            raise ValueError("The only valid rotor types for this method are GLOBAL_ROTOR and LOCAL_ROTOR.")
        self.m_basebone_constraint_type = rotor_type
        self.m_basebone_constraint_uv = constraint_axis.normalised()
        self.m_basebone_relative_constraint_uv.set(self.m_basebone_constraint_uv)
        self.get_bone(0).get_joint().set_as_ball_joint(angle_degs)

    def set_hinge_basebone_constraint(self, hinge_type, hinge_rotation_axis, cw_constraint_degs, acw_constraint_degs, hinge_reference_axis):
        if not self.m_chain:
            raise RuntimeError("Chain must contain a basebone before we can specify the basebone constraint type.")
        if hinge_rotation_axis.length() <= 0.0:
            raise ValueError("Hinge rotation axis cannot be zero.")
        if hinge_reference_axis.length() <= 0.0:
            raise ValueError("Hinge reference axis cannot be zero.")
        if not Vec3f.perpendicular(hinge_rotation_axis, hinge_reference_axis):
            raise ValueError("The hinge reference axis must be in the plane of the hinge rotation axis, that is, they must be perpendicular.")
        if hinge_type not in [BaseboneConstraintType3D.GLOBAL_HINGE, BaseboneConstraintType3D.LOCAL_HINGE]:
            raise ValueError("The only valid hinge types for this method are GLOBAL_HINGE and LOCAL_HINGE.")
        self.m_basebone_constraint_type = hinge_type
        self.m_basebone_constraint_uv.set(hinge_rotation_axis.normalised())
        hinge = FabrikJoint3D()
        if hinge_type == BaseboneConstraintType3D.GLOBAL_HINGE:
            hinge.set_hinge(JointType.GLOBAL_HINGE, hinge_rotation_axis, cw_constraint_degs, acw_constraint_degs, hinge_reference_axis)
        else:
            hinge.set_hinge(JointType.LOCAL_HINGE, hinge_rotation_axis, cw_constraint_degs, acw_constraint_degs, hinge_reference_axis)
        self.get_bone(0).set_joint(hinge)

    def set_basebone_constraint_uv(self, constraint_uv):
        if self.m_basebone_constraint_type == BaseboneConstraintType3D.NONE:
            raise ValueError("Specify the basebone constraint type with set_basebone_constraint_type_cannot specify a basebone constraint when the current constraint type is BaseboneConstraint.NONE.")
        constraint_uv.normalise()
        self.m_basebone_constraint_uv.set(constraint_uv)

    def set_base_location(self, base_location):
        self.m_fixed_base_location = base_location

    def connect_to_structure(self, structure, chain_number: int, bone_number: int):
        # Sanity check chain exists
        num_chains = structure.get_num_chains()
        if chain_number >= num_chains:
            raise ValueError(f"Structure does not contain a chain {chain_number} - it has {num_chains} chains.")

        # Sanity check bone exists
        num_bones = structure.get_chain(chain_number).get_num_bones()
        if bone_number >= num_bones:
            raise ValueError(f"Chain does not contain a bone {bone_number} - it has {num_bones} bones.")

        # All good? Set the connection details
        self.m_connected_chain_number = chain_number
        self.m_connected_bone_number = bone_number

    def set_fixed_base_mode(self, value):
        if not value and self.m_connected_chain_number != -1:
            raise RuntimeError("This chain is connected to another chain so must remain in fixed base mode.")
        if self.m_basebone_constraint_type == BaseboneConstraintType3D.GLOBAL_ROTOR and not value:
            raise RuntimeError("Cannot set a non-fixed base mode when the chain's constraint type is BaseboneConstraintType3D.GLOBAL_ABSOLUTE_ROTOR.")
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
        self.m_name = name[:100] if len(name) > 100 else name

    def set_solve_distance_threshold(self, solve_distance):
        if solve_distance < 0.0:
            raise ValueError("The solve distance threshold must be greater than or equal to zero.")
        self.m_solve_distance_threshold = solve_distance

    def set_colour(self, colour):
        for bone in self.m_chain:
            bone.set_colour(colour)

    def solve_for_embedded_target(self):
        if self.m_use_embedded_target:
            return self.solve_for_target(self.m_embedded_target)
        else:
            raise RuntimeError("This chain does not have embedded targets enabled - enable with set_embedded_target_mode(true).")

    def solve_for_target(self, new_target):
        if (self.m_last_target_location.approximately_equals(new_target, 0.001)
                and self.m_last_base_location.approximately_equals(self.get_base_location(), 0.001)
                and self.m_fixed_base_location.approximately_equals(self.get_base_location(), 0.001)):
            return self.m_current_solve_distance
        best_solution = self.clone_ik_chain()
        best_solve_distance = float('inf')
        last_pass_solve_distance = float('inf')
        for _ in range(self.m_max_iteration_attempts):
            solve_distance = self.solve_ik(new_target)
            if solve_distance < best_solve_distance:
                best_solve_distance = solve_distance
                best_solution = self.clone_ik_chain()
                if solve_distance <= self.m_solve_distance_threshold:
                    break
            elif abs(solve_distance - last_pass_solve_distance) < self.m_min_iteration_change:
                break
            last_pass_solve_distance = solve_distance
        self.m_current_solve_distance = best_solve_distance
        self.m_chain = best_solution
        self.m_last_base_location.set(self.get_base_location())
        self.m_last_target_location.set(new_target)
        return self.m_current_solve_distance

    def solve_for_target_xyz(self, target_x, target_y, target_z):
        return self.solve_for_target(Vec3f(target_x, target_y, target_z))

    def solve_ik(self, target):
        if not self.m_chain:
            raise RuntimeError("It makes no sense to solve an IK chain with zero bones.")
        for loop in range(len(self.m_chain) - 1, -1, -1):
            this_bone = self.m_chain[loop]
            this_bone_length = this_bone.length()
            this_bone_joint = this_bone.get_joint()
            this_bone_joint_type = this_bone.get_joint_type()
            if loop != len(self.m_chain) - 1:
                outer_bone_outer_to_inner_uv = self.m_chain[loop + 1].get_direction_uv().negated()
                this_bone_outer_to_inner_uv = this_bone.get_direction_uv().negated()
                if this_bone_joint_type == JointType.BALL:
                    angle_between_degs = Vec3f.get_angle_between_degs(outer_bone_outer_to_inner_uv, this_bone_outer_to_inner_uv)
                    constraint_angle_degs = this_bone_joint.get_ball_joint_constraint_degs()
                    if angle_between_degs > constraint_angle_degs:
                        this_bone_outer_to_inner_uv = Vec3f.get_angle_limited_unit_vector_degs(this_bone_outer_to_inner_uv, outer_bone_outer_to_inner_uv, constraint_angle_degs)
                elif this_bone_joint_type == JointType.GLOBAL_HINGE:
                    this_bone_outer_to_inner_uv = this_bone_outer_to_inner_uv.project_onto_plane(this_bone_joint.get_hinge_rotation_axis())
                elif this_bone_joint_type == JointType.LOCAL_HINGE:
                    m = Mat3f.create_rotation_matrix(self.m_chain[loop - 1].get_direction_uv()) if loop > 0 else Mat3f()
                    relative_hinge_rotation_axis = m.times(this_bone_joint.get_hinge_rotation_axis()).normalise()
                    this_bone_outer_to_inner_uv = this_bone_outer_to_inner_uv.project_onto_plane(relative_hinge_rotation_axis)
                new_start_location = this_bone.get_end_location().plus(this_bone_outer_to_inner_uv.times(this_bone_length))
                this_bone.set_start_location(new_start_location)
                if loop > 0:
                    self.m_chain[loop - 1].set_end_location(new_start_location)
            else:
                this_bone.set_end_location(target)
                this_bone_outer_to_inner_uv = this_bone.get_direction_uv().negated()
                if this_bone_joint_type == JointType.GLOBAL_HINGE:
                    this_bone_outer_to_inner_uv = this_bone_outer_to_inner_uv.project_onto_plane(this_bone_joint.get_hinge_rotation_axis())
                elif this_bone_joint_type == JointType.LOCAL_HINGE:
                    m = Mat3f.create_rotation_matrix(self.m_chain[loop - 1].get_direction_uv()) if loop > 0 else Mat3f()
                    relative_hinge_rotation_axis = m.times(this_bone_joint.get_hinge_rotation_axis()).normalise()
                    this_bone_outer_to_inner_uv = this_bone_outer_to_inner_uv.project_onto_plane(relative_hinge_rotation_axis)
                new_start_location = target.plus(this_bone_outer_to_inner_uv.times(this_bone_length))
                this_bone.set_start_location(new_start_location)
                if loop > 0:
                    self.m_chain[loop - 1].set_end_location(new_start_location)
        for loop in range(len(self.m_chain)):
            this_bone = self.m_chain[loop]
            this_bone_length = this_bone.length()
            if loop != 0:
                this_bone_inner_to_outer_uv = this_bone.get_direction_uv()
                prev_bone_inner_to_outer_uv = self.m_chain[loop - 1].get_direction_uv()
                this_bone_joint = this_bone.get_joint()
                joint_type = this_bone_joint.get_joint_type()
                if joint_type == JointType.BALL:
                    angle_between_degs = Vec3f.get_angle_between_degs(prev_bone_inner_to_outer_uv, this_bone_inner_to_outer_uv)
                    constraint_angle_degs = this_bone_joint.get_ball_joint_constraint_degs()
                    if angle_between_degs > constraint_angle_degs:
                        this_bone_inner_to_outer_uv = Vec3f.get_angle_limited_unit_vector_degs(this_bone_inner_to_outer_uv, prev_bone_inner_to_outer_uv, constraint_angle_degs)
                elif joint_type == JointType.GLOBAL_HINGE:
                    hinge_rotation_axis = this_bone_joint.get_hinge_rotation_axis()
                    this_bone_inner_to_outer_uv = this_bone_inner_to_outer_uv.project_onto_plane(hinge_rotation_axis)
                    cw_constraint_degs = -this_bone_joint.get_hinge_clockwise_constraint_degs()
                    acw_constraint_degs = this_bone_joint.get_hinge_anticlockwise_constraint_degs()
                    if not (abs(cw_constraint_degs + FabrikJoint3D.MAX_CONSTRAINT_ANGLE_DEGS) < 0.001 and abs(acw_constraint_degs - FabrikJoint3D.MAX_CONSTRAINT_ANGLE_DEGS) < 0.001):
                        hinge_reference_axis = this_bone_joint.get_hinge_reference_axis()
                        signed_angle_degs = Vec3f.get_signed_angle_between_degs(hinge_reference_axis, this_bone_inner_to_outer_uv, hinge_rotation_axis)
                        if signed_angle_degs > acw_constraint_degs:
                            this_bone_inner_to_outer_uv = Vec3f.rotate_about_axis_degs(hinge_reference_axis, acw_constraint_degs, hinge_rotation_axis).normalised()
                        elif signed_angle_degs < cw_constraint_degs:
                            this_bone_inner_to_outer_uv = Vec3f.rotate_about_axis_degs(hinge_reference_axis, cw_constraint_degs, hinge_rotation_axis).normalised()
                elif joint_type == JointType.LOCAL_HINGE:
                    m = Mat3f.create_rotation_matrix(prev_bone_inner_to_outer_uv)
                    relative_hinge_rotation_axis = m.times(this_bone_joint.get_hinge_rotation_axis()).normalise()
                    this_bone_inner_to_outer_uv = this_bone_inner_to_outer_uv.project_onto_plane(relative_hinge_rotation_axis)
                    cw_constraint_degs = -this_bone_joint.get_hinge_clockwise_constraint_degs()
                    acw_constraint_degs = this_bone_joint.get_hinge_anticlockwise_constraint_degs()
                    if not (abs(cw_constraint_degs + FabrikJoint3D.MAX_CONSTRAINT_ANGLE_DEGS) < 0.001 and abs(acw_constraint_degs - FabrikJoint3D.MAX_CONSTRAINT_ANGLE_DEGS) < 0.001):
                        relative_hinge_reference_axis = m.times(this_bone_joint.get_hinge_reference_axis()).normalise()
                        signed_angle_degs = Vec3f.get_signed_angle_between_degs(relative_hinge_reference_axis, this_bone_inner_to_outer_uv, relative_hinge_rotation_axis)
                        if signed_angle_degs > acw_constraint_degs:
                            this_bone_inner_to_outer_uv = Vec3f.rotate_about_axis_degs(relative_hinge_reference_axis, acw_constraint_degs, relative_hinge_rotation_axis).normalised()
                        elif signed_angle_degs < cw_constraint_degs:
                            this_bone_inner_to_outer_uv = Vec3f.rotate_about_axis_degs(relative_hinge_reference_axis, cw_constraint_degs, relative_hinge_rotation_axis).normalised()
                new_end_location = this_bone.get_start_location().plus(this_bone_inner_to_outer_uv.times(this_bone_length))
                this_bone.set_end_location(new_end_location)
                if loop < len(self.m_chain) - 1:
                    self.m_chain[loop + 1].set_start_location(new_end_location)
            else:
                if self.m_fixed_base_mode:
                    this_bone.set_start_location(self.m_fixed_base_location)
                else:
                    this_bone.set_start_location(this_bone.get_end_location().minus(this_bone.get_direction_uv().times(this_bone_length)))
                if self.m_basebone_constraint_type == BaseboneConstraintType3D.NONE:
                    new_end_location = this_bone.get_start_location().plus(this_bone.get_direction_uv().times(this_bone_length))
                    this_bone.set_end_location(new_end_location)
                    if len(self.m_chain) > 1:
                        self.m_chain[1].set_start_location(new_end_location)
                else:
                    if self.m_basebone_constraint_type == BaseboneConstraintType3D.GLOBAL_ROTOR:
                        this_bone_inner_to_outer_uv = this_bone.get_direction_uv()
                        angle_between_degs = Vec3f.get_angle_between_degs(self.m_basebone_constraint_uv, this_bone_inner_to_outer_uv)
                        constraint_angle_degs = this_bone.get_ball_joint_constraint_degs()
                        if angle_between_degs > constraint_angle_degs:
                            this_bone_inner_to_outer_uv = Vec3f.get_angle_limited_unit_vector_degs(this_bone_inner_to_outer_uv, self.m_basebone_constraint_uv, constraint_angle_degs)
                        new_end_location = this_bone.get_start_location().plus(this_bone_inner_to_outer_uv.times(this_bone_length))
                        this_bone.set_end_location(new_end_location)
                        if len(self.m_chain) > 1:
                            self.m_chain[1].set_start_location(new_end_location)
                    elif self.m_basebone_constraint_type == BaseboneConstraintType3D.LOCAL_ROTOR:
                        this_bone_inner_to_outer_uv = this_bone.get_direction_uv()
                        angle_between_degs = Vec3f.get_angle_between_degs(self.m_basebone_relative_constraint_uv, this_bone_inner_to_outer_uv)
                        constraint_angle_degs = this_bone.get_ball_joint_constraint_degs()
                        if angle_between_degs > constraint_angle_degs:
                            this_bone_inner_to_outer_uv = Vec3f.get_angle_limited_unit_vector_degs(
                                this_bone_inner_to_outer_uv, self.m_basebone_relative_constraint_uv,
                                constraint_angle_degs)
                            new_end_location = this_bone.get_start_location().plus(
                                this_bone_inner_to_outer_uv.times(this_bone_length))
                            this_bone.set_end_location(new_end_location)
                            if len(self.m_chain) > 1:
                                self.m_chain[1].set_start_location(new_end_location)
                        elif self.m_basebone_constraint_type == BaseboneConstraintType3D.GLOBAL_HINGE:
                            this_joint = this_bone.get_joint()
                            hinge_rotation_axis = this_joint.get_hinge_rotation_axis()
                            cw_constraint_degs = -this_joint.get_hinge_clockwise_constraint_degs()
                            acw_constraint_degs = this_joint.get_hinge_anticlockwise_constraint_degs()
                            this_bone_inner_to_outer_uv = this_bone.get_direction_uv().project_onto_plane(
                                hinge_rotation_axis)
                            if not (abs(cw_constraint_degs + FabrikJoint3D.MAX_CONSTRAINT_ANGLE_DEGS) < 0.001 and abs(
                                    acw_constraint_degs - FabrikJoint3D.MAX_CONSTRAINT_ANGLE_DEGS) < 0.001):
                                hinge_reference_axis = this_joint.get_hinge_reference_axis()
                                signed_angle_degs = Vec3f.get_signed_angle_between_degs(hinge_reference_axis,
                                                                                        this_bone_inner_to_outer_uv,
                                                                                        hinge_rotation_axis)
                                if signed_angle_degs > acw_constraint_degs:
                                    this_bone_inner_to_outer_uv = Vec3f.rotate_about_axis_degs(hinge_reference_axis,
                                                                                               acw_constraint_degs,
                                                                                               hinge_rotation_axis).normalised()
                                elif signed_angle_degs < cw_constraint_degs:
                                    this_bone_inner_to_outer_uv = Vec3f.rotate_about_axis_degs(hinge_reference_axis,
                                                                                               cw_constraint_degs,
                                                                                               hinge_rotation_axis).normalised()
                            new_end_location = this_bone.get_start_location().plus(
                                this_bone_inner_to_outer_uv.times(this_bone_length))
                            this_bone.set_end_location(new_end_location)
                            if len(self.m_chain) > 1:
                                self.m_chain[1].set_start_location(new_end_location)
                        elif self.m_basebone_constraint_type == BaseboneConstraintType3D.LOCAL_HINGE:
                            this_joint = this_bone.get_joint()
                            hinge_rotation_axis = self.m_basebone_relative_constraint_uv
                            cw_constraint_degs = -this_joint.get_hinge_clockwise_constraint_degs()
                            acw_constraint_degs = this_joint.get_hinge_anticlockwise_constraint_degs()
                            this_bone_inner_to_outer_uv = this_bone.get_direction_uv().project_onto_plane(
                                hinge_rotation_axis)
                            if not (abs(cw_constraint_degs + FabrikJoint3D.MAX_CONSTRAINT_ANGLE_DEGS) < 0.001 and abs(
                                    acw_constraint_degs - FabrikJoint3D.MAX_CONSTRAINT_ANGLE_DEGS) < 0.001):
                                hinge_reference_axis = self.m_basebone_relative_reference_constraint_uv
                                signed_angle_degs = Vec3f.get_signed_angle_between_degs(hinge_reference_axis,
                                                                                        this_bone_inner_to_outer_uv,
                                                                                        hinge_rotation_axis)
                                if signed_angle_degs > acw_constraint_degs:
                                    this_bone_inner_to_outer_uv = Vec3f.rotate_about_axis_degs(hinge_reference_axis,
                                                                                               acw_constraint_degs,
                                                                                               hinge_rotation_axis).normalised()
                                elif signed_angle_degs < cw_constraint_degs:
                                    this_bone_inner_to_outer_uv = Vec3f.rotate_about_axis_degs(hinge_reference_axis,
                                                                                               cw_constraint_degs,
                                                                                               hinge_rotation_axis).normalised()
                            new_end_location = this_bone.get_start_location().plus(
                                this_bone_inner_to_outer_uv.times(this_bone_length))
                            this_bone.set_end_location(new_end_location)
                            if len(self.m_chain) > 1:
                                self.m_chain[1].set_start_location(new_end_location)
        self.m_last_target_location.set(target)
        return Vec3f.distance_between(self.m_chain[-1].get_end_location(), target)

    def update_chain_length(self):
        self.m_chain_length = 0.0
        for bone in self.m_chain:
            self.m_chain_length += bone.length()

    def update_embedded_target(self, new_embedded_target):
        if self.m_use_embedded_target:
            self.m_embedded_target.set(new_embedded_target)
        else:
            raise RuntimeError(
                "This chain does not have embedded targets enabled - enable with set_embedded_target_mode(true).")

    def update_embedded_target_xyz(self, x, y, z):
        if self.m_use_embedded_target:
            self.m_embedded_target.set(Vec3f(x, y, z))
        else:
            raise RuntimeError(
                "This chain does not have embedded targets enabled - enable with set_embedded_target_mode(true).")

    def clone_ik_chain(self):
        cloned_chain = []
        for bone in self.m_chain:
            cloned_chain.append(
                FabrikBone3D(start_location=bone.get_start_location(),direction_uv=bone.get_direction_uv(),length=bone.length(),
                             colour=bone.get_colour()))
        return cloned_chain

    def get_max_iteration_attempts(self):
        return self.m_max_iteration_attempts

    def get_min_iteration_change(self):
        return self.m_min_iteration_change

    def get_solve_distance_threshold(self):
        return self.m_solve_distance_threshold

    def __eq__(self, other):
        if not isinstance(other, FabrikChain3D):
            return False
        return (self.m_basebone_constraint_type == other.m_basebone_constraint_type and
                self.m_basebone_constraint_uv == other.m_basebone_constraint_uv and
                self.m_basebone_relative_constraint_uv == other.m_basebone_relative_constraint_uv and
                self.m_basebone_relative_reference_constraint_uv == other.m_basebone_relative_reference_constraint_uv and
                self.m_chain == other.m_chain and
                self.m_chain_length == other.m_chain_length and
                self.m_connected_bone_number == other.m_connected_bone_number and
                self.m_connected_chain_number == other.m_connected_chain_number and
                self.m_constraint_line_width == other.m_constraint_line_width and
                self.m_current_solve_distance == other.m_current_solve_distance and
                self.m_embedded_target == other.m_embedded_target and
                self.m_fixed_base_location == other.m_fixed_base_location and
                self.m_fixed_base_mode == other.m_fixed_base_mode and
                self.m_last_base_location == other.m_last_base_location and
                self.m_last_target_location == other.m_last_target_location and
                self.m_max_iteration_attempts == other.m_max_iteration_attempts and
                self.m_min_iteration_change == other.m_min_iteration_change and
                self.m_name == other.m_name and
                self.m_solve_distance_threshold == other.m_solve_distance_threshold and
                self.m_use_embedded_target == other.m_use_embedded_target)

    def __hash__(self):
        return hash((self.m_basebone_constraint_type, self.m_basebone_constraint_uv,
                     self.m_basebone_relative_constraint_uv,
                     self.m_basebone_relative_reference_constraint_uv, tuple(self.m_chain),
                     self.m_chain_length,
                     self.m_connected_bone_number, self.m_connected_chain_number,
                     self.m_constraint_line_width,
                     self.m_current_solve_distance, self.m_embedded_target, self.m_fixed_base_location,
                     self.m_fixed_base_mode,
                     self.m_last_base_location, self.m_last_target_location,
                     self.m_max_iteration_attempts,
                     self.m_min_iteration_change, self.m_name, self.m_solve_distance_threshold,
                     self.m_use_embedded_target))

    def __str__(self):
        sb = ["--- FabrikChain3D: ", self.m_name, " ---\n"]
        if self.m_chain:
            sb.append(f"Bone count:    : {len(self.m_chain)}\n")
            sb.append(f"Base location  : {self.get_base_location()}\n")
            sb.append(f"Chain length   : {self.get_chain_length()}\n")
            sb.append(f"Fixed base mode: {'Yes' if self.m_fixed_base_mode else 'No'}\n")
            for bone in self.m_chain:
                sb.append(f"--- Bone: {bone} ---\n")
                sb.append(f"{str(bone)}\n")
        else:
            sb.append("Chain does not contain any bones.\n")
        return ''.join(map(str, sb))