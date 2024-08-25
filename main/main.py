from FabrikBone3D import FabrikBone3D
from FabrikChain3D import FabrikChain3D
from FabrikJoint3D import JointType
from FabrikStructure3D import FabrikStructure3D
from utils.Colour4f import Colour4f
from utils.Utils import Utils
from utils.Vec3f import Vec3f, X_AXIS, Y_AXIS, Z_AXIS

default_bone_direction = Z_AXIS.negated()
default_bone_length = 10.0
bone_line_width = 5.0
constraint_line_width = 2.0
base_rotation_amount_degs = 0.3

class DemoLocalHingesWithReferenceAxisConstraints:
    def __init__(self):
        self.structure = None

    def setup(self):
        self.structure = FabrikStructure3D("Demo - Local Hinges with Reference Axis Constraints")
        num_chains = 3

        # We'll create a circular arrangement of 3 chains with alternate bones each constrained about different local axes.
        # Note: Local hinge rotation axes are relative to the rotation matrix of the previous bone in the chain.
        hinge_rotation_axis = Vec3f()
        hinge_reference_axis = Vec3f()

        rot_step = 360.0 / num_chains
        for loop in range(num_chains):
            # Set colour and axes
            chain_colour = Colour4f()
            if loop % 3 == 0:
                chain_colour = Utils.RED
                hinge_rotation_axis = X_AXIS
                hinge_reference_axis = Y_AXIS
            elif loop % 3 == 1:
                chain_colour = Utils.GREEN
                hinge_rotation_axis = Y_AXIS
                hinge_reference_axis = X_AXIS
            elif loop % 3 == 2:
                chain_colour = Utils.BLUE
                hinge_rotation_axis = Z_AXIS
                hinge_reference_axis = Y_AXIS

            # Create a new chain
            chain = FabrikChain3D()

            # Set up the initial base bone location...
            start_loc = Vec3f(0.0, 0.0, -40.0)
            start_loc = Vec3f.rotate_y_degs(start_loc, rot_step * loop)
            end_loc = start_loc.plus(default_bone_direction.times(default_bone_length))

            # ...then create a base bone, set its colour, and add it to the chain.
            basebone = FabrikBone3D(start_location=start_loc, end_location=end_loc)
            basebone.set_colour(chain_colour)
            chain.add_bone(basebone)

            # Add alternating local hinge constrained and unconstrained bones to the chain
            constraint_angle_degs = 90.0
            for bone_loop in range(6):
                if bone_loop % 2 == 0:
                    chain.add_consecutive_hinged_bone(default_bone_direction, default_bone_length, JointType.LOCAL_HINGE, hinge_rotation_axis, constraint_angle_degs, constraint_angle_degs, hinge_reference_axis, Utils.GREY)
                else:
                    chain.add_consecutive_bone(default_bone_direction, default_bone_length, chain_colour)

            # Finally, add the chain to the structure
            self.structure.add_chain(chain)

    def get_structure(self):
        return self.structure


if __name__ == "__main__":
    demo = DemoLocalHingesWithReferenceAxisConstraints()
    demo.setup()
    structure = demo.get_structure()
    structure.solve_for_target(Vec3f(0.0, 0.0, 0.0))
    print(structure)
    print("Done")