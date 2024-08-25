from enum import Enum
from typing import List, Optional

from FabrikChain3D import FabrikChain3D, BaseboneConstraintType3D
from FabrikBone3D import BoneConnectionPoint
from utils.Mat3f import Mat3f
from utils.Utils import Utils
from utils.Vec3f import Vec3f



class FabrikStructure3D:
    def __init__(self, name: Optional[str] = None):
        self.m_name = name if name else ""
        self.m_chains = []

    def set_name(self, name: str):
        self.m_name = Utils.get_validated_name(name)

    def solve_for_target(self, new_target_location: Vec3f):
        num_chains = len(self.m_chains)
        for loop in range(num_chains):
            this_chain = self.m_chains[loop]
            connected_chain_number = this_chain.get_connected_chain_number()
            if connected_chain_number == -1:
                this_chain.solve_for_target(new_target_location)
            else:
                host_chain = self.m_chains[connected_chain_number]
                host_bone = host_chain.get_bone(this_chain.get_connected_bone_number())
                if host_bone.get_bone_connection_point() == BoneConnectionPoint.START:
                    this_chain.set_base_location(host_bone.get_start_location())
                else:
                    this_chain.set_base_location(host_bone.get_end_location())

                constraint_type = this_chain.get_basebone_constraint_type()
                if constraint_type in [BaseboneConstraintType3D.LOCAL_ROTOR, BaseboneConstraintType3D.LOCAL_HINGE]:
                    connection_bone_matrix = Mat3f.create_rotation_matrix(host_bone.get_direction_uv())
                    relative_basebone_constraint_uv = connection_bone_matrix.times(this_chain.m_basebone_constraint_uv).normalised()
                    this_chain.set_basebone_relative_constraint_uv(relative_basebone_constraint_uv)
                    if constraint_type == BaseboneConstraintType3D.LOCAL_HINGE:
                        this_chain.set_basebone_relative_reference_constraint_uv(connection_bone_matrix.times(this_chain.get_bone(0).get_joint().get_hinge_reference_axis()))

                if not this_chain.get_embedded_target_mode():
                    this_chain.solve_for_target(new_target_location)
                else:
                    this_chain.solve_for_embedded_target()

    def add_chain(self, chain: FabrikChain3D):
        self.m_chains.append(chain)

    def remove_chain(self, chain_index: int):
        self.m_chains.pop(chain_index)

    def connect_chain_3(self, new_chain: FabrikChain3D, existing_chain_number: int, existing_bone_number: int):
        if existing_chain_number >= len(self.m_chains):
            raise ValueError(f"Cannot connect to chain {existing_chain_number} - no such chain (remember that chains are zero indexed).")
        if existing_bone_number >= self.m_chains[existing_chain_number].get_num_bones():
            raise ValueError(f"Cannot connect to bone {existing_bone_number} of chain {existing_chain_number} - no such bone (remember that bones are zero indexed).")

        relative_chain = FabrikChain3D(source=new_chain)
        relative_chain.connect_to_structure(self, existing_chain_number, existing_bone_number)

        connection_point = self.m_chains[existing_chain_number].get_bone(existing_bone_number).get_bone_connection_point()
        connection_location = self.m_chains[existing_chain_number].get_bone(existing_bone_number).get_start_location() if connection_point == BoneConnectionPoint.START else self.m_chains[existing_chain_number].get_bone(existing_bone_number).get_end_location()
        relative_chain.set_base_location(connection_location)
        relative_chain.set_fixed_base_mode(True)

        for loop in range(relative_chain.get_num_bones()):
            orig_start = relative_chain.get_bone(loop).get_start_location()
            orig_end = relative_chain.get_bone(loop).get_end_location()
            translated_start = orig_start.plus(connection_location)
            translated_end = orig_end.plus(connection_location)
            relative_chain.get_bone(loop).start_location = translated_start
            relative_chain.get_bone(loop).end_location = translated_end

        self.add_chain(relative_chain)

    def connect_chain_4(self, new_chain: FabrikChain3D, existing_chain_number: int, existing_bone_number: int, bone_connection_point: BoneConnectionPoint):
        if existing_chain_number >= len(self.m_chains):
            raise ValueError(f"Cannot connect to chain {existing_chain_number} - no such chain (remember that chains are zero indexed).")
        if existing_bone_number >= self.m_chains[existing_chain_number].get_num_bones():
            raise ValueError(f"Cannot connect to bone {existing_bone_number} of chain {existing_chain_number} - no such bone (remember that bones are zero indexed).")

        relative_chain = FabrikChain3D(source=new_chain)
        relative_chain.connect_to_structure(self, existing_chain_number, existing_bone_number)

        self.m_chains[existing_chain_number].get_bone(existing_bone_number).set_bone_connection_point(bone_connection_point)
        connection_location = self.m_chains[existing_chain_number].get_bone(existing_bone_number).get_start_location() if bone_connection_point == BoneConnectionPoint.START else self.m_chains[existing_chain_number].get_bone(existing_bone_number).get_end_location()
        relative_chain.set_base_location(connection_location)
        relative_chain.set_fixed_base_mode(True)

        for loop in range(relative_chain.get_num_bones()):
            orig_start = relative_chain.get_bone(loop).get_start_location()
            orig_end = relative_chain.get_bone(loop).get_end_location()
            translated_start = orig_start.plus(connection_location)
            translated_end = orig_end.plus(connection_location)
            relative_chain.get_bone(loop).start_location = translated_start
            relative_chain.get_bone(loop).end_location = translated_end

        self.add_chain(relative_chain)

    def get_num_chains(self) -> int:
        return len(self.m_chains)

    def get_chain(self, chain_number: int) -> FabrikChain3D:
        return self.m_chains[chain_number]

    def set_fixed_base_mode(self, fixed_base_mode: bool):
        for loop in range(len(self.m_chains)):
            self.m_chains[loop].set_fixed_base_mode(fixed_base_mode)

    def get_name(self) -> str:
        return self.m_name

    def __str__(self) -> str:
        sb = [f"----- FabrikStructure3D: {self.m_name} -----\n", f"Number of chains: {len(self.m_chains)}\n"]
        for loop in range(len(self.m_chains)):
            sb.append(str(self.m_chains[loop]))
        return ''.join(sb)

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, FabrikStructure3D):
            return False
        return (self.m_chains == other.m_chains and
                self.m_name == other.m_name)

    def __hash__(self) -> int:
        return hash((self.m_chains, self.m_name))