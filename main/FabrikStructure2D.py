from enum import Enum
from typing import Optional


from FabrikChain2D import FabrikChain2D, BaseboneConstraintType2D, BoneConnectionPoint
from utils.Utils import Utils
from utils.Vec2f import Vec2f



class FabrikStructure2D:
    def __init__(self, name: Optional[str] = None):
        self.m_name = name if name else ""
        self.m_chains = []
        self.m_fixed_base_mode = True

    def set_name(self, name: str):
        self.m_name = Utils.get_validated_name(name)

    def solve_for_target(self, new_target_location: Vec2f):
        num_chains = len(self.m_chains)
        for loop in range(num_chains):
            this_chain = self.m_chains[loop]
            host_chain_number = this_chain.get_connected_chain_number()
            constraint_type = this_chain.get_basebone_constraint_type()

            if host_chain_number != -1 and constraint_type != BaseboneConstraintType2D.GLOBAL_ABSOLUTE:
                host_bone = self.m_chains[host_chain_number].get_bone(this_chain.get_connected_bone_number())
                connection_point = this_chain.get_bone_connection_point()
                connection_location = host_bone.get_start_location() if connection_point == BoneConnectionPoint.START else host_bone.get_end_location()
                this_chain.set_base_location(connection_location)

                host_bone_uv = host_bone.get_direction_uv()
                if constraint_type == BaseboneConstraintType2D.LOCAL_RELATIVE:
                    this_chain.set_basebone_constraint_uv(host_bone_uv)
                elif constraint_type == BaseboneConstraintType2D.LOCAL_ABSOLUTE:
                    angle_degs = Vec2f(0.0, 1.0).get_signed_angle_degs_to(host_bone_uv)
                    relative_constraint_uv = Vec2f.rotate_degs(this_chain.m_basebone_constraint_uv, angle_degs)
                    this_chain.set_basebone_relative_constraint_uv(relative_constraint_uv)

            if not this_chain.get_embedded_target_mode():
                this_chain.solve_for_target(new_target_location)
            else:
                this_chain.solve_for_embedded_target()

    def add_chain(self, chain: FabrikChain2D):
        self.m_chains.append(chain)

    def connect_chain_3(self, chain: FabrikChain2D, chain_number: int, bone_number: int):
        if chain_number >= len(self.m_chains):
            raise ValueError(f"Cannot connect to chain {chain_number} - no such chain (remember that chains are zero indexed).")
        if bone_number >= self.m_chains[chain_number].get_num_bones():
            raise ValueError(f"Cannot connect to bone {bone_number} of chain {chain_number} - no such bone (remember that bones are zero indexed).")

        relative_chain = FabrikChain2D(chain)
        relative_chain.set_connected_chain_number(chain_number)
        relative_chain.set_connected_bone_number(bone_number)

        connection_point = chain.get_bone_connection_point()
        connection_location = self.m_chains[chain_number].get_bone(bone_number).get_start_location() if connection_point == BoneConnectionPoint.START else self.m_chains[chain_number].get_bone(bone_number).get_end_location()
        relative_chain.set_base_location(connection_location)
        relative_chain.set_fixed_base_mode(True)

        for loop in range(chain.get_num_bones()):
            orig_start = relative_chain.get_bone(loop).get_start_location()
            orig_end = relative_chain.get_bone(loop).get_end_location()
            translated_start = orig_start.plus(connection_location)
            translated_end = orig_end.plus(connection_location)
            relative_chain.get_bone(loop).start_location = translated_start
            relative_chain.get_bone(loop).end_location = translated_end

        self.add_chain(relative_chain)

    def connect_chain_4(self, chain: FabrikChain2D, chain_number: int, bone_number: int, bone_connection_point: BoneConnectionPoint):
        chain.set_bone_connection_point(bone_connection_point)
        self.connect_chain_3(chain, chain_number, bone_number)

    def get_num_chains(self) -> int:
        return len(self.m_chains)

    def get_chain(self, chain_number: int) -> FabrikChain2D:
        return self.m_chains[chain_number]

    def set_fixed_base_mode(self, fixed_base_mode: bool):
        self.m_fixed_base_mode = fixed_base_mode
        self.m_chains[0].set_fixed_base_mode(self.m_fixed_base_mode)

    def get_name(self) -> str:
        return self.m_name

    def __str__(self) -> str:
        sb = [f"----- FabrikStructure2D: {self.m_name} -----{Utils.NEW_LINE}",
              f"Number of chains: {len(self.m_chains)}{Utils.NEW_LINE}"]
        for loop in range(len(self.m_chains)):
            sb.append(str(self.m_chains[loop]))
        return ''.join(sb)

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, FabrikStructure2D):
            return False
        return (self.m_chains == other.m_chains and
                self.m_fixed_base_mode == other.m_fixed_base_mode and
                self.m_name == other.m_name)

    def __hash__(self) -> int:
        return hash((self.m_chains, self.m_fixed_base_mode, self.m_name))