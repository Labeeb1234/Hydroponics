from typing import List

MOVE_GROUP_ARM: str = "robot_arm"

def joint_names(prefix: str = "") -> List[str]:
    return ["arm_base_joint",
            "shoulder_joint",
            "bottom_wrist_joint",
            "elbow_joint",
            "top_wrist_joint",]

def base_link_name(prefix: str = "") -> str:
    return "mani_base_1"


def end_effector_name(prefix: str = "") -> str:
    return "cam_link"
