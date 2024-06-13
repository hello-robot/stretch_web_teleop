"""
This file contains various constants relevant to stretch:
joint names, hard-coded configurations, speed profiles, control modes,
frames of reference, etc.
"""

# Standard Imports
from enum import Enum
from typing import Dict, List

# Third-Party Imports
import numpy as np


class Joint(Enum):
    """
    Joint names of Stretch.
    """

    BASE_ROTATION = "joint_mobile_base_rotation"
    ARM_LIFT = "joint_lift"
    ARM_L0 = "joint_arm_l0"
    ARM_L1 = "joint_arm_l1"
    ARM_L2 = "joint_arm_l2"
    ARM_L3 = "joint_arm_l3"
    COMBINED_ARM = "joint_arm"
    WRIST_YAW = "joint_wrist_yaw"
    WRIST_PITCH = "joint_wrist_pitch"
    WRIST_ROLL = "joint_wrist_roll"
    GRIPPER_RIGHT = "joint_gripper_finger_right"
    GRIPPER_LEFT = "joint_gripper_finger_left"
    RIGHT_WHEEL = "joint_right_wheel"
    LEFT_WHEEL = "joint_left_wheel"
    HEAD_PAN = "joint_head_pan"
    HEAD_TILT = "joint_head_tilt"

    @staticmethod
    def get_arm_joints():
        """
        Get the list of telescoping arm joints.
        """
        return [Joint.ARM_L0, Joint.ARM_L1, Joint.ARM_L2, Joint.ARM_L3]

    @staticmethod
    def get_wrist_joints():
        """
        Get the list of wrist joints.
        """
        return [Joint.WRIST_YAW, Joint.WRIST_PITCH, Joint.WRIST_ROLL]


class Frame(Enum):
    """
    Key frame names of reference for Stretch.
    """

    BASE_LINK = "base_link"
    END_EFFECTOR_LINK = "link_grasp_center"
    ODOM = "odom"


class ControlMode(Enum):
    """
    The control modes for the Stretch Driver.
    """

    POSITION = "position"
    NAVIGATION = "navigation"

    def get_service_name(self):
        """
        Get the service name for switching to this control mode.
        """
        return f"switch_to_{self.value}_mode"


class SpeedProfile(Enum):
    """
    The speed profile to use to get max velocities and accelerations. This
    should correspond to the speed profile in robot params.
    """

    SLOW = "slow"
    DEFAULT = "default"
    FAST = "fast"
    MAX = "max"


def get_stow_configuration(
    joints: List[Joint], partial: bool = False
) -> Dict[Joint, float]:
    """
    Get the joint configuration for stowing the arm.

    Note that in practice, commanding all these joints at the same time can create
    a motion with a larger footprint than desired, resulting in collisions. It is
    typically best practice to first command arm length, then command wrist/gripper,
    then command lift.

    Parameters
    ----------
    joints: The joints return.
    partial: If True, make the arm length stop slightly before the robot base, so that if the
        wrist is vertically down, it won't collide with the base.

    Returns
    -------
    Dict[Joint, float]: The joint configuration.
    """
    retval = {}
    for joint in joints:
        if joint == Joint.ARM_L0:
            retval[joint] = 0.1675 if partial else 0.0
        elif joint == Joint.ARM_LIFT:
            retval[
                joint
            ] = 0.40  # This is chosen so even when the gripper is pointing down, it doesn't hit the base.
        elif joint == Joint.WRIST_YAW:
            retval[joint] = 3.19579  # Should match src/shared/util.tsx
        elif joint == Joint.WRIST_PITCH:
            retval[joint] = -0.497  # Should match src/shared/util.tsx
        elif joint == Joint.WRIST_ROLL:
            retval[joint] = 0.0  # Should match src/shared/util.tsx
        elif joint == Joint.GRIPPER_LEFT:
            retval[
                joint
            ] = 0.0  # close gripper when stowed. An open gripper can sometimes get caught in the mast.
    return retval


def get_pregrasp_wrist_configuration(horizontal: bool) -> Dict[Joint, float]:
    """
    Get the wrist rotation for the pregrasp position.

    TODO: Add the option to specify 0 or 90 degree roll.

    Parameters
    ----------
    horizontal_grasp: Whether the pregrasp position is horizontal.

    Returns
    -------
    Dict[Joint, float]: The joint configuration.
    """
    if horizontal:
        return {
            Joint.WRIST_YAW: 0.0,
            Joint.WRIST_PITCH: 0.0,
            Joint.WRIST_ROLL: 0.0,
        }
    else:
        return {
            Joint.WRIST_YAW: 0.0,
            Joint.WRIST_PITCH: -np.pi / 2.0,
            Joint.WRIST_ROLL: 0.0,
        }


def get_gripper_configuration(closed: bool) -> Dict[Joint, float]:
    """
    Get the gripper configuration.

    Parameters
    ----------
    closed: Whether the gripper is closed. If false, returns the configuration
        where the gripper is fully open.

    Returns
    -------
    Dict[Joint, float]: The joint configuration.
    """
    return {
        # We only need to command one gripper joint, as the other is coupled.
        Joint.GRIPPER_LEFT: 0.0
        if closed
        else 0.84,
    }
