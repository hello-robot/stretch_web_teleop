import pathlib
import numpy as np
import stretch_body.hello_utils as hu
from typing import Dict, Optional, Tuple


def get_latest_urdf():
    """
    Fetches the latest calibrated URDF from the calibration directory.

    Returns
    -------
    str: Absolute filepath to the latest calibrated URDF.
    """
    try:
        fleet_dir = hu.get_fleet_directory()
    except:
        raise FileNotFoundError("Stretch data directory doesn't exist") from None
    calibration_dir = pathlib.Path(fleet_dir) / "exported_urdf"
    if not calibration_dir.is_dir():
        raise FileNotFoundError("URDF calibration directory doesn't exist")
    urdf_path = calibration_dir / "stretch.urdf"
    if not urdf_path.is_file():
        raise FileNotFoundError("URDF doesn't exist")
    urdf_filename = str(urdf_path.absolute())
    return urdf_filename


def get_joint_limits(use_original_limits=True):
    """
    Enables more conservative joint limits to be set than in the
    original URDF.

    If these limits are outside the originally permitted range,
    the original range is used. Joint limits. Where these limits
    have a value of None, the original limit is used.

    Parameters
    ----------
    use_original_limits: bool
        don't impose any additional limits

    Returns
    -------
    dict[str, tuple(float or None, float or None)]:
        mapping between joint and (lower limit, upper limit)
    """
    ik_joint_limits: Dict[str, Tuple[Optional[float], Optional[float]]] = {}
    if use_original_limits:
        ik_joint_limits = {
            "joint_mobile_base_translation": (None, None),
            "joint_mobile_base_rotation": (None, None),
            "joint_lift": (None, None),
            "joint_arm_l0": (None, None),
            "joint_wrist_yaw": (None, None),
            "joint_wrist_pitch": (-0.8 * (np.pi / 2.0), None), # Beware of gimbal lock if joint_wrist_pitch is too close to -90 deg
            "joint_wrist_roll": (None, None),
        }
    else:
        ik_joint_limits = {
            "joint_mobile_base_translation": (-0.25, 0.25),
            "joint_mobile_base_rotation": (-(np.pi / 2.0), np.pi / 2.0),
            "joint_lift": (0.01, 1.09),
            "joint_arm_l0": (0.01, 0.48),
            "joint_wrist_yaw": (-(np.pi / 4.0), np.pi),
            "joint_wrist_pitch": (-0.9 * (np.pi / 2.0), np.pi / 20.0),
            "joint_wrist_roll": (-(np.pi / 2.0), np.pi / 2.0),
        }

    return ik_joint_limits


def generate_urdf():
    pass


if __name__ == "__main__":
    print(get_latest_urdf())
    print('')
    import pprint; pprint.pprint(get_joint_limits())
    print('')
    pprint.pprint(get_joint_limits(use_original_limits=False))
