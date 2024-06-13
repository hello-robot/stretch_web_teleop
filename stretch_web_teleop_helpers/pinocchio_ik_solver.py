# Copied from stretchpy/src/stretch/motion/pinocchio_ik_solver.py
# TODO: Remove this file once the above package is finalized.
import sys
from typing import List, Tuple

import numpy as np
import pinocchio
from loguru import logger
from scipy.spatial.transform import Rotation as R

# --DEFAULTS--
# Error tolerances
POS_ERROR_TOL = 0.005
ORI_ERROR_TOL = [0.1, 0.1, np.pi / 2]

# CEM
CEM_MAX_ITERATIONS = 5
CEM_NUM_SAMPLES = 50
CEM_NUM_TOP = 10


def level_filter(level):
    """Filter log messages by level"""

    def is_level(record):
        return record["level"].name == level

    return is_level


logger.remove(0)
logger.add(sys.stderr, filter=level_filter(level="WARNING"))


class PinocchioIKSolver:
    """IK solver using pinocchio which can handle end-effector constraints for optimized IK solutions"""

    EPS = 1e-4
    DT = 1e-1
    DAMP = 1e-12

    def __init__(
        self,
        urdf_path: str,
        ee_link_name: str,
        controlled_joints: List[str],
        verbose: bool = False,
    ):
        """
        urdf_path: path to urdf file
        ee_link_name: name of the end-effector link
        controlled_joints: list of joint names to control
        """
        if verbose:
            print(f"{urdf_path=}")
        self.model = pinocchio.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.q_neutral = pinocchio.neutral(self.model)

        # print([f.name for f in self.model.frames])
        self.ee_frame_idx = [f.name for f in self.model.frames].index(ee_link_name)

        self.controlled_joints_by_name = {}
        self.controlled_joints = []
        self.controlled_joint_names = controlled_joints
        for joint in controlled_joints:
            if joint == "ignore":
                idx = -1
            else:
                jid = self.model.getJointId(joint)
                if jid >= len(self.model.idx_qs):
                    logger.error(f"{joint=} {jid=} not in model.idx_qs")
                    raise RuntimeError(
                        f"Invalid urdf at {urdf_path=}: missing {joint=}"
                    )
                else:
                    idx = self.model.idx_qs[jid]
            self.controlled_joints.append(idx)
            self.controlled_joints_by_name[joint] = idx

        logger.info(f"{controlled_joints=}")
        for j in controlled_joints:
            idx = self.model.getJointId(j)
            idx_q = self.model.idx_qs[idx]
            logger.info(f"{j=} {idx=} {idx_q=}")

    def get_dof(self) -> int:
        """returns dof for the manipulation chain"""
        return len(self.controlled_joints)

    def get_num_controllable_joints(self) -> int:
        """returns number of controllable joints under this solver's purview"""
        return len(self.controlled_joints)

    def get_all_joint_names(self) -> List[str]:
        """Return a list of joints"""
        return [self.model.names[i + 1] for i in range(self.model.nq)]

    def _qmap_control2model(self, q_input: np.ndarray) -> np.ndarray:
        """returns a full joint configuration from a partial joint configuration"""
        q_out = self.q_neutral.copy()
        if isinstance(q_input, dict):
            for joint_name, value in q_input.items():
                if joint_name in self.controlled_joints_by_name:
                    q_out[self.controlled_joints_by_name[joint_name]] = value
                else:
                    q_out[self.model.idx_qs[self.model.getJointId(joint_name)]] = value
        else:
            assert len(self.controlled_joints) == len(
                q_input
            ), "if not specifying by name, must match length"
            for i, joint_idx in enumerate(self.controlled_joints):
                q_out[joint_idx] = q_input[i]
        return q_out

    def _qmap_model2control(self, q_input: np.ndarray) -> np.ndarray:
        """returns a partial joint configuration from a full joint configuration"""
        q_out = np.empty(len(self.controlled_joints))
        for i, joint_idx in enumerate(self.controlled_joints):
            if joint_idx >= 0:
                q_out[i] = q_input[joint_idx]

        return q_out

    def compute_fk(self, config) -> Tuple[np.ndarray, np.ndarray]:
        """given joint values return end-effector position and quaternion associated with it"""
        q_model = self._qmap_control2model(config)
        pinocchio.forwardKinematics(self.model, self.data, q_model)
        pinocchio.updateFramePlacement(self.model, self.data, self.ee_frame_idx)
        pos = self.data.oMf[self.ee_frame_idx].translation
        quat = R.from_matrix(self.data.oMf[self.ee_frame_idx].rotation).as_quat()

        return pos.copy(), quat.copy()

    def compute_ik(
        self,
        pos_desired: np.ndarray,
        quat_desired: np.ndarray,
        q_init=None,
        max_iterations=100,
        num_attempts: int = 1,
        verbose: bool = False,
    ) -> Tuple[np.ndarray, bool, dict]:
        """given end-effector position and quaternion, return joint values.

        Two parameters are currently unused and might be implemented in the future:
            q_init: initial configuration for the optimization to start in; especially useful for
                    arms with redundant degrees of freedom
            num_attempts: start from multiple initial configs; included for compatibility with pb
            max iterations: time budget in number of steps; included for compatibility with pb
        """
        i = 0

        if q_init is None:
            q = self.q_neutral.copy()
            if num_attempts > 1:
                raise NotImplementedError(
                    "Sampling multiple initial configs not yet supported by Pinocchio solver."
                )
        else:
            q = self._qmap_control2model(q_init)
            # Override the number of attempts
            num_attempts = 1

        desired_ee_pose = pinocchio.SE3(
            R.from_quat(quat_desired).as_matrix(), pos_desired
        )
        while True:
            pinocchio.forwardKinematics(self.model, self.data, q)
            pinocchio.updateFramePlacement(self.model, self.data, self.ee_frame_idx)

            dMi = desired_ee_pose.actInv(self.data.oMf[self.ee_frame_idx])
            err = pinocchio.log(dMi).vector
            if verbose:
                print(f"[pinocchio_ik_solver] iter={i}; error={err}")
            if np.linalg.norm(err) < self.EPS:
                success = True
                break
            if i >= max_iterations:
                success = False
                break
            J = pinocchio.computeFrameJacobian(
                self.model,
                self.data,
                q,
                self.ee_frame_idx,
                pinocchio.ReferenceFrame.LOCAL,
            )
            v = -J.T.dot(np.linalg.solve(J.dot(J.T) + self.DAMP * np.eye(6), err))
            q = pinocchio.integrate(self.model, q, v * self.DT)
            i += 1

        q_control = self._qmap_model2control(q.flatten())
        debug_info = {"iter": i, "final_error": err}

        return q_control, success, debug_info

    def q_array_to_dict(self, arr: np.ndarray):
        state = {}
        assert len(arr) == len(self.controlled_joint_names)
        for i, name in enumerate(self.controlled_joint_names):
            state[name] = arr[i]
        return state
