#!/usr/bin/env python3
import subprocess
import time

import numpy as np
import PyKDL as kdl
from urdf import treeFromString


def jacobian_to_np(J):
    return np.array([[J[i, j] for j in range(J.columns())] for i in range(J.rows())])


if __name__ == "__main__":
    investigation_1 = False
    if investigation_1:
        # Investigation 1: Is it possible to get a chain and Jacobian
        # without modifying the URDF?
        urdf_string = subprocess.check_output(
            [
                "xacro",
                "/home/hello-robot/ament_ws/src/stretch_ros2/stretch_description/"
                "urdf/stretch_description_SE3_eoa_wrist_dw3_tool_sg3.xacro",
            ]
        )
        # print(urdf_string.decode())
        # raise Exception()
        ok, robot = treeFromString(urdf_string)
        # This chain has 8 non-fixed joints: joint_lift, joint_arm_l3, joint_arm_l2,
        # joint_arm_l1, joint_arm_l0, joint_wrist_yaw, joint_wrist_pitch, joint_wrist_roll
        orig_chain = robot.getChain("base_link", "link_grasp_center")
        print(orig_chain, type(orig_chain), orig_chain.getNrOfJoints())

        # # Add a dummy link for translation and rotation from the base link
        # for i in range(orig_chain.getNrOfSegments()):
        #     print("Segment", i)
        #     s = orig_chain.getSegment(i)
        #     print(" "*4, "Name", s.getName())
        #     print(" "*4, "Joint", s.getJoint())
        #     print(" "*4, "FrameToTip", s.getFrameToTip())
        #     print(" "*4, "RigidBodyIntertia", s.getInertia().getMass(), s.getInertia().getCOG())
        #     rot_intertia = s.getInertia().getRotationalInertia()
        #     print(" "*4, "RotationalIntertia", [rot_intertia[i] for i in range(8)])
        # test_rot_intertia = kdl.RotationalInertia(
        #     Ixx=0.291286,
        #     Iyy=0.004149,
        #     Izz=-0.000174,
        #     Ixy=0.001226,
        #     Ixz=0.000354,
        #     Iyz=0.291429
        # )
        # print("TestRotationalIntertia", [test_rot_intertia[i] for i in range(8)])
        # Segment 6
        #     Name link_arm_l0
        #     Joint joint_arm_l0:[TransAxis, axis: [-3.80121e-15,-1.76235e-29,           1], origin[           0,           0,    -0.01375]]
        #     FrameToTip [[           1,-2.62708e-15,-3.80121e-15;
        # 2.62708e-15,           1,-1.76235e-29;
        # 3.80121e-15, 7.63747e-30,           1]
        # [           0,           0,    -0.01375]]
        #     RigidBodyIntertia 0.427734 [    0.033681,   -0.000847,   -0.031723]
        #     RotationalIntertia [0.0021197564274712923, -1.5797687000661998e-05, 0.000940017680375442, -1.5797687000661998e-05, 0.00302267519196166, -2.2492947812654e-05, 0.000940017680375442, -2.2492947812654e-05]
        chain = kdl.Chain()
        chain.addSegment(
            kdl.Segment(
                name="link_base_translation",
                joint=kdl.Joint(
                    name="joint_base_translation",
                    origin=kdl.Vector(0, 0, 0),
                    axis=kdl.Vector(1, 0, 0),
                    type=kdl.Joint.TransAxis,
                ),
                f_tip=kdl.Frame(),
                I=kdl.RigidBodyInertia(
                    m=1.0,
                    oc=kdl.Vector(0, 0, 0),
                    Ic=kdl.RotationalInertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0),
                ),
            )
        )
        chain.addSegment(
            kdl.Segment(
                name="link_base_revolution",
                joint=kdl.Joint(
                    name="joint_base_revolution",
                    origin=kdl.Vector(0, 0, 0),
                    axis=kdl.Vector(1, 0, 0),
                    type=kdl.Joint.RotAxis,
                ),
                f_tip=kdl.Frame(),
                I=kdl.RigidBodyInertia(
                    m=1.0,
                    oc=kdl.Vector(0, 0, 0),
                    Ic=kdl.RotationalInertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0),
                ),
            )
        )
        chain.addChain(orig_chain)

        # Sample joints to test. Arm after homing.
        # name:
        # - joint_right_wheel
        # - joint_left_wheel
        # - joint_lift
        # - joint_arm_l3
        # - joint_arm_l2
        # - joint_arm_l1
        # - joint_arm_l0
        # - joint_wrist_yaw
        # - joint_head_pan
        # - joint_head_tilt
        # - joint_wrist_pitch
        # - joint_wrist_roll
        # - joint_gripper_finger_right
        # - joint_gripper_finger_left
        # position:
        # - 0.0
        # - 0.0
        # - 0.5994133216606548
        # - 0.024997924991407225
        # - 0.024997924991407225
        # - 0.024997924991407225
        # - 0.024997924991407225
        # - 0.009587379924285258
        # - 0.007731993378606676
        # - 0.06897162366246637
        # - -0.5399612373357457
        # - -0.0015339807878856412
        # - 0.4450037904233133
        # - 0.4450037904233133
        q_kdl_1 = kdl.JntArray(chain.getNrOfJoints())
        q_kdl_1[0] = 0.0
        q_kdl_1[1] = 0.0
        q_kdl_1[2] = 0.5994133216606548
        q_kdl_1[3] = 0.024997924991407225
        q_kdl_1[4] = 0.024997924991407225
        q_kdl_1[5] = 0.024997924991407225
        q_kdl_1[6] = 0.024997924991407225
        q_kdl_1[7] = 0.009587379924285258
        q_kdl_1[8] = -0.5399612373357457
        q_kdl_1[9] = -0.0015339807878856412
        # Arm with the wrist rotated in stow and lift lowered
        # name:
        # - joint_right_wheel
        # - joint_left_wheel
        # - joint_lift
        # - joint_arm_l3
        # - joint_arm_l2
        # - joint_arm_l1
        # - joint_arm_l0
        # - joint_wrist_yaw
        # - joint_head_pan
        # - joint_head_tilt
        # - joint_wrist_pitch
        # - joint_wrist_roll
        # - joint_gripper_finger_right
        # - joint_gripper_finger_left
        # position:
        # - 0.0
        # - 0.0
        # - 0.1030816611928582
        # - 0.024999421316935718
        # - 0.024999421316935718
        # - 0.024999421316935718
        # - 0.024999421316935718
        # - 3.1804535002162297
        # - -1.5983458915376596
        # - -1.0554362938577084
        # - -0.007669903939428206
        # - -0.0030679615757712823
        # - 0.4450037904233133
        # - 0.4450037904233133
        q_kdl_2 = kdl.JntArray(chain.getNrOfJoints())
        q_kdl_2[0] = 0.0
        q_kdl_2[1] = 0.0
        q_kdl_2[2] = 0.1030816611928582
        q_kdl_2[3] = 0.024999421316935718
        q_kdl_2[4] = 0.024999421316935718
        q_kdl_2[5] = 0.024999421316935718
        q_kdl_2[6] = 0.024999421316935718
        q_kdl_2[7] = 3.1804535002162297
        q_kdl_2[8] = -0.007669903939428206
        q_kdl_2[9] = -0.0030679615757712823

        # Get the Jacobian matrix for the chain
        jac_solver = kdl.ChainJntToJacSolver(chain)
        for q in [q_kdl_1, q_kdl_2]:
            Js = []
            for q01 in [(1.0, 3.14159 / 2), (q[0], q[1])]:
                J = kdl.Jacobian(chain.getNrOfJoints())
                q[0] = q01[0]
                q[1] = q01[1]
                jac_solver.JntToJac(q, J)
                J = jacobian_to_np(J)
                Js.append(J)
            print(Js[0])
            print(Js[1])
            print(np.isclose(Js[0], Js[1], atol=1e-6))

    investigation_2 = True
    # Investigation 2: Try out controlling the robot using the velocity
    # controller from visual servoing.
    if investigation_2:
        import normalized_velocity_control as nvc
        import stretch_body.robot as rb

        robot = rb.Robot()
        robot.startup()

        controller = nvc.NormalizedVelocityControl(robot)

        zero_vel = {
            "base_forward": 0.0,
            "base_counterclockwise": 0.0,
            "lift_up": 0.0,
            "arm_out": 0.0,
            "wrist_roll_counterclockwise": 0.0,
            "wrist_pitch_up": 0.0,
            "wrist_yaw_counterclockwise": 0.0,
            "head_pan_counterclockwise": 0.0,
            "head_tilt_up": 0.0,
            "gripper_open": 0.0,
        }

        vel = zero_vel.copy()
        vel["lift_up"] = -0.1
        controller.set_command(vel)
        time.sleep(2.0)
        controller.set_command(zero_vel)
        time.sleep(1.0)
