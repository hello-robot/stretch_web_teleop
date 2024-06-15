#!/usr/bin/env python3
import subprocess
import time

import numpy as np
import PyKDL as kdl


def jacobian_to_np(J):
    return np.array([[J[i, j] for j in range(J.columns())] for i in range(J.rows())])


if __name__ == "__main__":
    investigation_1 = False
    if investigation_1:
        from urdf import treeFromString
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

    investigation_2 = False
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

    investigation_3 = False
    # Investigation 3: Test Pinocchio IK
    if investigation_3:
        from stretch.motion.pinocchio_ik_solver import PinocchioIKSolver

        urdf_abs_path = "/home/hello-robot/stretchpy/src/stretch/motion/stretch_base_rotation_ik.urdf"
        ee_link = "link_grasp_center"
        # Note the distal telescoping joint is meant to act as all of them.
        all_joints = [
            "joint_mobile_base_rotation",
            "joint_lift",
            "joint_arm_l0",
            "joint_wrist_yaw",
            "joint_wrist_pitch",
            "joint_wrist_roll",
        ]
        controllable_joints = [
            "joint_mobile_base_rotation",
            "joint_lift",
            "joint_arm_l0",
            # "joint_wrist_yaw",
            # "joint_wrist_pitch",
            # "joint_wrist_roll"
        ]
        ik_solver = PinocchioIKSolver(urdf_abs_path, ee_link, controllable_joints)
        # raise Exception(list(ik_solver.model.names), ik_solver.model.nq)

        # Hardcoded joint limits
        joint_limits = {
            "joint_head_tilt": [-2.050932313403102, 0.440252486123179],
            "joint_head_pan": [-4.011359760320952, 1.6597672124922638],
            "joint_lift": [0.0, 1.100079983666348],
            "joint_wrist_pitch": [-1.5707963267948966, 0.45099035163837853],
            "joint_arm_l0": [
                0.0,
                0.5149896872699868,
            ],  # NOTE: The actual joint limits message had this for "joint_arm" instead
            "joint_wrist_roll": [-2.9114955354069467, 2.9176314585584895],
            "joint_wrist_yaw": [-1.227184630308513, 4.5003161364595],
            "gripper_aperture": [-0.12857142857142856, 0.28998923573735197],
            "joint_gripper_finger_left": [-0.3759398496240601, 0.8479217419220817],
            "joint_gripper_finger_right": [-0.3759398496240601, 0.8479217419220817],
        }

        non_wrist_init = {
            "joint_mobile_base_rotation": 0.0,
            "joint_lift": 0.0,  # 0.6957399712989648,
            "joint_arm_l0": 0.0,
        }
        vertical_grasp_wrist = {
            "joint_wrist_yaw": 0.0,  # -0.1195226697227562,
            "joint_wrist_pitch": -np.pi / 2.0,  # -1.560058461279697,
            "joint_wrist_roll": 0.0,  # -0.009203884727313847,
        }
        horizontal_grasp_wrist = {
            "joint_wrist_yaw": 0.0,  # -0.009587379924285258,
            "joint_wrist_pitch": 0.0,  # -0.03681553890925539,
            "joint_wrist_roll": 0.0,  # -0.0015339807878856412,
        }
        goals = [
            # Should not be reachable. This is a pose above a tennis ball a bit too far behind the robot. Vertical grasp.
            # Goal pose in base link frame: geometry_msgs.msg.PoseStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1717621762, nanosec=82198975), frame_id='base_link'), pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=-0.6382754335421884, y=-0.23710846176695716, z=0.12130922975443773), orientation=geometry_msgs.msg.Quaternion(x=0.6959541333313253, y=0.1250913438214004, z=-0.6959541333313254, w=0.12509134382140044)))
            {
                "q_init": non_wrist_init | vertical_grasp_wrist,
                "pos": [-0.6382754335421884, -0.23710846176695716, 0.12130922975443773],
                "quat": [
                    0.6959541333313253,
                    0.1250913438214004,
                    -0.6959541333313254,
                    0.12509134382140044,
                ],
                "reachable": False,
            },
            # Should be reachable. Similar to the above, but the tennis ball is closer. Vertical grasp.
            # Goal pose in base link frame: geometry_msgs.msg.PoseStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1717625858, nanosec=865458496), frame_id='base_link'), pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=-0.45058882040516846, y=-0.21159535596143234, z=0.10690844806130442), orientation=geometry_msgs.msg.Quaternion(x=0.6901383786048245, y=0.1539773307234015, z=-0.6901383786048246, w=0.1539773307234015)))
            {
                "q_init": non_wrist_init | vertical_grasp_wrist,
                "pos": [
                    -0.45058882040516846,
                    -0.21159535596143234,
                    0.10690844806130442,
                ],
                "quat": [
                    0.6901383786048245,
                    0.1539773307234015,
                    -0.6901383786048246,
                    0.1539773307234015,
                ],
                "reachable": True,
            },
            # Should be reachable. An object on the table in front of the arm. Horizontal grasp.
            # Goal pose in base link frame: geometry_msgs.msg.PoseStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1717621522, nanosec=842120361), frame_id='base_link'), pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=-0.053576778904789735, y=-0.5557921114276037, z=0.8378904578910882), orientation=geometry_msgs.msg.Quaternion(x=-0.0, y=-0.0, z=-0.7402541362529396, w=0.6723271627417818)))
            {
                "q_init": non_wrist_init | horizontal_grasp_wrist,
                "pos": [-0.053576778904789735, -0.5557921114276037, 0.8378904578910882],
                "quat": [0.0, 0.0, -0.7402541362529396, 0.6723271627417818],
                "reachable": True,
            },
            # Should not be reachable. This is the plastic box, and is too far from the robot. Horizontal grasp.
            # Goal pose in base link frame: geometry_msgs.msg.PoseStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1717621655, nanosec=803832275), frame_id='base_link'), pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=1.5007274996562687, y=0.38194357599373563, z=0.24187051527438452), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.1242851503754816, w=0.9922465426476144)))
            {
                "q_init": non_wrist_init | horizontal_grasp_wrist,
                "pos": [1.5007274996562687, 0.38194357599373563, 0.24187051527438452],
                "quat": [0.0, 0.0, 0.1242851503754816, 0.9922465426476144],
                "reachable": False,
            },
        ]
        # Takes < 10ms each, while the web app code is running latently in the background.
        for i, goal in enumerate(goals):
            start_time = time.time()
            q_init = np.array([goal["q_init"][joint] for joint in controllable_joints])
            pos = np.array(goal["pos"])
            quat = np.array(goal["quat"])
            reachable = goal["reachable"]
            ik_solver.q_neutral = np.array(
                [goal["q_init"][joint] for joint in all_joints]
            )
            q, success, debug_info = ik_solver.compute_ik(pos, quat, q_init=q_init)
            joint_positions = dict(zip(controllable_joints, q))
            for joint_name, joint_position in joint_positions.items():
                if joint_name in joint_limits:
                    if (
                        not joint_limits[joint_name][0]
                        <= joint_position
                        <= joint_limits[joint_name][1]
                    ):
                        print(f"Joint {joint_name} out of bounds: {joint_position}")
                        success = False
            duration = time.time() - start_time
            print(f"Goal {i}")
            print("    Goal Pose:", pos, quat)
            print("    Expected:", goal["reachable"], "Actual:", success)
            print("    Joint Values:", joint_positions)
            print("    Debug Info:", debug_info)
            print("    Duration:", duration)

    # Test Pinocchio Itself
    investigation_4 = True
    if investigation_4:
        from ament_index_python import get_package_share_directory
        import os
        import pinocchio as pin
        # urdf_abs_path = "/home/hello-robot/stretchpy/src/stretch/motion/stretch_base_rotation_ik.urdf"
        stretch_description_path = get_package_share_directory("stretch_description")
        urdf_abs_path = os.path.join(stretch_description_path, "urdf/stretch.urdf")
        model = pin.buildModelFromUrdf(urdf_abs_path)
        for jid in range(model.njoints):
            print(model.names[jid])
        
        joints_to_fix = [
            "joint_left_wheel",
            "joint_right_wheel",
            "joint_head_pan",
            "joint_head_tilt",
            "joint_arm_l3",
            "joint_arm_l2",
            "joint_arm_l1",
            "joint_gripper_finger_left",
            "joint_gripper_finger_right",
        ]
        jid_to_fix = [
            model.getJointId(joint_name)
            for joint_name in joints_to_fix
        ]
        q = pin.neutral(model)

        model_reduced = pin.buildReducedModel(model, jid_to_fix, q)
        for jid in range(model_reduced.njoints - 1):
            print(f"JID: {jid+1}")
            print(f"    Joint Name: {model_reduced.names[jid+1]}")
            print(f"    Joint Model: {model_reduced.joints[jid+1]}, {type(model_reduced.joints[jid+1])}")
            print(f"    Joint Placement: {model_reduced.jointPlacements[jid]}")
            print(f"    Max Effort: {model_reduced.effortLimit[jid]}")
            print(f"    Max Velocity: {model_reduced.velocityLimit[jid]}")
            print(f"    Min Config: {model_reduced.lowerPositionLimit[jid]}")
            print(f"    Max Config: {model_reduced.upperPositionLimit[jid]}")
            print(f"    Inertia: {model_reduced.inertias[jid]}")
            # print(type(model_reduced.joints[jid]))

        # Insert the mobile base rotation joint
        print(f"Pre-NQ: {model_reduced.nq}, {[name for name in model_reduced.names]}")
        model_reduced.addJoint(
            parent_id=0, # universe
            joint_model=pin.JointModel(pin.JointModelRZ()),
            joint_placement=pin.SE3.Identity(),
            joint_name="joint_mobile_base_rotation",
            max_effort=np.array([10.0]),
            max_velocity=np.array([1.0]),
            min_config=np.array([-np.pi]),
            max_config=np.array([np.pi]),
        )
        lift_jid = model_reduced.getJointId("joint_lift")
        model_reduced.parents[lift_jid] = model_reduced.getJointId("joint_mobile_base_rotation")
        for jid in range(model_reduced.njoints):
            print(model_reduced.names[jid])
        print(f"Post-NQ: {model_reduced.nq}, {[name for name in model_reduced.names]}")

        q = np.array([0.0, 0.5, 0.3, 0.0, 0.0, 0.0])
        J = pin.computeFrameJacobian(
            model_reduced,
            model_reduced.createData(),
            q,
            model_reduced.getJointId("link_grasp_center"),
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        print(J)

        urdf_abs_path = "/home/hello-robot/stretchpy/src/stretch/motion/stretch_base_rotation_ik.urdf"
        model_ground_truth = pin.buildModelFromUrdf(urdf_abs_path)
        model_ground_truth_data = model_ground_truth.createData()
        print(f"{model_ground_truth.nq}, {[name for name in model_ground_truth.names]}")
        J_ground_truth = pin.computeFrameJacobian(
            model_ground_truth,
            model_ground_truth_data,
            q,
            model_ground_truth.getJointId("link_grasp_center"),
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        print(J_ground_truth)
        # for i in range(model_ground_truth.nframes):
        #     print(model_ground_truth.frames[i])
        # raise Exception()

        # # Create a new model
        # model_new = pin.Model()
        # model_new.addJoint(
        #     parent_id=0, # universe
        #     joint_model=pin.JointModel(pin.JointModelRZ()),
        #     joint_placement=pin.SE3.Identity(),
        #     joint_name="joint_mobile_base_rotation",
        #     max_effort=np.array([10.0]),
        #     max_velocity=np.array([1.0]),
        #     min_config=np.array([-np.pi]),
        #     max_config=np.array([np.pi]),
        # )
        # parent_jid = 1
        # for joint_name in [
        #     "joint_lift", "joint_arm_l0", "joint_wrist_yaw", "joint_wrist_pitch", "joint_wrist_roll"
        # ]:
        #     jid = model.getJointId(joint_name)
        #     print(joint_name, "A")
        #     model_new.addJoint(
        #         parent_id=parent_jid, # universe
        #         joint_model=model.joints[jid],
        #         joint_placement=model.jointPlacements[jid].copy(),
        #         joint_name=joint_name,
        #         max_effort=np.array([model.effortLimit[jid]]),
        #         max_velocity=np.array([model.velocityLimit[jid]]),
        #         min_config=np.array([model.lowerPositionLimit[jid]]),
        #         max_config=np.array([model.upperPositionLimit[jid]]),
        #     )
        #     print(joint_name, "B")
        #     model_new.appendBodyToJoint(
        #         parent_jid,
        #         model.inertias[jid],
        #         pin.SE3.Identity(),
        #     )
        #     parent_jid += 1

        # model_new_data = model_new.createData()
        # print(f"{model_new.nq}, {[name for name in model_new.names]}")
        # J_new = pin.computeFrameJacobian(
        #     model_new,
        #     model_new_data,
        #     q,
        #     model_new.getJointId("link_grasp_center"),
        #     pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        # )
        # print(J_new)

