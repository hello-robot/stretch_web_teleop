import React from 'react'
import { ROSJointState, ROSCompressedImage, ValidJoints, VideoProps } from 'shared/util';
import ROSLIB, { Message, Ros } from "roslib";
import { JOINT_LIMITS, RobotPose, generateUUID } from 'shared/util';

export var robotMode: "navigation" | "position" = "position"
export var rosConnected = false;

export class Robot extends React.Component {
    private ros!: ROSLIB.Ros
    private jointState?: ROSJointState;
    private poseGoal?: ROSLIB.Goal;
    private trajectoryClient?: ROSLIB.ActionClient;
    private cmdVelTopic?: ROSLIB.Topic;
    private switchToNavigationService?: ROSLIB.Service;
    private switchToPositionService?: ROSLIB.Service;
    private setCameraPerspectiveService?: ROSLIB.Service;
    private setDepthSensingService?: ROSLIB.Service;
    private robotFrameTfClient?: ROSLIB.TFClient;
    private linkGripperFingerLeftTF?: ROSLIB.Transform
    private linkHeadTiltTF?: ROSLIB.Transform
    private jointStateCallback: (jointState: ROSJointState) => void
    private lookAtGripperInterval?: number // ReturnType<typeof setInterval>

    constructor(props: {jointStateCallback: (jointState: ROSJointState) => void}) {
        super(props);
        this.jointStateCallback = props.jointStateCallback
    }

    async connect(): Promise<void> {
        let ros = new ROSLIB.Ros({
            url: 'wss://localhost:9090'
        });

        return new Promise<void>((resolve, reject) => {
            ros.on('connection', async () => {
                await this.onConnect(ros);
                resolve()
            })
            ros.on('error', (error) => {
                reject(error)
            });

            ros.on('close', () => {
                reject('Connection to websocket has been closed.')
            });
        });
    }

    async onConnect(ros: ROSLIB.Ros) {
        this.ros = ros
        this.subscribeToJointState()
        this.createTrajectoryClient()
        this.createCmdVelTopic()
        this.createSwitchToNavigationService()
        this.createSwitchToPositionService()
        this.createSetCameraPerspectiveService()
        this.createDepthSensingService()
        this.createRobotFrameTFClient()
        this.subscribeToGripperFingerTF()
        this.subscribeToHeadTiltTF()

        return Promise.resolve()
    }

    subscribeToJointState() {
        const jointStateTopic: ROSLIB.Topic<ROSJointState> = new ROSLIB.Topic({
            ros: this.ros,
            name: '/stretch/joint_states/',
            messageType: 'sensor_msgs/JointState'
        });
    
        jointStateTopic.subscribe((msg: ROSJointState) => {
            this.jointState = msg
            if (this.jointStateCallback) this.jointStateCallback(msg)
        });
    };
    
    subscribeToVideo(props: VideoProps) {
        let topic: ROSLIB.Topic<ROSCompressedImage> = new ROSLIB.Topic({
            ros: this.ros,
            name: props.topicName,
            messageType: 'sensor_msgs/CompressedImage'
        });
        topic.subscribe(props.callback)
    }
    
    createTrajectoryClient() {
        this.trajectoryClient = new ROSLIB.ActionClient({
            ros: this.ros,
            serverName: '/stretch_controller/follow_joint_trajectory',
            actionName: 'control_msgs/FollowJointTrajectoryAction',
            timeout: 100
        });
    }
    
    createCmdVelTopic() {
        this.cmdVelTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/stretch/cmd_vel',
            messageType: 'geometry_msgs/Twist'
        });
    }
    
    createSwitchToNavigationService() {
        this.switchToNavigationService = new ROSLIB.Service({
            ros: this.ros,
            name: '/switch_to_navigation_mode',
            serviceType: 'std_srvs/Trigger'
        });
    }
    
    createSwitchToPositionService() {
        this.switchToPositionService = new ROSLIB.Service({
            ros: this.ros,
            name: '/switch_to_position_mode',
            serviceType: 'std_srvs/Trigger'
        });
    }
    
    createSetCameraPerspectiveService() {
        this.setCameraPerspectiveService = new ROSLIB.Service({
            ros: this.ros,
            name: '/camera_perspective',
            serviceType: 'stretch_web_interface_react/CameraPerspective'
        })
    }

    createDepthSensingService() {
        this.setDepthSensingService = new ROSLIB.Service({
            ros: this.ros,
            name: '/depth_ar',
            serviceType: 'stretch_web_interface_react/DepthAR'
        })
    }

    createRobotFrameTFClient() {
        this.robotFrameTfClient = new ROSLIB.TFClient({
            ros: this.ros,
            fixedFrame: 'base_link',
            angularThres: 0.01,
            transThres: 0.01,
            rate: 10
        });
    }

    subscribeToGripperFingerTF() {
        this.robotFrameTfClient?.subscribe('link_gripper_finger_left', transform => {
            this.linkGripperFingerLeftTF = transform;
        });
    }

    subscribeToHeadTiltTF () {
        this.robotFrameTfClient?.subscribe('link_head_tilt', transform => {
            this.linkHeadTiltTF = transform;
        })
    }

    setCameraPerspective(props: {camera: "overhead" | "realsense" | "gripper", perspective: string}) {
        var request = new ROSLIB.ServiceRequest({camera: props.camera, perspective: props.perspective})
        this.setCameraPerspectiveService?.callService(request, (response: boolean) => {
            response ? console.log("Set " + props.camera + " to " + props.perspective + " perspective") 
                     : console.error(props.perspective + " is not a valid perspective for " + props.camera + " camera!")
        })
    }

    setDepthSensing(toggle: boolean) {
        var requeset = new ROSLIB.ServiceRequest({enable: toggle})
        this.setDepthSensingService?.callService(requeset, (response: boolean) => {
            response ? console.log("Enable depth sensing") : console.log("Disabled depth sensing")
        })
    }

    switchToNavigationMode() {
        var request = new ROSLIB.ServiceRequest({});
        this.switchToNavigationService!.callService(request, () => {
            robotMode = "navigation"
            console.log("Switched to navigation mode")
        });
    }
    
    switchToPositionMode = () => {
        var request = new ROSLIB.ServiceRequest({});
        this.switchToPositionService!.callService(request, () => {
            robotMode = "position"
            console.log("Switched to position mode")
        });
    }

    executeBaseVelocity = (props: {linVel: number, angVel: number}): void => {
        let twist = new ROSLIB.Message({
            linear: {
                x: props.linVel,
                y: 0,
                z: 0
            },
            angular: {
                x: 0,
                y: 0,
                z: props.angVel
            }
        });
        if (!this.cmdVelTopic) throw 'trajectoryClient is undefined';
        this.cmdVelTopic.publish(twist)
    }

    makeIncrementalMoveGoal(jointName: ValidJoints, jointValueInc: number): ROSLIB.Goal | undefined {
        if (!this.jointState) throw 'jointState is undefined';
        let newJointValue = GetJointValue({ jointStateMessage: this.jointState, jointName: jointName })
        // Paper over Hello's fake joints
        if (jointName === "translate_mobile_base" || jointName === "rotate_mobile_base") {
            // These imaginary joints are floating, always have 0 as their reference
            newJointValue = 0
        } 

        let collision = inCollision({ jointStateMessage: this.jointState, jointName: jointName })
        // Negative joint increment is for lower/retract/rotate out
        // Positive joint increment is for lift/extend/rotate in
        let index = jointValueInc <= 0 ? 0 : 1 
        // If request to move the joint in the direction of collision, cancel movement
        if (collision[index]) return;

        newJointValue = newJointValue + jointValueInc

        // Make sure new joint value is within limits
        if (jointName in JOINT_LIMITS) {
            let inLimits = inJointLimitsHelper({ jointValue: newJointValue, jointName: jointName })
            if (!inLimits) throw 'invalid joint name'
            console.log(newJointValue, JOINT_LIMITS[jointName]![index], inLimits[index])
            if (!inLimits[index]) newJointValue = JOINT_LIMITS[jointName]![index]
        }

        let pose = { [jointName]: newJointValue }
        if (!this.trajectoryClient) throw 'trajectoryClient is undefined';
        return this.makePoseGoal(pose)
    }

    makePoseGoal(pose: RobotPose) {
        let jointNames: ValidJoints[] = []
        let jointPositions: number[] = []
        for (let key in pose) {
            jointNames.push(key as ValidJoints)
            jointPositions.push(pose[key as ValidJoints]!)
        }
    
        if (!this.trajectoryClient) throw 'trajectoryClient is undefined';
        let newGoal = new ROSLIB.Goal({
            actionClient: this.trajectoryClient,
            goalMessage: {
                trajectory: {
                    header: {
                        stamp: {
                            secs: 0,
                            nsecs: 0
                        }
                    },
                    joint_names: jointNames,
                    points: [
                        {
                            positions: jointPositions,
                            // The following might causing the jumpiness in continuous motions
                            time_from_start: {
                                secs: 0,
                                nsecs: 1
                            }
    
                        }
                    ]
                }
            }
        });
    
        newGoal.on('result', function (result) {
            console.log('Final Result: ' + result);
        });

        return newGoal
    }

    executePoseGoal(pose: RobotPose) {
        this.stopExecution();
        this.trajectoryClient?.cancel();
        this.poseGoal = this.makePoseGoal(pose)
        this.poseGoal.send()
    }

    executeIncrementalMove(jointName: ValidJoints, increment: number) {
        this.stopExecution();
        this.poseGoal = this.makeIncrementalMoveGoal(jointName, increment)
        if (!this.poseGoal) {
            console.log("Joint in collision!")
            return;
        }
        this.poseGoal.send()
    }

    stopExecution() {
        if (!this.trajectoryClient) throw 'trajectoryClient is undefined';
        this.trajectoryClient.cancel()
        if (this.poseGoal) {
            this.poseGoal.cancel()
            this.poseGoal = undefined
        }
    }

    setPanTiltFollowGripper(followGripper: boolean) {
        if (this.lookAtGripperInterval && followGripper) return;
        
        if (followGripper) { 
            let panOffset = 0;
            let tiltOffset = 0;
            let lookIfReadyAndRepeat = () => {
                if (this.linkGripperFingerLeftTF && this.linkHeadTiltTF) {
                    this.lookAtGripper(panOffset, tiltOffset);
                }
                this.lookAtGripperInterval = window.setTimeout(lookIfReadyAndRepeat, 500)
            }
            lookIfReadyAndRepeat()

            // this.lookAtGripperInterval = window.setTimeout(() => { 
            //     if (this.linkGripperFingerLeftTF && this.linkHeadTiltTF) {
            //         this.lookAtGripper(panOffset, tiltOffset);
            //     }
            // }, 500)
        } else {
            this.stopExecution()
            clearTimeout(this.lookAtGripperInterval)
            this.lookAtGripperInterval = undefined
        }
    }

    lookAtGripper(panOffset: number, tiltOffset: number) {
        if (!this.linkGripperFingerLeftTF) throw 'linkGripperFingerLeftTF is undefined';
        if (!this.linkHeadTiltTF) throw 'linkHeadTiltTF is undefined';
        let posDifference = {
            x: this.linkGripperFingerLeftTF.translation.x - this.linkHeadTiltTF.translation.x,
            y: this.linkGripperFingerLeftTF.translation.y - this.linkHeadTiltTF.translation.y,
            z: this.linkGripperFingerLeftTF.translation.z - this.linkHeadTiltTF.translation.z
        };

        // Normalize posDifference
        const scalar = Math.sqrt(posDifference.x ** 2 + posDifference.y ** 2 + posDifference.z ** 2);
        posDifference.x /= scalar;
        posDifference.y /= scalar;
        posDifference.z /= scalar;

        const pan = Math.atan2(posDifference.y, posDifference.x) + panOffset;
        const tilt = Math.atan2(posDifference.z, -posDifference.y) + tiltOffset;
        
        // Goals really close to current state cause some whiplash in these joints in simulation. 
        // Ignoring small goals is a temporary fix
        if (!this.jointState) throw 'jointState is undefined';
        let panDiff = Math.abs(GetJointValue({
            jointStateMessage: this.jointState, 
            jointName: "joint_head_pan"
        }) - pan);
        let tiltDiff = Math.abs(GetJointValue({
            jointStateMessage: this.jointState, 
            jointName: "joint_head_tilt"
        }) - tilt);
        if (panDiff < 0.02 && tiltDiff < 0.02) {
            return
        }

        this.executePoseGoal({
            'joint_head_pan': pan + panOffset,
            'joint_head_tilt': tilt + tiltOffset
        })
    }
}

export const GetJointValue = (props: { jointStateMessage: ROSJointState, jointName: ValidJoints }): number => {
    // Paper over Hello's fake joint implementation
    if (props.jointName === "wrist_extension") {
        return GetJointValue({jointStateMessage: props.jointStateMessage, jointName: "joint_arm_l0"}) +
               GetJointValue({jointStateMessage: props.jointStateMessage, jointName: "joint_arm_l1"}) +
               GetJointValue({jointStateMessage: props.jointStateMessage, jointName: "joint_arm_l2"}) +
               GetJointValue({jointStateMessage: props.jointStateMessage, jointName: "joint_arm_l3"});
    } else if (props.jointName === "translate_mobile_base" || props.jointName === "rotate_mobile_base") {
        return 0;
    }

    let jointIndex = props.jointStateMessage.name.indexOf(props.jointName)
    return props.jointStateMessage.position[jointIndex]
}

export function inJointLimits(props: { jointStateMessage: ROSJointState, jointName: ValidJoints }) {
    let jointValue = GetJointValue(props)
    return inJointLimitsHelper({ jointValue: jointValue, jointName: props.jointName})
}

function inJointLimitsHelper(props: { jointValue: number, jointName: ValidJoints }) {
    let jointLimits = JOINT_LIMITS[props.jointName]
    if (!jointLimits) return;

    var eps = 0.03
    let inLimits: [boolean, boolean] = [true, true]
    inLimits[0] = props.jointValue - eps >= jointLimits[0] // Lower joint limit
    inLimits[1] = props.jointValue + eps <= jointLimits[1] // Upper joint limit
    return inLimits
}

export function inCollision(props: { jointStateMessage: ROSJointState, jointName: ValidJoints }) {
    let inCollision: [boolean, boolean] = [false, false]
    const MAX_EFFORTS: { [key in ValidJoints]?: [number, number] } = {
        "joint_head_tilt": [-50, 50],
        "joint_head_pan": [-50, 50],
        "wrist_extension": [-20, 30],
        "joint_lift": [0, 40],
        "joint_wrist_yaw": [-10, 10],
    }

    if (!(props.jointName in MAX_EFFORTS)) return inCollision;

    let jointIndex = props.jointStateMessage.name.indexOf(props.jointName)
    // In collision if joint is applying more than 50% effort when moving downward/inward/backward
    inCollision[0] = props.jointStateMessage.effort[jointIndex] < MAX_EFFORTS[props.jointName]![0]
    // In collision if joint is applying more than 50% effort when moving upward/outward/forward
    inCollision[1] = props.jointStateMessage.effort[jointIndex] > MAX_EFFORTS[props.jointName]![1]

    return inCollision
}