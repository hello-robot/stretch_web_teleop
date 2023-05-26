import React from 'react'
import { ROSJointState, ROSCompressedImage, ValidJoints, VideoProps } from 'shared/util';
import ROSLIB, { Message, Ros } from "roslib";
import { JOINT_LIMITS, Pose2D, GoalMessage } from 'shared/util';

export var robotMode: "navigation" | "position" = "position"
export var rosConnected = false;

export class Robot extends React.Component {
    ros!: ROSLIB.Ros
    jointState?: ROSJointState;
    jointStateCallback: (jointState: ROSJointState) => void
    poseGoal?: ROSLIB.Goal;
    trajectoryClient?: ROSLIB.ActionClient;
    cmdVelTopic?: ROSLIB.Topic;
    switchToNavigationService?: ROSLIB.Service;
    switchToPositionService?: ROSLIB.Service;
    
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

    makePoseGoal(pose: {[jointName: string]: number}) {
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

    executeIncrementalMove(jointName: ValidJoints, increment: number) {
        this.stopExecution();
        // this.moveBaseClient?.cancel();
        this.trajectoryClient?.cancel();
        this.poseGoal = this.makeIncrementalMoveGoal(jointName, increment)
        if (!this.poseGoal) {
            console.log("Joint in collision!")
            return;
        }
        this.poseGoal.send()
        // this.affirmExecution()
    }

    stopExecution() {
        if (!this.trajectoryClient) throw 'trajectoryClient is undefined';
        this.trajectoryClient.cancel()
        // this.currentJointTrajectoryGoal = null

        // if (this.currentTrajectoryKillInterval) {
        //     clearTimeout(this.currentTrajectoryKillInterval)
        //     // this.currentTrajectoryKillInterval = null
        // }
        // this.moveBaseClient?.cancel()
        if (this.poseGoal) {
            this.poseGoal.cancel()
            this.poseGoal = undefined
        }
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