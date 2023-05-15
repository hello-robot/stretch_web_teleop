import React from 'react'
import { ROSJointState, ROSCompressedImage, ValidJoints, VideoProps } from 'utils/util';
import ROSLIB, { Message, Ros } from "roslib";

var trajectoryClient: ROSLIB.ActionClient;
var cmdVelTopic: ROSLIB.Topic;
var switchToNavigationService: ROSLIB.Service;
var switchToPositionService: ROSLIB.Service;

export var robotMode: "navigation" | "position" = "position"
export var rosConnected = false;

export class Robot extends React.Component {
    ros!: ROSLIB.Ros
    
    constructor(props) {
        super(props);
        this.state = {
            jointState: new Message({})
        }
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
        this.trajectoryClient()
        this.cmdVelTopic()
        this.switchToNavigationService()
        this.switchToPositionService()

        return Promise.resolve()
    }

    subscribeToJointState() {
        const jointStateTopic: ROSLIB.Topic<ROSJointState> = new ROSLIB.Topic({
            ros: this.ros,
            name: '/stretch/joint_states/',
            messageType: 'sensor_msgs/JointState'
        });
    
        jointStateTopic.subscribe((msg: Message) => {
            this.state['jointState'] = msg;
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
    
    trajectoryClient() {
        trajectoryClient = new ROSLIB.ActionClient({
            ros: this.ros,
            serverName: '/stretch_controller/follow_joint_trajectory',
            actionName: 'control_msgs/FollowJointTrajectoryAction',
            timeout: 100
        });
    }
    
    cmdVelTopic() {
        cmdVelTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/stretch/cmd_vel',
            messageType: 'geometry_msgs/Twist'
        });
    }
    
    switchToNavigationService() {
        switchToNavigationService = new ROSLIB.Service({
            ros: this.ros,
            name: '/switch_to_navigation_mode',
            serviceType: 'std_srvs/Trigger'
        });
    }
    
    switchToPositionService() {
        switchToPositionService = new ROSLIB.Service({
            ros: this.ros,
            name: '/switch_to_position_mode',
            serviceType: 'std_srvs/Trigger'
        });
    }
    
    switchToNavigationMode() {
        var request = new ROSLIB.ServiceRequest({});
        switchToNavigationService.callService(request, () => {
            robotMode = "navigation"
            console.log("Switched to navigation mode")
        });
    }
    
    switchToPositionMode = () => {
        var request = new ROSLIB.ServiceRequest({});
        switchToPositionService.callService(request, () => {
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
        cmdVelTopic.publish(twist)
    }
}

export const GetJointValue = (props: { jointStateMessage: ROSJointState, jointName: ValidJoints }): number => {
    // Paper over Hello's fake joint implementation
    if (props.jointName === "wrist_extension") {
    console.log("publish")
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