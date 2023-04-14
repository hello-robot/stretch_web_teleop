import React, { useEffect, useState } from 'react'
import { RosConnection, TopicListProvider, useRos, Subscriber, useMsg } from 'rosreact'
import { ROSJointState, ValidJoints } from '../util/util';
import ROSLIB, { Message, Ros } from "roslib";

var trajectoryClient: ROSLIB.ActionClient;
var cmdVelTopic: ROSLIB.Topic

export const Connect = () => {
    const [trigger, setTrigger] = useState(false);
    return (
        <div>
            <RosConnection url={"wss://localhost:9090"} autoConnect>
            <TopicListProvider
                    trigger={trigger} 
                    failedCallback={(e) => {console.log(e)}}
                >
                </TopicListProvider>
                <SubscribeToJointState/>
                <TrajectoryClient/>
                <CmdVelTopic/>
            </RosConnection>
        </div>
    );
}

const SubscribeToJointState = () => {
    const [jointState, setJointState] = useState(new Message({}));
    const ros = useRos();
    useEffect(() => {
        const jointStateTopic: ROSLIB.Topic<ROSJointState> = new ROSLIB.Topic({
            ros: ros,
            name: '/stretch/joint_states/',
            messageType: 'sensor_msgs/JointState'
        });
    
        jointStateTopic.subscribe(msg => {
            setJointState(msg);
        });
    }, []);
    
    return (<div></div>);
};

const TrajectoryClient = () => {
    const ros = useRos();
    trajectoryClient = new ROSLIB.ActionClient({
        ros: useRos(),
        serverName: '/stretch_controller/follow_joint_trajectory',
        actionName: 'control_msgs/FollowJointTrajectoryAction',
        timeout: 100
    });
    return (<></>)
}

const CmdVelTopic = () => {
    const ros = useRos();
    cmdVelTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/stretch/cmd_vel',
        messageType: 'geometry_msgs/Twist'
    });

    return (<></>)
}

export const ExecuteBaseVelocity = (props: {linVel: number, angVel: number}): void => {
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