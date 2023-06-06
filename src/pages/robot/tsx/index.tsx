import React from 'react';
import { createRoot } from 'react-dom/client';
import 'robot/css/index.css';
import { Robot, inJointLimits, inCollision } from 'robot/tsx/robot'
import { WebRTCConnection } from 'shared/webrtcconnections'
import { navigationProps, realsenseProps, gripperProps, WebRTCMessage, ValidJointStateDict, ROSJointState, ValidJoints, ValidJointStateMessage } from 'shared/util'
import { AllVideoStreamComponent, VideoStream } from './videostreams';

export const robot = new Robot({ jointStateCallback: forwardJointStates })
export let connection: WebRTCConnection;
export let navigationStream = new VideoStream(navigationProps);
export let realsenseStream = new VideoStream(realsenseProps)
export let gripperStream = new VideoStream(gripperProps);

robot.connect().then(() => {
    robot.subscribeToVideo({
        topicName: "/navigation_camera/image_raw/rotated/compressed",
        callback: navigationStream.updateImage
    })
    navigationStream.start()

    robot.subscribeToVideo({
        topicName: "/camera/color/image_raw/rotated/compressed",
        callback: realsenseStream.updateImage
    })
    realsenseStream.start()

    robot.subscribeToVideo({
        topicName: "/gripper_camera/image_raw/cropped/compressed",
        callback: gripperStream.updateImage
    })
    gripperStream.start()

    connection = new WebRTCConnection({
        peerRole: 'robot',
        polite: false,
        onRobotConnectionStart: handleSessionStart,
        onMessage: handleMessage
    })

    connection.joinRobotRoom()
})

setTimeout(() => {
    let isResolved = connection.connectionState() == 'connected' ? true : false
    console.log("connection state: ", isResolved)
    if (isResolved) {
        console.log('WebRTC connection is resolved.');
    } else {
        window.location.reload()
    }
}, 2000);

function handleSessionStart() {
    connection.removeTracks()

    console.log('adding local media stream to peer connection');

    let stream: MediaStream = navigationStream.outputVideoStream!;
    stream.getTracks().forEach(track => connection.addTrack(track, stream, "overhead"))

    stream = realsenseStream.outputVideoStream!;
    stream.getTracks().forEach(track => connection.addTrack(track, stream, "realsense"))

    stream = gripperStream.outputVideoStream!;
    stream.getTracks().forEach(track => connection.addTrack(track, stream, "gripper"))

    connection.openDataChannels()
}

function forwardJointStates(jointState: ROSJointState) {
    if (!connection) throw 'WebRTC connection undefined!'

    let jointValues: ValidJointStateDict = {}
    let effortValues: ValidJointStateDict = {}
    jointState.name.forEach((name?: ValidJoints) => {
        let inLimits = inJointLimits({ jointStateMessage: jointState, jointName: name! });
        let collision = inCollision({ jointStateMessage: jointState, jointName: name! });
        if (inLimits) jointValues[name!] = inLimits;
        if (collision) effortValues[name!] = collision;
    })

    connection.sendData({
        type: "validJointState",
        jointsInLimits: jointValues,
        jointsInCollision: effortValues
    } as ValidJointStateMessage);
}

function handleMessage(message: WebRTCMessage) {
    if (!("type" in message)) {
        console.error("Malformed message:", message)
        return
    }

    switch (message.type) {
        case "driveBase":
            robot.executeBaseVelocity(message.modifier)
            break;
        case "incrementalMove":
            robot.executeIncrementalMove(message.jointName, message.increment)
            break
        case "stop":
            robot.stopExecution()
            break
        case "setRobotMode":
            message.modifier == "navigation" ? robot.switchToNavigationMode() : robot.switchToPositionMode()
            break
        case "setCameraPerspective":
            robot.setCameraPerspective({ camera: message.camera, perspective: message.perspective })
            break
        case "setRobotPose":
            robot.executePoseGoal(message.pose)
            break
        case "setFollowGripper":
            robot.setPanTiltFollowGripper(message.toggle)
            break
        case "setDepthSensing":
            robot.setDepthSensing(message.toggle)
            break
    }
};

// New method of rendering in react 18
const container = document.getElementById('root');
const root = createRoot(container!); // createRoot(container!) if you use TypeScript
root.render(<AllVideoStreamComponent streams={[navigationStream, realsenseStream, gripperStream]} />);