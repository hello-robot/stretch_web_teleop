import React from 'react';
import { createRoot } from 'react-dom/client';
import 'robot/css/index.css';
import { Robot, GetJointValue } from 'robot/tsx/robot'
import { WebRTCConnection } from 'shared/webrtcconnections'
import { navigationProps, realsenseProps, gripperProps, WebRTCMessage, RobotPose, ROSJointState } from 'shared/util'
import { AllVideoStreamComponent, VideoStream } from 'operator/tsx/layoutcomponents/videostreams';

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
        peerName: 'ROBOT',
        polite: false,
        onRobotConnectionStart: handleSessionStart,
        onMessage: handleMessage
    })

    connection.joinRobotRoom()
    
    // connection.registerRequestResponder("jointState", async () => {
    //     let processedJointPositions: {[key in ValidJoints]?: number} = {};
    //     AllJoints.forEach((key, _) => {
    //         if (robot.jointState) {
    //             processedJointPositions[key] = GetJointValue({jointStateMessage: robot.jointState, jointName: key})
    //         }
    //     });
    //     return processedJointPositions
    // });
})

function handleSessionStart() {
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
    let values: RobotPose = {}
    jointState.name.forEach(name => {
        values[name!] = GetJointValue({ jointStateMessage: jointState, jointName: name! });
    })

    connection.sendData({
        type: 'jointState',
        jointState: values
    });
}

function handleMessage(message: WebRTCMessage) {
    if (!("type" in message)) {
        console.error("Malformed message:", message)
        return
    }

    console.log(message)
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
    }
};

// New method of rendering in react 18
const container = document.getElementById('root');
const root = createRoot(container!); // createRoot(container!) if you use TypeScript
root.render(<AllVideoStreamComponent streams={[navigationStream, realsenseStream, gripperStream]}/>);