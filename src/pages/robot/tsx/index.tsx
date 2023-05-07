import React from 'react';
import { createRoot } from 'react-dom/client';
import 'robot/css/index.css';
import { Robot } from 'robot/tsx/robot'
import { WebRTCConnection } from 'shared/webrtcconnections'
import { navigationProps, realsenseProps, gripperProps, WebRTCMessage } from 'utils/util'
import { VideoStreamComponent, VideoStream } from 'operator/tsx/videostreams';

export const robot = new Robot({})
export let connection: WebRTCConnection;
export let navigationStream = new VideoStream(navigationProps);
export let realsenseStream = new VideoStream(realsenseProps)
export let gripperStream = new VideoStream(gripperProps);

robot.connect().then(() => {
    robot.subscribeToVideo({
        topicName: "/rotatedNavCamera/compressed",
        callback: navigationStream.updateImage
    })
    navigationStream.start()

    robot.subscribeToVideo({
        topicName: "/rotatedCamera/compressed",
        callback: realsenseStream.updateImage
    })
    realsenseStream.start()

    robot.subscribeToVideo({
        topicName: "/gripper_camera/image_raw/compressed",
        callback: gripperStream.updateImage
    })
    gripperStream.start()

    connection = new WebRTCConnection({
        peerName: 'ROBOT',
        polite: false,
        onRobotConnectionStart: handleSessionStart,
        onMessage: handleMessage
    });
    connection.joinRobotRoom()
})

function handleSessionStart() {
    connection.openDataChannels()

    console.log('adding local media stream to peer connection');

    let stream: MediaStream = navigationStream.outputVideoStream!;
    stream.getTracks().forEach(track => connection.addTrack(track, stream, "navigation"))

    stream = realsenseStream.outputVideoStream!;
    stream.getTracks().forEach(track => connection.addTrack(track, stream, "realsense"))

    stream = gripperStream.outputVideoStream!;
    stream.getTracks().forEach(track => connection.addTrack(track, stream, "gripper"))
}

function handleMessage(message: WebRTCMessage) {
    if (!("type" in message)) {
        console.error("Malformed message:", message)
        return
    }
    console.log(message)
};

// New method of rendering in react 18
const container = document.getElementById('root');
const root = createRoot(container!); // createRoot(container!) if you use TypeScript
root.render(<VideoStreamComponent streams={[navigationStream, realsenseStream, gripperStream]}/>);