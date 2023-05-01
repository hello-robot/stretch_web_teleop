import React from 'react';
import ReactDOM from 'react-dom';
import '../css/index.css'

import { Robot } from './robot'
import { WebRTCConnection } from './webrtcconnections'
import { navigationProps, realsenseProps, gripperProps } from '../util/util'
import { VideoStreamComponent, VideoStream } from './videostreams';

export const robot = new Robot({})
export let connection: WebRTCConnection;
export let navigationStream = new VideoStream(navigationProps);
export let realsenseStream = new VideoStream(realsenseProps)
export let gripperStream = new VideoStream(gripperProps);

robot.connect().then(() => {
    robot.subscribeToVideo({
        topicName: "/rotatedNavCamera/compressed",
        callback: navigationStream.updateImage.bind(navigationStream)
    })
    navigationStream.start()

    robot.subscribeToVideo({
        topicName: "/rotatedCamera/compressed",
        callback: realsenseStream.updateImage.bind(realsenseStream)
    })
    realsenseStream.start()

    robot.subscribeToVideo({
        topicName: "/gripper_camera/image_raw/compressed",
        callback: gripperStream.updateImage.bind(gripperStream)
    })
    gripperStream.start()

    connection = new WebRTCConnection({ 
        peerName: 'ROBOT', 
        onConnectionStart: handleSessionStart 
    });
    connection.connectToRobot('ROBOT')
})

function handleSessionStart() {
    let stream: MediaStream = navigationStream.outputVideoStream!;
    stream.getTracks().forEach(track => connection.addTrack(track, stream))

    stream = realsenseStream.outputVideoStream!;
    stream.getTracks().forEach(track => connection.addTrack(track, stream))

    stream = gripperStream.outputVideoStream!;
    stream.getTracks().forEach(track => connection.addTrack(track, stream))
}

ReactDOM.render(
    <VideoStreamComponent streams={[navigationStream, realsenseStream, gripperStream]}/>,
    document.getElementById('root')
);

