import { createRoot } from 'react-dom/client';
import { VideoStreamComponent, VideoStream } from 'operator/tsx/videostreams';
import { WebRTCConnection } from 'shared/webrtcconnections';
import { WebRTCMessage } from 'utils/util';
import { RemoteRobot } from 'robot/tsx/remoterobot';
import { cmd } from 'utils/util';

let allRemoteStreams = new Map()
let remoteRobot: RemoteRobot;

let connection = new WebRTCConnection({
    peerName: "OPERATOR",
    polite: true,
    onMessage: handleMessage,
    onTrackAdded: handleRemoteTrackAdded,   
    onMessageChannelOpen: configureRobot,
});

connection.joinOperatorRoom()

function handleRemoteTrackAdded(event: RTCTrackEvent) {
    console.log('Remote track added.');
    const track = event.track;
    const stream = event.streams[0];
    console.log('got track id=' + track.id, track);
    if (stream) {
        console.log('stream id=' + stream.id, stream);
    }
    console.log('OPERATOR: adding remote tracks');

    let streamName = connection.cameraInfo[stream.id]
    allRemoteStreams.set(streamName, { 'track': track, 'stream': stream });
}

function handleMessage(message: WebRTCMessage | WebRTCMessage[]) {
    console.log("received message")
}

function configureRobot() {
    remoteRobot = new RemoteRobot({
        robotChannel: (message: cmd) => connection.sendData(message)
    });
    console.log("message channel open")
}

// console.log(allRemoteStreams.keys())
// // New method of rendering in react 18
// const container = document.getElementById('root');
// const root = createRoot(container!); // createRoot(container!) if you use TypeScript
// // root.render(<VideoStreamComponent streams={[navigationStream, realsenseStream, gripperStream]}/>);