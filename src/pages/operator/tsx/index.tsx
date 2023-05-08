import { createRoot } from 'react-dom/client';
import { WebRTCConnection } from 'shared/webrtcconnections';
import { WebRTCMessage } from 'utils/util';
import { RemoteRobot } from 'robot/tsx/remoterobot';
import { cmd } from 'utils/util';
import { Operator } from './operator';
import "operator/css/index.css"

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

const container = document.getElementById('root');
const root = createRoot(container!);
root.render(<Operator/>);
