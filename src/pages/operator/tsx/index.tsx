import { createRoot } from 'react-dom/client';
import { VideoControl } from 'operator/tsx/layoutcomponents/videostreams';
import { WebRTCConnection } from 'shared/webrtcconnections';
import { WebRTCMessage, RemoteStream } from 'shared/util';
import { RemoteRobot } from 'shared/remoterobot';
import { cmd } from 'shared/commands';
import { Operator } from './operator';
import "operator/css/index.css"

let allRemoteStreams: Map<string, RemoteStream> = new Map<string, RemoteStream>()
let remoteRobot: RemoteRobot;

let connection = new WebRTCConnection({
    peerName: "OPERATOR",
    polite: true,
    onMessage: handleMessage,
    onTrackAdded: handleRemoteTrackAdded,   
    onMessageChannelOpen: configureRobot,
    onConnectionEnd: disconnectFromRobot
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
    if (message instanceof Array) {
        for (const subMessage of message) {
            handleMessage(subMessage)
        }
        return
    }
    switch (message.type) {
        case 'jointState':
            remoteRobot.sensors.setJointState(message.jointState);
            break;
    }
}

function configureRobot() {
    remoteRobot = new RemoteRobot({
        robotChannel: (message: cmd) => connection.sendData(message),
    });

    const container = document.getElementById('root');
    const root = createRoot(container!);
    root.render(<Operator remoteRobot={remoteRobot} remoteStreams={allRemoteStreams}/>);
}

function disconnectFromRobot() {
    connection.hangup()
    // for (const stream in mediaStreams) {
    //     this.controls[control].removeRemoteStream()
    // }
}

function renderVideos() {
    const container = document.getElementById("root");
    const root = createRoot(container!);

    const videoControls = Array.from(allRemoteStreams.values()).map(
      (remoteStream) => (
        <VideoControl key={remoteStream.stream.id} stream={remoteStream.stream} />
    ));

    console.log(videoControls)

    root.render(<div>{videoControls}</div>);
  }
  
