import { createRoot } from 'react-dom/client';
import { VideoStreamComponent, VideoStream, VideoControlComponent, VideoControl } from 'operator/tsx/videostreams';
import { WebRTCConnection } from 'shared/webrtcconnections';
import { WebRTCMessage } from 'utils/util';
import { RemoteRobot } from 'robot/tsx/remoterobot';
import { cmd } from 'utils/util';
import ReactDOM from 'react-dom';
import { navigationStream } from 'robot/tsx/index'

type RemoteStream = {
    stream: MediaStream;
    track: MediaStreamTrack
}
let allRemoteStreams: Map<string, RemoteStream> = new Map<string, RemoteStream>()
let remoteRobot: RemoteRobot;
let mediaStreams: MediaStream[] = []

let connection = new WebRTCConnection({
    peerName: "OPERATOR",
    polite: true,
    onMessage: handleMessage,
    onTrackAdded: handleRemoteTrackAdded,   
    onMessageChannelOpen: configureRobot,
    onConnectionEnd: disconnectFromRobot
});

connection.joinOperatorRoom()

// const button = document.getElementById('video')?.addEventListener('click', e => connection.joinOperatorRoom());

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
    allRemoteStreams.forEach((values, keys) => {
        mediaStreams.push(values.stream)
    })
    console.log('connection ', connection.connectionState())

    const button = document.getElementById('video')?.addEventListener('click', e => renderVideos());

    console.log("remote streams ", allRemoteStreams)
    // overhead.addRemoteStream(allRemoteStreams.get("overhead").stream)
    // realsense.addRemoteStream(allRemoteStreams.get("realsense").stream)
    // gripper.addRemoteStream(allRemoteStreams.get("gripper").stream)
    // root.render(<VideoControlComponent streams={mediaStreams}/>);
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
  
// // root.render(<VideoStreamComponent streams={[navigationStream, realsenseStream, gripperStream]}/>);