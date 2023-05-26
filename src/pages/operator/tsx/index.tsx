import React from 'react'
import { createRoot } from 'react-dom/client';
import { WebRTCConnection } from 'shared/webrtcconnections';
import { WebRTCMessage, RemoteStream } from 'shared/util';
import { RemoteRobot } from 'shared/remoterobot';
import { cmd } from 'shared/commands';
import { Operator } from './operator';
import { FunctionProvider } from 'operator/tsx/functionprovider/functionprovider';
import { ButtonFunctionProvider } from 'operator/tsx/functionprovider/buttonpads'
import { PredictiveDisplayFunctionProvider } from 'operator/tsx/functionprovider/predictivedisplay'
import { VoiceFunctionProvider } from 'operator/tsx/functionprovider/voicecommands'
import { DEFAULT_VELOCITY_SCALE } from './staticcomponents/velocitycontrol';
import "operator/css/index.css"
import { LocalStorageHandler } from './utils/storageHandler';

let allRemoteStreams: Map<string, RemoteStream> = new Map<string, RemoteStream>()
let remoteRobot: RemoteRobot;
export var buttonFunctionProvider = new ButtonFunctionProvider();
export var voiceFunctionProvider = new VoiceFunctionProvider();
export var predicitiveDisplayFunctionProvider = new PredictiveDisplayFunctionProvider();

let connection = new WebRTCConnection({
    peerRole: "operator",
    polite: true,
    onMessage: handleMessage,
    onTrackAdded: handleRemoteTrackAdded,
    onMessageChannelOpen: configureRobot,
    onConnectionEnd: disconnectFromRobot
});

connection.joinOperatorRoom()

// Create root once when index is loaded
const container = document.getElementById('root');
const root = createRoot(container!);

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
        case 'validJointState':
            remoteRobot.sensors.checkValidJointState(
                message.jointsInLimits,
                message.jointsInCollision
            );
            break;
    }
}

function configureRobot() {
    remoteRobot = new RemoteRobot({
        robotChannel: (message: cmd) => connection.sendData(message),
    });
    remoteRobot.setRobotMode("navigation");
    remoteRobot.sensors.setFunctionProviderCallback(buttonFunctionProvider.updateJointStates);

    const storageHandler = new LocalStorageHandler();
    const layout = storageHandler.loadLayout();

    FunctionProvider.initialize(DEFAULT_VELOCITY_SCALE, layout.actionMode);
    FunctionProvider.addRemoteRobot(remoteRobot);

    root.render(
        <Operator
            remoteStreams={allRemoteStreams}
            layout={layout}
            storageHandler={storageHandler}
        />
    );
}

function disconnectFromRobot() {
    connection.hangup()
}

window.onbeforeunload = () => {
    connection.hangup()
};
