import React from 'react'
import { createRoot, Root } from 'react-dom/client';
import { WebRTCConnection } from 'shared/webrtcconnections';
import { WebRTCMessage, RemoteStream, RobotPose, ROSOccupancyGrid, delay, waitUntil } from 'shared/util';
import { RemoteRobot } from 'shared/remoterobot';
import { cmd } from 'shared/commands';
import { Operator } from './Operator';
import { DEFAULT_VELOCITY_SCALE } from './static_components/SpeedControl';
import { StorageHandler } from './storage_handler/StorageHandler';
import { FirebaseStorageHandler } from './storage_handler/FirebaseStorageHandler';
import { LocalStorageHandler } from './storage_handler/LocalStorageHandler';
import { FirebaseOptions } from "firebase/app"
import { ButtonFunctionProvider } from './function_providers/ButtonFunctionProvider';
import { FunctionProvider } from './function_providers/FunctionProvider';
import { PredictiveDisplayFunctionProvider } from './function_providers/PredictiveDisplayFunctionProvider';
import { UnderVideoFunctionProvider } from './function_providers/UnderVideoFunctionProvider';
import { MapFunctionProvider } from './function_providers/MapFunctionProvider';
import { UnderMapFunctionProvider } from './function_providers/UnderMapFunctionProvider';
import { MovementRecorderFunctionProvider } from './function_providers/MovementRecorderFunctionProvider';
import { ArucoMarkerFunctionProvider } from './function_providers/ArucoMarkerFunctionProvider';
import { MobileOperator } from './MobileOperator';
import {isMobile} from 'react-device-detect';
import "operator/css/index.css";
import { RunStopFunctionProvider } from './function_providers/RunStopFunctionProvider';
import { BatteryVoltageFunctionProvider } from './function_providers/BatteryVoltageFunctionProvider';

let allRemoteStreams: Map<string, RemoteStream> = new Map<string, RemoteStream>()
let remoteRobot: RemoteRobot;
let connection: WebRTCConnection;
let root: Root;
let isConnected: boolean;
export let occupancyGrid: ROSOccupancyGrid | undefined = undefined;
export let storageHandler: StorageHandler;

// Create the function providers. These abstract the logic between the React 
// components and remote robot.
export var buttonFunctionProvider = new ButtonFunctionProvider();
export var predicitiveDisplayFunctionProvider = new PredictiveDisplayFunctionProvider();
export var underVideoFunctionProvider = new UnderVideoFunctionProvider();
export var runStopFunctionProvider = new RunStopFunctionProvider();
export var batteryVoltageFunctionProvider = new BatteryVoltageFunctionProvider();
export var mapFunctionProvider: MapFunctionProvider;
export var underMapFunctionProvider: UnderMapFunctionProvider;
export var movementRecorderFunctionProvider: MovementRecorderFunctionProvider;
export var arucoMarkerFunctionProvider: ArucoMarkerFunctionProvider;

// Create the WebRTC connection and connect the operator room
connection = new WebRTCConnection({
    peerRole: "operator",
    polite: true,
    onMessage: handleWebRTCMessage,
    onTrackAdded: handleRemoteTrackAdded,
    onMessageChannelOpen: configureRemoteRobot,
    onConnectionEnd: disconnectFromRobot
});

// connection.joinOperatorRoom()

setTimeout(async () => {
    connection.joinOperatorRoom()
    await delay(2000)
    initializeOperator()
}, 1000)

// Create root once when index is loaded
const container = document.getElementById('root');
root = createRoot(container!);

/** Handle when the WebRTC connection adds a new track on a camera video stream. */
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

/**
 * Callback to handle a new WebRTC message from the robot browser.
 * @param message the {@link WebRTCMessage} or an array of messages.
 */
function handleWebRTCMessage(message: WebRTCMessage | WebRTCMessage[]) {
    if (message instanceof Array) {
        for (const subMessage of message) {
            // Recursive call to handle each message in the array
            handleWebRTCMessage(subMessage)
        }
        return
    }

    switch (message.type) {
        case 'validJointState':
            remoteRobot.sensors.checkValidJointState(
                message.robotPose,
                message.jointsInLimits,
                message.jointsInCollision
            );
            break;
        case 'isRunStopped':
            remoteRobot.setIsRunStopped(message.enabled)
            break;
        case 'occupancyGrid':
            if (!occupancyGrid) {
                occupancyGrid = message.message
            } else {
                occupancyGrid.data = occupancyGrid.data.concat(message.message.data)
            }
            break;
        case 'amclPose':
            remoteRobot.setMapPose(
                message.message
            )
            break;
        case 'goalStatus':
            remoteRobot.setGoalReached(true)
            break;
        case 'moveBaseState':
            console.log(message.message)
            underMapFunctionProvider.setMoveBaseState(message.message)
            break;
        case 'arucoMarkers':
            remoteRobot.setMarkers(message.message)
            break;
        case 'arucoNavigationState':
            arucoMarkerFunctionProvider.setArucoNavigationState(message.message)
            break;
        case 'relativePose':
            remoteRobot.setRelativePose(message.message)
            break;
        case 'batteryVoltage':
            remoteRobot.sensors.setBatteryVoltage(message.message)
            break;
        default:
            throw Error(`unhandled WebRTC message type ${message.type}`)
    }
}

/**
 * Sets up remote robot, creates the storage handler, 
 * and renders the operator browser.
 */
function initializeOperator() {
    // configureRemoteRobot();
    const storageHandlerReadyCallback = () => {
        underMapFunctionProvider = new UnderMapFunctionProvider(storageHandler)
        movementRecorderFunctionProvider = new MovementRecorderFunctionProvider(storageHandler)
        arucoMarkerFunctionProvider = new ArucoMarkerFunctionProvider(storageHandler)
        renderOperator(storageHandler);
    }
    storageHandler = createStorageHandler(storageHandlerReadyCallback);
}

/** 
 * Configures the remote robot, which connects with the robot browser over the 
 * WebRTC connection.
 */
function configureRemoteRobot() {
    remoteRobot = new RemoteRobot({
        robotChannel: (message: cmd) => connection.sendData(message),
    });
    occupancyGrid = undefined;
    // remoteRobot.getOccupancyGrid("getOccupancyGrid")
    FunctionProvider.addRemoteRobot(remoteRobot);
    mapFunctionProvider = new MapFunctionProvider();
    remoteRobot.sensors.setFunctionProviderCallback(buttonFunctionProvider.updateJointStates);
    remoteRobot.sensors.setBatteryFunctionProviderCallback(batteryVoltageFunctionProvider.updateVoltage)
}

/**
 * Creates a storage handler based on the `storage` property in the process 
 * environment.
 * @param storageHandlerReadyCallback callback when the storage handler is ready
 * @returns the storage handler
 */
function createStorageHandler(storageHandlerReadyCallback: () => void) {
    switch (process.env.storage) {
        case ('firebase'):
            const config: FirebaseOptions = {
                apiKey: process.env.apiKey,
                authDomain: process.env.authDomain,
                projectId: process.env.projectId,
                storageBucket: process.env.storageBucket,
                messagingSenderId: process.env.messagingSenderId,
                appId: process.env.appId,
                measurementId: process.env.measurementId
            }
            return new FirebaseStorageHandler(
                storageHandlerReadyCallback,
                config
            );
        default:
            return new LocalStorageHandler(storageHandlerReadyCallback);
    }
}

/**
 * Renders the operator browser.
 * 
 * @param storageHandler the storage handler
 */
function renderOperator(storageHandler: StorageHandler) {
    const layout = storageHandler.loadCurrentLayoutOrDefault();
    FunctionProvider.initialize(DEFAULT_VELOCITY_SCALE, layout.actionMode);
    
    !isMobile ?
        root.render(
            <Operator
                remoteStreams={allRemoteStreams}
                layout={layout}
                getRobotPose={(head: boolean, gripper: boolean, arm: boolean) => {
                    return remoteRobot.sensors.getRobotPose(head, gripper, arm)
                }}
                setRobotPose={(pose: RobotPose) => { remoteRobot.setRobotPose(pose) }}
                storageHandler={storageHandler}
            />
        ) 
        :
        root.render(
            <MobileOperator
                remoteStreams={allRemoteStreams}
                storageHandler={storageHandler}
            />
        )

    if (!isMobile) {
        var loader = document.createElement('div');
        loader.className = 'loader'
        var loaderText = document.createElement('div');
        loaderText.className = 'reconnecting-text'
        var text = document.createElement('p')
        text.textContent = 'Reconnecting...'
        loaderText.appendChild(text)
        var loaderBackground = document.createElement('div');
        loaderBackground.className = 'loader-background'

        setInterval(async() => {
            let connected = await connection.isConnected()
            if (!connected && !window.document.body.contains(loader)) {
                window.document.body.appendChild(loaderBackground)
                window.document.body.appendChild(loaderText)
                window.document.body.appendChild(loader)
                isConnected = connected
            } else if (connected && window.document.body.contains(loader)) {
                window.document.body.removeChild(loaderBackground)
                window.document.body.removeChild(loaderText)
                window.document.body.removeChild(loader)
                isConnected = connected
            }
        }, 1000)
    }
}

function disconnectFromRobot() {
    connection.hangup()
}

window.onbeforeunload = () => {
    connection.hangup()
};