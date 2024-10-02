import React from "react";
import { createRoot, Root } from "react-dom/client";
import { WebRTCConnection } from "shared/webrtcconnections";
import {
    WebRTCMessage,
    RemoteStream,
    RobotPose,
    ROSOccupancyGrid,
    StretchTool,
    delay,
    getStretchTool,
    waitUntil,
} from "shared/util";
import { RemoteRobot } from "shared/remoterobot";
import { cmd } from "shared/commands";
import { Operator } from "./Operator";
import { DEFAULT_VELOCITY_SCALE } from "./static_components/SpeedControl";
import { StorageHandler } from "./storage_handler/StorageHandler";
import { FirebaseStorageHandler } from "./storage_handler/FirebaseStorageHandler";
import { LocalStorageHandler } from "./storage_handler/LocalStorageHandler";
import { FirebaseOptions } from "firebase/app";
import { ButtonFunctionProvider } from "./function_providers/ButtonFunctionProvider";
import { FunctionProvider } from "./function_providers/FunctionProvider";
import { PredictiveDisplayFunctionProvider } from "./function_providers/PredictiveDisplayFunctionProvider";
import { UnderVideoFunctionProvider } from "./function_providers/UnderVideoFunctionProvider";
import { MapFunctionProvider } from "./function_providers/MapFunctionProvider";
import { UnderMapFunctionProvider } from "./function_providers/UnderMapFunctionProvider";
import { MovementRecorderFunctionProvider } from "./function_providers/MovementRecorderFunctionProvider";
import { TextToSpeechFunctionProvider } from "./function_providers/TextToSpeechFunctionProvider";
import { HomeTheRobotFunctionProvider } from "./function_providers/HomeTheRobotFunctionProvider";
import { MobileOperator } from "./MobileOperator";
import { isBrowser, isMobile, isTablet } from "react-device-detect";
import "operator/css/index.css";
import { RunStopFunctionProvider } from "./function_providers/RunStopFunctionProvider";
import { BatteryVoltageFunctionProvider } from "./function_providers/BatteryVoltageFunctionProvider";
import { waitUntilAsync } from "../../../shared/util";

let allRemoteStreams: Map<string, RemoteStream> = new Map<
    string,
    RemoteStream
>();
let remoteRobot: RemoteRobot;
let connection: WebRTCConnection;
let root: Root;
export let hasBetaTeleopKit: boolean;
export let stretchTool: StretchTool;
export let occupancyGrid: ROSOccupancyGrid | undefined = undefined;
export let storageHandler: StorageHandler;

// Create the function providers. These abstract the logic between the React
// components and remote robot.
export var buttonFunctionProvider = new ButtonFunctionProvider();
export var predicitiveDisplayFunctionProvider =
    new PredictiveDisplayFunctionProvider();
export var underVideoFunctionProvider = new UnderVideoFunctionProvider();
export var runStopFunctionProvider = new RunStopFunctionProvider();
export var batteryVoltageFunctionProvider =
    new BatteryVoltageFunctionProvider();
export var mapFunctionProvider: MapFunctionProvider;
export var underMapFunctionProvider: UnderMapFunctionProvider;
export var movementRecorderFunctionProvider: MovementRecorderFunctionProvider;
export var textToSpeechFunctionProvider: TextToSpeechFunctionProvider;
export var homeTheRobotFunctionProvider: HomeTheRobotFunctionProvider =
    new HomeTheRobotFunctionProvider();

// Create the WebRTC connection and connect the operator room
connection = new WebRTCConnection({
    peerRole: "operator",
    polite: true,
    onMessage: handleWebRTCMessage,
    onTrackAdded: handleRemoteTrackAdded,
    onMessageChannelOpen: configureRemoteRobot,
    onConnectionEnd: disconnectFromRobot,
});

new Promise<void>(async (resolve) => {
    let connected = false;
    while (!connected) {
        connection.hangup();

        // Attempt to join robot room
        let joinedRobotRoom = await connection.addOperatorToRobotRoom();
        if (!joinedRobotRoom) {
            console.log("Operator failed to join robot room");
            await delay(500);
            continue;
        }

        // Wait for WebRTC connection to resolve, timeout after 10 seconds
        let isResolved = await waitUntil(
            () => connection.connectionState() == "connected",
            10000,
        );
        if (!isResolved) {
            console.warn("WebRTC connection could not resolve");
            await delay(500);
            continue;
        }

        // Wait for data to flow through the data channel, timeout after 10 seconds
        connected = await waitUntilAsync(
            async () => await connection.isConnected(),
            10000,
        );
        if (!connected) {
            console.warn("No data flowing through data channel");
            await delay(500);
            continue;
        }

        await delay(1000); // 1 second delay to allow data to flow through data channel
        initializeOperator();
        resolve();
    }
});

// Create root once when index is loaded
const container = document.getElementById("root");
root = createRoot(container!);

/** Handle when the WebRTC connection adds a new track on a camera video stream. */
function handleRemoteTrackAdded(event: RTCTrackEvent) {
    const track = event.track;
    const stream = event.streams[0];
    let streamName = connection.cameraInfo[stream.id];
    console.log("Adding remote track", streamName);
    if (streamName != "audio") {
        console.log(stream.getVideoTracks()[0].getConstraints());
    }
    console.log("got track id=" + track.id, track);
    if (stream) {
        console.log("stream id=" + stream.id, stream);
    }
    console.log("OPERATOR: adding remote tracks");

    allRemoteStreams.set(streamName, { track: track, stream: stream });
}

/**
 * Callback to handle a new WebRTC message from the robot browser.
 * @param message the {@link WebRTCMessage} or an array of messages.
 */
function handleWebRTCMessage(message: WebRTCMessage | WebRTCMessage[]) {
    if (message instanceof Array) {
        for (const subMessage of message) {
            // Recursive call to handle each message in the array
            handleWebRTCMessage(subMessage);
        }
        return;
    }

    switch (message.type) {
        case "validJointState":
            remoteRobot.sensors.checkValidJointState(
                message.robotPose,
                message.jointsInLimits,
                message.jointsInCollision,
            );
            break;
        case "mode":
            remoteRobot.sensors.setMode(message.value);
            break;
        case "isHomed":
            remoteRobot.sensors.setIsHomed(message.value);
            break;
        case "isRunStopped":
            remoteRobot.sensors.setRunStopState(message.enabled);
            break;
        case "hasBetaTeleopKit":
            hasBetaTeleopKit = message.value;
            break;
        case "stretchTool":
            console.log("index stretchTool", message.value);
            stretchTool = getStretchTool(message.value);
            break;
        case "occupancyGrid":
            if (!occupancyGrid) {
                occupancyGrid = message.message;
            } else {
                occupancyGrid.data = occupancyGrid.data.concat(
                    message.message.data,
                );
            }
            break;
        case "amclPose":
            remoteRobot.setMapPose(message.message);
            break;
        case "goalStatus":
            console.log("goalStatus", message.message);
            remoteRobot.setGoalReached(true);
            break;
        case "moveBaseState":
            console.log("moveBaseState", message.message);
            underMapFunctionProvider.setMoveBaseState(message.message);
            break;
        case "moveToPregraspState":
            console.log("moveToPregraspState", message.message);
            underVideoFunctionProvider.setMoveToPregraspState(message.message);
            break;
        case "showTabletState":
            console.log("showTabletState", message.message);
            underVideoFunctionProvider.setShowTabletState(message.message);
            break;
        case "relativePose":
            remoteRobot.setRelativePose(message.message);
            break;
        case "batteryVoltage":
            remoteRobot.sensors.setBatteryVoltage(message.message);
            break;
        default:
            throw Error(`unhandled WebRTC message type ${message.type}`);
    }
}

/**
 * Sets up remote robot, creates the storage handler,
 * and renders the operator browser.
 */
function initializeOperator() {
    // configureRemoteRobot();
    const storageHandlerReadyCallback = () => {
        underMapFunctionProvider = new UnderMapFunctionProvider(storageHandler);
        movementRecorderFunctionProvider = new MovementRecorderFunctionProvider(
            storageHandler,
        );
        textToSpeechFunctionProvider = new TextToSpeechFunctionProvider(
            storageHandler,
        );
        renderOperator(storageHandler);
    };
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
    remoteRobot.getHasBetaTeleopKit("getHasBetaTeleopKit");
    remoteRobot.getStretchTool("getStretchTool");
    FunctionProvider.addRemoteRobot(remoteRobot);
    mapFunctionProvider = new MapFunctionProvider();
    remoteRobot.sensors.setFunctionProviderCallback(
        buttonFunctionProvider.updateJointStates,
    );
    remoteRobot.sensors.setJointStateFunctionProviderCallback(
        underVideoFunctionProvider.jointStateCallback,
    );
    remoteRobot.sensors.setBatteryFunctionProviderCallback(
        batteryVoltageFunctionProvider.updateVoltage,
    );
    remoteRobot.sensors.setModeFunctionProviderCallback(
        homeTheRobotFunctionProvider.updateModeState,
    );
    remoteRobot.sensors.setIsHomedFunctionProviderCallback(
        homeTheRobotFunctionProvider.updateIsHomedState,
    );
    remoteRobot.sensors.setRunStopFunctionProviderCallback(
        runStopFunctionProvider.updateRunStopState,
    );
}

/**
 * Creates a storage handler based on the `storage` property in the process
 * environment.
 * @param storageHandlerReadyCallback callback when the storage handler is ready
 * @returns the storage handler
 */
function createStorageHandler(storageHandlerReadyCallback: () => void) {
    switch (process.env.storage) {
        case "firebase":
            const config: FirebaseOptions = {
                apiKey: process.env.apiKey,
                authDomain: process.env.authDomain,
                projectId: process.env.projectId,
                storageBucket: process.env.storageBucket,
                messagingSenderId: process.env.messagingSenderId,
                appId: process.env.appId,
                measurementId: process.env.measurementId,
            };
            return new FirebaseStorageHandler(
                storageHandlerReadyCallback,
                config,
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

    isBrowser || isTablet
        ? root.render(
              <Operator
                  remoteStreams={allRemoteStreams}
                  layout={layout}
                  storageHandler={storageHandler}
              />,
          )
        : root.render(
              <MobileOperator
                  remoteStreams={allRemoteStreams}
                  storageHandler={storageHandler}
              />,
          );

    if (isBrowser || isTablet) {
        var loader = document.createElement("div");
        loader.className = "loader";
        var loaderText = document.createElement("div");
        loaderText.className = "reconnecting-text";
        var text = document.createElement("p");
        text.textContent = "Reconnecting...";
        loaderText.appendChild(text);
        var loaderBackground = document.createElement("div");
        loaderBackground.className = "loader-background";

        setInterval(async () => {
            let connected = await connection.isConnected();
            if (!connected && !window.document.body.contains(loader)) {
                window.document.body.appendChild(loaderBackground);
                window.document.body.appendChild(loaderText);
                window.document.body.appendChild(loader);
            } else if (connected && window.document.body.contains(loader)) {
                window.document.body.removeChild(loaderBackground);
                window.document.body.removeChild(loaderText);
                window.document.body.removeChild(loader);
            }
        }, 1000);
    }
}

function disconnectFromRobot() {
    connection.hangup();
    connection.stop();
}

window.onbeforeunload = () => {
    connection.hangup();
    connection.stop();
};

window.onunload = () => {
    connection.hangup();
    connection.stop();
};
