import React from "react";
import { createRoot } from "react-dom/client";
import "robot/css/index.css";
import { Robot } from "../../robot/tsx/robot";
import { WebRTCConnection } from "../../../shared/webrtcconnections";
import {
    navigationProps,
    realsenseProps,
    gripperProps,
    audioProps,
    WebRTCMessage,
    ValidJointStateDict,
    ValidJointStateMessage,
    ModeMessage,
    IsHomedMessage,
    IsRunStoppedMessage,
    RobotPose,
    ROSOccupancyGrid,
    OccupancyGridMessage,
    MapPoseMessage,
    GoalStatusMessage,
    ActionState,
    ActionStateMessage,
    ROSBatteryState,
    BatteryVoltageMessage,
} from "shared/util";
import { AllVideoStreamComponent, VideoStream } from "./videostreams";
import { AudioStream } from "./audiostreams";
import ROSLIB from "roslib";
import {
    HasBetaTeleopKitMessage,
    StretchToolMessage,
} from "../../../shared/util";
import { loginFirebaseSignalerAsRobot } from "shared/signaling/get_signaler";

export const robot = new Robot({
    jointStateCallback: forwardJointStates,
    batteryStateCallback: forwardBatteryState,
    occupancyGridCallback: forwardOccupancyGrid,
    moveBaseResultCallback: (goalState: ActionState) =>
        forwardActionState(goalState, "moveBaseState"),
    moveToPregraspResultCallback: (goalState: ActionState) =>
        forwardActionState(goalState, "moveToPregraspState"),
    showTabletResultCallback: (goalState: ActionState) =>
        forwardActionState(goalState, "showTabletState"),
    amclPoseCallback: forwardAMCLPose,
    modeCallback: forwardMode,
    isHomedCallback: forwardIsHomed,
    isRunStoppedCallback: forwardIsRunStopped,
    hasBetaTeleopKitCallback: forwardHasBetaTeleopKit,
    stretchToolCallback: forwardStretchTool,
});

export let connection: WebRTCConnection;
export let navigationStream = new VideoStream(navigationProps);
export let realsenseStream = new VideoStream(realsenseProps);
export let gripperStream = new VideoStream(gripperProps);
export let audioStream = new AudioStream(audioProps);
// let occupancyGrid: ROSOccupancyGrid | undefined;

connection = new WebRTCConnection({
    peerRole: "robot",
    polite: false,
    onRobotConnectionStart: handleSessionStart,
    onMessage: handleMessage,
    onConnectionEnd: disconnectFromRobot,
});
robot.setOnRosConnectCallback(async () => {
    robot.subscribeToVideo({
        topicName: "/navigation_camera/image_raw/rotated/compressed",
        callback: navigationStream.updateImage,
    });
    navigationStream.start();

    robot.subscribeToVideo({
        topicName: "/camera/color/image_raw/rotated/compressed",
        callback: realsenseStream.updateImage,
    });
    realsenseStream.start();

    robot.subscribeToVideo({
        topicName: "/gripper_camera/image_raw/cropped/compressed",
        callback: gripperStream.updateImage,
    });
    gripperStream.start();

    audioStream.start();

    robot.getOccupancyGrid();
    robot.getJointLimits();

    console.log("Waiting for configured signaler (i.e. logging in if using Firebase)")
    await loginFirebaseSignalerAsRobot();
    await connection.configure_signaler("");
    console.log("Signaler ready! Joining room.")
    let joinedRobotRoom = await connection.joinRobotRoom();
    while (!joinedRobotRoom) {
        await delay(500);
        joinedRobotRoom = await connection.joinRobotRoom();
    }


    return Promise.resolve();
});
robot.connect();

function handleSessionStart() {
    connection.removeTracks();

    console.log("adding local media stream to peer connection");

    let stream: MediaStream = navigationStream.outputVideoStream!;
    stream
        .getTracks()
        .forEach((track) => connection.addTrack(track, stream, "overhead"));

    stream = realsenseStream.outputVideoStream!;
    stream
        .getTracks()
        .forEach((track) => connection.addTrack(track, stream, "realsense"));

    stream = gripperStream.outputVideoStream!;
    stream
        .getTracks()
        .forEach((track) => connection.addTrack(track, stream, "gripper"));

    stream = audioStream.outputAudioStream!;
    stream
        .getTracks()
        .forEach((track) => connection.addTrack(track, stream, "audio"));

    connection.openDataChannels();
}

function forwardActionState(state: ActionState, type: string) {
    if (!connection) throw "WebRTC connection undefined!";

    if (state.alert_type != "info") {
        connection.sendData({
            type: "goalStatus",
            message: state,
        } as GoalStatusMessage);
    }

    connection.sendData({
        type: type,
        message: state,
    } as ActionStateMessage);
}

function forwardMode(mode: string) {
    if (!connection) throw "WebRTC connection undefined!";

    connection.sendData({
        type: "mode",
        value: mode,
    } as ModeMessage);
}

function forwardIsHomed(isHomed: boolean) {
    if (!connection) throw "WebRTC connection undefined!";

    connection.sendData({
        type: "isHomed",
        value: isHomed,
    } as IsHomedMessage);
}

function forwardIsRunStopped(isRunStopped: boolean) {
    if (!connection) throw "WebRTC connection undefined!";

    connection.sendData({
        type: "isRunStopped",
        enabled: isRunStopped,
    } as IsRunStoppedMessage);
}

function forwardHasBetaTeleopKit(value: boolean) {
    if (!connection) throw "WebRTC connection undefined!";

    connection.sendData({
        type: "hasBetaTeleopKit",
        value: value,
    } as HasBetaTeleopKitMessage);
}

function forwardStretchTool(value: string) {
    if (!connection) throw "WebRTC connection undefined!";

    connection.sendData({
        type: "stretchTool",
        value: value,
    } as StretchToolMessage);
}

function forwardJointStates(
    robotPose: RobotPose,
    jointValues: ValidJointStateDict,
    effortValues: ValidJointStateDict,
) {
    if (!connection) throw "WebRTC connection undefined!";

    connection.sendData({
        type: "validJointState",
        robotPose: robotPose,
        jointsInLimits: jointValues,
        jointsInCollision: effortValues,
    } as ValidJointStateMessage);
}

function forwardBatteryState(batteryState: ROSBatteryState) {
    if (!connection) throw "WebRTC connection undefined";

    connection.sendData({
        type: "batteryVoltage",
        message: batteryState.voltage,
    } as BatteryVoltageMessage);
}

function forwardOccupancyGrid(occupancyGrid: ROSOccupancyGrid) {
    if (!connection) throw "WebRTC connection undefined";

    let splitOccupancyGrid: ROSOccupancyGrid = {
        header: occupancyGrid.header,
        info: occupancyGrid.info,
        data: [],
    };

    const data_size = 50000;
    for (let i = 0; i < occupancyGrid.data.length; i += data_size) {
        const data_chunk = occupancyGrid.data.slice(i, i + data_size);
        splitOccupancyGrid.data = data_chunk;
        connection.sendData({
            type: "occupancyGrid",
            message: splitOccupancyGrid,
        } as OccupancyGridMessage);
    }

    // occupancyGrid.data = occupancyGrid.data.slice(0, 70000)
    // console.log('forwarding', occupancyGrid)
    // connection.sendData({
    //     type: 'occupancyGrid',
    //     message: occupancyGrid
    // } as OccupancyGridMessage);
}

function forwardAMCLPose(transform: ROSLIB.Transform) {
    if (!connection) throw "WebRTC connection undefined";

    connection.sendData({
        type: "amclPose",
        message: transform,
    } as MapPoseMessage);
}

function handleMessage(message: WebRTCMessage) {
    if (!("type" in message)) {
        console.error("Malformed message:", message);
        return;
    }

    switch (message.type) {
        case "driveBase":
            robot.executeBaseVelocity(message.modifier);
            break;
        case "incrementalMove":
            robot.executeIncrementalMove(message.jointName, message.increment);
            break;
        case "stopTrajectory":
            robot.stopTrajectoryClient();
            break;
        case "stopMoveBase":
            robot.stopMoveBaseClient();
            break;
        case "setRobotMode":
            message.modifier == "navigation"
                ? robot.switchToNavigationMode()
                : robot.switchToPositionMode();
            break;
        case "setCameraPerspective":
            robot.setCameraPerspective({
                camera: message.camera,
                perspective: message.perspective,
            });
            break;
        case "setRobotPose":
            robot.executePoseGoal(message.pose);
            break;
        case "playbackPoses":
            robot.executePoseGoals(message.poses, 0);
            break;
        case "moveBase":
            robot.executeMoveBaseGoal(message.pose);
            break;
        case "setFollowGripper":
            robot.setPanTiltFollowGripper(message.toggle);
            break;
        case "setRealsenseDepthSensing":
            robot.setRealsenseDepthSensing(message.toggle);
            break;
        case "setGripperDepthSensing":
            robot.setGripperDepthSensing(message.toggle);
            break;
        case "setExpandedGripper":
            robot.setExpandedGripper(message.toggle);
            break;
        case "setRealsenseBodyPoseEstimate":
            robot.setComputeBodyPose(message.toggle);
            robot.setRealsenseShowBodyPose(message.toggle);
            break;
        case "setRunStop":
            robot.setRunStop(message.toggle);
            break;
        case "lookAtGripper":
            robot.lookAtGripper(0, 0);
            break;
        case "getOccupancyGrid":
            robot.getOccupancyGrid();
            break;
        case "getHasBetaTeleopKit":
            robot.getHasBetaTeleopKit();
        case "moveToPregrasp":
            robot.executeMoveToPregraspGoal(
                message.scaled_x,
                message.scaled_y,
                message.horizontal,
            );
            break;
        case "stopMoveToPregrasp":
            robot.stopMoveToPregraspClient();
            break;
        case "getStretchTool":
            robot.getStretchTool();
            break;
        case "playTextToSpeech":
            robot.playTextToSpeech(
                message.text,
                message.override_behavior,
                message.is_slow,
            );
            break;
        case "stopTextToSpeech":
            robot.stopTextToSpeech();
            break;
        case "showTablet":
            robot.executeShowTabletGoal();
            break;
        case "stopShowTablet":
            robot.stopShowTabletClient();
            break;
        case "homeTheRobot":
            robot.homeTheRobot();
            break;
    }
}

function disconnectFromRobot() {
    robot.closeROSConnection();
    connection.hangup();
}

window.onbeforeunload = () => {
    robot.closeROSConnection();
    connection.hangup();
};

// New method of rendering in react 18
const container = document.getElementById("root");
const root = createRoot(container!); // createRoot(container!) if you use TypeScript
root.render(
    <AllVideoStreamComponent
        streams={[navigationStream, realsenseStream, gripperStream]}
    />,
);
