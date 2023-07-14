import React from 'react';
import { createRoot } from 'react-dom/client';
import 'robot/css/index.css';
import { Robot, inJointLimits, inCollision } from 'robot/tsx/robot'
import { WebRTCConnection } from 'shared/webrtcconnections'
import { navigationProps, realsenseProps, gripperProps, WebRTCMessage, ValidJointStateDict, ROSJointState, ValidJoints, ValidJointStateMessage, RobotPose, rosJointStatetoRobotPose, ROSOccupancyGrid, OccupancyGridMessage, MapPoseMessage, GoalStatus, GoalStatusMessage, Marker, MarkersMessage, MarkerArray } from 'shared/util'
import { AllVideoStreamComponent, VideoStream } from './videostreams';
import ROSLIB from 'roslib';

export const robot = new Robot({ 
    jointStateCallback: forwardJointStates,
    occupancyGridCallback: setOccupancyGrid,
    moveBaseResultCallback: forwardMoveBaseResult,
    amclPoseCallback: forwardAMCLPose,
    markerArrayCallback: forwardMarkers
})

export let connection: WebRTCConnection;
export let navigationStream = new VideoStream(navigationProps);
export let realsenseStream = new VideoStream(realsenseProps)
export let gripperStream = new VideoStream(gripperProps);
let occupancyGrid: ROSOccupancyGrid;

robot.connect().then(() => {
    robot.subscribeToVideo({
        topicName: "/navigation_camera/image_raw/rotated/compressed",
        callback: navigationStream.updateImage
    })
    navigationStream.start()

    robot.subscribeToVideo({
        topicName: "/camera/color/image_raw/rotated/compressed",
        callback: realsenseStream.updateImage
    })
    realsenseStream.start()

    robot.subscribeToVideo({
        topicName: "/gripper_camera/image_raw/cropped/compressed",
        callback: gripperStream.updateImage
    })
    gripperStream.start()

    connection = new WebRTCConnection({
        peerRole: 'robot',
        polite: false,
        onRobotConnectionStart: handleSessionStart,
        onMessage: handleMessage
    })

    connection.joinRobotRoom()
}).then(() => {
    setTimeout(() => {
        let isResolved = connection.connectionState() == 'connected' ? true : false
        console.log("connection state: ", isResolved)
        if (isResolved) {
            console.log('WebRTC connection is resolved.');
        } else {
            window.location.reload()
        }
    }, 6000);    
})

function handleSessionStart() {
    connection.removeTracks()

    console.log('adding local media stream to peer connection');

    let stream: MediaStream = navigationStream.outputVideoStream!;
    stream.getTracks().forEach(track => connection.addTrack(track, stream, "overhead"))

    stream = realsenseStream.outputVideoStream!;
    stream.getTracks().forEach(track => connection.addTrack(track, stream, "realsense"))

    stream = gripperStream.outputVideoStream!;
    stream.getTracks().forEach(track => connection.addTrack(track, stream, "gripper"))

    connection.openDataChannels()
}

function forwardMoveBaseResult(goalStatus: GoalStatus) {
    if (!connection) throw 'WebRTC connection undefined!'
    
    connection.sendData({
        type: "goalStatus",
        message: goalStatus
    } as GoalStatusMessage)
}

function forwardJointStates(jointState: ROSJointState) {
    if (!connection) throw 'WebRTC connection undefined!'

    let robotPose: RobotPose = rosJointStatetoRobotPose(jointState)
    let jointValues: ValidJointStateDict = {}
    let effortValues: ValidJointStateDict = {}
    jointState.name.forEach((name?: ValidJoints) => {
        let inLimits = inJointLimits({ jointStateMessage: jointState, jointName: name! });
        let collision = inCollision({ jointStateMessage: jointState, jointName: name! });
        if (inLimits) jointValues[name!] = inLimits;
        if (collision) effortValues[name!] = collision;
    })

    connection.sendData({
        type: "validJointState",
        robotPose: robotPose,
        jointsInLimits: jointValues,
        jointsInCollision: effortValues
    } as ValidJointStateMessage);
}

function forwardOccupancyGrid() {
    if (!connection) throw 'WebRTC connection undefined'

    let splitOccupancyGrid: ROSOccupancyGrid = {
        header: occupancyGrid.header,
        info: occupancyGrid.info,
        data: []
    }

    const data_size = 50000
    for (let i = 0; i < occupancyGrid.data.length; i += data_size) {
        const data_chunk = occupancyGrid.data.slice(i, i + data_size);
        splitOccupancyGrid.data = data_chunk
        connection.sendData({
            type: 'occupancyGrid',
            message: splitOccupancyGrid
        } as OccupancyGridMessage);
    }

    occupancyGrid.data = occupancyGrid.data.slice(0, 70000)
    console.log('forwarding', occupancyGrid)
    connection.sendData({
        type: 'occupancyGrid',
        message: occupancyGrid
    } as OccupancyGridMessage);
}

function forwardMarkers(markers: MarkerArray) {
    if (!connection) throw 'WebRTC connection undefined'

    connection.sendData({
        type: "arucoMarkers",
        message: markers
    } as MarkersMessage)
}

function forwardAMCLPose(transform: ROSLIB.Transform) {
    if (!connection) throw 'WebRTC connection undefined'

    connection.sendData({
        type: 'amclPose',
        message: transform
    } as MapPoseMessage)
}

function setOccupancyGrid(message: ROSOccupancyGrid) {
    occupancyGrid = message
}

function handleMessage(message: WebRTCMessage) {
    if (!("type" in message)) {
        console.error("Malformed message:", message)
        return
    }

    switch (message.type) {
        case "driveBase":
            robot.executeBaseVelocity(message.modifier)
            break;
        case "incrementalMove":
            robot.executeIncrementalMove(message.jointName, message.increment)
            break
        case "stop":
            robot.stopExecution()
            break
        case "setRobotMode":
            message.modifier == "navigation" ? robot.switchToNavigationMode() : robot.switchToPositionMode()
            break
        case "setCameraPerspective":
            robot.setCameraPerspective({ camera: message.camera, perspective: message.perspective })
            break
        case "setRobotPose":
            robot.executePoseGoal(message.pose)
            break
        case "playbackPoses":
            robot.executePoseGoals(message.poses, 0)
            break
        case "moveBase":
            robot.executeMoveBaseGoal(message.pose)
            break
        case "setFollowGripper":
            robot.setPanTiltFollowGripper(message.toggle)
            break
        case "setDepthSensing":
            robot.setDepthSensing(message.toggle)
            break
        case "setArucoMarkers":
            robot.setArucoMarkers(message.toggle)
            break
        case "lookAtGripper":
            robot.lookAtGripper(0, 0)
            break
        case "getOccupancyGrid":
            forwardOccupancyGrid()
            break
    }
};

// New method of rendering in react 18
const container = document.getElementById('root');
const root = createRoot(container!); // createRoot(container!) if you use TypeScript
root.render(<AllVideoStreamComponent streams={[navigationStream, realsenseStream, gripperStream]} />);