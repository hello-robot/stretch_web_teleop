import React from 'react';
import { createRoot } from 'react-dom/client';
import 'robot/css/index.css';
import { Robot, inJointLimits, inCollision } from 'robot/tsx/robot'
import { WebRTCConnection } from 'shared/webrtcconnections'
import { navigationProps, realsenseProps, gripperProps, WebRTCMessage, ValidJointStateDict, ROSJointState, ValidJoints, ValidJointStateMessage, RobotPose, rosJointStatetoRobotPose, ROSOccupancyGrid, OccupancyGridMessage, MapPoseMessage, GoalStatus, GoalStatusMessage, RelativePoseMessage, MarkersMessage, MarkerArray, ArucoNavigationStateMessage, ArucoNavigationState, MoveBaseActionResult, MoveBaseActionResultMessage, MoveBaseState, MoveBaseStateMessage } from 'shared/util'
import { AllVideoStreamComponent, VideoStream } from './videostreams';
import ROSLIB from 'roslib';

export const robot = new Robot({ 
    jointStateCallback: forwardJointStates,
    occupancyGridCallback: forwardOccupancyGrid,
    moveBaseResultCallback: forwardMoveBaseState,
    arucoNavigationStateCallback: forwardArucoNavigationState,
    amclPoseCallback: forwardAMCLPose,
    markerArrayCallback: forwardMarkers,
    relativePoseCallback: forwardRelativePose
})

export let connection: WebRTCConnection;
export let navigationStream = new VideoStream(navigationProps);
export let realsenseStream = new VideoStream(realsenseProps)
export let gripperStream = new VideoStream(gripperProps);
// let occupancyGrid: ROSOccupancyGrid | undefined;

robot.connect().then(() => {
    connection = new WebRTCConnection({
        peerRole: 'robot',
        polite: false,
        onRobotConnectionStart: handleSessionStart,
        onMessage: handleMessage,
        onConnectionEnd: disconnectFromRobot
    })

    connection.joinRobotRoom()

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

    robot.getOccupancyGrid()
}).then(() => {
    setTimeout(() => {
        console.log(connection.connectionState())
        let isResolved = connection.connectionState() == 'connected' ? true : false
        console.log("connection state: ", isResolved)
        if (isResolved) {
            console.log('WebRTC connection is resolved.');
        } else {
            window.location.reload()
        }
    }, 7000);    
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

function forwardMoveBaseState(state: MoveBaseState) {
    if (!connection) throw 'WebRTC connection undefined!'
    
    if (state.alert_type != "info") {
        connection.sendData({
            type: "goalStatus",
            message: state
        } as GoalStatusMessage)
    }

    connection.sendData({
        type: "moveBaseState",
        message: state
    } as MoveBaseStateMessage)
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

function forwardOccupancyGrid(occupancyGrid: ROSOccupancyGrid) {
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

function forwardArucoNavigationState(state: ArucoNavigationState) {
    if (!connection) throw 'WebRTC connection undefined'

    connection.sendData({
        type: "arucoNavigationState",
        message: state
    } as ArucoNavigationStateMessage)
}

function forwardAMCLPose(transform: ROSLIB.Transform) {
    if (!connection) throw 'WebRTC connection undefined'

    connection.sendData({
        type: 'amclPose',
        message: transform
    } as MapPoseMessage)
}

function forwardRelativePose(pose: ROSLIB.Transform) {
    if (!connection) throw 'WebRTC connection undefined'
    
    connection.sendData({
        type: "relativePose",
        message: pose
    } as RelativePoseMessage)
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
        case "stopTrajectory":
            robot.stopTrajectoryClient()
            break
        case "stopMoveBase":
            robot.stopMoveBaseClient()
            break
        case "stopArucoNavigation":
            robot.stopNavigateToArucoClient()
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
        case "navigateToAruco":
            robot.executeNavigateToArucoGoal(message.name, message.pose.transform)
            break
        case "setArucoMarkers":
            robot.setArucoMarkers(message.toggle)
            break
        case "lookAtGripper":
            robot.lookAtGripper(0, 0)
            break
        case "getOccupancyGrid":
            robot.getOccupancyGrid()
            break
        case "updateArucoMarkersInfo":
            robot.updateArucoMarkersInfo()
            break;
        case "setArucoMarkerInfo":
            robot.setArucoMarkerInfo(message.info)
            break;    
        case "getRelativePose":
            robot.getRelativePose(message.marker_name)
            break;
    }
};

function disconnectFromRobot() {
    robot.closeROSConnection()
    connection.hangup()
}

window.onbeforeunload = () => {
    robot.closeROSConnection()
    connection.hangup()
};

// New method of rendering in react 18
const container = document.getElementById('root');
const root = createRoot(container!); // createRoot(container!) if you use TypeScript
root.render(<AllVideoStreamComponent streams={[navigationStream, realsenseStream, gripperStream]} />);