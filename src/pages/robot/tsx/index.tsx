import React from "react";
import { createRoot } from "react-dom/client";
import "robot/css/index.css";
import { Robot } from "../../robot/tsx/robot";
import { WebRTCConnection } from "../../../shared/webrtcconnections";
import {
  navigationProps,
  realsenseProps,
  gripperProps,
  expandedGripperProps,
  WebRTCMessage,
  ValidJointStateDict,
  ValidJointStateMessage,
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
import ROSLIB from "roslib";
import { HasBetaTeleopKitMessage } from "../../../shared/util";

export const robot = new Robot({
  jointStateCallback: forwardJointStates,
  batteryStateCallback: forwardBatteryState,
  occupancyGridCallback: forwardOccupancyGrid,
  moveBaseResultCallback: (goalState: ActionState) =>
    forwardActionState(goalState, "moveBaseState"),
  moveToPregraspResultCallback: (goalState: ActionState) =>
    forwardActionState(goalState, "moveToPregraspState"),
  amclPoseCallback: forwardAMCLPose,
  isRunStoppedCallback: forwardIsRunStopped,
  hasBetaTeleopKitCallback: forwardHasBetaTeleopKit,
});

export let connection: WebRTCConnection;
export let navigationStream = new VideoStream(navigationProps);
export let realsenseStream = new VideoStream(realsenseProps);
export let gripperStream = new VideoStream(gripperProps);
export let expandedGripperStream = new VideoStream(expandedGripperProps);
// let occupancyGrid: ROSOccupancyGrid | undefined;

connection = new WebRTCConnection({
  peerRole: "robot",
  polite: false,
  onRobotConnectionStart: handleSessionStart,
  onMessage: handleMessage,
  onConnectionEnd: disconnectFromRobot,
});

robot.connect().then(() => {
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

  robot.subscribeToVideo({
    topicName: "/gripper_camera/image_raw/expanded/compressed",
    callback: expandedGripperStream.updateImage,
  });
  expandedGripperStream.start();

  robot.getOccupancyGrid();
  robot.getJointLimits();

  connection.joinRobotRoom();
});

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

  stream = expandedGripperStream.outputVideoStream!;
  stream
    .getTracks()
    .forEach((track) => connection.addTrack(track, stream, "expandedGripper"));

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
    case "setDepthSensing":
      robot.setDepthSensing(message.toggle);
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
      console.log("moveToPregrasp", message.scaled_x, message.scaled_y);
      robot.executeMoveToPregraspGoal(message.scaled_x, message.scaled_y);
      break;
    case "stopMoveToPregrasp":
      robot.stopMoveToPregraspClient();
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
    streams={[
      navigationStream,
      realsenseStream,
      gripperStream,
      expandedGripperStream,
    ]}
  />,
);
