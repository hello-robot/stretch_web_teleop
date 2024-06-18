import ROSLIB, { Message } from "roslib";
import { cmd } from "./commands";

export type ValidJoints =
  | "joint_head_tilt"
  | "joint_head_pan"
  | "joint_gripper_finger_left"
  | "joint_arm"
  | "wrist_extension"
  | "joint_lift"
  | "joint_wrist_roll"
  | "joint_wrist_pitch"
  | "joint_wrist_yaw"
  | "translate_mobile_base"
  | "rotate_mobile_base"
  | "gripper_aperture"
  | "joint_arm_l0"
  | "joint_arm_l1"
  | "joint_arm_l2"
  | "joint_arm_l3";

export type VelocityGoalArray = [
  { [key in ValidJoints]?: number },
  { [key in ValidJoints]?: number },
];

export type RemoteStream = {
  stream: MediaStream;
  track: MediaStreamTrack;
};

export const AllJoints: ValidJoints[] = [
  "joint_head_tilt",
  "joint_head_pan",
  "joint_gripper_finger_left",
  "joint_arm",
  "wrist_extension",
  "joint_lift",
  "joint_wrist_roll",
  "joint_wrist_pitch",
  "joint_wrist_yaw",
  "translate_mobile_base",
  "rotate_mobile_base",
];

export type ValidJointStateDict = { [key in ValidJoints]?: [boolean, boolean] };

export interface ROSJointState extends Message {
  name: [ValidJoints?];
  position: [number];
  effort: [number];
  velocity: [number];
}

export interface ROSBatteryState extends Message {
  voltage: number;
}

export interface ROSCompressedImage extends Message {
  header: string;
  format: "jpeg" | "png";
  data: string;
}

export interface CameraInfo {
  [key: string]: string;
}

export interface SignallingMessage {
  candidate?: RTCIceCandidate;
  sessionDescription?: RTCSessionDescription;
  cameraInfo?: CameraInfo;
}

export interface Transform {
  transform: ROSLIB.Transform;
}

export type WebRTCMessage =
  | ValidJointStateMessage
  | OccupancyGridMessage
  | MapPoseMessage
  | StopTrajectoryMessage
  | StopMoveBaseMessage
  | FollowJointTrajectoryActionResultMessage
  | MoveBaseActionResultMessage
  | BatteryVoltageMessage
  | MoveBaseStateMessage
  | IsRunStoppedMessage
  | HasBetaTeleopKitMessage
  | StretchToolMessage
  | cmd;

interface StopTrajectoryMessage {
  type: "stopTrajectory";
}

interface StopMoveBaseMessage {
  type: "stopMoveBase";
}

export type RobotPose = { [key in ValidJoints]?: number };

export interface ValidJointStateMessage {
  type: "validJointState";
  robotPose: RobotPose;
  jointsInLimits: { [key in ValidJoints]?: [boolean, boolean] };
  jointsInCollision: { [key in ValidJoints]?: [boolean, boolean] };
}

export interface IsRunStoppedMessage {
  type: "isRunStopped";
  enabled: boolean;
}

export interface HasBetaTeleopKitMessage {
  type: "hasBetaTeleopKit";
  value: boolean;
}

export interface StretchToolMessage {
  type: "stretchTool";
  value: string;
}

export interface FollowJointTrajectoryActionResultMessage {
  type: "goalStatus";
  message: FollowJointTrajectoryActionResult;
}

export interface FollowJointTrajectoryActionResult {
  header: string;
  status: GoalStatus;
  result: string;
}

export interface NavigateToPoseActionStatusList {
  status_list: NavigateToPoseActionStatus[];
}

export interface NavigateToPoseActionStatus {
  status: number;
}

export interface GoalStatus {
  goal_id: string;
  status: number;
  text: string;
}

export interface MoveBaseState {
  state: string;
  alert_type: string;
}

export interface MoveBaseStateMessage {
  type: "moveBaseState";
  message: MoveBaseState;
}

export interface MoveBaseActionResultMessage {
  type: "moveBaseActionResult";
  message: MoveBaseActionResult;
}

export interface MoveBaseActionResult {
  header: string;
  status: GoalStatus;
  result: string;
}

export interface OccupancyGridMessage {
  type: "occupancyGrid";
  message: ROSOccupancyGrid;
}

export interface MapPoseMessage {
  type: "amclPose";
  message: ROSLIB.Transform;
}

export interface BatteryVoltageMessage {
  type: "batteryVoltage";
  message: number;
}

export interface AMCLPose extends Message {
  header: string;
  pose: {
    pose: ROSPose;
    covariance: number[];
  };
}

export interface MarkerArray {
  markers: Marker[];
}

export interface Marker {
  text: string;
  id: number;
}

export interface ROSPoint extends Message {
  x: number;
  y: number;
  z: number;
}

export interface ROSQuaternion extends Message {
  x: number;
  y: number;
  z: number;
  w: number;
}

export interface ROSPose extends Message {
  position: ROSPoint;
  orientation: ROSQuaternion;
}

export interface ROSMapMetaData extends Message {
  map_load_time: number;
  resolution: number;
  width: number;
  height: number;
  origin: ROSPose;
}

export interface ROSOccupancyGrid {
  header: string;
  info: ROSMapMetaData;
  data: number[];
}

export const STOW_WRIST: RobotPose = {
  joint_wrist_roll: 0.0,
  joint_wrist_pitch: -0.497,
  joint_wrist_yaw: 3.19579,
};

export const CENTER_WRIST: RobotPose = {
  joint_wrist_roll: 0.0,
  joint_wrist_pitch: 0.0,
  joint_wrist_yaw: 0.0,
};

export const WRIST_ROLL_ZERO: RobotPose = {
  joint_wrist_roll: 0.0,
};

export const WRIST_ROLL_NINETY: RobotPose = {
  joint_wrist_roll: 1.57079632679,
};

export const REALSENSE_FORWARD_POSE: RobotPose = {
  joint_head_pan: 0.075,
  joint_head_tilt: 0.0,
};

export const REALSENSE_BASE_POSE: RobotPose = {
  joint_head_pan: 0.075,
  joint_head_tilt: -1.1,
};

export const REALSENSE_GRIPPER_POSE: RobotPose = {
  joint_head_pan: -1.7,
  joint_head_tilt: -1.35,
};

export const JOINT_LIMITS: { [key in ValidJoints]?: [number, number] } = {
  wrist_extension: [0.001, 0.518],
  joint_wrist_roll: [-2.95, 2.94],
  joint_wrist_pitch: [-1.57, 0.57],
  joint_wrist_yaw: [-1.37, 4.41],
  joint_lift: [0.001, 1.1],
  translate_mobile_base: [-30.0, 30.0],
  rotate_mobile_base: [-3.14, 3.14],
  joint_gripper_finger_left: [-0.37, 0.17],
  joint_head_tilt: [-1.6, 0.3],
  joint_head_pan: [-3.95, 1.7],
};

export const JOINT_VELOCITIES: { [key in ValidJoints]?: number } = {
  joint_head_tilt: 0.3,
  joint_head_pan: 0.3,
  wrist_extension: 0.04,
  joint_lift: 0.04,
  joint_wrist_roll: 0.1,
  joint_wrist_pitch: 0.1,
  joint_wrist_yaw: 0.4,
  translate_mobile_base: 0.1,
  rotate_mobile_base: 0.3,
};

export const JOINT_INCREMENTS: { [key in ValidJoints]?: number } = {
  joint_head_tilt: 0.1,
  joint_head_pan: 0.1,
  joint_gripper_finger_left: 0.075,
  wrist_extension: 0.075,
  joint_lift: 0.075,
  joint_wrist_roll: 0.2,
  joint_wrist_pitch: 0.2,
  joint_wrist_yaw: 0.2,
  translate_mobile_base: 0.1,
  rotate_mobile_base: 0.2,
};

export const navigationProps = {
  width: 768, // 800,
  height: 768, // 1280,
  scale: 1,
  fps: 6.0,
};

export const realsenseProps = {
  width: 360,
  height: 640,
  scale: 1,
  fps: 6.0,
};

export const gripperProps = {
  width: 768, // 1024
  height: 768,
  scale: 1,
  fps: 6.0,
};

export const expandedGripperProps = {
  width: 768, // 480
  height: 768, // 480
  scale: 1,
  fps: 6.0,
};

export interface VideoProps {
  topicName: string;
  callback: (message: ROSCompressedImage) => void;
}

export function rosJointStatetoRobotPose(jointState: ROSJointState): RobotPose {
  let robotPose: RobotPose = {};
  const names = jointState.name;
  const positions = jointState.position;
  names.map((name, index) => {
    if (name) {
      robotPose[name] = positions[index];
    }
  });
  return robotPose;
}
////////////////////////////////////////////////////////////
// safelyParseJSON code copied from
// https://stackoverflow.com/questions/29797946/handling-bad-json-parse-in-node-safely
// on August 18, 2017
export function safelyParseJSON<T = any>(json: string): T {
  // This function cannot be optimized, it's best to
  // keep it small!
  let parsed;

  try {
    parsed = JSON.parse(json);
  } catch (e) {
    console.warn(e);
  }

  return parsed; // Could be undefined!
}

export type uuid = string;
// From: https://stackoverflow.com/a/2117523/6454085
export function generateUUID(): uuid {
  return "xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx".replace(/[xy]/g, function (c) {
    var r = (Math.random() * 16) | 0,
      v = c == "x" ? r : (r & 0x3) | 0x8;
    return v.toString(16);
  });
}

export async function waitUntil(
  condition,
  timeout = 5000,
  checkInterval = 100,
) {
  let interval;
  let waitPromise = new Promise((resolve) => {
    interval = setInterval(() => {
      if (!condition()) {
        return;
      }
      clearInterval(interval);
      resolve(true);
    }, checkInterval);
  });
  let timeoutPromise = new Promise((resolve) =>
    setTimeout(() => {
      clearInterval(interval);
      resolve(false);
    }, timeout),
  );
  return await Promise.any([waitPromise, timeoutPromise]);
}

export async function waitUntilAsync(
  condition,
  timeout = 5000,
  checkInterval = 100,
) {
  let interval;
  let waitPromise = new Promise((resolve) => {
    interval = setInterval(async () => {
      let value = await condition();
      if (!value) {
        return;
      }
      clearInterval(interval);
      resolve(true);
    }, checkInterval);
  });
  let timeoutPromise = new Promise((resolve) =>
    setTimeout(() => {
      clearInterval(interval);
      resolve(false);
    }, timeout),
  );
  return await Promise.any([waitPromise, timeoutPromise]);
}

export const delay = (ms: number) => new Promise((res) => setTimeout(res, ms));

/**
 * Creates a class name string based on a base class name and additional flags
 * to include.
 *
 * @param baseName base name of the class
 * @param flags additional flags to append to the class name
 * @returns returns a string for the class name
 *
 * @example ```js
 * className("foreground-button", {"active": false, "customizing": true})
 * // returns "foreground-button customizing"
 * ```
 */
export function className(baseName: string, flags: {}): string {
  let className = baseName;
  for (const [k, b] of Object.entries(flags)) {
    if (b) {
      className += " " + k;
    }
  }
  return className;
}
