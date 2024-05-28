import React from "react";
import ROSLIB from "roslib";
import {
  cmd,
  DriveCommand,
  CameraPerspectiveCommand,
  IncrementalMove,
  setRobotModeCommand,
  VelocityCommand,
  RobotPoseCommand,
  ToggleCommand,
  LookAtGripper,
  GetOccupancyGrid,
  MoveBaseCommand,
  PlaybackPosesCommand,
} from "shared/commands";
import {
  ValidJointStateDict,
  RobotPose,
  ValidJoints,
  ROSPose,
  waitUntil,
} from "shared/util";
import { GetHasBetaTeleopKit } from "./commands";

export type robotMessageChannel = (message: cmd) => void;

export class RemoteRobot extends React.Component<{}, any> {
  robotChannel: robotMessageChannel;
  sensors: RobotSensors;
  isRunStopped: boolean;
  batteryVoltage: number;
  mapPose: ROSLIB.Transform;
  moveBaseGoalReached: boolean;
  moveBaseState?: string;

  constructor(props: { robotChannel: robotMessageChannel }) {
    super(props);
    this.robotChannel = props.robotChannel;
    this.sensors = new RobotSensors({});
    this.isRunStopped = false;
    this.batteryVoltage = 13.0;
    this.mapPose = {
      translation: {
        x: 0,
        y: 0,
        z: 0,
      } as ROSLIB.Vector3,
      rotation: {
        x: 0,
        y: 0,
        z: 0,
        w: 0,
      } as ROSLIB.Quaternion,
    } as ROSLIB.Transform;
    this.moveBaseGoalReached = false;
  }

  setGoalReached(reached: boolean) {
    this.moveBaseGoalReached = reached;
  }

  isGoalReached() {
    return this.moveBaseGoalReached;
  }

  driveBase(linVel: number, angVel: number): VelocityCommand {
    let cmd: DriveCommand = {
      type: "driveBase",
      modifier: { linVel: linVel, angVel: angVel },
    };
    this.robotChannel(cmd);

    return {
      stop: () => {
        let stopEvent: DriveCommand = {
          type: "driveBase",
          modifier: { linVel: 0, angVel: 0 },
        };
        this.robotChannel(stopEvent);
      },
      affirm: () => {
        let affirmEvent: DriveCommand = {
          type: "driveBase",
          modifier: { linVel: linVel, angVel: angVel },
        };
        this.robotChannel(affirmEvent);
      },
    };
  }

  incrementalMove(jointName: ValidJoints, increment: number): VelocityCommand {
    let cmd: IncrementalMove = {
      type: "incrementalMove",
      jointName: jointName,
      increment: increment,
    };
    this.robotChannel(cmd);

    return {
      stop: () => {
        this.robotChannel({ type: "stopTrajectory" });
      },
    };
  }

  setRobotMode(mode: "position" | "navigation") {
    let cmd: setRobotModeCommand = {
      type: "setRobotMode",
      modifier: mode,
    };
    this.robotChannel(cmd);
  }

  setCameraPerspective(
    camera: "overhead" | "realsense" | "gripper",
    perspective: string,
  ) {
    let cmd: CameraPerspectiveCommand = {
      type: "setCameraPerspective",
      camera: camera,
      perspective: perspective,
    };
    this.robotChannel(cmd);
  }

  setRobotPose(pose: RobotPose) {
    let cmd: RobotPoseCommand = {
      type: "setRobotPose",
      pose: pose,
    };
    this.robotChannel(cmd);
  }

  playbackPoses(poses: RobotPose[]) {
    let cmd: PlaybackPosesCommand = {
      type: "playbackPoses",
      poses: poses,
    };
    this.robotChannel(cmd);
  }

  moveBase(pose: ROSPose) {
    let cmd: MoveBaseCommand = {
      type: "moveBase",
      pose: pose,
    };
    this.robotChannel(cmd);
  }

  moveToPregrasp(x: number, y: number) {
    let cmd: MoveBaseCommand = {
      type: "moveToPregrasp",
      x: x,
      y: y,
    };
    this.robotChannel(cmd);
  }

  setToggle(
    type: "setFollowGripper" | "setDepthSensing" | "setRunStop",
    toggle: boolean,
  ) {
    let cmd: ToggleCommand = {
      type: type,
      toggle: toggle,
    };
    this.robotChannel(cmd);
  }

  lookAtGripper(type: "lookAtGripper") {
    let cmd: LookAtGripper = {
      type: type,
    };
    this.robotChannel(cmd);
  }

  getOccupancyGrid(type: "getOccupancyGrid") {
    let cmd: GetOccupancyGrid = {
      type: type,
    };
    this.robotChannel(cmd);
  }

  getHasBetaTeleopKit(type: "getHasBetaTeleopKit") {
    let cmd: GetHasBetaTeleopKit = {
      type: type,
    };
    this.robotChannel(cmd);
  }

  setMapPose(pose: ROSLIB.Transform) {
    this.mapPose = pose;
  }

  getMapPose() {
    return this.mapPose;
  }

  stopTrajectory() {
    this.robotChannel({ type: "stopTrajectory" });
  }

  stopMoveBase() {
    this.robotChannel({ type: "stopMoveBase" });
  }

  stopMoveToPregrasp() {
    this.robotChannel({ type: "stopMoveToPregrasp" });
  }
}

class RobotSensors extends React.Component {
  private batteryVoltage: number = 0.0;
  private robotPose: RobotPose = {};
  private inJointLimits: ValidJointStateDict = {};
  private inCollision: ValidJointStateDict = {};
  private runStopEnabled: boolean = false;
  private functionProviderCallback?: (
    inJointLimits: ValidJointStateDict,
    inCollision: ValidJointStateDict,
  ) => void;
  private batteryFunctionProviderCallback?: (voltage: number) => void;
  private runStopFunctionProviderCallback?: (enabled: boolean) => void;

  constructor(props: {}) {
    super(props);
    this.functionProviderCallback = () => {};
    this.batteryFunctionProviderCallback = () => {};
    this.runStopFunctionProviderCallback = () => {};
    this.setFunctionProviderCallback =
      this.setFunctionProviderCallback.bind(this);
    this.setBatteryFunctionProviderCallback =
      this.setBatteryFunctionProviderCallback.bind(this);
    this.setRunStopFunctionProviderCallback =
      this.setRunStopFunctionProviderCallback.bind(this);
  }

  /**
   * Handler for joint state messages with information about if individual
   * joints are in collision or at their limit.
   *
   * @param jointValues mapping of joint name to a pair of booleans for
   *                    [joint is within lower limit, joint is within upper limit]
   * @param effortValues mapping for joint name to pair of booleans for
   *                     [joint in collision at lower end, joint is in
   *                     collision at upper end]
   */
  checkValidJointState(
    robotPose: RobotPose,
    jointValues: ValidJointStateDict,
    effortValues: ValidJointStateDict,
  ) {
    if (robotPose !== this.robotPose) {
      this.robotPose = robotPose;
    }

    // Remove existing values from list
    let change = false;
    Object.keys(jointValues).forEach((k) => {
      const key = k as ValidJoints;

      const same =
        key in this.inJointLimits
          ? jointValues[key]![0] == this.inJointLimits[key]![0] &&
            jointValues[key]![1] == this.inJointLimits[key]![1]
          : false;
      // If same value, remove from dict so not passed to callback
      if (same) delete jointValues[key];
      else {
        change = true;
        this.inJointLimits[key] = jointValues[key];
      }
    });
    Object.keys(effortValues).forEach((k) => {
      const key = k as ValidJoints;
      const same =
        key in this.inCollision
          ? effortValues[key]![0] == this.inCollision[key]![0] &&
            effortValues[key]![1] == this.inCollision[key]![1]
          : false;
      // If same value, remove from dict so not passed to callback
      if (same) delete effortValues[key];
      else {
        change = true;
        this.inCollision[key] = effortValues[key];
      }
    });

    // Only callback when value has changed
    if (change && this.functionProviderCallback) {
      this.functionProviderCallback(jointValues, effortValues);
    }
  }

  /**
   * Records a callback from the function provider. The callback is called
   * whenever a joint state "at limit" or "in collision" changes.
   *
   * @param callback callback to function provider
   */
  setFunctionProviderCallback(
    callback: (
      inJointLimits: ValidJointStateDict,
      inCollision: ValidJointStateDict,
    ) => void,
  ) {
    this.functionProviderCallback = callback;
  }

  setBatteryVoltage(voltage: number) {
    let change = Math.abs(this.batteryVoltage - voltage) > 0.01;
    if (change && this.batteryFunctionProviderCallback) {
      this.batteryVoltage = voltage;
      this.batteryFunctionProviderCallback(this.batteryVoltage);
    }
  }

  setRunStopState(enabled: boolean) {
    this.runStopEnabled = enabled;
    if (this.runStopFunctionProviderCallback)
      this.runStopFunctionProviderCallback(this.runStopEnabled);
  }

  /**
   * Records a callback from the function provider. The callback is called
   * whenever the battery voltage changes.
   *
   * @param callback callback to function provider
   */
  setBatteryFunctionProviderCallback(callback: (voltage: number) => void) {
    this.batteryFunctionProviderCallback = callback;
  }

  /**
   * Records a callback from the function provider. The callback is called
   * whenever the battery voltage changes.
   *
   * @param callback callback to function provider
   */
  setRunStopFunctionProviderCallback(callback: (enabled: boolean) => void) {
    this.runStopFunctionProviderCallback = callback;
  }

  /**
   * @returns current robot pose
   */
  getRobotPose(head: boolean, gripper: boolean, arm: boolean): RobotPose {
    let filteredPose: RobotPose = {};
    if (head) {
      filteredPose["joint_head_tilt"] = this.robotPose["joint_head_tilt"];
      filteredPose["joint_head_pan"] = this.robotPose["joint_head_pan"];
    }
    if (gripper) {
      filteredPose["joint_wrist_roll"] = this.robotPose["joint_wrist_roll"];
      filteredPose["joint_wrist_pitch"] = this.robotPose["joint_wrist_pitch"];
      filteredPose["joint_wrist_yaw"] = this.robotPose["joint_wrist_yaw"];
      filteredPose["joint_gripper_finger_left"] =
        this.robotPose["joint_gripper_finger_left"];
    }
    if (arm) {
      filteredPose["joint_lift"] = this.robotPose["joint_lift"];
      filteredPose["wrist_extension"] = this.robotPose["wrist_extension"];
    }
    return filteredPose;
  }
}
