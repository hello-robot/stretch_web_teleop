import React from "react";
import ROSLIB from "roslib";
import {
  ROSJointState,
  ROSCompressedImage,
  ValidJoints,
  VideoProps,
  ROSOccupancyGrid,
  ROSPose,
  MoveBaseState,
  NavigateToPoseActionResult,
  NavigateToPoseActionStatusList,
  ROSBatteryState,
} from "shared/util";
import {
  rosJointStatetoRobotPose,
  ValidJointStateDict,
  RobotPose,
  IsRunStoppedMessage,
} from "../../../shared/util";

export var robotMode: "navigation" | "position" = "position";
export var rosConnected = false;

export class Robot extends React.Component {
  private ros: ROSLIB.Ros;
  private jointLimits: { [key in ValidJoints]?: [number, number] } = {};
  private jointState?: ROSJointState;
  private poseGoal?: ROSLIB.ActionGoal;
  private poseGoalComplete?: boolean;
  private isRunStopped?: boolean;
  private moveBaseGoal?: ROSLIB.ActionGoal;
  private trajectoryClient?: ROSLIB.ActionClient;
  private moveBaseClient?: ROSLIB.ActionClient;
  private cmdVelTopic?: ROSLIB.Topic;
  private switchToNavigationService?: ROSLIB.Service;
  private switchToPositionService?: ROSLIB.Service;
  private setCameraPerspectiveService?: ROSLIB.Service;
  private setDepthSensingService?: ROSLIB.Service;
  private setRunStopService?: ROSLIB.Service;
  private robotFrameTfClient?: ROSLIB.TFClient;
  private mapFrameTfClient?: ROSLIB.TFClient;
  private linkGripperFingerLeftTF?: ROSLIB.Transform;
  private linkHeadTiltTF?: ROSLIB.Transform;
  private jointStateCallback: (
    robotPose: RobotPose,
    jointValues: ValidJointStateDict,
    effortValues: ValidJointStateDict,
  ) => void;
  private batteryStateCallback: (batteryState: ROSBatteryState) => void;
  private occupancyGridCallback: (occupancyGrid: ROSOccupancyGrid) => void;
  private moveBaseResultCallback: (goalState: MoveBaseState) => void;
  private amclPoseCallback: (pose: ROSLIB.Transform) => void;
  private isRunStoppedCallback: (isRunStopped: boolean) => void;
  private hasBetaTeleopKitCallback: (value: boolean) => void;
  private stretchToolCallback: (value: string) => void;
  private lookAtGripperInterval?: number; // ReturnType<typeof setInterval>
  private subscriptions: ROSLIB.Topic[] = [];
  private hasBetaTeleopKitParam: ROSLIB.Param;
  private stretchToolParam: ROSLIB.Param;

  constructor(props: {
    jointStateCallback: (
      robotPose: RobotPose,
      jointValues: ValidJointStateDict,
      effortValues: ValidJointStateDict,
    ) => void;
    batteryStateCallback: (batteryState: ROSBatteryState) => void;
    occupancyGridCallback: (occupancyGrid: ROSOccupancyGrid) => void;
    moveBaseResultCallback: (goalState: MoveBaseState) => void;
    amclPoseCallback: (pose: ROSLIB.Transform) => void;
    isRunStoppedCallback: (isRunStopped: boolean) => void;
    hasBetaTeleopKitCallback: (value: boolean) => void;
    stretchToolCallback: (value: string) => void;
  }) {
    super(props);
    this.jointStateCallback = props.jointStateCallback;
    this.batteryStateCallback = props.batteryStateCallback;
    this.occupancyGridCallback = props.occupancyGridCallback;
    this.moveBaseResultCallback = props.moveBaseResultCallback;
    this.amclPoseCallback = props.amclPoseCallback;
    this.isRunStoppedCallback = props.isRunStoppedCallback;
    this.hasBetaTeleopKitCallback = props.hasBetaTeleopKitCallback;
    this.stretchToolCallback = props.stretchToolCallback;
  }

  async connect(): Promise<void> {
    this.ros = new ROSLIB.Ros({
      // set this to false to use the new service interface to
      // tf2_web_republisher. true is the default and means roslibjs
      // will use the action interface
      groovyCompatibility: false,
      url: "wss://localhost:9090",
    });

    return new Promise<void>((resolve, reject) => {
      this.ros.on("connection", async () => {
        await this.onConnect();
        resolve();
      });
      this.ros.on("error", (error) => {
        reject(error);
      });

      this.ros.on("close", () => {
        reject("Connection to websocket has been closed.");
      });
    });
  }

  async onConnect() {
    this.subscribeToJointState();
    this.subscribeToJointLimits();
    this.subscribeToBatteryState();
    this.subscribeToMoveBaseResult();
    this.subscribeToIsRunStopped();
    this.createTrajectoryClient();
    this.createMoveBaseClient();
    this.createCmdVelTopic();
    this.createSwitchToNavigationService();
    this.createSwitchToPositionService();
    this.createDepthSensingService();
    this.createRunStopService();
    this.createRobotFrameTFClient();
    this.createMapFrameTFClient();
    this.subscribeToGripperFingerTF();
    this.subscribeToHeadTiltTF();
    this.subscribeToMapTF();

    return Promise.resolve();
  }

  closeROSConnection() {
    this.subscriptions.forEach((topic) => {
      topic.unsubscribe();
    });
    this.ros.close();
  }

  isROSConnected() {
    return this.ros.isConnected;
  }

  subscribeToJointState() {
    const jointStateTopic: ROSLIB.Topic<ROSJointState> = new ROSLIB.Topic({
      ros: this.ros,
      name: "/stretch/joint_states",
      messageType: "sensor_msgs/msg/JointState",
    });
    this.subscriptions.push(jointStateTopic);

    jointStateTopic.subscribe((msg: ROSJointState) => {
      this.jointState = msg;
      let robotPose: RobotPose = rosJointStatetoRobotPose(this.jointState);
      let jointValues: ValidJointStateDict = {};
      let effortValues: ValidJointStateDict = {};
      this.jointState.name.forEach((name?: ValidJoints) => {
        let inLimits = this.inJointLimits(name);
        let collision = this.inCollision(name);
        if (inLimits) jointValues[name!] = inLimits;
        if (collision) effortValues[name!] = collision;
      });

      if (this.jointStateCallback)
        this.jointStateCallback(robotPose, jointValues, effortValues);
    });
  }

  subscribeToJointLimits() {
    const jointLimitsTopic: ROSLIB.Topic<ROSJointState> = new ROSLIB.Topic({
      ros: this.ros,
      name: "/joint_limits",
      messageType: "sensor_msgs/msg/JointState",
    });
    this.subscriptions.push(jointLimitsTopic);

    jointLimitsTopic.subscribe((msg: ROSJointState) => {
      msg.name.forEach((name, idx) => {
        if (name == "joint_arm") name = "wrist_extension";
        this.jointLimits[name] = [msg.position[idx], msg.velocity[idx]];
      });
    });
  }

  subscribeToBatteryState() {
    const batteryStateTopic: ROSLIB.Topic<ROSBatteryState> = new ROSLIB.Topic({
      ros: this.ros,
      name: "/battery",
      messageType: "sensor_msgs/msg/BatteryState",
    });
    this.subscriptions.push(batteryStateTopic);

    batteryStateTopic.subscribe((msg: ROSBatteryState) => {
      if (this.batteryStateCallback) this.batteryStateCallback(msg);
    });
  }

  subscribeToVideo(props: VideoProps) {
    let topic: ROSLIB.Topic<ROSCompressedImage> = new ROSLIB.Topic({
      ros: this.ros,
      name: props.topicName,
      messageType: "sensor_msgs/CompressedImage",
    });
    topic.subscribe(props.callback);
    this.subscriptions.push(topic);
  }

  getHasBetaTeleopKit() {
    this.hasBetaTeleopKitParam = new ROSLIB.Param({
      ros: this.ros,
      name: "/configure_video_streams:has_beta_teleop_kit",
    });

    this.hasBetaTeleopKitParam.get((value: boolean) => {
      console.log("has beta teleop kit: ", value);
      if (this.hasBetaTeleopKitCallback) this.hasBetaTeleopKitCallback(value);
    });
  }

  getStretchTool() {
    this.stretchToolParam = new ROSLIB.Param({
      ros: this.ros,
      name: "/configure_video_streams:stretch_tool",
    });

    this.stretchToolParam.get((value: string) => {
      console.log("stretch tool: ", value);
      if (this.stretchToolCallback) this.stretchToolCallback(value);
    });
  }

  getOccupancyGrid() {
    let getMapService = new ROSLIB.Service({
      ros: this.ros,
      name: "/map_server/map",
      serviceType: "nav2_msgs/srv/GetMap",
    });

    var request = new ROSLIB.ServiceRequest({});
    getMapService?.callService(
      request,
      (response: { map: ROSOccupancyGrid }) => {
        if (this.occupancyGridCallback)
          this.occupancyGridCallback(response.map);
      },
    );
  }

  getJointLimits() {
    let getJointLimitsService = new ROSLIB.Service({
      ros: this.ros,
      name: "/get_joint_states",
      serviceType: "std_srvs/Trigger",
    });

    var request = new ROSLIB.ServiceRequest({});
    getJointLimitsService.callService(request, () => {});
  }

  subscribeToMoveBaseResult() {
    let topic: ROSLIB.Topic<NavigateToPoseActionResult> = new ROSLIB.Topic({
      ros: this.ros,
      name: "/navigate_to_pose/_action/status",
      messageType: "action_msgs/msg/GoalStatusArray",
    });
    this.subscriptions.push(topic);

    topic.subscribe((msg: NavigateToPoseActionStatusList) => {
      let status = msg.status_list.pop()?.status;
      if (this.moveBaseResultCallback) {
        if (status == 5)
          this.moveBaseResultCallback({
            state: "Navigation cancelled!",
            alert_type: "error",
          });
        else if (status == 4)
          this.moveBaseResultCallback({
            state: "Navigation succeeded!",
            alert_type: "success",
          });
        else if (status == 6)
          this.moveBaseResultCallback({
            state: "Navigation failed!",
            alert_type: "error",
          });
      }
    });
  }

  subscribeToIsRunStopped() {
    let topic: ROSLIB.Topic = new ROSLIB.Topic({
      ros: this.ros,
      name: "is_runstopped",
      messageType: "std_msgs/msg/Bool",
    });
    this.subscriptions.push(topic);

    topic.subscribe((msg) => {
      if (this.isRunStoppedCallback) this.isRunStoppedCallback(msg.data);
    });
  }

  createTrajectoryClient() {
    this.trajectoryClient = new ROSLIB.ActionHandle({
      ros: this.ros,
      name: "/stretch_controller/follow_joint_trajectory",
      actionType: "control_msgs/action/FollowJointTrajectory",
    });
  }

  createMoveBaseClient() {
    this.moveBaseClient = new ROSLIB.ActionHandle({
      ros: this.ros,
      name: "/navigate_to_pose",
      actionType: "nav2_msgs/action/NavigateToPose",
      // timeout: 100
    });
  }

  createCmdVelTopic() {
    this.cmdVelTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: "/stretch/cmd_vel",
      messageType: "geometry_msgs/Twist",
    });
  }

  createSwitchToNavigationService() {
    this.switchToNavigationService = new ROSLIB.Service({
      ros: this.ros,
      name: "/switch_to_navigation_mode",
      serviceType: "std_srvs/Trigger",
    });
  }

  createSwitchToPositionService() {
    this.switchToPositionService = new ROSLIB.Service({
      ros: this.ros,
      name: "/switch_to_position_mode",
      serviceType: "std_srvs/Trigger",
    });
  }

  createDepthSensingService() {
    this.setDepthSensingService = new ROSLIB.Service({
      ros: this.ros,
      name: "/depth_ar",
      serviceType: "std_srvs/srv/SetBool",
    });
  }

  createRunStopService() {
    this.setRunStopService = new ROSLIB.Service({
      ros: this.ros,
      name: "/runstop",
      serviceType: "std_srvs/srv/SetBool",
    });
  }

  createRobotFrameTFClient() {
    this.robotFrameTfClient = new ROSLIB.TFClient({
      ros: this.ros,
      fixedFrame: "base_link",
      angularThres: 0.001,
      transThres: 0.001,
      rate: 10,
    });
  }

  createMapFrameTFClient() {
    this.mapFrameTfClient = new ROSLIB.TFClient({
      ros: this.ros,
      fixedFrame: "map",
      angularThres: 0.001,
      transThres: 0.001,
      rate: 10,
    });
  }

  subscribeToGripperFingerTF() {
    this.robotFrameTfClient?.subscribe(
      "link_gripper_finger_left",
      (transform) => {
        this.linkGripperFingerLeftTF = transform;
      },
    );
  }

  subscribeToHeadTiltTF() {
    this.robotFrameTfClient?.subscribe("link_head_tilt", (transform) => {
      this.linkHeadTiltTF = transform;
    });
  }

  subscribeToMapTF() {
    this.mapFrameTfClient?.subscribe("base_link", (transform) => {
      if (this.amclPoseCallback) this.amclPoseCallback(transform);
    });
  }

  setDepthSensing(toggle: boolean) {
    var request = new ROSLIB.ServiceRequest({ data: toggle });
    this.setDepthSensingService?.callService(request, (response: boolean) => {
      response
        ? console.log("Enable depth sensing")
        : console.log("Disabled depth sensing");
    });
  }

  setRunStop(toggle: boolean) {
    var request = new ROSLIB.ServiceRequest({ data: toggle });
    this.setRunStopService?.callService(request, (response: boolean) => {});
  }

  switchToNavigationMode() {
    var request = new ROSLIB.ServiceRequest({});
    if (robotMode !== "navigation") {
      this.switchToNavigationService!.callService(request, () => {
        robotMode = "navigation";
        console.log("Switched to navigation mode");
      });
    }
  }

  switchToPositionMode = () => {
    var request = new ROSLIB.ServiceRequest({});
    if (robotMode !== "position") {
      this.switchToPositionService!.callService(request, () => {
        robotMode = "position";
        console.log("Switched to position mode");
      });
    }
  };

  executeBaseVelocity = (props: { linVel: number; angVel: number }): void => {
    this.switchToNavigationMode();
    this.stopExecution();
    let twist = new ROSLIB.Message({
      linear: {
        x: props.linVel,
        y: 0,
        z: 0,
      },
      angular: {
        x: 0,
        y: 0,
        z: props.angVel,
      },
    });
    if (!this.cmdVelTopic) throw "trajectoryClient is undefined";
    this.cmdVelTopic.publish(twist);
  };

  makeIncrementalMoveGoal(
    jointName: ValidJoints,
    jointValueInc: number,
  ): ROSLIB.Goal | undefined {
    if (!this.jointState) throw "jointState is undefined";
    let newJointValue = this.getJointValue(jointName);
    // Paper over Hello's fake joints
    if (
      jointName === "translate_mobile_base" ||
      jointName === "rotate_mobile_base"
    ) {
      // These imaginary joints are floating, always have 0 as their reference
      newJointValue = 0;
    }

    let collision = this.inCollision({
      jointStateMessage: this.jointState,
      jointName: jointName,
    });
    let collisionIndex = jointValueInc <= 0 ? 0 : 1;
    if (jointName === "joint_wrist_yaw") {
      collisionIndex = jointValueInc <= 0 ? 1 : 0;
    }
    // Negative joint increment is for lower/retract/wrist out
    // Positive joint increment is for lift/extend/wrist in
    let index = jointValueInc <= 0 ? 0 : 1;
    // If request to move the joint in the direction of collision, cancel movement
    if (collision[collisionIndex]) return;

    newJointValue = newJointValue + jointValueInc;

    // Make sure new joint value is within limits
    if (jointName in this.jointLimits) {
      let inLimits = this.inJointLimitsHelper(newJointValue, jointName);
      if (!inLimits) throw "invalid joint name";
      // console.log(newJointValue, this.jointLimits[jointName]![index], inLimits[index])
      if (!inLimits[index]) newJointValue = this.jointLimits[jointName]![index];
    }

    let pose = { [jointName]: newJointValue };
    if (!this.trajectoryClient) throw "trajectoryClient is undefined";
    return this.makePoseGoal(pose);
  }

  makeMoveBaseGoal(pose: ROSPose) {
    if (!this.moveBaseClient) throw "moveBaseClient is undefined";

    let newGoal = new ROSLIB.ActionGoal({
      pose: {
        header: {
          frame_id: "map",
        },
        pose: pose,
      },
    });

    return newGoal;
  }

  makePoseGoal(pose: RobotPose) {
    let jointNames: ValidJoints[] = [];
    let jointPositions: number[] = [];
    for (let key in pose) {
      jointNames.push(key as ValidJoints);
      jointPositions.push(pose[key as ValidJoints]!);
    }

    console.log(this.trajectoryClient);

    if (!this.trajectoryClient) throw "trajectoryClient is undefined";
    let newGoal = new ROSLIB.ActionGoal({
      trajectory: {
        header: {
          stamp: {
            secs: 0,
            nsecs: 0,
          },
        },
        joint_names: jointNames,
        points: [
          {
            positions: jointPositions,
            // The following might causing the jumpiness in continuous motions
            time_from_start: {
              secs: 1,
              nsecs: 0,
            },
          },
        ],
      },
    });

    return newGoal;
  }

  makePoseGoals(poses: RobotPose[]) {
    let jointNames: ValidJoints[] = [];
    for (let key in poses[0]) {
      jointNames.push(key as ValidJoints);
    }

    let points: any = [];
    let jointPositions: number[] = [];
    poses.forEach((pose, index) => {
      jointPositions = [];
      for (let key in pose) {
        jointPositions.push(pose[key as ValidJoints]!);
      }
      points.push({
        positions: jointPositions,
        time_from_start: {
          secs: 10,
          nsecs: 0,
        },
      });
    });

    if (!this.trajectoryClient) throw "trajectoryClient is undefined";
    let newGoal = new ROSLIB.ActionGoal({
      trajectory: {
        header: {
          stamp: {
            secs: 0,
            nsecs: 0,
          },
        },
        joint_names: jointNames,
        points: points,
      },
    });

    return newGoal;
  }

  executePoseGoal(pose: RobotPose) {
    this.switchToPositionMode();
    this.poseGoal = this.makePoseGoal(pose);
    this.trajectoryClient.createClient(this.poseGoal);
  }

  async executePoseGoals(poses: RobotPose[], index: number) {
    this.switchToPositionMode();
    this.poseGoal = this.makePoseGoals(poses);
    this.trajectoryClient.createClient(this.poseGoal);
  }

  executeMoveBaseGoal(pose: ROSPose) {
    this.switchToNavigationMode();
    // this.stopExecution()
    this.moveBaseGoal = this.makeMoveBaseGoal(pose);
    this.moveBaseClient.createClient(this.moveBaseGoal);
    // this.moveBaseResultCallback({state: "Navigating to selected goal...", alert_type: "info"})
    // this.moveBaseGoal.send()
  }

  executeIncrementalMove(jointName: ValidJoints, increment: number) {
    this.switchToPositionMode();
    this.poseGoal = this.makeIncrementalMoveGoal(jointName, increment);
    this.trajectoryClient.createClient(this.poseGoal);
  }

  stopExecution() {
    this.stopTrajectoryClient();
    this.stopMoveBaseClient();
  }

  stopTrajectoryClient() {
    if (!this.trajectoryClient) throw "trajectoryClient is undefined";
    if (this.poseGoal) {
      this.trajectoryClient.cancelGoal();
      // this.poseGoal.cancel()
      this.poseGoal = undefined;
    }
  }

  stopMoveBaseClient() {
    if (!this.moveBaseClient) throw "moveBaseClient is undefined";
    if (this.moveBaseGoal) {
      this.moveBaseClient.cancelGoal();
      // this.moveBaseGoal.cancel()
      this.moveBaseGoal = undefined;
    }
  }

  setPanTiltFollowGripper(followGripper: boolean) {
    if (this.lookAtGripperInterval && followGripper) return;

    if (followGripper) {
      let panOffset = 0;
      let tiltOffset = 0;
      let lookIfReadyAndRepeat = () => {
        if (this.linkGripperFingerLeftTF && this.linkHeadTiltTF) {
          this.lookAtGripper(panOffset, tiltOffset);
        }
        this.lookAtGripperInterval = window.setTimeout(
          lookIfReadyAndRepeat,
          500,
        );
      };
      lookIfReadyAndRepeat();
    } else {
      this.stopExecution();
      clearTimeout(this.lookAtGripperInterval);
      this.lookAtGripperInterval = undefined;
    }
  }

  lookAtGripper(panOffset: number, tiltOffset: number) {
    if (!this.linkGripperFingerLeftTF)
      throw "linkGripperFingerLeftTF is undefined";
    if (!this.linkHeadTiltTF) throw "linkHeadTiltTF is undefined";
    let posDifference = {
      x:
        this.linkGripperFingerLeftTF.translation.x -
        this.linkHeadTiltTF.translation.x,
      y:
        this.linkGripperFingerLeftTF.translation.y -
        this.linkHeadTiltTF.translation.y,
      z:
        this.linkGripperFingerLeftTF.translation.z -
        this.linkHeadTiltTF.translation.z,
    };

    // Normalize posDifference
    const scalar = Math.sqrt(
      posDifference.x ** 2 + posDifference.y ** 2 + posDifference.z ** 2,
    );
    posDifference.x /= scalar;
    posDifference.y /= scalar;
    posDifference.z /= scalar;

    const pan = Math.atan2(posDifference.y, posDifference.x) + panOffset;
    const tilt = Math.atan2(posDifference.z, -posDifference.y) + tiltOffset;

    // Goals really close to current state cause some whiplash in these joints in simulation.
    // Ignoring small goals is a temporary fix
    if (!this.jointState) throw "jointState is undefined";
    let panDiff = Math.abs(this.getJointValue("joint_head_pan") - pan);
    let tiltDiff = Math.abs(this.getJointValue("joint_head_tilt") - tilt);
    if (panDiff < 0.02 && tiltDiff < 0.02) {
      return;
    }

    this.executePoseGoal({
      joint_head_pan: pan + panOffset,
      joint_head_tilt: tilt + tiltOffset,
    });
  }

  getJointValue(jointName: ValidJoints): number {
    // Paper over Hello's fake joint implementation
    if (jointName === "joint_arm" || jointName === "wrist_extension") {
      return (
        this.getJointValue("joint_arm_l0") +
        this.getJointValue("joint_arm_l1") +
        this.getJointValue("joint_arm_l2") +
        this.getJointValue("joint_arm_l3")
      );
    } else if (
      jointName === "translate_mobile_base" ||
      jointName === "rotate_mobile_base"
    ) {
      return 0;
    }

    let jointIndex = this.jointState.name.indexOf(jointName);
    return this.jointState.position[jointIndex];
  }

  inJointLimits(jointName: ValidJoints) {
    let jointValue = this.getJointValue(jointName);
    return this.inJointLimitsHelper(jointValue, jointName);
  }

  inJointLimitsHelper(jointValue: number, jointName: ValidJoints) {
    let jointLimits = this.jointLimits[jointName];
    if (!jointLimits) return;

    var eps = 0.03;
    let inLimits: [boolean, boolean] = [true, true];
    inLimits[0] = jointValue - eps >= jointLimits[0]; // Lower joint limit
    inLimits[1] = jointValue + eps <= jointLimits[1]; // Upper joint limit
    return inLimits;
  }

  inCollision(jointName: ValidJoints) {
    let inCollision: [boolean, boolean] = [false, false];
    const MAX_EFFORTS: { [key in ValidJoints]?: [number, number] } = {
      joint_head_tilt: [-50, 50],
      joint_head_pan: [-50, 50],
      wrist_extension: [-40, 40],
      joint_lift: [0, 70],
      // "joint_wrist_yaw": [-10, 10],
      // "joint_wrist_pitch": [-10, 10],
      // "joint_wrist_roll": [-10, 10],
    };

    if (!(jointName in MAX_EFFORTS)) return inCollision;

    let jointIndex = this.jointState.name.indexOf(jointName);
    // In collision if joint is applying more than 50% effort when moving downward/inward/backward
    inCollision[0] =
      this.jointState.effort[jointIndex] < MAX_EFFORTS[jointName]![0];
    // In collision if joint is applying more than 50% effort when moving upward/outward/forward
    inCollision[1] =
      this.jointState.effort[jointIndex] > MAX_EFFORTS[jointName]![1];

    return inCollision;
  }
}
