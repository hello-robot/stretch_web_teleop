import { FunctionProvider } from "./FunctionProvider";
import {
  CENTER_WRIST,
  Marker,
  REALSENSE_BASE_POSE,
  REALSENSE_FORWARD_POSE,
  REALSENSE_GRIPPER_POSE,
  RobotPose,
  STOW_WRIST_GRIPPER,
  STOW_WRIST_TABLET,
  TabletOrientation,
  TABLET_ORIENTATION_LANDSCAPE,
  TABLET_ORIENTATION_PORTRAIT,
} from "../../../../shared/util";
import { stretchTool } from "..";

export enum UnderVideoButton {
  DriveView = "Drive View",
  GripperView = "Gripper View",
  LookAtGripper = "Look At Gripper",
  LookAtBase = "Look At Base",
  LookAhead = "Look Ahead",
  FollowGripper = "Follow Gripper",
  DepthSensing = "Depth Sensing",
  ToggleArucoMarkers = "Toggle Aruco Markers",
  CenterWrist = "Center Wrist",
  StowWrist = "Stow Wrist",
  ToggleTabletOrientation = "Toggle Tablet Orientation",
  GetTabletOrientation = "Get Tablet Orientation",
}

/** Array of different perspectives for the overhead camera */
export const overheadButtons: UnderVideoButton[] = [
  UnderVideoButton.DriveView,
  UnderVideoButton.GripperView,
];
/** Type to specify the different overhead camera perspectives */
export type OverheadButtons = (typeof overheadButtons)[number];
/** Array of different perspectives for the realsense camera */
export const realsenseButtons: UnderVideoButton[] = [
  UnderVideoButton.LookAhead,
  UnderVideoButton.LookAtBase,
  UnderVideoButton.LookAtGripper,
];

/** Array of different actions for the wrist */
export const wristButtons: UnderVideoButton[] = [
  UnderVideoButton.CenterWrist,
  UnderVideoButton.StowWrist,
];
/** Type to specify the different realsense camera perspectives */
export type RealsenseButtons = (typeof realsenseButtons)[number];

export type UnderVideoButtonFunctions = {
  onClick?: () => void;
  onCheck?: (toggle: boolean) => void;
  get?: () => any;
  send?: (name: string) => void;
};

export class UnderVideoFunctionProvider extends FunctionProvider {
  private tabletOrientation: TabletOrientation;
  /**
   * Callback function for when the tablet orientation changes
   */
  private operatorCallback?: (tabletOrientation: TabletOrientation) => void =
    undefined;

  constructor() {
    super();
    this.tabletOrientation = TabletOrientation.LANDSCAPE;
    this.provideFunctions = this.provideFunctions.bind(this);
    this.jointStateCallback = this.jointStateCallback.bind(this);
  }

  public provideFunctions(button: UnderVideoButton): UnderVideoButtonFunctions {
    switch (button) {
      case UnderVideoButton.DriveView:
        return {
          onClick: () =>
            FunctionProvider.remoteRobot?.setCameraPerspective(
              "overhead",
              "nav",
            ),
        };
      case UnderVideoButton.GripperView:
        return {
          onClick: () =>
            FunctionProvider.remoteRobot?.setCameraPerspective(
              "overhead",
              "manip",
            ),
        };
      case UnderVideoButton.LookAtBase:
        return {
          onClick: () =>
            FunctionProvider.remoteRobot?.setRobotPose(REALSENSE_BASE_POSE),
        };
      case UnderVideoButton.LookAhead:
        return {
          onClick: () =>
            FunctionProvider.remoteRobot?.setRobotPose(REALSENSE_FORWARD_POSE),
        };
      case UnderVideoButton.LookAtGripper:
        return {
          onClick: () =>
            FunctionProvider.remoteRobot?.lookAtGripper("lookAtGripper"), //setRobotPose(REALSENSE_GRIPPER_POSE)
        };
      case UnderVideoButton.FollowGripper:
        return {
          onCheck: (toggle: boolean) =>
            FunctionProvider.remoteRobot?.setToggle("setFollowGripper", toggle),
        };
      case UnderVideoButton.DepthSensing:
        return {
          onCheck: (toggle: boolean) =>
            FunctionProvider.remoteRobot?.setToggle("setDepthSensing", toggle),
        };
      case UnderVideoButton.ToggleArucoMarkers:
        return {
          onCheck: (toggle: boolean) =>
            FunctionProvider.remoteRobot?.setToggle("setArucoMarkers", toggle),
        };
      case UnderVideoButton.CenterWrist:
        return {
          onClick: () =>
            FunctionProvider.remoteRobot?.setRobotPose(CENTER_WRIST),
        };
      case UnderVideoButton.StowWrist:
        if (stretchTool === "eoa_wrist_dw3_tool_tablet_12in") {
          return {
            onClick: () =>
              FunctionProvider.remoteRobot?.setRobotPose(STOW_WRIST_TABLET),
          };
        } else {
          return {
            onClick: () =>
              FunctionProvider.remoteRobot?.setRobotPose(STOW_WRIST_GRIPPER),
          };
        }
      case UnderVideoButton.ToggleTabletOrientation:
        return {
          onCheck: (isPortrait: boolean) =>
            FunctionProvider.remoteRobot?.setRobotPose(
              isPortrait
                ? TABLET_ORIENTATION_LANDSCAPE
                : TABLET_ORIENTATION_PORTRAIT,
            ),
        };
      case UnderVideoButton.GetTabletOrientation:
        return {
          get: () => {
            return this.tabletOrientation;
          },
        };
      default:
        throw Error(
          `Cannot get function for unknown UnderVideoButton ${button}`,
        );
    }
  }

  /**
   * Callback for when a new joint state is received.
   */
  public jointStateCallback(robotPose: RobotPose) {
    let prevTabletOrientation = this.tabletOrientation;
    // Update the tablet orientation
    let wristRoll = 0.0;
    if (robotPose.joint_wrist_roll) {
      wristRoll = robotPose.joint_wrist_roll;
    }
    let diff = Math.abs(wristRoll % Math.PI);
    let isLandscape = false;
    if (diff <= Math.PI / 4.0 || diff >= (3.0 * Math.PI) / 4.0) {
      isLandscape = true;
    }
    if (isLandscape) {
      this.tabletOrientation = TabletOrientation.LANDSCAPE;
    } else {
      this.tabletOrientation = TabletOrientation.PORTRAIT;
    }
    if (
      this.operatorCallback &&
      prevTabletOrientation !== this.tabletOrientation
    ) {
      this.operatorCallback(this.tabletOrientation);
    }
  }

  /**
   * Sets the local pointer to the operator's callback function, to be called
   * whenever the tablet orientation changes.
   *
   * @param callback operator's callback function to update aruco navigation state
   */
  public setOperatorCallback(
    callback: (tabeltOrientation: TabletOrientation) => void,
  ) {
    this.operatorCallback = callback;
  }
}
