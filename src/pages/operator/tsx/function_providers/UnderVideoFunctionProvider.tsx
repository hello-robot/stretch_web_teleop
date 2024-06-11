import { FunctionProvider } from "./FunctionProvider";
import {
  ActionState,
  CENTER_WRIST,
  Marker,
  REALSENSE_BASE_POSE,
  REALSENSE_FORWARD_POSE,
  REALSENSE_GRIPPER_POSE,
  STOW_WRIST,
} from "../../../../shared/util";

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
  StartMoveToPregraspHorizontal = "Gripper Horizontal",
  StartMoveToPregraspVertical = "Gripper Vertical",
  CancelMoveToPregrasp = "Cancel Goal",
  MoveToPregraspGoalReached = "Goal Reached",
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

/** Array of different options for the MoveToPregrasp feature on the realsense camera */
export const realsenseMoveToPregraspButtons: UnderVideoButton[] = [
  UnderVideoButton.StartMoveToPregraspHorizontal,
  UnderVideoButton.StartMoveToPregraspVertical,
];

/** Array of different actions for the wrist */
export const wristButtons: UnderVideoButton[] = [
  UnderVideoButton.CenterWrist,
  UnderVideoButton.StowWrist,
];
/** Type to specify the different realsense camera perspectives */
export type RealsenseButtons = (typeof realsenseButtons)[number];

export type UnderVideoButtonFunctions = {
  onClick?: (...args: any[]) => void;
  onCheck?: (toggle: boolean) => void;
  getMarkers?: () => string[];
  send?: (name: string) => void;
  getFuture?: () => Promise<any>;
};

export class UnderVideoFunctionProvider extends FunctionProvider {
  constructor() {
    super();
    this.provideFunctions = this.provideFunctions.bind(this);
  }

  /**
   * Callback function to update the move base state in the operator
   * interface (e.g., show alerts).
   */
  private operatorCallback?: (state: ActionState) => void = undefined;
  /**
   * Store the timestam at which the last moveToPregrasp state was received
   */
  private lastMoveToPregraspStateTimestamp: number = 0;

  /**
   * Called when a response is received from the robot for the move to pregrasp.
   * @param state the move to pregrasp state to set
   */
  public setMoveToPregraspState(state: ActionState) {
    this.lastMoveToPregraspStateTimestamp = Date.now();
    if (this.operatorCallback) this.operatorCallback(state);
  }

  public provideFunctions(button: UnderVideoButton): UnderVideoButtonFunctions {
    let horizontal = false;
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
      case UnderVideoButton.StartMoveToPregraspHorizontal:
        horizontal = true;
      case UnderVideoButton.StartMoveToPregraspVertical:
        return {
          onClick: (scaledXY: [number, number] | null) => {
            if (!scaledXY) {
              console.log("No scaledXY");
              return;
            }
            FunctionProvider.remoteRobot?.moveToPregrasp(
              scaledXY[0],
              scaledXY[1],
              horizontal,
            );
          },
        };
      case UnderVideoButton.MoveToPregraspGoalReached:
        // TODO: Add timeouts to this and the other GoalReached promises!
        return {
          getFuture: () => {
            let currentTimestamp = Date.now();
            let that = this;
            const promise = new Promise((resolve, reject) => {
              let interval = setInterval(() => {
                let goalReached =
                  that.lastMoveToPregraspStateTimestamp > currentTimestamp;
                if (goalReached) {
                  clearInterval(interval);
                  resolve(true);
                }
              });
            });
            return promise;
          },
        };
      case UnderVideoButton.CancelMoveToPregrasp:
        return {
          onClick: () => FunctionProvider.remoteRobot?.stopMoveToPregrasp(),
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
        return {
          onClick: () => FunctionProvider.remoteRobot?.setRobotPose(STOW_WRIST),
        };
      default:
        throw Error(
          `Cannot get function for unknown UnderVideoButton ${button}`,
        );
    }
  }

  /**
   * Sets the local pointer to the operator's callback function, to be called
   * whenever the move to pregrasp state changes.
   *
   * @param callback operator's callback function to update aruco navigation state
   */
  public setOperatorCallback(callback: (state: ActionState) => void) {
    this.operatorCallback = callback;
  }
}
