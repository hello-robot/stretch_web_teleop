import { FunctionProvider } from "./FunctionProvider";
import {
    ActionState,
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
    StretchTool,
    BoundingBox2D,
    findMinimumEncapsulatingBoxForSelected,
} from "../../../../shared/util";
import { stretchTool } from "..";
import React from "react";

export enum UnderVideoButton {
    DriveView = "Drive View",
    GripperView = "Gripper View",
    LookAtGripper = "Look At Gripper",
    LookAtBase = "Look At Base",
    LookAhead = "Look Ahead",
    FollowGripper = "Follow Gripper",
    PredictiveDisplay = "Predictive Display",
    RealsenseDepthSensing = "Realsense Depth Sensing",
    SelectObject = "Select Object",
    GripperDepthSensing = "Gripper Depth Sensing",
    ExpandedGripperView = "Expanded Gripper View",
    ToggleArucoMarkers = "Toggle Aruco Markers",
    CenterWrist = "Center Wrist",
    StowWrist = "Stow Wrist",
    StartMoveToPregraspHorizontal = "Gripper Horizontal",
    StartMoveToPregraspVertical = "Gripper Vertical",
    CancelMoveToPregrasp = "Cancel Goal",
    MoveToPregraspGoalReached = "Goal Reached",
    ToggleTabletOrientation = "Toggle Tablet Orientation",
    GetTabletOrientation = "Get Tablet Orientation",
    GetDetectedObjects = "Get Detected Objects",
    DetectObjects = "Detect Objects",
    RealsenseBodyPoseEstimate = "Show Body Pose",
    RealsenseShowTablet = "Show Tablet",
    RealsenseStopShowTablet = "Stop Show Tablet",
    RealsenseShowTabletGoalReached = "Show Tablet Goal Reached",
    VoiceSelectObject = "Voice Select Object",
    VoiceSelectBite = "Voice Select Bite",
    VoiceMoveToPregraspHorizontal = "Voice Gripper Horizonal",
    VoiceMoveToPregraspVertical = "Voice Gripper Vertical",
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
    get?: () => any;
    set?: (value: any) => void;
    send?: (name: string) => void;
    getFuture?: () => Promise<any>;
};

export class UnderVideoFunctionProvider extends FunctionProvider {
    private tabletOrientation: TabletOrientation;
    private selectedObjectCallback: (object: number) => void;
    public detectedObjects: BoundingBox2D[];
    private selectedObject: number | undefined;

    constructor() {
        super();
        this.tabletOrientation = TabletOrientation.LANDSCAPE;
        this.detectedObjects = [];
        this.provideFunctions = this.provideFunctions.bind(this);
        this.jointStateCallback = this.jointStateCallback.bind(this);
        this.detectedObjectsCallback = this.detectedObjectsCallback.bind(this);
        this.setSelectedObjectCallback = this.setSelectedObjectCallback.bind(this);
    }

    /**
     * Callback function for when the tablet orientation changes
     */
    private tabletOrientationOperatorCallback?: (
        tabletOrientation: TabletOrientation,
    ) => void = undefined;
    /**
     * Callback function to update the move base state in the operator
     * interface (e.g., show alerts).
     */
    private moveToPregraspOperatorCallback?: (state: ActionState) => void =
        undefined;
    /**
     * CAllback function to update the show tablet state in the operator
     * interface (e.g., show alerts).
     */
    private showTabletOperatorCallback?: (state: ActionState) => void =
        undefined;
    /**
     * Store the timestamp at which the last moveToPregrasp state was received
     */
    private lastMoveToPregraspStateTimestamp: number = 0;
    /**
     * Store the timestamp at which the last showTablet state was received
     */
    private lastShowTabletStateTimestamp: number = 0;

    /**
     * Called when a response is received from the robot for the move to pregrasp.
     * @param state the move to pregrasp state to set
     */
    public setMoveToPregraspState(state: ActionState) {
        this.lastMoveToPregraspStateTimestamp = Date.now();
        if (this.moveToPregraspOperatorCallback)
            this.moveToPregraspOperatorCallback(state);
    }
    /**
     * Called when a response is received from the robot for the show tablet action.
     * @param state the show tablet state to set
     */
    public setShowTabletState(state: ActionState) {
        this.lastShowTabletStateTimestamp = Date.now();
        if (this.showTabletOperatorCallback)
            this.showTabletOperatorCallback(state);
    }

    public provideFunctions(
        button: UnderVideoButton,
    ): UnderVideoButtonFunctions {
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
                        FunctionProvider.remoteRobot?.setRobotPose(
                            REALSENSE_BASE_POSE,
                        ),
                };
            case UnderVideoButton.LookAhead:
                return {
                    onClick: () =>
                        FunctionProvider.remoteRobot?.setRobotPose(
                            REALSENSE_FORWARD_POSE,
                        ),
                };
            case UnderVideoButton.LookAtGripper:
                return {
                    onClick: () =>
                        FunctionProvider.remoteRobot?.lookAtGripper(
                            "lookAtGripper",
                        ), //setRobotPose(REALSENSE_GRIPPER_POSE)
                };
            case UnderVideoButton.FollowGripper:
                return {
                    onCheck: (toggle: boolean) =>
                        FunctionProvider.remoteRobot?.setToggle(
                            "setFollowGripper",
                            toggle,
                        ),
                };
            case UnderVideoButton.RealsenseDepthSensing:
                return {
                    onCheck: (toggle: boolean) =>
                        FunctionProvider.remoteRobot?.setToggle(
                            "setRealsenseDepthSensing",
                            toggle,
                        ),
                };
            case UnderVideoButton.GripperDepthSensing:
                return {
                    onCheck: (toggle: boolean) =>
                        FunctionProvider.remoteRobot?.setToggle(
                            "setGripperDepthSensing",
                            toggle,
                        ),
                };
            case UnderVideoButton.ExpandedGripperView:
                return {
                    onCheck: (toggle: boolean) =>
                        FunctionProvider.remoteRobot?.setToggle(
                            "setExpandedGripper",
                            toggle,
                        ),
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
                                    that.lastMoveToPregraspStateTimestamp >
                                    currentTimestamp;
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
                    onClick: () =>
                        FunctionProvider.remoteRobot?.stopMoveToPregrasp(),
                };
            case UnderVideoButton.ToggleArucoMarkers:
                return {
                    onCheck: (toggle: boolean) =>
                        FunctionProvider.remoteRobot?.setToggle(
                            "setArucoMarkers",
                            toggle,
                        ),
                };
            case UnderVideoButton.CenterWrist:
                return {
                    onClick: () =>
                        FunctionProvider.remoteRobot?.setRobotPose(
                            CENTER_WRIST,
                        ),
                };
            case UnderVideoButton.StowWrist:
                if (stretchTool === StretchTool.TABLET) {
                    return {
                        onClick: () =>
                            FunctionProvider.remoteRobot?.setRobotPose(
                                STOW_WRIST_TABLET,
                            ),
                    };
                } else {
                    return {
                        onClick: () =>
                            FunctionProvider.remoteRobot?.setRobotPose(
                                STOW_WRIST_GRIPPER,
                            ),
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
            case UnderVideoButton.GetDetectedObjects:
                return {
                    get: () => {
                        return this.detectedObjects;
                    },
                };
            case UnderVideoButton.DetectObjects:
                return {
                    onCheck: (toggle: boolean) =>
                        FunctionProvider.remoteRobot?.setToggle(
                            "setDetectObjects",
                            toggle,
                        ),
                };
            case UnderVideoButton.VoiceSelectObject:
                return {
                    set: (value: number) => {
                        if (value < this.detectedObjects.length) {
                            this.selectedObjectCallback(value)
                            this.selectedObject = value;
                        }
                    }
                };
            case UnderVideoButton.VoiceSelectBite:
                return {
                    set: (value: number) => {
                        if (value < this.detectedObjects.length) {
                            this.selectedObjectCallback(value)
                            this.selectedObject = value;
                            let bbox: BoundingBox2D = this.detectedObjects[this.selectedObject]
                            let encapsulatingBox: BoundingBox2D = findMinimumEncapsulatingBoxForSelected(bbox, this.detectedObjects)
                            let plate_x = encapsulatingBox.center.position.x + (encapsulatingBox.center.position.x - bbox.center.position.x)
                            console.log(this.detectedObjects.indexOf(bbox), this.detectedObjects.indexOf(encapsulatingBox))
                            console.log(bbox.center.position.x, encapsulatingBox.center.position.x, plate_x)
                            this.provideFunctions(
                                UnderVideoButton.StartMoveToPregraspVertical
                            ).onClick([bbox.center.position.x, bbox.center.position.y])
                            this.provideFunctions(
                                UnderVideoButton.DetectObjects,
                            ).onCheck!(false);
                            this.provideFunctions(
                                UnderVideoButton.MoveToPregraspGoalReached
                            ).getFuture!().then(() => {
                                this.provideFunctions(
                                    UnderVideoButton.StartMoveToPregraspVertical
                                ).onClick([plate_x, bbox.center.position.y])
                                this.provideFunctions(
                                    UnderVideoButton.MoveToPregraspGoalReached
                                ).getFuture!().then(() => {
                                    this.provideFunctions(
                                        UnderVideoButton.DetectObjects,
                                    ).onCheck!(true);
                                });
                            });
                        }
                    }
                };
            case UnderVideoButton.VoiceMoveToPregraspVertical:
                return {
                    onClick: () => {
                        let bbox: BoundingBox2D = this.detectedObjects[this.selectedObject]
                        this.provideFunctions(
                            UnderVideoButton.StartMoveToPregraspVertical
                        ).onClick([bbox.center.position.x, bbox.center.position.y])
                        this.provideFunctions(
                            UnderVideoButton.MoveToPregraspGoalReached
                        ).getFuture!();
                        this.provideFunctions(
                            UnderVideoButton.DetectObjects,
                        ).onCheck!(true);
                    }
                }
            case UnderVideoButton.VoiceMoveToPregraspHorizontal:
                return {
                    onClick: () => {
                        let bbox: BoundingBox2D = this.detectedObjects[this.selectedObject]
                        this.provideFunctions(
                            UnderVideoButton.StartMoveToPregraspHorizontal
                        ).onClick([bbox.center.position.x, bbox.center.position.y])
                        this.provideFunctions(
                            UnderVideoButton.MoveToPregraspGoalReached
                        ).getFuture!();
                        this.provideFunctions(
                            UnderVideoButton.DetectObjects,
                        ).onCheck!(true);
                    }
                }
            case UnderVideoButton.RealsenseBodyPoseEstimate:
                return {
                    onCheck: (toggle: boolean) =>
                        FunctionProvider.remoteRobot?.setToggle(
                            "setRealsenseBodyPoseEstimate",
                            toggle,
                        ),
                };
            case UnderVideoButton.RealsenseShowTablet:
                return {
                    onClick: () => FunctionProvider.remoteRobot?.showTablet(),
                };
            case UnderVideoButton.RealsenseStopShowTablet:
                return {
                    onClick: () =>
                        FunctionProvider.remoteRobot?.stopShowTablet(),
                };
            case UnderVideoButton.RealsenseShowTabletGoalReached:
                // TODO: Add timeouts to this and the other GoalReached promises!
                return {
                    getFuture: () => {
                        let currentTimestamp = Date.now();
                        let that = this;
                        const promise = new Promise((resolve, reject) => {
                            let interval = setInterval(() => {
                                let goalReached =
                                    that.lastShowTabletStateTimestamp >
                                    currentTimestamp;
                                if (goalReached) {
                                    clearInterval(interval);
                                    resolve(true);
                                }
                            });
                        });
                        return promise;
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
            this.tabletOrientationOperatorCallback &&
            prevTabletOrientation !== this.tabletOrientation
        ) {
            this.tabletOrientationOperatorCallback(this.tabletOrientation);
        }
    }

    public detectedObjectsCallback(objects: BoundingBox2D[]) {
        this.detectedObjects = objects
        console.log("updating detected objects")
    }

    public setSelectedObjectCallback(callback: (object: number) => void) {
        this.selectedObjectCallback = callback;
    }
    // public detectedObjectCallback(bbox: BoundingBox) {
    //     console.log("detected object: ", bbox)
    //     let scaledY = Math.round((bbox.x_max + bbox.x_min)/2)
    //     let scaledX = Math.round((bbox.y_max + bbox.y_min)/2)
    //     FunctionProvider.remoteRobot?.moveToPregrasp(
    //         scaledY,
    //         scaledX,
    //         false
    //     );
    // }

    /**
     * Sets the local pointer to the operator's callback function, to be called
     * whenever the move to pregrasp state changes.
     *
     * @param callback operator's callback function to update aruco navigation state
     */
    public setMoveToPregraspOperatorCallback(
        callback: (state: ActionState) => void,
    ) {
        this.moveToPregraspOperatorCallback = callback;
    }

    /**
     * Sets the local pointer to the operator's callback function, to be called
     * whenever the show tablet state changes.
     */
    public setShowTabletOperatorCallback(
        callback: (state: ActionState) => void,
    ) {
        this.showTabletOperatorCallback = callback;
    }

    /**
     * Sets the local pointer to the operator's callback function, to be called
     * whenever the tablet orientation changes.
     *
     * @param callback operator's callback function to update aruco navigation state
     */
    public setTabletOrientationOperatorCallback(
        callback: (tabletOrientation: TabletOrientation) => void,
    ) {
        this.tabletOrientationOperatorCallback = callback;
    }
}
