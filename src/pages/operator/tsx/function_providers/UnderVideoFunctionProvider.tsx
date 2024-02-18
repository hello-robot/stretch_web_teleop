import { FunctionProvider } from "./FunctionProvider"
import { CENTER_WRIST, Marker, REALSENSE_BASE_POSE, REALSENSE_FORWARD_POSE, REALSENSE_GRIPPER_POSE, STOW_WRIST } from "shared/util"

export enum UnderVideoButton {
    DriveView = "Drive View",
    GripperView = "Gripper View",
    LookAtGripper = "Look At Gripper",
    LookAtBase = "Look At Base",
    LookAhead= "Look Ahead",
    FollowGripper = "Follow Gripper",
    DepthSensing = "Depth Sensing",
    ToggleArucoMarkers = "Toggle Aruco Markers",
    CenterWrist = "Center Wrist",
    StowWrist = "Stow Wrist",
}

/** Array of different perspectives for the overhead camera */
export const overheadButtons: UnderVideoButton[] = [
    UnderVideoButton.DriveView, 
    UnderVideoButton.GripperView
]
/** Type to specify the different overhead camera perspectives */
export type OverheadButtons = typeof overheadButtons[number]
/** Array of different perspectives for the realsense camera */
export const realsenseButtons: UnderVideoButton[] = [
    UnderVideoButton.LookAhead,
    UnderVideoButton.LookAtBase, 
    UnderVideoButton.LookAtGripper, 
]

/** Array of different actions for the wrist */
export const wristButtons: UnderVideoButton[] = [
    UnderVideoButton.CenterWrist,
    UnderVideoButton.StowWrist 
]
/** Type to specify the different realsense camera perspectives */
export type RealsenseButtons = typeof realsenseButtons[number]

export type UnderVideoButtonFunctions = {
    onClick?: () => void
    onCheck?: (toggle: boolean) => void
    getMarkers?: () => string[]
    send?: (name: string) => void
}

export class UnderVideoFunctionProvider extends FunctionProvider {
    constructor() {
        super()
        this.provideFunctions = this.provideFunctions.bind(this)
    }

    public provideFunctions(button: UnderVideoButton): UnderVideoButtonFunctions {
        switch (button) {
            case UnderVideoButton.DriveView:
                return {
                    onClick: () => FunctionProvider.remoteRobot?.setCameraPerspective("overhead", "nav") 
                }
            case UnderVideoButton.GripperView:
                return {
                    onClick: () => FunctionProvider.remoteRobot?.setCameraPerspective("overhead", "manip") 
                }
            case UnderVideoButton.LookAtBase:
                return {
                    onClick: () => FunctionProvider.remoteRobot?.setRobotPose(REALSENSE_BASE_POSE)
                }
            case UnderVideoButton.LookAhead:
                return {
                    onClick: () => FunctionProvider.remoteRobot?.setRobotPose(REALSENSE_FORWARD_POSE)
                }
            case UnderVideoButton.LookAtGripper:
                return {
                    onClick: () => FunctionProvider.remoteRobot?.lookAtGripper("lookAtGripper") //setRobotPose(REALSENSE_GRIPPER_POSE)
                }
            case UnderVideoButton.FollowGripper:
                return {
                    onCheck: (toggle: boolean) => FunctionProvider.remoteRobot?.setToggle("setFollowGripper", toggle)
                }
            case UnderVideoButton.DepthSensing:
                return {
                    onCheck: (toggle: boolean) => FunctionProvider.remoteRobot?.setToggle("setDepthSensing", toggle)
                }
            case UnderVideoButton.ToggleArucoMarkers:
                return {
                    onCheck: (toggle: boolean) => FunctionProvider.remoteRobot?.setToggle("setArucoMarkers", toggle)
                }
            case UnderVideoButton.CenterWrist:
                return {
                    onClick: () => FunctionProvider.remoteRobot?.setRobotPose(CENTER_WRIST)
                }
            case UnderVideoButton.StowWrist:
                return {
                    onClick: () => FunctionProvider.remoteRobot?.setRobotPose(STOW_WRIST)
                }
            default:
                throw Error(`Cannot get function for unknown UnderVideoButton ${button}`)
        }
    }
}