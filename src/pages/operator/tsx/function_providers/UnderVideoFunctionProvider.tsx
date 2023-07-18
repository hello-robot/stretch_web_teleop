import { FunctionProvider } from "./FunctionProvider"
import { Marker, REALSENSE_BASE_POSE, REALSENSE_GRIPPER_POSE } from "shared/util"

export enum UnderVideoButton {
    DriveView = "Drive View",
    GripperView = "Gripper View",
    LookAtGripper = "Look At Gripper",
    LookAtBase = "Look At Base",
    FollowGripper = "Follow Gripper",
    DepthSensing = "Depth Sensing",
    ToggleArucoMarkers = "Toggle Aruco Markers",
    GetArucoMarkerNames = "Get Aruco Marker Names",
    NavigateToMarker = "Navigate to Marker"
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
    UnderVideoButton.LookAtBase, 
    UnderVideoButton.LookAtGripper, 
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
            case UnderVideoButton.GetArucoMarkerNames:
                return {
                    getMarkers: () => { 
                        const markers = FunctionProvider.remoteRobot?.getMarkers() 
                        let marker_names: string[] = []
                        if (markers) {
                            markers.markers.forEach(marker => {
                                marker_names.push(marker.text)
                            }) 
                            return marker_names
                        } 
                        return []
                    }
                }
            case UnderVideoButton.NavigateToMarker:
                return {
                    send: (name: string) => {
                        FunctionProvider.remoteRobot?.navigateToMarker(name)
                    }
                }
                break
            default:
                throw Error(`Cannot get function for unknown UnderVideoButton ${button}`)
        }
    }
}