import { ROSPose, RobotPose } from "./util"
import { ValidJoints } from "./util"

export type cmd = DriveCommand | IncrementalMove | setRobotModeCommand | CameraPerspectiveCommand | RobotPoseCommand | ToggleCommand | LookAtGripper | GetOccupancyGrid | MoveBaseCommand | StopCommand | PlaybackPosesCommand

export interface VelocityCommand {
    stop: () => void,
    affirm?: () => void
}

export interface DriveCommand {
    type: "driveBase",
    modifier: {
        linVel: number,
        angVel: number
    }
}

export interface IncrementalMove {
    type: "incrementalMove"
    jointName: ValidJoints,
    increment: number
}

export interface RobotPoseCommand {
    type: "setRobotPose"
    pose: RobotPose
}

export interface PlaybackPosesCommand {
    type: "playbackPoses"
    poses: RobotPose[]
}

export interface setRobotModeCommand {
    type: "setRobotMode",
    modifier: "position" | "navigation"
}

export interface CameraPerspectiveCommand {
    type: "setCameraPerspective"
    camera: "overhead" | "realsense" | "gripper"
    perspective: string
}

export interface ToggleCommand {
    type: "setFollowGripper" | "setDepthSensing" | "setArucoMarkers"
    toggle: boolean
}

export interface LookAtGripper {
    type: "lookAtGripper"
}

export interface GetOccupancyGrid {
    type: "getOccupancyGrid"
}

export interface MoveBaseCommand {
    type: "moveBase"
    pose: ROSPose
}

export interface StopCommand {
    type: "stop"
}