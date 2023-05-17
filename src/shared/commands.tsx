import { Pose2D, RobotPose, uuid } from "./util"
import { ValidJoints } from "./util"

export type cmd = DriveCommand | IncrementalMove | setRobotModeCommand

export interface VelocityCommand { stop: () => void }

interface GeneralCommand {
    msg: string
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

export interface NavGoalCommand {
    type: "navGoal",
    goal: Pose2D,
    id: uuid,
}

export interface NamedPose {
    name: string,
    description: string,
    jointState: RobotPose
}

export interface PoseGoalCommand {
    type: "poseGoal",
    goal: NamedPose,
    id: uuid
}


export interface setRobotModeCommand {
    type: "setRobotMode",
    modifier: "position" | "navigation"
}