import ROSLIB from "roslib";
import { ROSPose, RobotPose } from "./util";
import { ValidJoints } from "./util";

export type cmd =
    | DriveCommand
    | IncrementalMove
    | setRobotModeCommand
    | CameraPerspectiveCommand
    | RobotPoseCommand
    | ToggleCommand
    | LookAtGripper
    | GetOccupancyGrid
    | MoveBaseCommand
    | StopTrajectoryCommand
    | StopMoveBaseCommand
    | MoveToPregraspCommand
    | StopMoveToPregraspCommand
    | PlaybackPosesCommand
    | GetBatteryVoltageCommand
    | GetHasBetaTeleopKit
    | GetStretchTool
    | PlayTextToSpeech
    | StopTextToSpeech
    | ShowTabletCommand
    | StopShowTabletCommand
    | HomeTheRobotCommand;

export interface VelocityCommand {
    stop: () => void;
    affirm?: () => void;
}

export interface DriveCommand {
    type: "driveBase";
    modifier: {
        linVel: number;
        angVel: number;
    };
}

export interface IncrementalMove {
    type: "incrementalMove";
    jointName: ValidJoints;
    increment: number;
}

export interface RobotPoseCommand {
    type: "setRobotPose";
    pose: RobotPose;
}

export interface PlaybackPosesCommand {
    type: "playbackPoses";
    poses: RobotPose[];
}

export interface setRobotModeCommand {
    type: "setRobotMode";
    modifier: "position" | "navigation";
}

export interface CameraPerspectiveCommand {
    type: "setCameraPerspective";
    camera: "overhead" | "realsense" | "gripper";
    perspective: string;
}

export interface ToggleCommand {
    type:
        | "setFollowGripper"
        | "setRealsenseDepthSensing"
        | "setGripperDepthSensing"
        | "setRealsenseBodyPoseEstimate"
        | "setRunStop";
    toggle: boolean;
}

export interface LookAtGripper {
    type: "lookAtGripper";
}

export interface GetOccupancyGrid {
    type: "getOccupancyGrid";
}

export interface GetHasBetaTeleopKit {
    type: "getHasBetaTeleopKit";
}

export interface GetStretchTool {
    type: "getStretchTool";
}

export interface MoveBaseCommand {
    type: "moveBase";
    pose: ROSPose;
}

export interface StopTrajectoryCommand {
    type: "stopTrajectory";
}

export interface StopMoveBaseCommand {
    type: "stopMoveBase";
}

export interface MoveToPregraspCommand {
    type: "moveToPregrasp";
    x: number;
    y: number;
}

export interface StopMoveToPregraspCommand {
    type: "stopMoveToPregrasp";
}

export interface ShowTabletCommand {
    type: "showTablet";
    url: string;
}

export interface StopShowTabletCommand {
    type: "stopShowTablet";
}

export interface GetBatteryVoltageCommand {
    type: "getBatteryVoltage";
}

export interface PlayTextToSpeech {
    type: "playTextToSpeech";
    text: string;
    override_behavior: number;
    is_slow: boolean;
}

export interface StopTextToSpeech {
    type: "stopTextToSpeech";
}

export interface HomeTheRobotCommand {
    type: "homeTheRobot";
}
