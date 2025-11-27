import { FunctionProvider } from "./FunctionProvider";
import {
    MovementRecorderFunctions,
    MovementRecorderFunction,
} from "../layout_components/MovementRecorder";
import {
    CENTER_WRIST,
    POINT_WRIST_DOWN,
    RobotPose,
    STOW_WRIST_GRIPPER,
    ValidJoints,
} from "shared/util";
import { StorageHandler } from "../storage_handler/StorageHandler";

export class MovementRecorderFunctionProvider extends FunctionProvider {
    private recordPosesHeartbeat?: number; // ReturnType<typeof setInterval>
    private poses: RobotPose[];
    private storageHandler: StorageHandler;
    private quickActionNames: string[] = [
        "Center Wrist",
        "Tuck Wrist",
        "Point Wrist Down",
    ];
    private quickActionPoses: RobotPose[] = [
        CENTER_WRIST,
        STOW_WRIST_GRIPPER,
        POINT_WRIST_DOWN,
    ];

    constructor(storageHandler: StorageHandler) {
        super();
        this.provideFunctions = this.provideFunctions.bind(this);
        this.poses = [];
        this.storageHandler = storageHandler;
    }

    public provideFunctions(poseRecordFunction: MovementRecorderFunction) {
        switch (poseRecordFunction) {
            case MovementRecorderFunction.Record:
                return (
                    head: boolean,
                    arm: boolean,
                    lift: boolean,
                    wrist_roll: boolean,
                    wrist_pitch: boolean,
                    wrist_yaw: boolean,
                    gripper: boolean
                ) => {
                    let prevJoint: ValidJoints | undefined;
                    let prevJointDirection: number | undefined;
                    let currJointDirection: number | undefined;
                    this.recordPosesHeartbeat = window.setInterval(() => {
                        const currentPose: RobotPose =
                            FunctionProvider.remoteRobot!.sensors.getRobotPose(
                                head,
                                arm,
                                lift,
                                wrist_roll,
                                wrist_pitch,
                                wrist_yaw,
                                gripper
                            );
                        const prevPose =
                            this.poses.length == 0
                                ? undefined
                                : this.poses[this.poses.length - 1];
                        if (prevPose) {
                            Object.keys(currentPose).map((key, index) => {
                                if (
                                    Math.abs(
                                        currentPose[key as ValidJoints]! -
                                            prevPose[key as ValidJoints]!
                                    ) > 0.025
                                ) {
                                    // If there is no prevJoint or the current joint moving has changed
                                    if (!prevJoint || prevJoint != key) {
                                        prevJoint = key as ValidJoints;
                                        prevJointDirection = Math.sign(
                                            currentPose[key as ValidJoints] -
                                                prevPose[
                                                    prevJoint as ValidJoints
                                                ]
                                        );
                                        this.poses.push(currentPose);
                                        return;
                                    }

                                    currJointDirection = Math.sign(
                                        currentPose[key as ValidJoints] -
                                            prevPose[prevJoint as ValidJoints]
                                    );

                                    // If the direction of joint movement has not been changed
                                    if (
                                        currJointDirection ===
                                        prevJointDirection
                                    ) {
                                        this.poses[this.poses.length - 1][
                                            prevJoint
                                        ] = currentPose[key as ValidJoints];
                                        return;
                                    }

                                    // If the direction of joint movement has changed
                                    else {
                                        prevJointDirection = currJointDirection;
                                        this.poses.push(currentPose);
                                        return;
                                    }
                                }
                            });
                        } else {
                            this.poses.push(currentPose);
                        }
                    }, 150);
                };
            case MovementRecorderFunction.SaveRecording:
                return (name: string) => {
                    if (this.recordPosesHeartbeat) {
                        clearInterval(this.recordPosesHeartbeat);
                        this.recordPosesHeartbeat = undefined;
                    }
                    this.storageHandler.savePoseRecording(name, this.poses);
                    this.poses = [];
                };
            case MovementRecorderFunction.StopRecording:
                return () => {
                    if (this.recordPosesHeartbeat) {
                        clearInterval(this.recordPosesHeartbeat);
                        this.recordPosesHeartbeat = undefined;
                    }
                    this.poses = [];
                };
            case MovementRecorderFunction.SavedRecordingNames:
                return () => {
                    return [
                        ...this.quickActionNames,
                        ...this.storageHandler.getRecordingNames(),
                    ];
                };
            case MovementRecorderFunction.DeleteRecording:
                return (recordingID: number) => {
                    if (recordingID < this.quickActionNames.length) return;
                    let recordingNames =
                        this.storageHandler.getRecordingNames();
                    this.storageHandler.deleteRecording(
                        recordingNames[
                            recordingID - this.quickActionNames.length
                        ]
                    );
                };
            case MovementRecorderFunction.DeleteRecordingName:
                return (name: string) => {
                    this.storageHandler.deleteRecording(name);
                };
            case MovementRecorderFunction.LoadRecording:
                return (recordingID: number) => {
                    if (recordingID < this.quickActionNames.length) {
                        FunctionProvider.remoteRobot?.setRobotPose(
                            this.quickActionPoses[recordingID]
                        );
                    } else {
                        let recordingNames =
                            this.storageHandler.getRecordingNames();
                        let recording = this.storageHandler.getRecording(
                            recordingNames[
                                recordingID - this.quickActionNames.length
                            ]
                        );
                        FunctionProvider.remoteRobot?.playbackPoses(recording);
                    }
                };
            case MovementRecorderFunction.LoadRecordingName:
                return (name: string) => {
                    let recording = this.storageHandler.getRecording(name);
                    FunctionProvider.remoteRobot?.playbackPoses(recording);
                };
            case MovementRecorderFunction.Cancel:
                return () => FunctionProvider.remoteRobot?.stopTrajectory();
        }
    }
}
