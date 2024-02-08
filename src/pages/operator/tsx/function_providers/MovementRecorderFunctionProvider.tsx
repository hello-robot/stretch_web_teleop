import { FunctionProvider } from "./FunctionProvider"
import { MovementRecorderFunctions, MovementRecorderFunction } from "../layout_components/MovementRecorder"
import { RobotPose, ValidJoints } from "shared/util"
import { StorageHandler } from "../storage_handler/StorageHandler"

export class MovementRecorderFunctionProvider extends FunctionProvider {
    private recordPosesHeartbeat?: number // ReturnType<typeof setInterval>
    private poses: RobotPose[]
    private storageHandler: StorageHandler

    constructor(storageHandler: StorageHandler) {
        super()
        this.provideFunctions = this.provideFunctions.bind(this)
        this.poses = []
        this.storageHandler = storageHandler
    }

    public provideFunctions(poseRecordFunction: MovementRecorderFunction) {
        switch (poseRecordFunction) {
            case MovementRecorderFunction.Record:
                return () => { 
                    let lastJoint: ValidJoints | undefined;
                    this.recordPosesHeartbeat = window.setInterval(() => {
                        const currentPose: RobotPose = FunctionProvider.remoteRobot!.sensors.getRobotPose(
                            true, true, true 
                        )
                        const lastPose = this.poses.length == 0 ? undefined : this.poses[this.poses.length - 1]
                        if (lastPose) {
                            Object.keys(currentPose).map((key, index) => {
                                if (Math.abs(currentPose[key as ValidJoints]! - lastPose[key as ValidJoints]!) > 0.025) {
                                    if (!lastJoint || lastJoint != key) {
                                        lastJoint = key as ValidJoints
                                        this.poses.push(currentPose)
                                        return;
                                    } else {
                                        this.poses[this.poses.length - 1][lastJoint] = currentPose[key as ValidJoints]
                                    }
                                }
                            })
                        } else {
                            this.poses.push(currentPose)
                        }
                    }, 50)
                }
            case MovementRecorderFunction.SaveRecording:
                return (name: string) => { 
                        if (this.recordPosesHeartbeat) {
                            clearInterval(this.recordPosesHeartbeat)
                            this.recordPosesHeartbeat = undefined
                        }
                        this.storageHandler.savePoseRecording(name, this.poses)
                        this.poses = []
                    }
            case MovementRecorderFunction.StopRecording:
                return () => {
                    if (this.recordPosesHeartbeat) {
                        clearInterval(this.recordPosesHeartbeat)
                        this.recordPosesHeartbeat = undefined
                    }
                    this.poses = []
                }
            case MovementRecorderFunction.SavedRecordingNames:
                return () => { 
                        return this.storageHandler.getRecordingNames()
                    }
            case MovementRecorderFunction.DeleteRecording:
                return (recordingID: number) => { 
                        let recordingNames = this.storageHandler.getRecordingNames()
                        this.storageHandler.deleteRecording(recordingNames[recordingID])
                    }
            case MovementRecorderFunction.DeleteRecordingName:
                return (name: string) => { 
                        this.storageHandler.deleteRecording(name)
                    }
            case MovementRecorderFunction.LoadRecording:
                return (recordingID: number) => { 
                        let recordingNames = this.storageHandler.getRecordingNames()
                        let recording = this.storageHandler.getRecording(recordingNames[recordingID])
                        FunctionProvider.remoteRobot?.playbackPoses(recording)
                    }
            case MovementRecorderFunction.LoadRecordingName:
                return (name: string) => { 
                        let recording = this.storageHandler.getRecording(name)
                        FunctionProvider.remoteRobot?.playbackPoses(recording)
                    }
            case MovementRecorderFunction.Cancel:
                return () => FunctionProvider.remoteRobot?.stopTrajectory()
        }
    }
}