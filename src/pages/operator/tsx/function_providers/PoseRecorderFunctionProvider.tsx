import { FunctionProvider } from "./FunctionProvider"
import { PoseRecorderFunctions, PoseRecorderFunction } from "../layout_components/PoseRecorder"
import { RobotPose, ValidJoints } from "shared/util"
import { StorageHandler } from "../storage_handler/StorageHandler"

export class PoseRecorderFunctionProvider extends FunctionProvider {
    private recordPosesHeartbeat?: number // ReturnType<typeof setInterval>
    private poses: RobotPose[]
    private storageHandler: StorageHandler

    constructor(storageHandler: StorageHandler) {
        super()
        this.provideFunctions = this.provideFunctions.bind(this)
        this.poses = []
        this.storageHandler = storageHandler
    }

    public provideFunctions(poseRecordFunction: PoseRecorderFunction) {
        switch (poseRecordFunction) {
            case PoseRecorderFunction.Record:
                return () => { 
                        this.recordPosesHeartbeat = window.setInterval(() => {
                            const currentPose: RobotPose = FunctionProvider.remoteRobot!.sensors.getRobotPose(
                                true, true, true 
                            )
                            const lastPose = this.poses.length == 0 ? undefined : this.poses[this.poses.length - 1]
                            let diff: boolean = false
                            if (!lastPose) {
                                diff = true
                            } else {
                                Object.keys(currentPose).map((key, index) => {
                                    if (Math.abs(currentPose[key as ValidJoints]! - lastPose[key as ValidJoints]!) > 0.05) {
                                        diff = true
                                    }
                                })
                            }
                            if (diff) {
                                this.poses.push(currentPose)
                            }
                        }, 1000)
                    }
            case PoseRecorderFunction.SaveRecording:
                return (name: string) => { 
                        if (this.recordPosesHeartbeat) {
                            clearInterval(this.recordPosesHeartbeat)
                            this.recordPosesHeartbeat = undefined
                        }
                        this.storageHandler.savePoseRecording(name, this.poses)
                        this.poses = []
                    }
            case PoseRecorderFunction.SavedRecordingNames:
                return () => { 
                        return this.storageHandler.getRecordingNames()
                    }
            case PoseRecorderFunction.DeleteRecording:
                return (recordingID: number) => { 
                        let recordingNames = this.storageHandler.getRecordingNames()
                        this.storageHandler.deleteRecording(recordingNames[recordingID])
                    }
            case PoseRecorderFunction.LoadRecording:
                return (recordingID: number) => { 
                        let recordingNames = this.storageHandler.getRecordingNames()
                        let recording = this.storageHandler.getRecording(recordingNames[recordingID])
                        FunctionProvider.remoteRobot?.playbackPoses(recording)
                    }
        }
    }
}