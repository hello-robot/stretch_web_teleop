import { MoveBaseState, ROSPose, waitUntil } from "shared/util"
import { StorageHandler } from "../storage_handler/StorageHandler"
import { FunctionProvider } from "./FunctionProvider"
import { resolve } from "path"

export enum UnderMapButton {
    SelectGoal,
    DeleteGoal,
    CancelGoal,
    SaveGoal,
    LoadGoal,
    GetPose,
    GetSavedPoseNames,
    GetSavedPoseTypes,
    GetSavedPoses,
    NavigateToAruco,
    GoalReached
}

export class UnderMapFunctionProvider extends FunctionProvider {
    private selectGoal: boolean
    private storageHandler: StorageHandler
    private navigationSuccess?: boolean
    /** 
     * Callback function to update the move base state in the operator
     */
    private operatorCallback?: (state: MoveBaseState) => void = undefined;

    constructor(storageHandler: StorageHandler) {
        super()
        this.provideFunctions = this.provideFunctions.bind(this)
        this.selectGoal = false
        this.storageHandler = storageHandler
    }

    public setMoveBaseState(state: MoveBaseState) {
        if (state.alert_type == "success") this.navigationSuccess = true
        if (this.operatorCallback) this.operatorCallback(state)
    }

    public provideFunctions(button: UnderMapButton) {
        switch (button) {
            case UnderMapButton.SelectGoal:
                return (toggle: boolean) => {
                        this.selectGoal = toggle
                    } 
            case UnderMapButton.CancelGoal:
                return () => FunctionProvider.remoteRobot?.stopMoveBase() 
            case UnderMapButton.DeleteGoal:        
                return (idx: number) => {
                    let poses = this.storageHandler.getMapPoseNames()
                    this.storageHandler.deleteMapPose(poses[idx])
                }
            case UnderMapButton.SaveGoal:
                return (name: string) => {
                    let pose = FunctionProvider.remoteRobot?.getMapPose()
                    if (!pose) throw 'Cannot save undefined map pose!'
                    this.storageHandler.saveMapPose(name, pose, "MAP")
                }
            case UnderMapButton.LoadGoal:
                return (idx: number) => {
                    this.navigationSuccess = undefined
                    let poses = this.storageHandler.getMapPoseNames()
                    let pose = this.storageHandler.getMapPose(poses[idx])
                    let rosPose = {
                        position: {
                            x: pose.translation.x,
                            y: pose.translation.y,
                            z: 0
                        },
                        orientation: {
                            x: pose.rotation.x,
                            y: pose.rotation.y,
                            z: pose.rotation.z,
                            w: pose.rotation.w,
                        }
                    } as ROSPose
                    FunctionProvider.remoteRobot?.moveBase(rosPose)
                    return pose.translation
                }
            case UnderMapButton.NavigateToAruco:
                return (idx: number) => {
                    let poseTypes = this.storageHandler.getMapPoseTypes()
                    if (poseTypes[idx] != "ARUCO") return
                    waitUntil(() => this.navigationSuccess != undefined, 120000).then(() => {
                        // If navigation failed don't try navigating to marker
                        if (!this.navigationSuccess) {
                            this.navigationSuccess = undefined
                            return
                        }

                        this.navigationSuccess = undefined
                        let poseNames = this.storageHandler.getMapPoseNames()
                        let name = poseNames[idx]
                        let markerNames = this.storageHandler.getArucoMarkerNames()
                        let markerIndex = markerNames.indexOf(name)
                        if (markerIndex == -1) {
                            this.setMoveBaseState({ state: "Cannot find Aruco Marker", alert_type: "error" })
                            return
                        }
                        let markerIDs = this.storageHandler.getArucoMarkerIDs()
                        let markerID = markerIDs[markerIndex]
                        let marker_info = this.storageHandler.getArucoMarkerInfo()
                        let pose = marker_info.aruco_marker_info[markerID].pose
                        if (!pose) {
                            this.setMoveBaseState({ state: "Cannot find Aruco Marker", alert_type: "error" })
                            return
                        }
                        FunctionProvider.remoteRobot?.navigateToAruco(name, pose)
                    })
                } 
            case UnderMapButton.GetPose:
                return () => { return FunctionProvider.remoteRobot?.getMapPose() }
            case UnderMapButton.GetSavedPoseNames:
                return () => { return this.storageHandler.getMapPoseNames() }
            case UnderMapButton.GetSavedPoseTypes:
                return () => { return this.storageHandler.getMapPoseTypes() }
            case UnderMapButton.GetSavedPoses:
                return () => { return this.storageHandler.getMapPoses() }
            case UnderMapButton.GoalReached:
                return () => { 
                    const promise = new Promise((resolve, reject) => { 
                        let interval = setInterval(() => { 
                            let goalReached = FunctionProvider.remoteRobot?.isGoalReached()
                            if (goalReached) {
                                clearInterval(interval)
                                resolve(true)
                            }
                        });
                    });
                    return promise; 
                }
            default:
                throw Error(`Cannot get function for unknown UnderMapButton ${button}`)
        }
    }

    /**
     * Sets the local pointer to the operator's callback function, to be called 
     * whenever the move base state changes.
     * 
     * @param callback operator's callback function to update aruco navigation state
     */
    public setOperatorCallback(callback: (state: MoveBaseState) => void) {
        this.operatorCallback = callback;
    }
}