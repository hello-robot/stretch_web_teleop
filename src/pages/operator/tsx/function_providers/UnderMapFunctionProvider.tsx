import { ActionState, ROSPose, waitUntil } from "shared/util";
import { StorageHandler } from "../storage_handler/StorageHandler";
import { FunctionProvider } from "./FunctionProvider";
import { resolve } from "path";
import ROSLIB from "roslib";

export enum UnderMapButton {
    SelectGoal,
    DeleteGoal,
    DeleteMapPose,
    CancelGoal,
    SaveGoal,
    LoadGoal,
    NavigateToPose,
    GetPose,
    GetSavedPoseNames,
    GetSavedPoseTypes,
    GetSavedPoses,
    GoalReached,
    RenamePose,
}

export class UnderMapFunctionProvider extends FunctionProvider {
    private selectGoal: boolean;
    private storageHandler: StorageHandler;
    private navigationSuccess?: boolean;
    private mapPoseCallback?: (pose: ROSLIB.Vector3) => void = undefined;
    private operatorCallback?: (state: ActionState) => void = undefined;

    constructor(storageHandler: StorageHandler) {
        super();
        this.provideFunctions = this.provideFunctions.bind(this);
        this.selectGoal = true;
        this.storageHandler = storageHandler;
    }

    public setMoveBaseState(state: ActionState) {
        if (state.alert_type == "success") this.navigationSuccess = true;
        if (this.operatorCallback) this.operatorCallback(state);
    }

    public provideFunctions(button: UnderMapButton) {
        switch (button) {
            case UnderMapButton.SelectGoal:
                return (toggle: boolean) => {
                    this.selectGoal = toggle;
                };
            case UnderMapButton.CancelGoal:
                return () => FunctionProvider.remoteRobot?.stopMoveBase();

            case UnderMapButton.DeleteGoal:
                return (idx: number) => {
                    let poses = this.storageHandler.getMapPoseNames();
                    this.storageHandler.deleteMapPose(poses[idx]);
                };
            case UnderMapButton.DeleteMapPose:
                return (poseName: string) => {
                    let poses = this.storageHandler.getMapPoseNames();
                    this.storageHandler.deleteMapPose(poseName);
                };

            case UnderMapButton.SaveGoal:
                return (name: string) => {
                    let pose = FunctionProvider.remoteRobot?.getMapPose();
                    if (!pose) throw "Cannot save undefined map pose!";
                    this.storageHandler.saveMapPose(name, pose, "MAP");
                };
            case UnderMapButton.LoadGoal:
                return (poseName: string) => {
                    this.navigationSuccess = undefined;
                    let pose = this.storageHandler.getMapPose(poseName);
                    return pose;
                };
            case UnderMapButton.NavigateToPose:
                return (pose: ROSLIB.Transform) => {
                    let rosPose = {
                        position: {
                            x: pose.translation.x,
                            y: pose.translation.y,
                            z: 0,
                        },
                        orientation: {
                            x: pose.rotation.x,
                            y: pose.rotation.y,
                            z: pose.rotation.z,
                            w: pose.rotation.w,
                        },
                    } as ROSPose;
                    FunctionProvider.remoteRobot?.moveBase(rosPose);
                }
            case UnderMapButton.GetPose:
                return () => {
                    return FunctionProvider.remoteRobot?.getMapPose();
                };
            case UnderMapButton.GetSavedPoseNames:
                return () => {
                    return this.storageHandler.getMapPoseNames();
                };
            case UnderMapButton.GetSavedPoseTypes:
                return () => {
                    return this.storageHandler.getMapPoseTypes();
                };
            case UnderMapButton.GetSavedPoses:
                return () => {
                    return this.storageHandler.getMapPoses();
                };
            case UnderMapButton.GoalReached:
                return () => {
                    const promise = new Promise((resolve, reject) => {
                        let interval = setInterval(() => {
                            let goalReached =
                                FunctionProvider.remoteRobot?.isGoalReached();
                            if (goalReached) {
                                clearInterval(interval);
                                resolve(true);
                            }
                        });
                    });
                    return promise;
                };
            case UnderMapButton.RenamePose:
                return (poseNameOld: string, poseNameNew: string) => {
                    return this.storageHandler.renamePose(poseNameOld, poseNameNew);
                };
            default:
                throw Error(
                    `Cannot get function for unknown UnderMapButton ${button}`,
                );
        }
    }

    /**
     * Sets the local pointer to the operator's callback function, to be called
     * whenever the move base state changes.
     *
     * @param callback operator's callback function to update aruco navigation state
     */
    public setOperatorCallback(callback: (state: ActionState) => void) {
        this.operatorCallback = callback;
    }

    /**
     * Registers a callback function to be invoked when a map pose is set.
     *
     * @param callback - A function that receives a `ROSLIB.Vector3` object representing the pose on the map.
     */
    public setMapPoseCallback(callback: (pose: ROSLIB.Vector3) => void) {
        this.mapPoseCallback = callback;
    }
}
