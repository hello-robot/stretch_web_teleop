import { FunctionProvider } from "./FunctionProvider"
import { ArucoMarkersFunction } from "../layout_components/ArucoMarkers"
import { ArucoNavigationState, RobotPose } from "shared/util"
import { StorageHandler } from "../storage_handler/StorageHandler"
import { useState } from "react"

export enum ArucoNavigationResult {
    NAVIGATION_COMPLETE = "Navigation succeeded!", 
    NAVIGATION_FAIL = "Navigation failed, please try again.",     
    MARKER_FAIL = "Could not find Aruco Marker. Stretch may be too far away, try moving it closer to the marker.", 
    POSE_FIND_FAIL = "Could not find saved pose. Please save a pose for this marker.",
    POSE_GET_FAIL = "Could not get pose for this marker. Please try again.",
    POSE_SAVED = "Saved pose for this marker!",
    MARKER_SAVED = "Marker saved!",
    MARKER_DELETE_SUCCESS = "Marker deleted!",        
    MARKER_DELETE_FAIL = "Cannot delete this marker."
}


export class ArucoMarkerFunctionProvider extends FunctionProvider {
    private recordPosesHeartbeat?: number // ReturnType<typeof setInterval>
    private poses: RobotPose[]
    private storageHandler: StorageHandler
    public remoteRobotState: any
    /** 
     * Callback function to update the aruco navigation state in the operator
     */
    private operatorCallback?: (state: ArucoNavigationState) => void = undefined;

    constructor(storageHandler: StorageHandler) {
        super()
        this.provideFunctions = this.provideFunctions.bind(this)
        this.poses = []
        this.storageHandler = storageHandler
        let marker_info = this.storageHandler.getArucoMarkerInfo()
        FunctionProvider.remoteRobot?.setArucoMarkerInfo(marker_info)
        this.remoteRobotState = FunctionProvider.remoteRobot?.state
    }

    public setArucoNavigationState(state: ArucoNavigationState) {
        if (this.operatorCallback) this.operatorCallback(state)
    }

    public provideFunctions(arucoMarkerFunction: ArucoMarkersFunction) {
        switch (arucoMarkerFunction) {
            case ArucoMarkersFunction.SaveMarker:
                return (markerID: string, name: string) => {
                    this.storageHandler.saveMarker(markerID, name)
                    let marker_info = this.storageHandler.getArucoMarkerInfo()
                    FunctionProvider.remoteRobot?.setArucoMarkerInfo(marker_info)
                    FunctionProvider.remoteRobot?.updateArucoMarkersInfo()
                    return { state: ArucoNavigationResult.MARKER_SAVED, alertType: "success" }
                }
            case ArucoMarkersFunction.NavigateToMarker:
                return (markerIndex: number) => {
                    FunctionProvider.remoteRobot?.stopMoveBase()
                    let markerNames = this.storageHandler.getArucoMarkerNames()
                    let name = markerNames[markerIndex]
                    let markerIDs = this.storageHandler.getArucoMarkerIDs()
                    let markerID = markerIDs[markerIndex]
                    let marker_info = this.storageHandler.getArucoMarkerInfo()
                    let pose = marker_info.aruco_marker_info[markerID].pose
                    console.log(pose)
                    if (!pose) return { state: ArucoNavigationResult.POSE_FIND_FAIL, alertType: "error" }
                    FunctionProvider.remoteRobot?.navigateToAruco(name, pose)
                    // let result = await FunctionProvider.remoteRobot?.getMoveBaseState()
                    // if (!result) {
                    //     FunctionProvider.remoteRobot?.stopExecution()
                    //     return { state: ArucoNavigationResult.NAVIGATION_FAIL, alertType: "error" }
                    // }
                    // return { state: result, alertType: result == ArucoNavigationResult.NAVIGATION_COMPLETE ? "success" : "error" };
                }

            case ArucoMarkersFunction.SavedMarkerNames:
                return () => {
                    return this.storageHandler.getArucoMarkerNames()
                }
            case ArucoMarkersFunction.DeleteMarker:
                return (markerIndex: number) => {
                    let markerNames = this.storageHandler.getArucoMarkerNames()
                    if (markerNames[markerIndex] == 'docking_station') return { state: ArucoNavigationResult.MARKER_DELETE_FAIL, alertType: "error" }
                    this.storageHandler.deleteMarker(markerNames[markerIndex])
                    let marker_info = this.storageHandler.getArucoMarkerInfo()
                    FunctionProvider.remoteRobot?.setArucoMarkerInfo(marker_info)
                    FunctionProvider.remoteRobot?.updateArucoMarkersInfo()
                    let poses = this.storageHandler.getMapPoseNames()
                    let mapPoseIdx = poses.indexOf(markerNames[markerIndex])
                    if (mapPoseIdx != -1) this.storageHandler.deleteMapPose(poses[mapPoseIdx])
                    return { state: ArucoNavigationResult.MARKER_DELETE_SUCCESS, alertType: "success" }
                }
            case ArucoMarkersFunction.SaveRelativePose:
                return async (markerIndex: number, saveToMap: boolean) => {
                    let markerNames = this.storageHandler.getArucoMarkerNames()
                    let name = markerNames[markerIndex]
                    let pose = await FunctionProvider.remoteRobot?.getRelativePose(name)
                    let markerIDs = this.storageHandler.getArucoMarkerIDs()
                    let markerID = markerIDs[markerIndex]
                    if (!pose) return { state: ArucoNavigationResult.POSE_GET_FAIL, alertType: "error" }
                    this.storageHandler.saveRelativePose(markerID, pose)
                    if (saveToMap) {
                        let mapPose = FunctionProvider.remoteRobot?.getMapPose()
                        if (!mapPose) return { state: ArucoNavigationResult.POSE_GET_FAIL, alertType: "error" }
                        this.storageHandler.saveMapPose(name, mapPose, "ARUCO")
                    }
                    return { state: ArucoNavigationResult.POSE_SAVED, alertType: "success" }
                }
        }
    }

    /**
     * Sets the local pointer to the operator's callback function, to be called 
     * whenever the aruco navigation state changes.
     * 
     * @param callback operator's callback function to update aruco navigation state
     */
    public setOperatorCallback(callback: (state: ArucoNavigationState) => void) {
        this.operatorCallback = callback;
    }
}