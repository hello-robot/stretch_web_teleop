import { FunctionProvider } from "./FunctionProvider"
import { ArucoMarkersFunction } from "../layout_components/ArucoMarkers"
import { RobotPose, ValidJoints } from "shared/util"
import { StorageHandler } from "../storage_handler/StorageHandler"

export class ArucoMarkerFunctionProvider extends FunctionProvider {
    private recordPosesHeartbeat?: number // ReturnType<typeof setInterval>
    private poses: RobotPose[]
    private storageHandler: StorageHandler

    constructor(storageHandler: StorageHandler) {
        super()
        this.provideFunctions = this.provideFunctions.bind(this)
        this.poses = []
        this.storageHandler = storageHandler
        let marker_info = this.storageHandler.getArucoMarkerInfo()
        console.log('remote robot: ', FunctionProvider.remoteRobot)
        FunctionProvider.remoteRobot?.setArucoMarkerInfo(marker_info)
    }

    public provideFunctions(arucoMarkerFunction: ArucoMarkersFunction) {
        switch (arucoMarkerFunction) {
            case ArucoMarkersFunction.SaveMarker:
                return (markerID: string, name: string) => {
                    this.storageHandler.saveMarker(markerID, name)
                    let marker_info = this.storageHandler.getArucoMarkerInfo()
                    FunctionProvider.remoteRobot?.setArucoMarkerInfo(marker_info)
                    FunctionProvider.remoteRobot?.updateArucoMarkersInfo()
                }
            case ArucoMarkersFunction.NavigateToMarker:
                return async (markerIndex: number) => {
                    FunctionProvider.remoteRobot?.stopExecution()
                    let markerNames = this.storageHandler.getArucoMarkerNames()
                    let name = markerNames[markerIndex]
                    let markerIDs = this.storageHandler.getArucoMarkerIDs()
                    let markerID = markerIDs[markerIndex]
                    let marker_info = this.storageHandler.getArucoMarkerInfo()
                    let pose = marker_info.aruco_marker_info[markerID].pose
                    console.log(name, pose)
                    FunctionProvider.remoteRobot?.navigateToMarker(name, pose)
                    let result = await FunctionProvider.remoteRobot?.getMoveBaseState()
                    if (!result) {
                        FunctionProvider.remoteRobot?.stopExecution()
                        return 'navigation failed'
                    }
                    return result;
                }

            case ArucoMarkersFunction.SavedMarkerNames:
                return () => {
                    return this.storageHandler.getArucoMarkerNames()
                }
            case ArucoMarkersFunction.DeleteMarker:
                return (markerIndex: number) => {
                    let markerNames = this.storageHandler.getArucoMarkerNames()
                    this.storageHandler.deleteMarker(markerNames[markerIndex])
                    let marker_info = this.storageHandler.getArucoMarkerInfo()
                    FunctionProvider.remoteRobot?.setArucoMarkerInfo(marker_info)
                    FunctionProvider.remoteRobot?.updateArucoMarkersInfo()
                }
            case ArucoMarkersFunction.SaveRelativePose:
                return async (markerIndex: number) => {
                    let markerNames = this.storageHandler.getArucoMarkerNames()
                    let name = markerNames[markerIndex]
                    let pose = await FunctionProvider.remoteRobot?.getRelativePose(name)
                    let markerIDs = this.storageHandler.getArucoMarkerIDs()
                    let markerID = markerIDs[markerIndex]
                    if (!pose) return;
                    this.storageHandler.saveRelativePose(markerID, pose)
                }
        }
    }
}