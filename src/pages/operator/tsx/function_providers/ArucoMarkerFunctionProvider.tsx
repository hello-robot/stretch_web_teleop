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
    }

    public provideFunctions(arucoMarkerFunction: ArucoMarkersFunction) {
        switch (arucoMarkerFunction) {
            case ArucoMarkersFunction.SaveMarker:
                return (markerID: string, name: string) => {
                    FunctionProvider.remoteRobot?.updateArucoMarkersInfo()
                    this.storageHandler.saveMarker(markerID, name)
                }
            case ArucoMarkersFunction.NavigateToMarker:
                return (name: string) => {
                    FunctionProvider.remoteRobot?.navigateToMarker(name)
                }
            case ArucoMarkersFunction.SavedMarkerNames:
                return () => {
                    return this.storageHandler.getArucoMarkerNames()
                }
            case ArucoMarkersFunction.DeleteMarker:
                return (markerIndex: number) => {
                    FunctionProvider.remoteRobot?.updateArucoMarkersInfo()
                    let markerNames = this.storageHandler.getArucoMarkerNames()
                    this.storageHandler.deleteMarker(markerNames[markerIndex])
                }
        }
    }
}