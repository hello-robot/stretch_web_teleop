import { ROSPose } from "shared/util"
import { MapFunction } from "../layout_components/Map"
import { FunctionProvider } from "./FunctionProvider"
import { occupancyGrid } from "operator/tsx/index"

export class MapFunctionProvider extends FunctionProvider {
    constructor() {
        super()
        this.provideFunctions = this.provideFunctions.bind(this)
    }

    public provideFunctions(mapFunction: MapFunction) {
        switch (mapFunction) {
            case MapFunction.GetMap:
                FunctionProvider.remoteRobot?.getOccupancyGrid("getOccupancyGrid")
                console.log(occupancyGrid)
                return occupancyGrid
            case MapFunction.GetPose:
                return () => { return FunctionProvider.remoteRobot?.getMapPose() }
            case MapFunction.MoveBase:
                return (pose: ROSPose) => FunctionProvider.remoteRobot?.moveBase(pose)
        }
        
    }
}