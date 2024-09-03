import { ROSPose } from "shared/util";
import { MapFunction } from "../layout_components/Map";
import { FunctionProvider } from "./FunctionProvider";
import { occupancyGrid } from "operator/tsx/index";

export class MapFunctionProvider extends FunctionProvider {
    constructor() {
        super();
        this.provideFunctions = this.provideFunctions.bind(this);
        FunctionProvider.remoteRobot?.getOccupancyGrid("getOccupancyGrid");
    }

    public provideFunctions(mapFunction: MapFunction) {
        switch (mapFunction) {
            case MapFunction.GetMap:
                return occupancyGrid;
            case MapFunction.GetPose:
                return () => {
                    return FunctionProvider.remoteRobot?.getMapPose();
                };
            case MapFunction.MoveBase:
                return (pose: ROSPose) => {
                    // FunctionProvider.remoteRobot?.stopExecution()
                    FunctionProvider.remoteRobot?.moveBase(pose);
                };
            case MapFunction.GoalReached:
                return () => {
                    let goalReached =
                        FunctionProvider.remoteRobot?.isGoalReached();
                    if (goalReached) {
                        FunctionProvider.remoteRobot?.setGoalReached(false);
                        return true;
                    }
                    return false;
                };
        }
    }
}
