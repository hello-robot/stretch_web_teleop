import { FunctionProvider } from "./FunctionProvider";
import { HomeTheRobotFunction } from "../layout_components/HomeTheRobot";

export class HomeTheRobotFunctionProvider extends FunctionProvider {

    constructor() {
        super();
        this.provideFunctions = this.provideFunctions.bind(this);
    }

    public provideFunctions(homeTheRobotFunction: HomeTheRobotFunction) {
        switch (homeTheRobotFunction) {
            case HomeTheRobotFunction.Home:
                return () => {
                    FunctionProvider.remoteRobot?.homeTheRobot();
                };
        }
    }
}
