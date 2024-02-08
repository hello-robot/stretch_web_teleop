import { FunctionProvider } from "./FunctionProvider"

export type RunStopFunctions = {
    onClick: () => void
    isEnabled: () => boolean
}

export class RunStopFunctionProvider extends FunctionProvider {
    constructor() {
        super()
        this.provideFunctions = this.provideFunctions.bind(this)
    }

    public provideFunctions(): RunStopFunctions {
        return {
            onClick: () => { 
                let isEnabled = FunctionProvider.remoteRobot?.getIsRunStopped()
                FunctionProvider.remoteRobot?.setToggle("setRunStop", !isEnabled)
            },
            isEnabled: () => {
                return FunctionProvider.remoteRobot?.getIsRunStopped()
            }
        }
    } 
    
}