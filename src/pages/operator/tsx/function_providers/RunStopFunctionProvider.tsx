import { FunctionProvider } from "./FunctionProvider"

export type RunStopFunctions = {
    onClick: () => void
}

export class RunStopFunctionProvider extends FunctionProvider {
    public enabled: boolean; 

    constructor() {
        super()
        this.provideFunctions = this.provideFunctions.bind(this)
        this.updateRunStopState = this.updateRunStopState.bind(this)
    }

    public updateRunStopState(enabled: boolean): void {
        this.enabled = enabled
    }

    public provideFunctions(): RunStopFunctions {
        return {
            onClick: () => { 
                FunctionProvider.remoteRobot?.setToggle("setRunStop", !this.enabled)
            }
        }
    } 
    
}