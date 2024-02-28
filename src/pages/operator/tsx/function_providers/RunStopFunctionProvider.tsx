import { FunctionProvider } from "./FunctionProvider"

export type RunStopFunctions = {
    onClick: () => void
}

export class RunStopFunctionProvider extends FunctionProvider {
    private enabled: boolean; 
    private runStopStateChangeCallback: (enabled: boolean) => void;

    constructor() {
        super()
        this.provideFunctions = this.provideFunctions.bind(this)
        this.updateRunStopState = this.updateRunStopState.bind(this)
    }

    /**
     * Records a callback from the function provider. The callback is called 
     * whenever the runstop state changes.
     * 
     * @param callback callback to function provider
     */
    public setRunStopStateChangeCallback(callback: (enabled: boolean) => void) {
        this.runStopStateChangeCallback = callback;
    }

    public updateRunStopState(enabled: boolean): void {
        this.enabled = enabled
        if (this.runStopStateChangeCallback) this.runStopStateChangeCallback(this.enabled)
    }

    public provideFunctions(): RunStopFunctions {
        return {
            onClick: () => { 
                FunctionProvider.remoteRobot?.setToggle("setRunStop", !this.enabled)
            }
        }
    } 
    
}