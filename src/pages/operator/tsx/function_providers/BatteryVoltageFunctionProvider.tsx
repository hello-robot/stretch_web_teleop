import { FunctionProvider } from "./FunctionProvider"

export type BatteryVoltageFunctions = {
    getColor: () => string
}

export class BatteryVoltageFunctionProvider extends FunctionProvider {
    private voltage: number = 0.0;

    /** 
    * Callback function to update the battery voltage in the operator
    */
    private operatorCallback?: (voltage: number) => void = undefined;

    constructor() {
        super()
        this.provideFunctions = this.provideFunctions.bind(this)
        this.updateVoltage = this.updateVoltage.bind(this)
    }

    public updateVoltage(voltage: number): void {
        this.voltage = voltage
        if (this.operatorCallback) this.operatorCallback(voltage)
    }

     /**
     * Sets the local pointer to the operator's callback function, to be called 
     * whenever the battery voltage updates.
     * 
     * @param callback operator's callback function to update the battery voltage
     */
      public setOperatorCallback(callback: (voltage: number) => void) {
        this.operatorCallback = callback;
    }

    public provideFunctions(): BatteryVoltageFunctions {
        return {
            getColor: () => {
                let vbat_min = 10.0
                let vbat_max = 12.0
                let dv = (vbat_max - vbat_min)/4.0
                
                if (!this.voltage) return 'red' // throw 'Cannot retrieve battery voltage'

                if (this.voltage < vbat_min) {
                    return 'red' // [64, 0, 0]
                } else if (this.voltage >= vbat_min && this.voltage < (vbat_min + dv)) {
                    return 'orange-red' // [64, 32, 0]
                } else if (this.voltage >= (vbat_min + dv) && this.voltage < (vbat_min + (2 * dv))) {
                    return 'orange-yellow' // [64, 64, 0]
                } else if (this.voltage >= (vbat_min + (2 * dv)) && this.voltage < (vbat_min + (3 * dv))) {
                    return 'yellow' // [64, 64, 0]
                } else if (this.voltage >= (vbat_min + (3 * dv)) && this.voltage < vbat_max) {
                    return 'yellow-green' // [32, 64, 0]
                } else { 
                    return 'green' // [0, 64, 0]
                }
            }
        }
    } 
    
}