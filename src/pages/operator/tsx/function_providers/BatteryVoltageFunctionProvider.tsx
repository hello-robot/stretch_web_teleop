import { FunctionProvider } from "./FunctionProvider";

export type BatteryVoltageFunctions = {
    getColor: () => string;
    getBars: () => number;
};

export class BatteryVoltageFunctionProvider extends FunctionProvider {
    public voltage: number = 0.0;
    public voltageColorChangeCallback: (color: string) => void;
    public voltageBarChangeCallback: (bars: number) => void;

    constructor() {
        super();
        this.updateVoltage = this.updateVoltage.bind(this);
    }

    public updateVoltage(voltage: number): void {
        this.voltage = voltage;
        if (this.voltageColorChangeCallback)
            this.voltageColorChangeCallback(this.getColor());
        if (this.voltageBarChangeCallback)
            this.voltageBarChangeCallback(this.getBars())
    }

    private getBars() {
        let vbat_min = 10.0;
        let vbat_max = 12.0;
        let dv = (vbat_max - vbat_min) / 4.0;

        if (!this.voltage) return "red"; // throw 'Cannot retrieve battery voltage'

        if (this.voltage < vbat_min) {
            return 0; // [64, 0, 0]
        } else if (this.voltage >= vbat_min && this.voltage < vbat_min + dv) {
            return 1; // [64, 32, 0]
        } else if (
            this.voltage >= vbat_min + dv &&
            this.voltage < vbat_min + 2 * dv
        ) {
            return 2; // [64, 64, 0]
        } else if (
            this.voltage >= vbat_min + 2 * dv &&
            this.voltage < vbat_min + 3 * dv
        ) {
            return 3; // [64, 64, 0]
        } else if (
            this.voltage >= vbat_min + 3 * dv &&
            this.voltage < vbat_max
        ) {
            return 4; // [32, 64, 0]
        } else {
            return 5; // [0, 64, 0]
        }
    }

    private getColor() {
        let vbat_min = 10.0;
        let vbat_max = 12.0;
        let dv = (vbat_max - vbat_min) / 4.0;

        if (!this.voltage) return "red"; // throw 'Cannot retrieve battery voltage'

        if (this.voltage < vbat_min) {
            return "red"; // [64, 0, 0]
        } else if (this.voltage >= vbat_min && this.voltage < vbat_min + dv) {
            return "orange-red"; // [64, 32, 0]
        } else if (
            this.voltage >= vbat_min + dv &&
            this.voltage < vbat_min + 2 * dv
        ) {
            return "orange-yellow"; // [64, 64, 0]
        } else if (
            this.voltage >= vbat_min + 2 * dv &&
            this.voltage < vbat_min + 3 * dv
        ) {
            return "yellow"; // [64, 64, 0]
        } else if (
            this.voltage >= vbat_min + 3 * dv &&
            this.voltage < vbat_max
        ) {
            return "yellow-green"; // [32, 64, 0]
        } else {
            return "green"; // [0, 64, 0]
        }
    }

    /**
     * Records a callback from the function provider. The callback is called
     * whenever the battery voltage changes.
     *
     * @param callback callback to function provider
     */
    public setVoltageChangeCallback(callback: (color: string) => void) {
        this.voltageColorChangeCallback = callback;
    }

    public setVoltageBarChangeCallback(callback: (bars: number) => void) {
        this.voltageBarChangeCallback = callback;
    }
}
