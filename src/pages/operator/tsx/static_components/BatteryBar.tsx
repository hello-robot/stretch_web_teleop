import "operator/css/BatteryBar.css"
import batteryBar from "operator/icons/battery.svg"
import React, { useState } from "react";
import { batteryVoltageFunctionProvider } from "..";

export const BatteryBar = () => {
    const [numBars, setNumBars] = useState(5);

    // TODO: Implement the logic to determine the number of bars based on battery voltage
    // batteryVoltageFunctionProvider.setVoltageBarChangeCallback(setNumBars);

    return (
        <div className="batteryBarContainer">
            <img src={batteryBar} className="batteryBar" />
            <div className="barsContainer">
                {numBars > 4 ? <div className="bar"></div> : <></>}
                {numBars > 3 ? <div className="bar"></div> : <></>}
                {numBars > 2 ? <div className="bar"></div> : <></>}
                {numBars > 1 ? <div className="bar"></div> : <></>}
                <div className="bar"></div>
            </div>
        </div>
    )
}
