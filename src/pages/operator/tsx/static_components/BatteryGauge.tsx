import { memo } from "react";

import "operator/css/BatteryGauge.css";
// import batteryGauge from "operator/icons/Battery_Gauge.svg";
import React from "react";

export const BatteryGauge = memo(() => {
    // const batteryColors = [
    //     'green',
    //     'yellow-green',
    //     'yellow',
    //     'orange',
    //     'orange-red',
    //     'red',
    // ]
    // const fakeBatteryColor = batteryColors[Math.floor(Math.random() * batteryColors.length)]; // Simulating battery level for demo purposes

    return (
        <div className="battery-gauge-container">
            {/* <img src={batteryGauge} className={"battery-gauge-image " + fakeBatteryColor} aria-disabled="true" /> */}
        </div>
    );
});

