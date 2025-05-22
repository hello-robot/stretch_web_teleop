import "operator/css/BatteryGauge.css";
import batteryGauge from "operator/icons/Battery_Gauge.svg";
import { batteryVoltageFunctionProvider } from "..";
import React from "react";

export const BatteryGauge = () => {
    const { getColor } = batteryVoltageFunctionProvider;

    return (
        <div className="batteryGaugeContainer">
            <img src={batteryGauge} className={"batteryGauge " + getColor()} />
            <span>Battery Gauge</span>
        </div>
    );
};
