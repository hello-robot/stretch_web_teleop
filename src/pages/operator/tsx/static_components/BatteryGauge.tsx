import "operator/css/BatteryGuage.css";
import batteryGauge from "operator/icons/Battery_Gauge.svg";
import { batteryVoltageFunctionProvider } from "..";
import React from "react";

export const BatteryGuage = () => {
    const { getColor } = batteryVoltageFunctionProvider;

    return (
        <div className="batteryGaugeContainer">
            <img src={batteryGauge} className={"batteryGauge " + getColor()} />
            <span>Battery Gauge</span>
        </div>
    );
};
