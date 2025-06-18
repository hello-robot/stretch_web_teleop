import "operator/css/BatteryGuage.css";
import { useState } from "react";
import { className } from "shared/util";
import { CustomizableComponentProps } from "../layout_components/CustomizableComponent";
import batteryGauge from "operator/icons/Battery_Gauge.svg";
import { batteryVoltageFunctionProvider } from "..";
import React from "react";
import { BatteryVoltageFunctions } from "../function_providers/BatteryVoltageFunctionProvider";

export default (props: CustomizableComponentProps) => {
    const [color, setColor] = useState("green");

    batteryVoltageFunctionProvider.setVoltageChangeCallback(setColor);

    return (
        <div className="batteryGaugeContainer">
            <img src={batteryGauge} className={"batteryGauge " + color} />
            {/* <div className="barsContainer">
                <div className="bar"></div>
                <div className="bar"></div>
                <div className="bar"></div>
                <div className="bar"></div>
                <div className="bar"></div>
            </div> */}
            <span>Battery Gauge</span>
        </div>
    );
};
