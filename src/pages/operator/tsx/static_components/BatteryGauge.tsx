import "operator/css/BatteryGuage.css"
import { useState } from "react";
import { className } from "shared/util";
import { CustomizableComponentProps } from "../layout_components/CustomizableComponent";
import batteryGauge from "operator/icons/Battery_Gauge.svg"
import { batteryVoltageFunctionProvider } from "..";
import React from "react";
import { BatteryVoltageFunctions } from "../function_providers/BatteryVoltageFunctionProvider";

export const BatteryGuage = (props: CustomizableComponentProps) => {
    const functs: BatteryVoltageFunctions = batteryVoltageFunctionProvider.provideFunctions();
    const [color, setColor] = useState(functs.getColor())

    React.useEffect(() => {
        setColor(functs.getColor())
    }, [batteryVoltageFunctionProvider.voltage])
    
    return (
        <div className="batteryGaugeContainer">
            <img src={batteryGauge} className={"batteryGauge " + color }/>
            {/* <div className="barsContainer">
                <div className="bar"></div>
                <div className="bar"></div>
                <div className="bar"></div>
                <div className="bar"></div>
                <div className="bar"></div>
            </div> */}
            <span>Battery Gauge</span>
        </div>
    )
}