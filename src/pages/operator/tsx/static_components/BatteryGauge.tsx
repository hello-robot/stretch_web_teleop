import "operator/css/BatteryGuage.css"
import { useState } from "react";
import { className } from "shared/util";
import { CustomizableComponentProps } from "../layout_components/CustomizableComponent";
import batteryGauge from "operator/icons/battery.svg"

export const BatteryGuage = (props: CustomizableComponentProps) => {
    return (
        <div className="batteryGaugeContainer">
            <img src={batteryGauge} className="batteryGauge"/>
            <div className="barsContainer">
                <div className="bar"></div>
                <div className="bar"></div>
                <div className="bar"></div>
                <div className="bar"></div>
                <div className="bar"></div>
            </div>
            <span>100%</span>
        </div>
    )
}