import "operator/css/RunStopButton.css"
import { className } from "shared/util";
import { runStopFunctionProvider } from "..";
import { RunStopFunctions } from "../function_providers/RunStopFunctionProvider";
import { CustomizableComponentProps } from "../layout_components/CustomizableComponent";
import runStopButton from "operator/icons/button.svg"
import React from "react";

export const RunStopButton = (props: CustomizableComponentProps) => {
    const functs: RunStopFunctions = runStopFunctionProvider.provideFunctions();
    let enabled = runStopFunctionProvider.enabled;

    React.useEffect(() => {
        enabled = runStopFunctionProvider.enabled;
    }, [runStopFunctionProvider.enabled])

    return (
        <div className="runStopContainer">
            <img 
                src={runStopButton}
                onClick={functs.onClick}
                className={className("run-stop-button", {enabled})}
            />
            {enabled ? <span>Run Stop: Enabled</span> : <span>Run Stop: Disabled</span>}
        </div>
    )
}