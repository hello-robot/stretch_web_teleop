import "operator/css/RunStop.css";
import { className } from "shared/util";
import { runStopFunctionProvider } from "..";
import { RunStopFunctions } from "../function_providers/RunStopFunctionProvider";
import { CustomizableComponentProps } from "../layout_components/CustomizableComponent";
import runStopButton from "operator/icons/button.svg";
import React, { useState } from "react";

export const RunStop = () => {
    const functs: RunStopFunctions = runStopFunctionProvider.provideFunctions();
    const [enabled, setEnabled] = useState<boolean>(false);

    runStopFunctionProvider.setRunStopStateChangeCallback(setEnabled);

    return (
        <div className="runStopContainer">
            <img
                src={runStopButton}
                onClick={functs.onClick}
                className={className("run-stop-button", { enabled })}
            />
            {enabled ? (
                <span>Run Stop: Enabled</span>
            ) : (
                <span>Run Stop: Disabled</span>
            )}
        </div>
    );
};
