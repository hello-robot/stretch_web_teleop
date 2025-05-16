import "operator/css/RunStopButton.css";
import { className } from "shared/util";
import { runStopFunctionProvider } from "..";
import { RunStopFunctions } from "../function_providers/RunStopFunctionProvider";
import { CustomizableComponentProps } from "../layout_components/CustomizableComponent";
import React, { useState } from "react";

export const RunStopButton = (props: CustomizableComponentProps) => {
    const functs: RunStopFunctions = runStopFunctionProvider.provideFunctions();
    const [enabled, setEnabled] = useState<boolean>(false);

    runStopFunctionProvider.setRunStopStateChangeCallback(setEnabled);

    return (
        <div className="run-stop-container">
            <button
                onPointerDown={functs.onClick}
                className={className("run-stop-button", { enabled })}
                aria-label={`${!enabled ? 'Enable' : "Disable"} Run Stop`}
            >Run Stop</button>
            <span className="text-run-stop">{enabled ? 'Enabled' : "Disabled"}</span>
        </div>
    );
};
