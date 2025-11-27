import "operator/css/RunStop.css";
import { className } from "shared/util";
import { runStopFunctionProvider } from "..";
import { RunStopFunctions } from "../function_providers/RunStopFunctionProvider";
import React, { useState } from "react";

export const RunStop = () => {
    const functs: RunStopFunctions = runStopFunctionProvider.provideFunctions();
    const [enabled, setEnabled] = useState<boolean>(false);

    runStopFunctionProvider.setRunStopStateChangeCallback(setEnabled);

    return (
        <div className="run-stop-container">
            <button
                onPointerDown={functs.onPointerDown}
                className={className("run-stop-button", { enabled })}
                aria-label={`${!enabled ? 'Enable' : "Disable"} Run Stop`}
            >Run Stop</button>
            {/* <span className="text-run-stop">{enabled ? 'Enabled' : "Disabled"}</span> */}
        </div>
    );
};
