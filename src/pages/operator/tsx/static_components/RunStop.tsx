import "operator/css/RunStopButton.css"
import { useState } from "react";
import { className } from "shared/util";
import { runStopFunctionProvider } from "..";
import { RunStopFunctions } from "../function_providers/RunStopFunctionProvider";
import { CustomizableComponentProps } from "../layout_components/CustomizableComponent";

export const RunStopButton = (props: CustomizableComponentProps) => {
    const functs: RunStopFunctions = runStopFunctionProvider.provideFunctions();
    const [enabled, setEnabled] = useState(functs.isEnabled())

    return (
        <button
            onClick={() => {
                setEnabled(!enabled)
                functs.onClick()
            }}
            className={className("run-stop-button", { enabled })}
        >
        </button>
    )
}