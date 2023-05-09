import React from "react";
import { VelocityControl, DEFAULT_SPEED } from "operator/tsx/velocitycontrol"
import { LayoutArea } from "./layoutarea";
import { ActionMode, ActionModeButton } from "operator/tsx/actionmodebutton"
import "operator/css/operator.css"
import { mockFunctionProvider } from "./functionprovider";
import { UserInteractionFunction } from "./buttonpads";
import { CustomizeButton } from "./customizebutton";
import { Sidebar } from "./sidebar";

/** Operator interface webpage */
export const Operator = () => {
    /** Speed of the robot. */
    let speed = DEFAULT_SPEED;
    let actionMode = ActionMode.StepActions;
    const [customizing, setCustomizing] = React.useState(false);

    return (
        <div id="operator">
            <div id="operator-header">
                <ActionModeButton
                    default={actionMode}
                    onChange={(newAm) => (actionMode = newAm)}
                />
                <VelocityControl
                    initialSpeed={speed}
                    onChange={(newSpeed) => (speed = newSpeed)}
                />
                <CustomizeButton
                    customizing={customizing}
                    onClick={() => setCustomizing(!customizing)}
                />
            </div>
            <div id="operator-body">
                <LayoutArea
                    customizing={customizing}
                    functionProvider={(bf: UserInteractionFunction) => mockFunctionProvider(actionMode, bf)}
                />
                <Sidebar hidden={!customizing} />
            </div>
        </div>
    )
}