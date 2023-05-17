import React from "react";
import { VelocityControl, DEFAULT_VELOCITY_SCALE } from "operator/tsx/velocitycontrol"
import { LayoutArea } from "./layoutarea";
import { ActionMode, ActionModeButton } from "operator/tsx/actionmodebutton"
import "operator/css/operator.css"
import { ButtonFunctionProvider } from "./functionprovider";
import { UserInteractionFunction } from "./buttonpads";
import { CustomizeButton } from "./customizebutton";
import { Sidebar } from "./sidebar";
import { SharedState } from "./customizablecomponent";
import { ComponentDefinition } from "./componentdefinitions";
import { DEFAULT_LAYOUT } from "./defaultlayout";
import { RemoteRobot } from "robot/tsx/remoterobot";
import { RemoteStream } from "utils/util";
import { addToLayout, moveInLayout } from "utils/layouthelpers";

/** Operator interface webpage */
export const Operator = (props: {
    remoteRobot: RemoteRobot,
    remoteStreams: Map<string, RemoteStream>
}) => {
    /** Speed of the robot. */
    let velocityScale = DEFAULT_VELOCITY_SCALE;
    const [actionMode, setActionMode] = React.useState(ActionMode.StepActions);
    const [layout, setLayout] = React.useState(DEFAULT_LAYOUT);
    const [customizing, setCustomizing] = React.useState(false);
    const [activePath, setActivePath] = React.useState<string | undefined>();
    const [activeDef, setActiveDef] = React.useState<ComponentDefinition | undefined>();

    let remoteRobot = props.remoteRobot
    let remoteStreams = props.remoteStreams

    let btnFnProvider = new ButtonFunctionProvider({
        actionMode: actionMode,
        velocityScale: velocityScale,
        remoteRobot: remoteRobot
    })

    /**
     * Callback when the user clicks on a drop zone, moves the active component
     * into the drop zone
     * @param path path to the clicked drop zone
     */
    const handleDrop = (path: string) => {
        console.log("handleDrop", path);
        if (!activeDef) throw Error('Active definition undefined on drop event')
        let newPath: string = path;
        if (!activePath) {
            // New element not already in the layout
            addToLayout(activeDef, path, layout);
        } else {
            newPath = moveInLayout(activePath, path, layout);
        }
        setActivePath(newPath);
        console.log('new active path', newPath)
        setLayout(layout);
    }

    /**
     * Callback when a component is selected during customization
     * @param path path to the selected component
     * @param def definition of the selected component
     */
    const handleSelect = (def: ComponentDefinition, path?: string) => {
        console.log('selected', path);
        if (!customizing) return;
        if (activePath && activePath == path) {
            setActiveDef(undefined);
            setActivePath(undefined);
            return;
        }
        setActiveDef(def);
        setActivePath(path);
    }

    /**
     * Callback when the customization button is clicked.
     */
    const handleCustomize = () => {
        setCustomizing(!customizing);
        setActiveDef(undefined);
        setActivePath(undefined);
    }

    /** State passed from the operator and shared by all components */
    const sharedState: SharedState = {
        customizing: customizing,
        onSelect: handleSelect,
        remoteStreams: remoteStreams,
        functionProvider: (bf: UserInteractionFunction) => btnFnProvider.provideFunctions(bf),
        activePath: activePath,
        dropZoneState: {
            onDrop: handleDrop,
            activeDef: activeDef
        }
    }

    return (
        <div id="operator">
            <div id="operator-header">
                <ActionModeButton
                    actionMode={actionMode}
                    onChange={(am) => setActionMode(am)}
                />
                <VelocityControl
                    initialVelocityScale={velocityScale}
                    onChange={(newVelocityScale) => btnFnProvider.handleVelocityScaleUpdate(newVelocityScale)}
                />
                <CustomizeButton
                    customizing={customizing}
                    onClick={handleCustomize}
                />
            </div>
            <div id="operator-body">
                <LayoutArea
                    layout={layout}
                    sharedState={sharedState}
                />
                <Sidebar hidden={!customizing} />
            </div>
        </div>
    )
}