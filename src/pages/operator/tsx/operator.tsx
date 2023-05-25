import React from "react";
import { VelocityControl } from "operator/tsx/staticcomponents/velocitycontrol"
import { LayoutArea } from "./staticcomponents/layoutarea";
import { ActionMode, ActionModeButton } from "operator/tsx/staticcomponents/actionmodebutton"
import { CustomizeButton } from "./staticcomponents/customizebutton";
import { Sidebar } from "./staticcomponents/sidebar";
import { SharedState } from "./layoutcomponents/customizablecomponent";
import { ComponentDefinition } from "./utils/componentdefinitions";
import { DEFAULT_LAYOUT } from "./utils/defaultlayout";
import { VoiceCommands } from "./staticcomponents/voicecommands";
import { RemoteStream } from "shared/util";
import { addToLayout, moveInLayout, removeFromLayout } from "operator/tsx/utils/layouthelpers";
import "operator/css/operator.css"
import { FunctionProvider } from "operator/tsx/functionprovider/functionprovider";
import { buttonFunctionProvider } from ".";
import { ButtonStateMap } from "./functionprovider/buttonpads";

/** Operator interface webpage */
export const Operator = (props: {
    remoteStreams: Map<string, RemoteStream>
}) => {
    const [layout, setLayout] = React.useState(DEFAULT_LAYOUT);
    const [customizing, setCustomizing] = React.useState(false);
    const [activePath, setActivePath] = React.useState<string | undefined>();
    const [activeDef, setActiveDef] = React.useState<ComponentDefinition | undefined>();
    const [displayVoiceControl, setDisplayVoiceControl] = React.useState<boolean>(true);

    // Just used as a flag to force the operator to rerender when the button state map
    // has been updated
    const [buttonStateMapRerender, setButtonStateMapRerender] = React.useState<boolean>(false);
    const buttonStateMap = React.useRef<ButtonStateMap>();
    function operatorCallback(bsm: ButtonStateMap) {
        buttonStateMap.current = bsm;
        setButtonStateMapRerender(!buttonStateMapRerender);
    }
    buttonFunctionProvider.setOperatorCallback(operatorCallback);

    // Store as state to cause rerender when velocity scale or action mode are changed
    const [velocityScale, setVelocityScale] = React.useState<number>(FunctionProvider.velocityScale);
    const [actionMode, setActionMode] = React.useState<ActionMode>(FunctionProvider.actionMode);

    let remoteStreams = props.remoteStreams;

    /** Rerenders the operator */
    function updateLayout() {
        console.log('update layout');
        setButtonStateMapRerender(!buttonStateMapRerender);
    }

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
        updateLayout();
    }

    /**
     * Callback when a component is selected during customization
     * @param path path to the selected component
     * @param def definition of the selected component
     */
    const handleSelect = (def: ComponentDefinition, path?: string) => {
        console.log('selected', path);
        if (!customizing) return;

        // If reselected the same component at the same path, or the same component
        // without a path from the sidebar, then unactivate it
        const pathsMatch = activePath && activePath == path;
        const defsMatch = !activePath && def.type === activeDef?.type && def.id === activeDef?.id;
        if (pathsMatch || defsMatch) {
            setActiveDef(undefined);
            setActivePath(undefined);
            return;
        }

        // Activate the selected component
        setActiveDef(def);
        setActivePath(path);
    }

    /** Callback when the delete button in the sidebar is clicked */
    const handleDelete = () => {
        if (!activePath) throw Error('handleDelete called when activePath is undefined');
        removeFromLayout(activePath, layout);
        updateLayout();
        setActivePath(undefined);
        setActiveDef(undefined);
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
        activePath: activePath,
        dropZoneState: {
            onDrop: handleDrop,
            activeDef: activeDef
        },
        buttonStateMap: buttonStateMap.current
    }

    return (
        <div id="operator">
            <div id="operator-header">
                <ActionModeButton
                    actionMode={actionMode}
                    onChange={(am) => { setActionMode(am); FunctionProvider.actionMode = am; }}
                />
                <VelocityControl
                    scale={velocityScale}
                    onChange={(newScale: number) => { setVelocityScale(newScale); FunctionProvider.velocityScale = newScale; }}
                />
                <CustomizeButton
                    customizing={customizing}
                    onClick={handleCustomize}
                />
            </div>
            <div id="operator-voice" hidden={!displayVoiceControl}>
                <VoiceCommands
                    onUpdateVelocityScale=
                    {(newScale: number) => { setVelocityScale(newScale); FunctionProvider.velocityScale = newScale; }}
                />
            </div>
            <div id="operator-body">
                <LayoutArea
                    layout={layout}
                    sharedState={sharedState}
                />
            </div>
            <Sidebar
                hidden={!customizing}
                onDelete={handleDelete}
                activeDef={activeDef}
                activePath={activePath}
                updateLayout={updateLayout}
                onSelect={handleSelect}
                displayVoiceControl={displayVoiceControl}
                setDisplayVoiceControl={setDisplayVoiceControl}
            />
        </div>
    )
}