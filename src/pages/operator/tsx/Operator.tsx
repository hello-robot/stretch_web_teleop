import React from "react";
import { SpeedControl } from "operator/tsx/static_components/SpeedControl"
import { LayoutArea } from "./static_components/LayoutArea";
import { CustomizeButton } from "./static_components/CustomizeButton";
import { GlobalOptionsProps, Sidebar } from "./static_components/Sidebar";
import { SharedState } from "./layout_components/CustomizableComponent";
import { ActionMode, ComponentDefinition, LayoutDefinition } from "./utils/component_definitions";
import { VoiceCommands } from "./static_components/VoiceCommands";
import { RemoteStream } from "shared/util";
import { buttonFunctionProvider } from ".";
import { ButtonStateMap } from "./function_providers/ButtonFunctionProvider";
import { Dropdown } from "./basic_components/Dropdown";
import { DEFAULT_LAYOUTS, DefaultLayoutName, StorageHandler } from "./storage_handler/StorageHandler";
import { FunctionProvider } from "./function_providers/FunctionProvider";
import { addToLayout, moveInLayout, removeFromLayout } from "./utils/layout_helpers";
import "operator/css/Operator.css"

/** Operator interface webpage */
export const Operator = (props: {
    remoteStreams: Map<string, RemoteStream>,
    layout: LayoutDefinition,
    storageHandler: StorageHandler
}) => {
    const [customizing, setCustomizing] = React.useState(false);
    const [selectedPath, setSelectedPath] = React.useState<string | undefined>(undefined);
    const [selectedDefinition, setSelectedDef] = React.useState<ComponentDefinition | undefined>(undefined);
    const [velocityScale, setVelocityScale] = React.useState<number>(FunctionProvider.velocityScale);
    const layout = React.useRef<LayoutDefinition>(props.layout);

    // Just used as a flag to force the operator to rerender when the button state map
    // has been updated
    const [buttonStateMapRerender, setButtonStateMapRerender] = React.useState<boolean>(false);
    const buttonStateMap = React.useRef<ButtonStateMap>();
    function operatorCallback(bsm: ButtonStateMap) {
        buttonStateMap.current = bsm;
        setButtonStateMapRerender(!buttonStateMapRerender);
    }
    buttonFunctionProvider.setOperatorCallback(operatorCallback);

    let remoteStreams = props.remoteStreams;

    /** Rerenders the operator */
    function updateLayout() {
        console.log('update layout');
        setButtonStateMapRerender(!buttonStateMapRerender);
    }

    /** 
     * Updates the action mode in the layout (visually) and in the fuction
     * provider (functionally).
     */
    function setActionMode(actionMode: ActionMode) {
        layout.current.actionMode = actionMode;
        FunctionProvider.actionMode = actionMode;
        props.storageHandler.saveCurrentLayout(layout.current);
        updateLayout();
    }

    /**
     * Sets the voice control component to display or hidden.
     * 
     * @param displayVoiceControl if the voice control component at the 
     *                            top of the operator body should be displayed
     */
    function setDisplayVoiceControl(displayVoiceControl: boolean) {
        layout.current.displayVoiceControl = displayVoiceControl;
        updateLayout();
    }

    /**
     * Callback when the user clicks on a drop zone, moves the active component
     * into the drop zone
     * @param path path to the clicked drop zone
     */
    function handleDrop(path: string) {
        console.log("handleDrop", path);
        if (!selectedDefinition) throw Error('Active definition undefined on drop event')
        let newPath: string = path;
        if (!selectedPath) {
            // New element not already in the layout
            addToLayout(selectedDefinition, path, layout.current);
        } else {
            newPath = moveInLayout(selectedPath, path, layout.current);
        }
        setSelectedPath(newPath);
        console.log('new active path', newPath)
        updateLayout();
    }

    /**
     * Callback when a component is selected during customization
     * @param path path to the selected component
     * @param def definition of the selected component
     */
    function handleSelect(def: ComponentDefinition, path?: string) {
        console.log('selected', path);
        if (!customizing) return;

        // If reselected the same component at the same path, or the same component
        // without a path from the sidebar, then unactivate it
        const pathsMatch = selectedPath && selectedPath == path;
        const defsMatch = !selectedPath && def.type === selectedDefinition?.type && def.id === selectedDefinition?.id;
        if (pathsMatch || defsMatch) {
            setSelectedDef(undefined);
            setSelectedPath(undefined);
            return;
        }

        // Activate the selected component
        setSelectedDef(def);
        setSelectedPath(path);
    }

    /** Callback when the delete button in the sidebar is clicked */
    function handleDelete () {
        if (!selectedPath) throw Error('handleDelete called when selectedPath is undefined');
        removeFromLayout(selectedPath, layout.current);
        updateLayout();
        setSelectedPath(undefined);
        setSelectedDef(undefined);
    }

    /**
     * Callback when the customization button is clicked.
     */
    const handleToggleCustomize = () => {
        if (customizing) {
            console.log('saving layout');
            props.storageHandler.saveCurrentLayout(layout.current);
        }
        setCustomizing(!customizing);
        setSelectedDef(undefined);
        setSelectedPath(undefined);
    }

    /** Un-select current component when click inside of header */
    function handleClickHeader() {
        setSelectedDef(undefined);
        setSelectedPath(undefined);
    }

    /** State passed from the operator and shared by all components */
    const sharedState: SharedState = {
        customizing: customizing,
        onSelect: handleSelect,
        remoteStreams: remoteStreams,
        selectedPath: selectedPath,
        dropZoneState: {
            onDrop: handleDrop,
            selectedDefinition: selectedDefinition
        },
        buttonStateMap: buttonStateMap.current
    }

    /** Properties for the global options area of the sidebar */
    const globalOptionsProps: GlobalOptionsProps = {
        displayVoiceControl: layout.current.displayVoiceControl,
        setDisplayVoiceControl: setDisplayVoiceControl,
        defaultLayouts: Object.keys(DEFAULT_LAYOUTS),
        customLayouts: props.storageHandler.getCustomLayoutNames(),
        loadLayout: (layoutName: string, dflt: boolean) => {
            layout.current = dflt ?
                props.storageHandler.loadDefaultLayout(layoutName as DefaultLayoutName) :
                props.storageHandler.loadCustomLayout(layoutName);
            updateLayout();
        },
        saveLayout: (layoutName: string) => { props.storageHandler.saveCustomLayout(layout.current, layoutName); }
    }

    const actionModes = Object.values(ActionMode);
    return (
        <div id="operator">
            <div id="operator-header" onClick={handleClickHeader}>
                {/* Action mode button */}
                <Dropdown
                    onChange={(idx) => setActionMode(actionModes[idx])}
                    selectedIndex={actionModes.indexOf(layout.current.actionMode)}
                    possibleOptions={actionModes}
                    showActive
                />
                <SpeedControl
                    scale={velocityScale}
                    onChange={(newScale: number) => { setVelocityScale(newScale); FunctionProvider.velocityScale = newScale; }}
                />
                <CustomizeButton
                    customizing={customizing}
                    onClick={handleToggleCustomize}
                />
            </div>
            <div id="operator-voice" hidden={!layout.current.displayVoiceControl}>
                <VoiceCommands
                    onUpdateVelocityScale=
                    {(newScale: number) => { setVelocityScale(newScale); FunctionProvider.velocityScale = newScale; }}
                />
            </div>
            <div id="operator-body">
                <LayoutArea
                    layout={layout.current}
                    sharedState={sharedState}
                />
            </div>
            <Sidebar
                hidden={!customizing}
                onDelete={handleDelete}
                updateLayout={updateLayout}
                onSelect={handleSelect}
                selectedDefinition={selectedDefinition}
                selectedPath={selectedPath}
                globalOptionsProps={globalOptionsProps}
            />
        </div>
    )
}