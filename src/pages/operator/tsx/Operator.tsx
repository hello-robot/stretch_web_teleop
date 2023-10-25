import React from "react";
import { SpeedControl } from "operator/tsx/static_components/SpeedControl"
import { LayoutArea } from "./static_components/LayoutArea";
import { CustomizeButton } from "./static_components/CustomizeButton";
import { GlobalOptionsProps, Sidebar } from "./static_components/Sidebar";
import { SharedState } from "./layout_components/CustomizableComponent";
import { ActionMode, ComponentDefinition, LayoutDefinition } from "./utils/component_definitions";
import { VoiceCommands } from "./static_components/VoiceCommands";
import { ArucoNavigationState, className, MoveBaseState, RemoteStream, RobotPose } from "shared/util";
import { arucoMarkerFunctionProvider, buttonFunctionProvider, underMapFunctionProvider } from ".";
import { ButtonPadButton, ButtonState, ButtonStateMap } from "./function_providers/ButtonFunctionProvider";
import { Dropdown } from "./basic_components/Dropdown";
import { DEFAULT_LAYOUTS, DefaultLayoutName, StorageHandler } from "./storage_handler/StorageHandler";
import { FunctionProvider } from "./function_providers/FunctionProvider";
import { addToLayout, moveInLayout, removeFromLayout } from "./utils/layout_helpers";
import { PoseLibrary } from "./static_components/PoseLibrary";
import { MovementRecorder } from "./layout_components/MovementRecorder";
import { ArucoMarkers } from "./layout_components/ArucoMarkers";
import { Alert } from "./basic_components/Alert";
import "operator/css/Operator.css"

/** Operator interface webpage */
export const Operator = (props: {
    remoteStreams: Map<string, RemoteStream>,
    getRobotPose: (head: boolean, gripper: boolean, arm: boolean) => RobotPose,
    setRobotPose: (pose: RobotPose) => void,
    layout: LayoutDefinition,
    storageHandler: StorageHandler
}) => {
    const [customizing, setCustomizing] = React.useState(false);
    const [selectedPath, setSelectedPath] = React.useState<string | undefined>(undefined);
    const [selectedDefinition, setSelectedDef] = React.useState<ComponentDefinition | undefined>(undefined);
    const [velocityScale, setVelocityScale] = React.useState<number>(FunctionProvider.velocityScale);
    const [buttonCollision, setButtonCollision] = React.useState<ButtonPadButton[]>([]);
    const [arucoNavigationState, setArucoNavigationState] = React.useState<ArucoNavigationState>()
    const [moveBaseState, setMoveBaseState] = React.useState<MoveBaseState>()

    const layout = React.useRef<LayoutDefinition>(props.layout);

    // Just used as a flag to force the operator to rerender when the button state map
    // has been updated
    const [buttonStateMapRerender, setButtonStateMapRerender] = React.useState<boolean>(false);
    const buttonStateMap = React.useRef<ButtonStateMap>();
    function operatorCallback(bsm: ButtonStateMap) {
        let collisionButtons: ButtonPadButton[] = []
        bsm.forEach((state, button) => {
            if (state == ButtonState.Collision) collisionButtons.push(button)
        })
        setButtonCollision(collisionButtons)
        buttonStateMap.current = bsm;
        setButtonStateMapRerender(!buttonStateMapRerender);
    }
    buttonFunctionProvider.setOperatorCallback(operatorCallback);

    function arucoNavigationStateCallback(state: ArucoNavigationState) {
        setArucoNavigationState(state)
    }
    arucoMarkerFunctionProvider.setOperatorCallback(arucoNavigationStateCallback);
    let arucoAlertTimeout: NodeJS.Timeout;
    React.useEffect(() => {
        if (arucoNavigationState && arucoNavigationState.alert_type != "info") {
            if (arucoAlertTimeout) clearTimeout(arucoAlertTimeout)
            arucoAlertTimeout = setTimeout(() => {
                setArucoNavigationState(undefined)
            }, 5000)
        }
    }, [arucoNavigationState])

    function moveBaseStateCallback(state: MoveBaseState) {
        setMoveBaseState(state)
    }
    underMapFunctionProvider.setOperatorCallback(moveBaseStateCallback);
    let moveBaseAlertTimeout: NodeJS.Timeout;
    React.useEffect(() => {
        if (moveBaseState && moveBaseState.alert_type != "info") {
            if (moveBaseAlertTimeout) clearTimeout(moveBaseAlertTimeout)
            moveBaseAlertTimeout = setTimeout(() => {
                setMoveBaseState(undefined)
            }, 5000)
        }
    }, [moveBaseState])

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
     * Sets the pose library component to display or hidden.
     * 
     * @param displayPoseLibrary if the pose library component at the 
     *                           top of the operator body should be displayed
     */
    function setDisplayPoseLibrary(displayPoseLibrary: boolean) {
        layout.current.displayPoseLibrary = displayPoseLibrary;
        updateLayout();
    }

    /**
     * Sets the movement recorder component to display or hidden.
     * 
     * @param displayMovementRecorder if the pose recorder component at the 
     *                             top of the operator body should be displayed
     */
    function setDisplayMovementRecorder(displayMovementRecorder: boolean) {
        layout.current.displayMovementRecorder = displayMovementRecorder;
        updateLayout();
    }

    /**
     * Sets the aruco markers component to display or hidden.
     * 
     * @param displayArucoMarkers if the pose recorder component at the 
     *                             top of the operator body should be displayed
     */
    function setDisplayArucoMarkers(displayArucoMarkers: boolean) {
        layout.current.displayArucoMarkers = displayArucoMarkers;
        updateLayout();
    }

    /**
     * Sets the display labels property to display or hidden.
     * 
     * @param displayLabels if the button text labels should be displayed 
     */
    function setDisplayLabels(displayLabels: boolean) {
        layout.current.displayLabels = displayLabels;
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
            newPath = addToLayout(selectedDefinition, path, layout.current);
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
    function handleDelete() {
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
        buttonStateMap: buttonStateMap.current,
        hideLabels: !layout.current.displayLabels
    }

    /** Properties for the global options area of the sidebar */
    const globalOptionsProps: GlobalOptionsProps = {
        displayVoiceControl: layout.current.displayVoiceControl,
        displayPoseLibrary: layout.current.displayPoseLibrary,
        displayMovementRecorder: layout.current.displayMovementRecorder,
        displayArucoMarkers: layout.current.displayArucoMarkers,
        displayLabels: layout.current.displayLabels,
        setDisplayVoiceControl: setDisplayVoiceControl,
        setDisplayPoseLibrary: setDisplayPoseLibrary,
        setDisplayMovementRecorder: setDisplayMovementRecorder,
        setDisplayArucoMarkers: setDisplayArucoMarkers,
        setDisplayLabels: setDisplayLabels,
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
                    placement="bottom"
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
            <div id="operator-global-controls">
                {
                    <div className="operator-collision-alerts">
                        <div className={className('operator-alert', { fadeIn: buttonCollision.length > 0, fadeOut: buttonCollision.length == 0 })}>
                            <Alert type="warning">
                                <span>
                                    {buttonCollision.length > 0 ? buttonCollision.join(', ') + " in collision!" : ""}
                                </span>
                            </Alert>
                        </div>
                    </div>
                }
                {arucoNavigationState &&
                    <div className="operator-collision-alerts">
                        <div className={className('operator-alert', { fadeIn: arucoNavigationState !== undefined, fadeOut: arucoNavigationState == undefined })}>
                            <Alert type={arucoNavigationState.alert_type} message={arucoNavigationState.state} />
                        </div>
                    </div>
                }
                {moveBaseState && !arucoNavigationState &&
                    <div className="operator-collision-alerts">
                        <div className={className('operator-alert', { fadeIn: moveBaseState !== undefined, fadeOut: moveBaseState == undefined })}>
                            <Alert type={moveBaseState.alert_type} message={moveBaseState.state} />
                        </div>
                    </div>
                }
                <div className="operator-voice" hidden={!layout.current.displayVoiceControl}>
                    <VoiceCommands
                        onUpdateVelocityScale=
                        {(newScale: number) => { setVelocityScale(newScale); FunctionProvider.velocityScale = newScale; }}
                    />
                </div>
                <div className={className("operator-pose-library", { hideLabels: !layout.current.displayLabels })} hidden={!layout.current.displayPoseLibrary}>
                    <PoseLibrary
                        savePose={(poseName: string, head: boolean, gripper: boolean, arm: boolean) => {
                            props.storageHandler.savePose(poseName, props.getRobotPose(head, gripper, arm));
                        }}
                        deletePose={((poseName: string) => { props.storageHandler.deletePose(poseName) })}
                        savedPoseNames={() => { return props.storageHandler.getPoseNames() }}
                        setRobotPose={(poseName: string) => {
                            let pose: RobotPose = props.storageHandler.getPose(poseName)
                            props.setRobotPose(pose)
                        }}
                        hideLabels={!layout.current.displayLabels}
                    />
                </div>
                <div className={className("operator-pose-recorder", {hideLabels: !layout.current.displayLabels})} hidden={!layout.current.displayMovementRecorder}>
                    <MovementRecorder hideLabels={!layout.current.displayLabels} />
                </div>
                <div className={className("operator-aruco-markers", { hideLabels: !layout.current.displayLabels })}  hidden={!layout.current.displayArucoMarkers}>
                    <ArucoMarkers setArucoNavigationState={arucoNavigationStateCallback} hideLabels={!layout.current.displayLabels} />
                </div>
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