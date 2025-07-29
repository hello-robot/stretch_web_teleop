import React from "react";
import { AudioControl } from "./static_components/AudioControl";
import { SpeedControl } from "./static_components/SpeedControl";
import { LayoutArea } from "./static_components/LayoutArea";
import { CustomizeButton } from "./static_components/CustomizeButton";
import { GlobalOptionsProps, Sidebar } from "./static_components/Sidebar";
import { SharedState } from "./layout_components/CustomizableComponent";
import {
    ActionMode,
    ComponentDefinition,
    LayoutDefinition,
} from "./utils/component_definitions";
import { className, ActionState, RemoteStream, RobotPose } from "shared/util";
import {
    buttonFunctionProvider,
    underMapFunctionProvider,
    underVideoFunctionProvider,
    homeTheRobotFunctionProvider,
    hasBetaTeleopKit,
    stretchTool,
} from ".";
import {
    ButtonPadButton,
    ButtonState,
    ButtonStateMap,
} from "./function_providers/ButtonFunctionProvider";
import { Dropdown } from "./basic_components/Dropdown";
import {
    DEFAULT_LAYOUTS,
    DefaultLayoutName,
    StorageHandler,
} from "./storage_handler/StorageHandler";
import { FunctionProvider } from "./function_providers/FunctionProvider";
import {
    addToLayout,
    moveInLayout,
    removeFromLayout,
} from "./utils/layout_helpers";
import { MovementRecorder } from "./layout_components/MovementRecorder";
import { Alert } from "./basic_components/Alert";
import "operator/css/Operator.css";
import { TextToSpeech } from "./layout_components/TextToSpeech";
import { HomeTheRobot } from "./layout_components/HomeTheRobot";
import { RosbagRecorder } from "./layout_components/RosbagRecorder";

/** Operator interface webpage */
export const Operator = (props: {
    remoteStreams: Map<string, RemoteStream>;
    layout: LayoutDefinition;
    storageHandler: StorageHandler;
    isReconnecting?: boolean;
}) => {
    const [customizing, setCustomizing] = React.useState<boolean>(false);
    const [selectedDefinition, setSelectedDefinition] =
        React.useState<ComponentDefinition>();
    const [selectedPath, setSelectedPath] = React.useState<string>();
    const [buttonStateMapRerender, setButtonStateMapRerender] =
        React.useState<boolean>(false);
    const [tabletOrientationRerender, setTabletOrientationRerender] =
        React.useState<boolean>(false);
    const [velocityScale, setVelocityScale] = React.useState<number>(0.8);
    const [isHumanMode, setIsHumanMode] = React.useState<boolean>(true);
    const [showPopup, setShowPopup] = React.useState<boolean>(false);
    const [programMode, setProgramMode] = React.useState<string>("Demonstrate");
    const [buttonCollision, setButtonCollision] = React.useState<
        ButtonPadButton[]
    >([]);
    const [moveBaseState, setMoveBaseState] = React.useState<ActionState>();
    const [moveToPregraspState, setMoveToPregraspState] =
        React.useState<ActionState>();
    const [showTabletState, setShowTabletState] =
        React.useState<ActionState>();
    const [robotNotHomed, setRobotNotHomed] =
        React.useState<ActionState>();
    function showHomeTheRobotGlobalControl(isHomed: boolean) {
        setRobotNotHomed(isHomed ? undefined : { state: "not_homed", alert_type: "warning" });
    }
    homeTheRobotFunctionProvider.setIsHomedCallback(
        showHomeTheRobotGlobalControl
    );

    const layout = React.useRef<LayoutDefinition>(props.layout);
    
    // Mode-specific layouts
    const [modeLayouts, setModeLayouts] = React.useState<{ [mode: string]: LayoutDefinition }>({
        "Demonstrate": props.layout,
        "Create Program": props.layout,
        "Run Program": props.layout,
    });

    // Initialize mode-specific layouts on first load
    React.useEffect(() => {
        const initializeModeLayouts = () => {
            const programModes = ["Demonstrate", "Create Program", "Run Program"];
            const initialLayouts: { [mode: string]: LayoutDefinition } = {};
            
            programModes.forEach(mode => {
                const savedLayout = props.storageHandler.loadCurrentLayout(mode);
                if (savedLayout) {
                    initialLayouts[mode] = savedLayout;
                } else {
                    // Use a fresh default layout for each mode
                    initialLayouts[mode] = props.storageHandler.loadCurrentLayoutOrDefault(mode);
                }
            });
            
            setModeLayouts(initialLayouts);
            // Set current layout to the current mode
            layout.current = initialLayouts[programMode];
        };
        
        initializeModeLayouts();
    }, []);

    // Just used as a flag to force the operator to rerender when the button state map
    // has been updated
    const buttonStateMap = React.useRef<ButtonStateMap>();
    function operatorCallback(bsm: ButtonStateMap) {
        let collisionButtons: ButtonPadButton[] = [];
        bsm.forEach((state, button) => {
            if (state == ButtonState.Collision) collisionButtons.push(button);
        });
        setButtonCollision(collisionButtons);
        buttonStateMap.current = bsm;
        setButtonStateMapRerender(!buttonStateMapRerender);
    }
    buttonFunctionProvider.setOperatorCallback(operatorCallback);

    // Just used as a flag to force the operator to rerender when the tablet orientation
    // changes.
    underVideoFunctionProvider.setTabletOrientationOperatorCallback((_) => {
        setTabletOrientationRerender(!tabletOrientationRerender);
    });

    // Callback for when the move base state is updated (e.g., the ROS2 action returns)
    // Used to render alerts to the operator.
    function moveBaseStateCallback(state: ActionState) {
        setMoveBaseState(state);
    }
    underMapFunctionProvider.setOperatorCallback(moveBaseStateCallback);
    let moveBaseAlertTimeout: NodeJS.Timeout;
    React.useEffect(() => {
        if (moveBaseState && moveBaseState.alert_type != "info") {
            if (moveBaseAlertTimeout) clearTimeout(moveBaseAlertTimeout);
            moveBaseAlertTimeout = setTimeout(() => {
                setMoveBaseState(undefined);
            }, 5000);
        }
    }, [moveBaseState]);

    // Callback for when the move to pregrasp state is updated (e.g., the ROS2 action returns)
    // Used to render alerts to the operator.
    function moveToPregraspStateCallback(state: ActionState) {
        setMoveToPregraspState(state);
    }
    underVideoFunctionProvider.setMoveToPregraspOperatorCallback(
        moveToPregraspStateCallback
    );
    let moveToPregraspAlertTimeout: NodeJS.Timeout;
    React.useEffect(() => {
        if (moveToPregraspState && moveToPregraspState.alert_type != "info") {
            if (moveToPregraspAlertTimeout)
                clearTimeout(moveToPregraspAlertTimeout);
            moveToPregraspAlertTimeout = setTimeout(() => {
                setMoveToPregraspState(undefined);
            }, 5000);
        }
    }, [moveToPregraspState]);

    // Callback for when the show tablet state is updated (e.g., the ROS2 action returns)
    // Used to render alerts to the operator.
    function showTabletStateCallback(state: ActionState) {
        setShowTabletState(state);
    }
    underVideoFunctionProvider.setShowTabletOperatorCallback(
        showTabletStateCallback
    );
    let showTabletAlertTimeout: NodeJS.Timeout;
    React.useEffect(() => {
        if (showTabletState && showTabletState.alert_type != "info") {
            if (showTabletAlertTimeout) clearTimeout(showTabletAlertTimeout);
            showTabletAlertTimeout = setTimeout(() => {
                setShowTabletState(undefined);
            }, 5000);
        }
    }, [showTabletState]);

    let remoteStreams = props.remoteStreams;

    /** Rerenders the operator */
    function updateLayout() {
        console.log("update layout");
        setButtonStateMapRerender(!buttonStateMapRerender);
        setTabletOrientationRerender(!tabletOrientationRerender);
    }

    /**
     * Updates the action mode in the layout (visually) and in the function
     * provider (functionally).
     */
    function setActionMode(actionMode: ActionMode) {
        layout.current.actionMode = actionMode;
        FunctionProvider.actionMode = actionMode;
        props.storageHandler.saveCurrentLayout(layout.current, programMode);
        updateLayout();
    }

    /**
     * Sets the movement recorder component to display or hidden.
     *
     * @param displayMovementRecorder if the movement recorder component at the
     *                             top of the operator body should be displayed
     */
    function setDisplayMovementRecorder(displayMovementRecorder: boolean) {
        layout.current.displayMovementRecorder = displayMovementRecorder;
        updateLayout();
    }

    /**
     * Sets the text-to-speech component to display or hidden.
     *
     * @param displayTextToSpeech whether the text-to-speech component should
     *    be displayed.
     */
    function setDisplayTextToSpeech(displayTextToSpeech: boolean) {
        layout.current.displayTextToSpeech = displayTextToSpeech;
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
     * Sets the RosbagRecorder component to display or hidden.
     *
     * @param displayRosbagRecorder whether the RosbagRecorder component should
     *    be displayed.
     */
    function setDisplayRosbagRecorder(displayRosbagRecorder: boolean) {
        layout.current.displayRosbagRecorder = displayRosbagRecorder;
        updateLayout();
    }

    /**
     * Callback when the user clicks on a drop zone, moves the active component
     * into the drop zone
     * @param path path to the clicked drop zone
     */
    function handleDrop(path: string) {
        console.log("handleDrop", path);
        if (!selectedDefinition)
            throw Error("Active definition undefined on drop event");
        let newPath: string = path;
        if (!selectedPath) {
            // New element not already in the layout
            newPath = addToLayout(selectedDefinition, path, layout.current);
        } else {
            newPath = moveInLayout(selectedPath, path, layout.current);
        }
        setSelectedPath(newPath);
        console.log("new active path", newPath);
        updateLayout();
    }

    /**
     * Callback when a component is selected during customization
     * @param path path to the selected component
     * @param def definition of the selected component
     */
    function handleSelect(def: ComponentDefinition, path?: string) {
        console.log("selected", path);
        if (!customizing) return;

        // If reselected the same component at the same path, or the same component
        // without a path from the sidebar, then unactivate it
        const pathsMatch = selectedPath && selectedPath == path;
        const defsMatch =
            !selectedPath &&
            def.type === selectedDefinition?.type &&
            def.id === selectedDefinition?.id;
        if (pathsMatch || defsMatch) {
            setSelectedDefinition(undefined);
            setSelectedPath(undefined);
            return;
        }

        // Activate the selected component
        setSelectedDefinition(def);
        setSelectedPath(path);
    }

    /** Callback when the delete button in the sidebar is clicked */
    function handleDelete() {
        if (!selectedPath)
            throw Error("handleDelete called when selectedPath is undefined");
        removeFromLayout(selectedPath, layout.current);
        updateLayout();
        setSelectedPath(undefined);
        setSelectedDefinition(undefined);
    }

    /**
     * Callback when the customization button is clicked.
     */
    const handleToggleCustomize = () => {
        if (customizing) {
            console.log("saving layout");
            props.storageHandler.saveCurrentLayout(layout.current, programMode);
        }
        setCustomizing(!customizing);
        setSelectedDefinition(undefined);
        setSelectedPath(undefined);
    };

    /** Un-select current component when click inside of header */
    function handleClickHeader() {
        setSelectedDefinition(undefined);
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
            selectedDefinition: selectedDefinition,
        },
        buttonStateMap: buttonStateMap.current,
        hideLabels: !layout.current.displayLabels,
        hasBetaTeleopKit: hasBetaTeleopKit,
        stretchTool: stretchTool,
        robotNotHomed: robotNotHomed,
        // Only pass human mode information in Run Program mode
        isHumanMode: programMode === "Run Program" ? isHumanMode : true,
        storageHandler: props.storageHandler,
    };

    /** Properties for the global options area of the sidebar */
    const globalOptionsProps: GlobalOptionsProps = {
        displayMovementRecorder: layout.current.displayMovementRecorder,
        displayTextToSpeech: layout.current.displayTextToSpeech,
        displayLabels: layout.current.displayLabels,
        displayRosbagRecorder: layout.current.displayRosbagRecorder,
        setDisplayMovementRecorder: setDisplayMovementRecorder,
        setDisplayTextToSpeech: setDisplayTextToSpeech,
        setDisplayLabels: setDisplayLabels,
        setDisplayRosbagRecorder: setDisplayRosbagRecorder,
        defaultLayouts: Object.keys(DEFAULT_LAYOUTS),
        customLayouts: props.storageHandler.getCustomLayoutNames(),
        loadLayout: (layoutName: string, dflt: boolean) => {
            layout.current = dflt
                ? props.storageHandler.loadDefaultLayout(
                      layoutName as DefaultLayoutName
                  )
                : props.storageHandler.loadCustomLayout(layoutName);
            updateLayout();
        },
        saveLayout: (layoutName: string) => {
            props.storageHandler.saveCustomLayout(layout.current, layoutName);
        },
    };

    const actionModes = Object.values(ActionMode);
    const programModes = ["Demonstrate", "Create Program", "Run Program"];

    // Function to switch layouts when program mode changes
    const switchToModeLayout = (newMode: string) => {
        // Save current layout for current mode
        if (modeLayouts[programMode]) {
            const updatedLayouts = { ...modeLayouts };
            updatedLayouts[programMode] = layout.current;
            setModeLayouts(updatedLayouts);
            props.storageHandler.saveCurrentLayout(layout.current, programMode);
        }
        
        // Load layout for new mode
        const newModeLayout = props.storageHandler.loadCurrentLayout(newMode);
        if (newModeLayout) {
            layout.current = newModeLayout;
        } else {
            // If no saved layout for this mode, start with a clean default layout
            layout.current = props.storageHandler.loadCurrentLayoutOrDefault(newMode);
        }
        
        updateLayout();
    };

    return (
        <div id="operator">
            {/* Persistent banner for control mode - only show in Run Program mode */}
            {programMode === "Run Program" && (
                <div
                    style={{
                        width: "100%",
                        background: isHumanMode ? "#4caf50" : "#ff9800",
                        color: "white",
                        textAlign: "center",
                        fontWeight: "bold",
                        fontSize: "1.2em",
                        padding: "8px 0",
                        position: "relative",
                        zIndex: 1,
                        opacity: props.isReconnecting ? 0.5 : 1,
                        filter: props.isReconnecting ? "grayscale(1)" : "none",
                        pointerEvents: props.isReconnecting ? "none" : "auto"
                    }}
                >
                    {isHumanMode ? "You are in control" : "Robot in control"}
                </div>
            )}
            {/* Global controls (Demonstration Recorder, etc.) */}
            <div id="operator-global-controls">
                <div
                    className={className("operator-pose-recorder", {
                        hideLabels: !layout.current.displayLabels,
                    })}
                    hidden={!layout.current.displayMovementRecorder}
                >
                    <MovementRecorder
                        hideLabels={!layout.current.displayLabels}
                    />
                </div>
                <div
                    className={className("operator-text-to-speech", {
                        hideLabels: !layout.current.displayLabels,
                    })}
                    hidden={!layout.current.displayTextToSpeech}
                >
                    <TextToSpeech hideLabels={!layout.current.displayLabels} />
                </div>
                <div
                    className={className("operator-rosbag-recorder", {
                        hideLabels: !layout.current.displayLabels,
                    })}
                    hidden={!layout.current.displayRosbagRecorder}
                >
                    <RosbagRecorder hideLabels={!layout.current.displayLabels} />
                </div>
            </div>
            <div id="operator-header" onClick={handleClickHeader} style={{ display: "flex", alignItems: "center" }}>
                {/* Program mode dropdown */}
                <Dropdown
                    onChange={(idx) => {
                        const newMode = programModes[idx];
                        setProgramMode(newMode);
                        switchToModeLayout(newMode);
                    }}
                    selectedIndex={programModes.indexOf(programMode)}
                    possibleOptions={programModes}
                    showActive
                    placement="bottom"
                />
                {/* Action mode dropdown */}
                <div style={{ marginLeft: 16 }}>
                    <Dropdown
                        onChange={(idx) => setActionMode(actionModes[idx])}
                        selectedIndex={actionModes.indexOf(
                            layout.current.actionMode
                        )}
                        possibleOptions={actionModes}
                        showActive
                        placement="bottom"
                    />
                </div>
                <AudioControl remoteStreams={remoteStreams} />
                <SpeedControl
                    scale={velocityScale}
                    onChange={(newScale: number) => {
                        setVelocityScale(newScale);
                        FunctionProvider.velocityScale = newScale;
                    }}
                />
                <CustomizeButton
                    customizing={customizing}
                    onClick={handleToggleCustomize}
                />
            </div>
            {robotNotHomed && (
                <div className="operator-collision-alerts">
                    <div
                        className={className("operator-alert", {
                            fadeIn: robotNotHomed,
                            fadeOut: !robotNotHomed,
                        })}
                    >
                        <HomeTheRobot
                            hideLabels={!layout.current.displayLabels}
                        />
                    </div>
                </div>
            )}
            {
                <div className="operator-collision-alerts">
                    <div
                        className={className("operator-alert", {
                            fadeIn: buttonCollision.length > 0,
                            fadeOut: buttonCollision.length == 0,
                        })}
                    >
                        <Alert type="warning">
                            <span>
                                {buttonCollision.length > 0
                                    ? buttonCollision.join(", ") +
                                      " in collision!"
                                    : ""}
                            </span>
                        </Alert>
                    </div>
                </div>
            }
            {moveBaseState && (
                <div className="operator-collision-alerts">
                    <div
                        className={className("operator-alert", {
                            fadeIn: moveBaseState !== undefined,
                            fadeOut: moveBaseState == undefined,
                        })}
                    >
                        <Alert
                            type={moveBaseState.alert_type}
                            message={moveBaseState.state}
                        />
                    </div>
                </div>
            )}
            {moveToPregraspState && (
                <div className="operator-collision-alerts">
                    <div
                        className={className("operator-alert", {
                            fadeIn: moveToPregraspState !== undefined,
                            fadeOut: moveToPregraspState == undefined,
                        })}
                    >
                        <Alert
                            type={moveToPregraspState.alert_type}
                            message={moveToPregraspState.state}
                        />
                    </div>
                </div>
            )}
            {showTabletState && (
                <div className="operator-collision-alerts">
                    <div
                        className={className("operator-alert", {
                            fadeIn: showTabletState !== undefined,
                            fadeOut: showTabletState == undefined,
                        })}
                    >
                        <Alert
                            type={showTabletState.alert_type}
                            message={showTabletState.state}
                        />
                    </div>
                </div>
            )}
            {/* Pop-up Modal */}
            {showPopup && programMode === "Run Program" && (
                <div style={{
                    position: "fixed",
                    top: 0,
                    left: 0,
                    width: "100vw",
                    height: "100vh",
                    background: "rgba(0,0,0,0.4)",
                    zIndex: 1000,
                    display: "flex",
                    alignItems: "center",
                    justifyContent: "center"
                }}>
                    <div style={{
                        background: "white",
                        borderRadius: 8,
                        padding: 32,
                        minWidth: 320,
                        boxShadow: "0 2px 16px rgba(0,0,0,0.2)",
                        textAlign: "center"
                    }}>
                        <div style={{ fontSize: "1.2em", marginBottom: 24 }}>
                            Insert text specified by the user
                        </div>
                        <div style={{ display: "flex", justifyContent: "center", gap: 16 }}>
                            <button
                                style={{
                                    backgroundColor: "orange",
                                    color: "white",
                                    border: "none",
                                    padding: "10px 20px",
                                    fontWeight: "bold",
                                    borderRadius: "5px",
                                    cursor: "pointer"
                                }}
                                onClick={() => {
                                    setIsHumanMode(true);
                                    setShowPopup(false);
                                }}
                            >
                                Reset
                            </button>
                            <button
                                style={{
                                    background: "#4caf50",
                                    color: "white",
                                    border: "none",
                                    borderRadius: 4,
                                    padding: "8px 20px",
                                    fontWeight: "bold",
                                    fontSize: "1em",
                                    cursor: "pointer"
                                }}
                                onClick={() => setShowPopup(false)}
                            >
                                Confirm and proceed
                            </button>
                        </div>
                    </div>
                </div>
            )}
            <div id="operator-body">
                <LayoutArea layout={layout.current} sharedState={sharedState} />
            </div>
            {/* Bottom left controls - only show in Run Program mode */}
            {programMode === "Run Program" && (
                <div style={{
                    position: "fixed",
                    bottom: 20,
                    left: 20,
                    zIndex: 100,
                    display: "flex",
                    alignItems: "center",
                    gap: 12
                }}>
                    {/* Human/Robot mode toggle */}
                    <button
                        style={{
                            background: isHumanMode ? "#4caf50" : "#ff9800",
                            color: "white",
                            border: "none",
                            borderRadius: 4,
                            padding: "8px 16px",
                            fontWeight: "bold",
                            cursor: "pointer",
                            fontSize: "0.9em",
                            boxShadow: "0 2px 8px rgba(0,0,0,0.2)"
                        }}
                        onClick={e => {
                            e.stopPropagation();
                            setIsHumanMode(mode => !mode);
                        }}
                    >
                        {isHumanMode ? "Human" : "Robot"} Mode
                    </button>
                    {/* Pop-up button */}
                    {!isHumanMode && (
                        <button
                            style={{
                                background: "#fff",
                                color: "#ff9800",
                                border: "1px solid #ff9800",
                                borderRadius: 4,
                                padding: "8px 16px",
                                fontWeight: "bold",
                                cursor: "pointer",
                                fontSize: "0.9em",
                                boxShadow: "0 2px 8px rgba(0,0,0,0.2)"
                            }}
                            onClick={e => {
                                e.stopPropagation();
                                setShowPopup(true);
                            }}
                        >
                            Pop-up
                        </button>
                    )}
                </div>
            )}
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
    );
};
