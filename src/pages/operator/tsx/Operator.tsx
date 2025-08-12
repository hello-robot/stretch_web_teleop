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
import "operator/css/HomeRobotButton.css";
import { TextToSpeech } from "./layout_components/TextToSpeech";
import { HomeTheRobot, HomeTheRobotFunction } from "./layout_components/HomeTheRobot";
import { RosbagRecorder } from "./layout_components/RosbagRecorder";
import HomeIcon from "@mui/icons-material/Home";
import CheckIcon from "@mui/icons-material/Check";



/** Operator interface webpage */
export const Operator = (props: {
    remoteStreams: Map<string, RemoteStream>;
    layout: LayoutDefinition;
    storageHandler: StorageHandler;
    isReconnecting?: boolean;
}) => {
    // Layout and customization state
    const [customizing, setCustomizing] = React.useState<boolean>(false);
    const [selectedDefinition, setSelectedDefinition] = React.useState<ComponentDefinition>();
    const [selectedPath, setSelectedPath] = React.useState<string>();
    const [buttonStateMapRerender, setButtonStateMapRerender] = React.useState<boolean>(false);
    const [tabletOrientationRerender, setTabletOrientationRerender] = React.useState<boolean>(false);
    
    // Program execution state
    const [velocityScale, setVelocityScale] = React.useState<number>(0.8);
    const [isExecutingProgram, setIsExecutingProgram] = React.useState<boolean>(false);
    const [currentExecutingLine, setCurrentExecutingLine] = React.useState<number | undefined>(undefined);
    const [showExecutionMessage, setShowExecutionMessage] = React.useState<boolean>(false);
    const [waitingForUserConfirmation, setWaitingForUserConfirmation] = React.useState<boolean>(false);
    const [pauseAndConfirmMessage, setPauseAndConfirmMessage] = React.useState<string>("");
    const [executionError, setExecutionError] = React.useState<{ type: 'syntax' | 'invalid_input' | 'unknown_pose'; message: string } | null>(null);
    const [errorLineNumber, setErrorLineNumber] = React.useState<number | null>(null);
    const [isTakeControlActive, setIsTakeControlActive] = React.useState<boolean>(false);
    
    // Program mode state
    const [showPopup, setShowPopup] = React.useState<boolean>(false);
    const [programMode, setProgramMode] = React.useState<string>("Demonstrate");
    
    // Function to update current executing line
    const updateCurrentExecutingLine = (lineNumber: number | undefined) => {
        setCurrentExecutingLine(lineNumber);
    };
    
    // Effect to handle execution message timing for Program Editor mode
    React.useEffect(() => {
        if (programMode === "Program Editor" && isExecutingProgram) {
            setShowExecutionMessage(true);
            const timer = setTimeout(() => {
                setShowExecutionMessage(false);
            }, 5000);
            return () => clearTimeout(timer);
        } else {
            setShowExecutionMessage(false);
        }
    }, [programMode, isExecutingProgram]);
    
    // Function to handle "Done teleoperating" button click
    const handleDoneTeleoperating = () => {
        setWaitingForUserConfirmation(false);
        setIsTakeControlActive(false);
        // Set execution state back to true
        if ((window as any).buttonFunctionProvider) {
            (window as any).buttonFunctionProvider.setExecutionState(true);
        }
        if ((window as any).resumeProgramExecution) {
            (window as any).resumeProgramExecution();
            (window as any).resumeProgramExecution = null;
        }
    };
    
    // Effect to detect when TakeControl is called
    React.useEffect(() => {
        const checkForTakeControl = () => {
            if (!waitingForUserConfirmation && (window as any).resumeProgramExecution) {
                setWaitingForUserConfirmation(true);
                setIsTakeControlActive(true);
            }
        };
        
        const interval = setInterval(checkForTakeControl, 100);
        return () => clearInterval(interval);
    }, [waitingForUserConfirmation]);

    // Function to handle "Confirm and Proceed" button click
    const handleConfirmAndProceed = () => {
        if ((window as any).pauseAndConfirmResolve) {
            (window as any).pauseAndConfirmResolve();
            (window as any).pauseAndConfirmResolve = null;
            (window as any).pauseAndConfirmMessage = null;
        }
    };

    // Function to handle "Reset" button click
    const handleReset = () => {
        // Home the robot
        if ((window as any).remoteRobot) {
            (window as any).remoteRobot.homeTheRobot();
        }
        // Also resolve the promise to continue program execution
        if ((window as any).pauseAndConfirmResolve) {
            (window as any).pauseAndConfirmResolve();
            (window as any).pauseAndConfirmResolve = null;
            (window as any).pauseAndConfirmMessage = null;
        }
    };

    // Effect to detect when PauseAndConfirm is called
    React.useEffect(() => {
        const checkForPauseAndConfirm = () => {
            if (isExecutingProgram && !showPopup && (window as any).pauseAndConfirmResolve && (window as any).pauseAndConfirmMessage) {
                setPauseAndConfirmMessage((window as any).pauseAndConfirmMessage);
                setShowPopup(true);
            }
        };
        
        const interval = setInterval(checkForPauseAndConfirm, 100);
        return () => clearInterval(interval);
    }, [isExecutingProgram, showPopup]);
    const [buttonCollision, setButtonCollision] = React.useState<
        ButtonPadButton[]
    >([]);
    const [moveBaseState, setMoveBaseState] = React.useState<ActionState>();
    const [moveToPregraspState, setMoveToPregraspState] =
        React.useState<ActionState>();
    const [showTabletState, setShowTabletState] =
        React.useState<ActionState>();
    const [robotNotHomed, setRobotNotHomed] =
        React.useState<boolean>(false);
    function showHomeTheRobotGlobalControl(isHomed: boolean) {
        setRobotNotHomed(!isHomed);
    }
    homeTheRobotFunctionProvider.setIsHomedCallback(
        showHomeTheRobotGlobalControl
    );

    const layout = React.useRef<LayoutDefinition>(props.layout);
    
    // Mode-specific layouts
    const [modeLayouts, setModeLayouts] = React.useState<{ [mode: string]: LayoutDefinition }>({
        "Demonstrate": props.layout,
        "Program Editor": props.layout,
        "Execution Monitor": props.layout,
    });

    // Initialize mode-specific layouts
    React.useEffect(() => {
        const initializeModeLayouts = () => {
            const programModes = ["Demonstrate", "Program Editor", "Execution Monitor"];
            const initialLayouts: { [mode: string]: LayoutDefinition } = {};
            
            programModes.forEach(mode => {
                const savedLayout = props.storageHandler.loadCurrentLayout(mode);
                // always use default layouts to ensure proper structure
                if (mode === "Demonstrate") {
                    initialLayouts[mode] = props.storageHandler.loadDefaultLayout("Basic Layout" as any);
                } else if (mode === "Program Editor") {
                    initialLayouts[mode] = props.storageHandler.loadDefaultLayout("Program Editor Layout" as any);
                } else if (mode === "Execution Monitor") {
                    initialLayouts[mode] = props.storageHandler.loadDefaultLayout("Execution Monitor Layout" as any);
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
    
    // Set up execution state callback
    function executionStateCallback(isExecuting: boolean) {
        setIsExecutingProgram(isExecuting);
        // Reset TakeControl state when execution starts
        if (isExecuting) {
            setIsTakeControlActive(false);
        }
    }
    buttonFunctionProvider.setExecutionStateCallback(executionStateCallback);

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
        props.storageHandler.saveCurrentLayout(layout.current);
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
            props.storageHandler.saveCurrentLayout(layout.current);
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
        // Get isExecutingProgram from the state managed by ButtonFunctionProvider
        isExecutingProgram: isExecutingProgram,
        isTakeControlActive: isTakeControlActive,
        programMode: programMode,
        currentExecutingLine: currentExecutingLine,
        updateCurrentExecutingLine: updateCurrentExecutingLine,
        waitingForUserConfirmation: waitingForUserConfirmation,
        handleDoneTeleoperating: handleDoneTeleoperating,
        setExecutionError: setExecutionError,
        clearExecutionError: () => {
            setExecutionError(null);
            setErrorLineNumber(null);
        },
        setErrorLineNumber: setErrorLineNumber,
        executionError: executionError,
        errorLineNumber: errorLineNumber,
    };
    


    /** Properties for the global options area of the sidebar */
    const globalOptionsProps: GlobalOptionsProps = {
        displayMovementRecorder: layout.current.displayMovementRecorder,
        displayTextToSpeech: layout.current.displayTextToSpeech,
        displayRosbagRecorder: layout.current.displayRosbagRecorder,
        displayLabels: layout.current.displayLabels,
        setDisplayMovementRecorder: setDisplayMovementRecorder,
        setDisplayTextToSpeech: setDisplayTextToSpeech,
        setDisplayRosbagRecorder: setDisplayRosbagRecorder,
        setDisplayLabels: setDisplayLabels,
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
    const programModes = ["Demonstrate", "Program Editor", "Execution Monitor"];

    // Function to switch layouts when program mode changes
    const switchToModeLayout = (newMode: string) => {
        // Save current layout for current mode
        if (modeLayouts[programMode]) {
            const updatedLayouts = { ...modeLayouts };
            updatedLayouts[programMode] = layout.current;
            setModeLayouts(updatedLayouts);
            props.storageHandler.saveCurrentLayout(layout.current, programMode);
        }
        
        // Load layout for new mode from our initialized modeLayouts
        if (modeLayouts[newMode]) {
            layout.current = modeLayouts[newMode];
        } else {
            const newModeLayout = props.storageHandler.loadCurrentLayout(newMode);
            if (newModeLayout) {
                layout.current = newModeLayout;
            } else {
                // Load default layouts for each mode
                if (newMode === "Demonstrate") {
                    layout.current = props.storageHandler.loadDefaultLayout("Basic Layout" as any);
                } else if (newMode === "Program Editor") {
                    layout.current = props.storageHandler.loadDefaultLayout("Program Editor Layout" as any);
                } else if (newMode === "Execution Monitor") {
                    layout.current = props.storageHandler.loadDefaultLayout("Execution Monitor Layout" as any);
                }
            }
        }
        
        updateLayout();
    };

    return (
        <div id="operator">
            {/* Persistent banner for control mode - only show in Execution Monitor mode when program is executing */}
            {programMode === "Execution Monitor" && isExecutingProgram && (
                <div
                    style={{
                        width: "100%",
                        background: isExecutingProgram ? "#ff9800" : "#4caf50",
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
                    {isExecutingProgram ? "Robot in control" : "You are in control"}
                </div>
            )}
            




            {/* Global controls */}
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
            <div id="operator-header" onClick={handleClickHeader} style={{ 
                display: "flex", 
                flexDirection: "column", 
                padding: window.innerWidth < 1200 ? "8px 12px" : "12px 20px" 
            }}>
                {/* Centered Title */}
                <div style={{ 
                    display: "flex", 
                    justifyContent: "center", 
                    marginBottom: window.innerWidth < 1200 ? "3px" : "5px",
                    width: "100%"
                }}>
                    <span style={{
                        fontSize: window.innerWidth < 1200 ? "16px" : "18px",
                        fontWeight: "bold",
                        color: "var(--text-color)",
                        textTransform: "capitalize"
                    }}>
                        {programMode}
                    </span>
                </div>
                
                {/* Header Controls */}
                <div style={{ 
                    display: "flex", 
                    alignItems: "center", 
                    justifyContent: "space-between",
                    width: "100%",
                    minHeight: window.innerWidth < 1200 ? "32px" : "40px",
                    flexWrap: window.innerWidth < 1000 ? "wrap" : "nowrap",
                    gap: window.innerWidth < 1000 ? "8px" : "0"
                }}>
                    {/* Left side controls */}
                    <div style={{ display: "flex", alignItems: "center", flex: "0 0 auto" }}>
                        {/* Program mode dropdown */}
                        <div style={{ 
                            display: "flex", 
                            alignItems: "center",
                            height: "40px"
                        }}>
                            <style>
                                {`
                                    .header-dropdown .dropdown-button {
                                        padding-top: 0.5rem !important;
                                        padding-bottom: 0.5rem !important;
                                        height: 40px !important;
                                        display: flex !important;
                                        align-items: center !important;
                                    }
                                `}
                            </style>
                            <div className="header-dropdown">
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
                            </div>
                        </div>
                        {/* Action mode dropdown - hide in Program Editor mode */}
                        {programMode !== "Program Editor" && (
                            <div style={{ 
                                marginLeft: window.innerWidth < 1200 ? 8 : 16, 
                                height: window.innerWidth < 1200 ? "32px" : "40px" 
                            }}>
                                <div className="header-dropdown">
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
                            </div>
                        )}
                    </div>
                    
                    {/* Center controls - hide in Program Editor mode */}
                    {programMode !== "Program Editor" && (
                        <div style={{ 
                            display: "flex", 
                            alignItems: "center", 
                            justifyContent: "center",
                            flex: "1 1 auto",
                            gap: window.innerWidth < 1200 ? "12px" : "16px"
                        }}>
                            <AudioControl remoteStreams={remoteStreams} />
                            <SpeedControl
                                scale={velocityScale}
                                onChange={(newScale: number) => {
                                    setVelocityScale(newScale);
                                    FunctionProvider.velocityScale = newScale;
                                }}
                            />
                        </div>
                    )}
                    
                    {/* Right side controls */}
                    <div style={{ 
                        display: "flex", 
                        alignItems: "center", 
                        flex: "0 0 auto",
                        gap: "8px"
                    }}>
                        <CustomizeButton
                            customizing={customizing}
                            onClick={handleToggleCustomize}
                        />
                        {/* Home Robot Button */}
                        <button
                            onClick={async () => {
                                if ((window as any).remoteRobot) {
                                    const retractedPose = { wrist_extension: 0.00211174 };
                                    (window as any).remoteRobot.setRobotPose(retractedPose);
                                    console.log(`Arm retraction command sent to robot!`);
                                    console.log(`Waiting for arm retraction...`);
                                    await new Promise(resolve => setTimeout(resolve, 2000));
                                    (window as any).remoteRobot.homeTheRobot();
                                    console.log(`Home robot command sent to robot!`);
                                } else {
                                    console.error("RemoteRobot not available");
                                }
                            }}
                            id="home-robot-button"
                            className="btn-turquoise font-white"
                            style={{
                                display: "flex",
                                alignItems: "center",
                                gap: "4px"
                            }}
                            title="Home the robot to its default position"
                        >
                            <HomeIcon />
                            <span>Home Robot</span>
                        </button>
                    </div>
                </div>
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
            {showPopup && programMode === "Execution Monitor" && (
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
                            {pauseAndConfirmMessage || "Ready to continue? Please confirm before the robot proceeds or reset to revise."}
                        </div>
                        <div style={{ display: "flex", justifyContent: "center", gap: 16 }}>
                            <button
                                style={{
                                    backgroundColor: "#f44336",
                                    color: "white",
                                    border: "none",
                                    padding: "10px 20px",
                                    fontWeight: "bold",
                                    borderRadius: "5px",
                                    cursor: "pointer"
                                }}
                                onClick={() => {
                                    setShowPopup(false);
                                    handleReset();
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
                                    cursor: "pointer",
                                    display: "flex",
                                    alignItems: "center",
                                    gap: "4px"
                                }}
                                onClick={() => {
                                    setShowPopup(false);
                                    handleConfirmAndProceed();
                                }}
                            >
                                <CheckIcon style={{ fontSize: "1em" }} />
                                Confirm and proceed
                            </button>
                        </div>
                    </div>
                </div>
            )}
            <div id="operator-body">
                <LayoutArea layout={layout.current} sharedState={sharedState} />
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
    );
};
