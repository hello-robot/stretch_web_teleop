import React, { PointerEventHandler, useState } from "react";
import {
    ActionMode,
    ButtonPadIdMobile,
    CameraViewId,
    ComponentDefinition,
    ComponentType,
    MapDefinition,
} from "./utils/component_definitions";
import {
    ActionState,
    ButtonAction,
    className,
    ActionState as MoveBaseState,
    RemoteStream,
} from "shared/util";
import {
    buttonFunctionProvider,
    hasBetaTeleopKit,
    stretchTool,
    movementRecorderFunctionProvider,
    underMapFunctionProvider,
    underVideoFunctionProvider,
    homeTheRobotFunctionProvider,
} from ".";
import {
    ButtonPadButton,
    ButtonState,
    ButtonStateMap,
} from "./function_providers/ButtonFunctionProvider";
import { StorageHandler } from "./storage_handler/StorageHandler";
import { FunctionProvider } from "./function_providers/FunctionProvider";
import "operator/css/MobileOperator.css";
import { SimpleCameraView } from "./layout_components/SimpleCameraView";
import { SharedState } from "./layout_components/CustomizableComponent";
import { VirtualJoystick } from "./layout_components/VirtualJoystick";
import { ButtonPad } from "./layout_components/ButtonPad";
import Swipe from "./static_components/Swipe";
import { Map } from "./layout_components/Map";
import { TabGroup } from "./basic_components/TabGroup";
import { RadioFunctions, RadioGroup } from "./basic_components/RadioGroup";
import {
    MovementRecorder,
    MovementRecorderFunction,
} from "./layout_components/MovementRecorder";
import { CheckToggleButton } from "./basic_components/CheckToggleButton";
import { UnderVideoButton } from "./function_providers/UnderVideoFunctionProvider";
import { Alert } from "./basic_components/Alert";
import PhotoCameraIcon from "@mui/icons-material/PhotoCamera";
import RadioButtonCheckedIcon from "@mui/icons-material/RadioButtonChecked";
import MapIcon from "@mui/icons-material/Map";
import { SpeedControl } from "./static_components/SpeedControl";
import { ScreenRecorder } from "./static_components/ScreenRecorder";
import { HomeTheRobot } from "./layout_components/HomeTheRobot";
import { PopupModal } from "./basic_components/PopupModal";
import { isBrowser, isTablet } from "react-device-detect";

/** Operator interface webpage */
export const MobileOperator = (props: {
    remoteStreams: Map<string, RemoteStream>;
    storageHandler: StorageHandler;
}) => {
    const [buttonCollision, setButtonCollision] = React.useState<
        ButtonPadButton[]
    >([]);
    const [moveBaseState, setMoveBaseState] = React.useState<MoveBaseState>();
    const [cameraID, setCameraID] = React.useState<CameraViewId>(
        CameraViewId.realsense,
    );
    const [velocityScale, setVelocityScale] = React.useState<number>(0.8);
    const [hideMap, setHideMap] = React.useState<boolean>(true);
    const [hideControls, setHideControls] = React.useState<boolean>(false);
    const [activeMainGroupTab, setActiveMainGroupTab] =
        React.useState<number>(0);
    const [activeControlTab, setActiveControlTab] = React.useState<number>(0);
    const [isRecording, setIsRecording] = React.useState<boolean>();
    const [depthSensing, setDepthSensing] = React.useState<boolean>(false);
    const [showAlert, setShowAlert] = React.useState<boolean>(true);
    const controlTabLabels = ["Drive", "Arm", "Gripper"];
    const headingTabLabels = ["Controls", "Recordings"];
    const [showRecordModal, setShowRecordModal] = React.useState<boolean>(true);
    
    React.useEffect(() => {
        setTimeout(function () {
            setShowAlert(false);
        }, 5000);
    }, []);

    FunctionProvider.actionMode = ActionMode.PressAndHold;

    // Just used as a flag to force the operator to rerender when the button state map
    // has been updated
    const [buttonStateMapRerender, setButtonStateMapRerender] =
        React.useState<boolean>(false);
    const buttonStateMap = React.useRef<ButtonStateMap>();
    function operatorCallback(bsm: ButtonStateMap) {
        let collisionButtons: ButtonPadButton[] = [];
        bsm.forEach((state, button) => {
            if (state == ButtonState.Collision) collisionButtons.push(button);
        });
        setButtonCollision(collisionButtons);
        if (bsm !== buttonStateMap.current) {
            buttonStateMap.current = bsm;
            setButtonStateMapRerender(!buttonStateMapRerender);
        }
    }
    buttonFunctionProvider.setOperatorCallback(operatorCallback);

    // Just used as a flag to force the operator to rerender when the tablet orientation
    // changes.
    const [tabletOrientationRerender, setTabletOrientationRerender] =
        React.useState<boolean>(false);
    underVideoFunctionProvider.setTabletOrientationOperatorCallback((_) => {
        setTabletOrientationRerender(!tabletOrientationRerender);
    });

    const [robotNotHomed, setRobotNotHomed] =
        React.useState<ActionState>(false);
    homeTheRobotFunctionProvider.setIsHomedCallback(
        (isHomed: ActionState) => setRobotNotHomed(!isHomed)
    );

    function moveBaseStateCallback(state: MoveBaseState) {
        setMoveBaseState(state);
    }
    underMapFunctionProvider.setOperatorCallback(moveBaseStateCallback);
    let moveBaseAlertTimeout: NodeJS.Timeout;
    React.useEffect(() => {
        if (moveBaseState && moveBaseState.alertType != "info") {
            if (moveBaseAlertTimeout) clearTimeout(moveBaseAlertTimeout);
            moveBaseAlertTimeout = setTimeout(() => {
                setMoveBaseState(undefined);
            }, 5000);
        }
    }, [moveBaseState]);

    let remoteStreams = props.remoteStreams;

    /** State passed from the operator and shared by all components */
    const sharedState: SharedState = {
        customizing: false,
        onSelect: () => {},
        remoteStreams: remoteStreams,
        selectedPath: "deselected",
        dropZoneState: {
            onDrop: () => {},
            selectedDefinition: undefined,
        },
        buttonStateMap: buttonStateMap.current,
        hideLabels: false,
        hasBetaTeleopKit: hasBetaTeleopKit,
        stretchTool: stretchTool,
    };

    function updateScreens() {
        if (hideMap) {
            setHideMap(false);
            setHideControls(true);
        } else {
            setHideControls(false);
            setHideMap(true);
        }
    }
    const swipeHandlers = Swipe({
        onSwipedLeft: () => updateScreens(),
        onSwipedRight: () => updateScreens(),
    });

    const driveMode = (show: boolean) => {
        return show ? (
            <React.Fragment key={"drive-mode"}>
                {/* <VirtualJoystick
                {...{
                    path: "",
                    definition: { type: ComponentType.VirtualJoystick },
                    sharedState: sharedState
                }}
            /> */}
                <ButtonPad
                    {...{
                        path: "",
                        definition: {
                            type: ComponentType.ButtonPad,
                            id: ButtonPadIdMobile.Drive,
                        },
                        sharedState: sharedState,
                        overlay: false,
                        aspectRatio: 3.35,
                    }}
                />
            </React.Fragment>
        ) : (
            <></>
        );
    };

    const armMode = (show: boolean) => {
        return show ? (
            <React.Fragment key={"arm-mode"}>
                <ButtonPad
                    {...{
                        path: "",
                        definition: {
                            type: ComponentType.ButtonPad,
                            id: ButtonPadIdMobile.Arm,
                        },
                        sharedState: sharedState,
                        overlay: false,
                        aspectRatio: 3.35,
                    }}
                />
            </React.Fragment>
        ) : (
            <></>
        );
    };

    const gripperMode = (show: boolean) => {
        return show ? (
            <React.Fragment key={"gripper-mode"}>
                <ButtonPad
                    {...{
                        path: "",
                        definition: {
                            type: ComponentType.ButtonPad,
                            id: ButtonPadIdMobile.Gripper,
                        },
                        sharedState: sharedState,
                        overlay: false,
                        aspectRatio: 3.35,
                    }}
                />
            </React.Fragment>
        ) : (
            <></>
        );
    };

    const ControlModes = () => {
        return (
            <>
                <SpeedControl
                    scale={velocityScale}
                    onChange={(newScale: number, newLabel?: string) => {
                        setVelocityScale(newScale);
                        FunctionProvider.velocityScale = newScale;
                        if (newLabel) 
                            FunctionProvider.logButtonAction(ButtonAction.CLICK, newLabel)
                    }}
                />
                {/* <div className="slider-container">
                    <span className="label">Slow</span>
                    <input
                        type="range"
                        className="slider"
                        min="0.2"
                        max="1.6"
                        defaultValue={velocityScale}
                        step="0.01"
                        onPointerUp={(event) => {
                            setVelocityScale(FunctionProvider.velocityScale);
                        }}
                        onChange={(event) => {
                            FunctionProvider.velocityScale = Number(
                                event.target.value,
                            );
                        }}
                    />
                    <span className="label">Fast</span>
                </div> */}
                <TabGroup
                    tabLabels={controlTabLabels}
                    tabContent={[driveMode, armMode, gripperMode]}
                    startIdx={activeControlTab}
                    onChange={(index: number) => {
                        setActiveControlTab(index)
                        FunctionProvider.logButtonAction(ButtonAction.CLICK, controlTabLabels[index] + " Tab")
                    }}
                    pill={true}
                    key={"controls-group"}
                />
            </>
        );
    };

    const controlModes = (show: boolean) => {
        return show ? <ControlModes key={"control-modes"} /> : <></>;
    };
    const recordingList = (show: boolean) => {
        return (
            <MovementRecorder
                globalRecord={show}
                hideLabels={false}
                isRecording={isRecording}
            />
        );
    };

    const LoomRecordingModal = (props: {
        setShow: (show: boolean) => void;
        show: boolean;
    }) => {

        function handleAccept() {
            props.setShow(false)
        }

        return (
            <PopupModal
                setShow={props.setShow}
                show={props.show}
                onAccept={handleAccept}
                id="start-recording-modal"
                acceptButtonText="Continue"
                size={!isBrowser && !isTablet  ? "small" : "large"}
                mobile={!isBrowser && !isTablet }
            >
                <p>Please start the Loom recording, then press "Continue".</p>
            </PopupModal>
        );
    };

    return (
        <div id="mobile-operator" onContextMenu={(e) => e.preventDefault()}>
            <div id="mobile-operator-body">
                {/* {showAlert ? (
                    <div className="mobile-alert">
                        <Alert type="error">
                            <span>Beta feature, use at your own risk</span>
                        </Alert>
                    </div>
                ) : (
                    <></>
                )} */}
                {showRecordModal && <LoomRecordingModal setShow={setShowRecordModal} show={showRecordModal}/>}
                {robotNotHomed && (
                    <div className="operator-collision-alerts">
                        <div
                            className={className("mobile-alert", {
                                fadeIn: robotNotHomed,
                                fadeOut: !robotNotHomed,
                            })}
                        >
                            <HomeTheRobot
                                hideLabels={false}
                            />
                        </div>
                    </div>
                )}
                {/* <ScreenRecorder/> */}
                <div className={className("controls", { hideControls })}>
                    <div className={"switch-camera"}>
                        <button
                            onPointerDown={() => {
                                let newCameraID = cameraID
                                if (cameraID == CameraViewId.realsense)
                                    newCameraID = CameraViewId.overhead;
                                else if (cameraID == CameraViewId.overhead)
                                    newCameraID = CameraViewId.gripper;
                                else if (cameraID == CameraViewId.gripper)
                                    newCameraID = CameraViewId.realsense;

                                setCameraID(newCameraID)
                                FunctionProvider.logButtonAction(ButtonAction.CLICK, newCameraID + " Camera")
                            }}
                        >
                            <PhotoCameraIcon
                                className="material-icons icon"
                            />
                        </button>
                        <button
                            onPointerDown={() => {
                                setHideMap(false);
                                setHideControls(true);
                                FunctionProvider.logButtonAction(ButtonAction.CLICK, "Map")
                            }}
                        >
                            <MapIcon
                                className="material-icons icon"
                            />
                        </button>
                    </div>
                    {/* {cameraID == CameraViewId.realsense && (
                        <div className="depth-sensing">
                            <CheckToggleButton
                                checked={depthSensing}
                                onClick={() => {
                                    setDepthSensing(!depthSensing);
                                    underVideoFunctionProvider.provideFunctions(
                                        UnderVideoButton.DepthSensing,
                                    ).onCheck!(!depthSensing);
                                }}
                                label="Depth Sensing"
                            />
                        </div>
                    )} */}
                    {/* <button
                        className="record"
                        onPointerDown={() => {
                            setIsRecording(!isRecording);
                        }}
                    >
                        {!isRecording ? (
                            <>
                                <RadioButtonCheckedIcon/>
                                <i>Record</i>
                            </>
                        ) : (
                            <>
                                <div className="recording"></div>
                                <div className="record-circle"></div>
                                <i>Stop Recording</i>
                            </>
                        )}
                    </button> */}
                    {/* <div {...swipeHandlers}> */}
                    <div>
                        <SimpleCameraView
                            id={cameraID}
                            remoteStreams={remoteStreams}
                        />
                    </div>
                    <TabGroup
                        tabLabels={headingTabLabels}
                        tabContent={[controlModes, recordingList]}
                        startIdx={activeMainGroupTab}
                        onChange={(index: number) => {
                            setActiveMainGroupTab(index)
                            FunctionProvider.logButtonAction(ButtonAction.CLICK, headingTabLabels[index] + " Tab")
                        }}
                        pill={false}
                        key={"main-group"}
                    />
                </div>
                {/* <div className={className('map', {hideMap})} {...swipeHandlers}> */}
                <div className={className("map", { hideMap })}>
                    <div className={"switch-camera"}>
                        <button
                            onPointerDown={() => {
                                setHideMap(true);
                                setHideControls(false);
                                FunctionProvider.logButtonAction(ButtonAction.CLICK, cameraID + " Camera")
                            }}
                        >
                            <PhotoCameraIcon
                                className="material-icons icon"
                            />
                        </button>
                    </div>
                    <Map
                        {...{
                            path: "",
                            definition: {
                                type: ComponentType.Map,
                                selectGoal: false,
                            } as MapDefinition,
                            sharedState: sharedState,
                        }}
                    />
                </div>
            </div>
        </div>
    );
};
