import React, { PointerEventHandler, useState } from "react";
import {
    ActionMode,
    ButtonPadIdMobile,
    CameraViewId,
    ComponentDefinition,
    ComponentType,
    MapDefinition,
    LayoutDefinition
} from "./utils/component_definitions";
import {
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
import FooterControls from "./layout_components/FooterControls";
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

/** Operator interface webpage */
export const MobileOperator = (props: {
    remoteStreams: Map<string, RemoteStream>;
    layout: LayoutDefinition;
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
    const [isCameraVeilVisible, isCameraVeilVisibleSet] = useState(false);

    React.useEffect(() => {
        setTimeout(function () {
            setShowAlert(false);
        }, 5000);
    }, []);

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




    FunctionProvider.actionMode = ActionMode.PressAndHold;
    const layout = React.useRef<LayoutDefinition>(props.layout);
    const actionModes = Object.values(ActionMode);
    const actionModesIdx = actionModes.indexOf(
        layout.current.actionMode
    )
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
    // onChange={(idx) => setActionMode(actionModes[idx])}





    // Just used as a flag to force the operator to rerender when the tablet orientation
    // changes.
    const [tabletOrientationRerender, setTabletOrientationRerender] =
        React.useState<boolean>(false);
    underVideoFunctionProvider.setTabletOrientationOperatorCallback((_) => {
        setTabletOrientationRerender(!tabletOrientationRerender);
    });

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
        onSelect: () => { },
        remoteStreams: remoteStreams,
        selectedPath: "deselected",
        dropZoneState: {
            onDrop: () => { },
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

                <TabGroup
                    // tabLabels={["Drive", "Arm", "Gripper"]}
                    // tabContent={[driveMode, armMode, gripperMode]}
                    tabLabels={["Drive"]}
                    tabContent={[driveMode]}
                    startIdx={activeControlTab}
                    onChange={(index: number) => setActiveControlTab(index)}
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

    return (
        <div id="mobile-operator" onContextMenu={(e) => e.preventDefault()}>
            <div id="mobile-operator-body">
                {showAlert ? (
                    <div className="mobile-alert">
                        <Alert type="error">
                            <span>Beta feature, use at your own risk</span>
                        </Alert>
                    </div>
                ) : (
                    <></>
                )}
                <div className={className("controls", { hideControls })}>
                    <div onPointerDown={() => {
                        if (cameraID == CameraViewId.realsense)
                            setCameraID(CameraViewId.overhead);
                        else if (cameraID == CameraViewId.overhead)
                            setCameraID(CameraViewId.gripper);
                        else if (cameraID == CameraViewId.gripper)
                            setCameraID(CameraViewId.realsense);
                    }}
                        className="simple-camera-view-wrapper_XP"
                    >
                        <SimpleCameraView
                            id={cameraID}
                            remoteStreams={remoteStreams}
                            isCameraVeilVisible={isCameraVeilVisible}
                        />
                    </div>
                    <TabGroup
                        tabLabels={["Controls", "Recordings"]}
                        tabContent={[controlModes, recordingList]}
                        startIdx={activeMainGroupTab}
                        onChange={(index: number) =>
                            setActiveMainGroupTab(index)
                        }
                        pill={false}
                        key={"main-group"}
                    />
                </div>
                <div className={className("map", { hideMap })}>
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
                <FooterControls
                    actionModes={actionModes}
                    actionModeCurrent={actionModes[actionModesIdx]}
                    isCameraVeilVisibleSet={isCameraVeilVisibleSet}
                />
            </div>
        </div>
    );
};
