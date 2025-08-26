import React, { PointerEventHandler, useState } from "react";
import {
    ActionModeType,
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
import { SharedState } from "./layout_components/CustomizableComponent";
import { ButtonPad } from "./layout_components/ButtonPad";
import { TabGroup } from "./basic_components/TabGroup";
import SwipeableViews from 'react-swipeable-views';
import {
    MovementRecorder,
    MovementRecorderFunction,
} from "./layout_components/MovementRecorder";
import GripperCam from "./layout_components/GripperCam";
import HeadCam from "./layout_components/HeadCam";
import AutoNav from "./layout_components/AutoNav";
import { SimpleCameraView } from "./layout_components/SimpleCameraView";
import FooterHeadCam from "./layout_components/FooterHeadCam";
import { Map } from "./layout_components/Map";
import { CheckToggleButton } from "./basic_components/CheckToggleButton";
import { UnderVideoButton } from "./function_providers/UnderVideoFunctionProvider";
import { Alert } from "./basic_components/Alert";
import { VirtualJoystick } from "./layout_components/VirtualJoystick";
import { RadioFunctions, RadioGroup } from "./basic_components/RadioGroup";

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
    const [velocityScale, setVelocityScale] = React.useState<number>(
        FunctionProvider.velocityScale
    );
    const [activeMainGroupTab, setActiveMainGroupTab] =
        React.useState<number>(0);
    const [activeControlTab, setActiveControlTab] = React.useState<number>(0);
    const [isRecording, setIsRecording] = React.useState<boolean>();
    const [depthSensing, setDepthSensing] = React.useState<boolean>(false);
    const [showAlert, setShowAlert] = React.useState<boolean>(true);
    // Manage the blurring+darkening of the camera feed
    const [isCameraVeilVisible, isCameraVeilVisibleSet] = useState(false);
    const [swipeableViewsIdx, swipeableViewsIdxSet] = useState<number>(0);
    const [swipeableViewsStyles, swipeableViewsStylesSet] = useState([
        { filter: 'brightness(1) blur(0px)' },
        { filter: 'brightness(1) blur(0px)' },
    ]);

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

    const layout = React.useRef<LayoutDefinition>(props.layout);
    FunctionProvider.actionMode = layout.current.actionMode;
    const actionModes = Object.values(ActionModeType);
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
    function setActionMode(actionMode: ActionModeType) {
        layout.current.actionMode = actionMode;
        FunctionProvider.actionMode = actionMode;
        props.storageHandler.saveCurrentLayout(layout.current);
        // updateLayout();
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
                            id: ButtonPadIdMobile.OmniDrive,
                        },
                        sharedState: sharedState,
                        overlay: false,
                        aspectRatio: 3.35,
                        isCameraVeilVisible: isCameraVeilVisible,
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
                        isCameraVeilVisible: isCameraVeilVisible,
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
                        isCameraVeilVisible: isCameraVeilVisible,
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
                <SwipeableViews
                    className="swipeable-views"
                    index={swipeableViewsIdx}
                    onChangeIndex={(idx: number) => (swipeableViewsIdxSet(idx))}
                    enableMouseEvents={true}
                    containerStyle={{ height: '100%' }}
                    slideStyle={{ overflowX: 'hidden', position: 'relative' }}
                    springConfig={{
                        duration: '0.2s',
                        easeFunction: 'cubic-bezier(0.15, 0.3, 0.25, 1)',
                        delay: '0s'
                    }}
                    // This "style" prop is required...
                    // CSS via "className" won't be applied.
                    style={{ overflowX: 'visible', height: '100%' }}
                    // Handler for animation brightness/blur fx
                    // as user is swiping between views
                    onSwitching={(slideOffset, type) => {
                        if (type === 'move') {
                            // Calculate filter values based on slide offset
                            const newStyles = swipeableViewsStyles.map((style, index) => {
                                // Determine the position of each slide relative to the active slide
                                const relativePosition = index - slideOffset;
                                const absPosition = Math.abs(relativePosition);

                                // Apply brightness and blur: reduce brightness and increase blur as slides move away
                                const brightness = Math.max(0.2, 1 - absPosition * 0.7); // Min brightness 0.5
                                const blur = Math.min(10, absPosition * 5); // Max blur 5px

                                return {
                                    filter: `brightness(${brightness}) blur(${blur}px)`,
                                };
                            });
                            swipeableViewsStylesSet(newStyles);
                        } else if (type === 'end') {
                            // Reset styles when transition ends
                            const newStyles = swipeableViewsStyles.map(() => ({
                                filter: 'brightness(1) blur(0px)',
                            }));
                            swipeableViewsStylesSet(newStyles);
                        }
                    }}
                >
                    <div
                        style={swipeableViewsStyles[0]}
                        className="gripper-wrapper"
                    >
                        <GripperCam
                            cameraID={CameraViewId.gripper}
                            isCameraVeilVisible={isCameraVeilVisible}
                            remoteStreams={remoteStreams}
                            setVelocityScale={setVelocityScale}
                            setActionMode={setActionMode}
                            isCameraVeilVisibleSet={isCameraVeilVisibleSet}
                            swipeableViewsIdxSet={swipeableViewsIdxSet} />
                    </div>

                    <div
                        style={swipeableViewsStyles[1]}
                        className="head-cam-wrapper"
                    >
                        <HeadCam
                            cameraID={CameraViewId.overhead}
                            isCameraVeilVisible={isCameraVeilVisible}
                            remoteStreams={remoteStreams}
                            tabContent={[controlModes, recordingList]}
                            activeMainGroupTab={activeMainGroupTab}
                            setActiveMainGroupTab={setActiveMainGroupTab}
                            setVelocityScale={setVelocityScale}
                            setActionMode={setActionMode}
                            isCameraVeilVisibleSet={isCameraVeilVisibleSet}
                            swipeableViewsIdxSet={swipeableViewsIdxSet}
                        />
                    </div>
                    <div
                        style={swipeableViewsStyles[2]}
                        className="auto-nav-wrapper"
                    >
                        <AutoNav
                            sharedState={sharedState}
                            swipeableViewsIdx={swipeableViewsIdx}
                            swipeableViewsIdxSet={swipeableViewsIdxSet}
                        />
                    </div>
                </SwipeableViews>
            </div>
        </div >
    );
};
