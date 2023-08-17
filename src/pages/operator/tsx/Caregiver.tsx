import React from "react";
import { ActionMode, ButtonPadId, CameraViewId, ComponentDefinition, ComponentType } from "./utils/component_definitions";
import { ArucoNavigationState, className, MoveBaseState, RemoteStream } from "shared/util";
import { arucoMarkerFunctionProvider, buttonFunctionProvider, underMapFunctionProvider } from ".";
import { ButtonPadButton, ButtonState, ButtonStateMap } from "./function_providers/ButtonFunctionProvider";
import { StorageHandler } from "./storage_handler/StorageHandler";
import { FunctionProvider } from "./function_providers/FunctionProvider";
import "operator/css/Caregiver.css"
import { SimpleCameraView } from "./layout_components/SimpleCameraView";
import { SharedState } from "./layout_components/CustomizableComponent";
import { VirtualJoystick } from "./layout_components/VirtualJoystick";
import { ButtonPad } from "./layout_components/ButtonPad";
import { getIcon } from "./utils/svg";

/** Operator interface webpage */
export const Caregiver = (props: {
    remoteStreams: Map<string, RemoteStream>,
    storageHandler: StorageHandler
}) => {
    const [velocityScale, setVelocityScale] = React.useState<number>(FunctionProvider.velocityScale);
    const [buttonCollision, setButtonCollision] = React.useState<ButtonPadButton[]>([]);
    const [arucoNavigationState, setArucoNavigationState] = React.useState<ArucoNavigationState>()
    const [moveBaseState, setMoveBaseState] = React.useState<MoveBaseState>()
    const [cameraID, setCameraID] = React.useState<CameraViewId>(CameraViewId.realsense)
    const [rerender, setRerender] = React.useState<boolean>(false);

    FunctionProvider.actionMode = ActionMode.PressRelease;

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
        if (arucoNavigationState && arucoNavigationState.alertType != "info") {
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
        if (moveBaseState && moveBaseState.alertType != "info") {
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

    /** State passed from the operator and shared by all components */
    const sharedState: SharedState = {
        customizing: false,
        onSelect: () => {},
        remoteStreams: remoteStreams,
        selectedPath: "deselected",
        dropZoneState: {
            onDrop: () => {},
            selectedDefinition: undefined
        },
        buttonStateMap: buttonStateMap.current,
        hideLabels: false
    }

    return (
        <div id="caregiver" onContextMenu={(e)=> e.preventDefault()}>
            <div id="caregiver-body">
                {/* <LayoutArea
                    layout={layout.current}
                    sharedState={sharedState}
                /> */}
                <div className={'switch-camera'}>
                    <button onPointerDown={() => {
                        if (cameraID == CameraViewId.realsense) setCameraID(CameraViewId.overhead)
                        else if (cameraID == CameraViewId.overhead) setCameraID(CameraViewId.gripper)
                        else if (cameraID == CameraViewId.gripper) setCameraID(CameraViewId.realsense)
                        setRerender(!rerender)
                    }}>
                        <span className="material-icons">
                            photo_camera
                        </span>
                    </button>
                </div>
                <SimpleCameraView id={ cameraID } remoteStreams={ remoteStreams }/>
                <div className="control-modes">
                    <button>Drive</button>
                    <button>Arm</button>
                    <button>Gripper</button>
                </div>
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
                        id: ButtonPadId.ArmMobile 
                        },
                        sharedState: sharedState,
                        overlay: false,
                        aspectRatio: 1.66
                    }}
                />
            </div>
        </div>
    )
}