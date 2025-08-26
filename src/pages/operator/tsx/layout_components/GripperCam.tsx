import React from 'react';
import { SimpleCameraView } from './SimpleCameraView';
import { CameraViewId } from "../utils/component_definitions";
import {
    underVideoFunctionProvider,
    buttonFunctionProvider,
} from "..";
import {
    UnderVideoButton,
} from "../function_providers/UnderVideoFunctionProvider";
// import { TabGroup } from "../basic_components/TabGroup";
import FooterHeadCam from './FooterHeadCam';
import {
    ButtonFunctions,
    ButtonPadButton,
} from "../function_providers/ButtonFunctionProvider";
import { FunctionProvider } from "../function_providers/FunctionProvider";
import "../../css/GripperCam.css";

interface GripperCamProps {
    cameraID: CameraViewId;
    remoteStreams: any; // Replace 'any' with the actual type if known
    isCameraVeilVisible: boolean;
    isCameraVeilVisibleSet: (visible: boolean) => void;
    setVelocityScale: (speed: number) => void;
    setActionMode: (mode: string) => void;
    swipeableViewsIdxSet: (idx: number) => void;
}

const GripperCam: React.FC<GripperCamProps> = ({
    cameraID,
    remoteStreams,
    isCameraVeilVisible,
    isCameraVeilVisibleSet,
    setVelocityScale,
    setActionMode,
    swipeableViewsIdxSet,
}) => {

    // By default, circular masking should be off
    React.useEffect(() => {
        underVideoFunctionProvider.provideFunctions(
            UnderVideoButton.ExpandedGripperView,
        ).onCheck(true);
    }, []);

    const clickPropsGet = (funct) => {
        const functs: ButtonFunctions = buttonFunctionProvider.provideFunctions(funct);
        return {
            onPointerDown: functs.onClick,
            onPointerUp: functs.onRelease,
            onPointerCancel: functs.onRelease,
            onPointerLeave: functs.onLeave,
        };
    }


    return (
        <div className="gripper-cam-wrapper">
            <div className="simple-camera-view-wrapper_XP">
                <SimpleCameraView
                    id={cameraID}
                    remoteStreams={remoteStreams}
                    isCameraVeilVisible={isCameraVeilVisible}
                />
                {/* START Buttons! */}
                <button {...clickPropsGet(ButtonPadButton.ArmLift)}>Up</button>
                <button {...clickPropsGet(ButtonPadButton.ArmLower)}>Down</button>
                {/* END Buttons! */}
            </div>
            <FooterHeadCam
                actionSpeedCurrent={FunctionProvider.velocityScale}
                onActionSpeedChange={(newSpeed: number) => {
                    setVelocityScale(newSpeed);
                    FunctionProvider.velocityScale = newSpeed;
                }}
                actionModeCurrent={FunctionProvider.actionMode}
                onActionModeChange={setActionMode}
                isCameraVeilVisible={isCameraVeilVisible}
                isCameraVeilVisibleSet={isCameraVeilVisibleSet}
                swipeableViewsIdxSet={swipeableViewsIdxSet}
            />
        </div>
    );
};

export default GripperCam;