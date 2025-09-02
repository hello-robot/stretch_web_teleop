import React, { useState, useCallback, useMemo, useEffect } from 'react';
import SwipeableViews from 'react-swipeable-views';

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
import FooterGripperCam from './FooterGripperCam';
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

type ClickProps = {
    onPointerDown?: React.PointerEventHandler<HTMLButtonElement>;
    onPointerUp?: React.PointerEventHandler<HTMLButtonElement>;
    onPointerCancel?: React.PointerEventHandler<HTMLButtonElement>;
    onPointerLeave?: React.PointerEventHandler<HTMLButtonElement>;
};

const GripperCam: React.FC<GripperCamProps> = ({
    cameraID,
    remoteStreams,
    isCameraVeilVisible,
    isCameraVeilVisibleSet,
    setVelocityScale,
    setActionMode,
    swipeableViewsIdxSet,
}) => {
    const [activeTabIndex, setActiveTabIndex] = useState<number>(0);

    useEffect(() => {
        // By default, circular masking should be off
        underVideoFunctionProvider
            .provideFunctions(UnderVideoButton.ExpandedGripperView)
            .onCheck(true);
    }, []);

    const clickPropsGet = useCallback((funct: ButtonPadButton): ClickProps => {
        const functs: ButtonFunctions = buttonFunctionProvider.provideFunctions(funct);
        return {
            onPointerDown: functs.onClick,
            onPointerUp: functs.onRelease,
            onPointerCancel: functs.onRelease,
            onPointerLeave: functs.onLeave,
        };
    }, []);

    const onChangeIndex = useCallback((index: number) => {
        setActiveTabIndex(index);
    }, []);

    const tabButtons = useMemo(
        () => [
            { name: 'Arm & Gripper' },
            { name: 'Wrist' },
            { name: 'Grab Assist' },
        ],
        []
    );
    return (
        <div className="grippercam">
            <div className="simple-camera-view-wrapper_XP">
                <SimpleCameraView
                    id={cameraID}
                    remoteStreams={remoteStreams}
                    isCameraVeilVisible={isCameraVeilVisible}
                />
                <div className="grippercam-controls">
                    <div className="grippercam-tab-buttons">
                        {tabButtons.map((item, index) => {
                            return (
                                <button onClick={() => onChangeIndex(index)} className={`grippercam-tab-button  ${activeTabIndex === index ? "active" : "inactive"}`}>
                                    {item.name}
                                </button>

                            )
                        })}
                    </div>
                    <SwipeableViews index={activeTabIndex} onChangeIndex={onChangeIndex} className="grippercam-buttons">
                        <div className="grippercam-buttons-group-wrapper">
                            <div className="grippercam-buttons-group">
                                <p>Arm</p>
                                <div className="button-group">
                                    <button {...clickPropsGet(ButtonPadButton.ArmLift)}>Up</button>
                                    <button {...clickPropsGet(ButtonPadButton.ArmLower)}>Down</button>
                                </div>
                            </div>
                            <div className="grippercam-buttons-group">
                                <p>Gripper</p>
                                <div className="button-group">
                                    <button {...clickPropsGet(ButtonPadButton.GripperOpen)}>Open</button>
                                    <button {...clickPropsGet(ButtonPadButton.GripperClose)}>Close</button>
                                </div>
                            </div>
                            <div className="grippercam-buttons-group">
                                <p>Arm</p>
                                <div className="button-group">
                                    <button {...clickPropsGet(ButtonPadButton.ArmRetract)}>In</button>
                                    <button {...clickPropsGet(ButtonPadButton.ArmExtend)}>Out</button>
                                </div>
                            </div>
                        </div>
                        <div className="grippercam-buttons-group-wrapper">
                            <div className="grippercam-buttons-group">
                                <p>Turn Wrist</p>
                                <div className="button-group">
                                    <button {...clickPropsGet(ButtonPadButton.WristRotateIn)}>Left</button>
                                    <button {...clickPropsGet(ButtonPadButton.WristRotateOut)}>Right</button>
                                </div>
                            </div>
                            <div className="grippercam-buttons-group">
                                <p>Bend Wrist</p>
                                <div className="button-group">
                                    <button {...clickPropsGet(ButtonPadButton.WristPitchUp)}>Up</button>
                                    <button {...clickPropsGet(ButtonPadButton.WristPitchDown)}>Down</button>
                                </div>
                            </div>
                            <div className="grippercam-buttons-group">
                                <p>Rotate Wrist</p>
                                <div className="button-group">
                                    <button {...clickPropsGet(ButtonPadButton.WristRollLeft)}>Left</button>
                                    <button {...clickPropsGet(ButtonPadButton.WristRollRight)}>Right</button>
                                </div>
                            </div>
                        </div>
                        <div className="grippercam-buttons-group-wrapper">
                            <div style={{ marginTop: 20 }}>
                                😊
                            </div>
                        </div>
                    </SwipeableViews>
                </div>
            </div>
            <FooterGripperCam
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
        </div >
    );
};

export default GripperCam;