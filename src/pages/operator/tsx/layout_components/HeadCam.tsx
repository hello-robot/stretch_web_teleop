import React from 'react';
import { SimpleCameraView } from './SimpleCameraView';
import { TabGroup } from "../basic_components/TabGroup";
import FooterHeadCam from './FooterHeadCam';
import { FunctionProvider } from "../function_providers/FunctionProvider";
import { CameraViewId } from "../utils/component_definitions";
import "../../css/HeadCam.css";

interface HeadCamProps {
    cameraID: CameraViewId;
    remoteStreams: any; // Replace 'any' with the actual type if known
    isCameraVeilVisible: boolean;
    isCameraVeilVisibleSet: (visible: boolean) => void;
    tabContent: ((active: boolean) => React.JSX.Element)[];
    activeMainGroupTab: number;
    setActiveMainGroupTab: (index: number) => void;
    setVelocityScale: (speed: number) => void;
    setActionMode: (mode: string) => void;
    swipeableViewsIdxSet: (idx: number) => void;
}

const HeadCam: React.FC<HeadCamProps> = ({
    cameraID,
    remoteStreams,
    isCameraVeilVisible,
    isCameraVeilVisibleSet,
    tabContent,
    activeMainGroupTab,
    setActiveMainGroupTab,
    setVelocityScale,
    setActionMode,
    swipeableViewsIdxSet,
}) => {
    return (
        <div className="head-cam-wrapper">
            <div className="controls">
                <div className="simple-camera-view-wrapper_XP">
                    <SimpleCameraView
                        id={cameraID}
                        remoteStreams={remoteStreams}
                        isCameraVeilVisible={isCameraVeilVisible}
                    />
                </div>
                <TabGroup
                    tabLabels={["Controls", "Recordings"]}
                    tabContent={tabContent}
                    startIdx={activeMainGroupTab}
                    onChange={(index: number) => setActiveMainGroupTab(index)}
                    pill={false}
                    key={"main-group"}
                />
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

export default HeadCam;