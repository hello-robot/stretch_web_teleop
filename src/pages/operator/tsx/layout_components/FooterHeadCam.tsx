import React from 'react'
import { BatteryGauge } from '../static_components/BatteryGauge';
import { ActionSpeed } from '../static_components/ActionSpeed';
import { ActionModeType } from '../utils/component_definitions';
import { ActionMode } from '../static_components/ActionMode';
import "operator/css/FooterHeadCam.css";

interface FooterControlsProps {
    actionSpeedCurrent?: number;
    onActionSpeedChange: (newSpeed: number) => void;
    actionModeCurrent?: ActionModeType;
    onActionModeChange?: (newMode: ActionModeType) => void;
    isCameraVeilVisible: boolean;
    isCameraVeilVisibleSet: React.Dispatch<React.SetStateAction<boolean>>;
    swipeableViewsIdxSet: React.Dispatch<React.SetStateAction<number>>;
}

const FooterHeadCam: React.FC<FooterControlsProps> = ({ actionSpeedCurrent, onActionSpeedChange, actionModeCurrent, onActionModeChange, isCameraVeilVisibleSet, isCameraVeilVisible, swipeableViewsIdxSet }) => {

    return (
        <div className="footer-head-cam_XP">
            <ActionSpeed
                speed={actionSpeedCurrent}
                onChange={onActionSpeedChange}
                isCameraVeilVisible={isCameraVeilVisible}
                setCameraVeilCallback={isCameraVeilVisibleSet}
            />
            <ActionMode
                mode={actionModeCurrent}
                onChange={onActionModeChange}
                isCameraVeilVisible={isCameraVeilVisible}
                setCameraVeilCallback={isCameraVeilVisibleSet}
            />
            <div className="auto-nav-toggle-wrapper">
                <button
                    onClick={() => swipeableViewsIdxSet(2)}
                    className="auto-nav-toggle"
                >
                    <span className="auto-nav-toggle-icon" />
                </button>
            </div>
        </div>
    )
}

export default FooterHeadCam;
