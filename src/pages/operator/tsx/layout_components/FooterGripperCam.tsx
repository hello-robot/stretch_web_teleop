import React from 'react'
import { ActionSpeed } from '../static_components/ActionSpeed';
import { ActionModeType } from '../utils/component_definitions';
import { ActionMode } from '../static_components/ActionMode';
import "operator/css/FooterGripperCam.css";

interface FooterControlsProps {
    actionSpeedCurrent?: number;
    onActionSpeedChange: (newSpeed: number) => void;
    actionModeCurrent?: ActionModeType;
    onActionModeChange?: (newMode: ActionModeType) => void;
    isCameraVeilVisible: boolean;
    isCameraVeilVisibleSet: React.Dispatch<React.SetStateAction<boolean>>;
    swipeableViewsIdxSet: React.Dispatch<React.SetStateAction<number>>;
}

const FooterGripperCam: React.FC<FooterControlsProps> = ({ actionSpeedCurrent, onActionSpeedChange, actionModeCurrent, onActionModeChange, isCameraVeilVisibleSet, isCameraVeilVisible, swipeableViewsIdxSet }) => {

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
            <div className="drive-toggle-wrapper">
                <button
                    onClick={() => swipeableViewsIdxSet(1)}
                    className="drive-toggle"
                >
                    <span className="drive-toggle-icon" />
                </button>
            </div>
        </div>
    )
}

export default FooterGripperCam;
