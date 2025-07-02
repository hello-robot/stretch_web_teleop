import React from 'react'
import { BatteryGauge } from '../static_components/BatteryGauge';
import { ActionSpeed } from '../static_components/ActionSpeed';
import { ActionModeType } from '../utils/component_definitions';
import { ActionMode } from '../static_components/ActionMode';
import "operator/css/FooterControls.css";

interface FooterControlsProps {
    actionSpeedCurrent?: number;
    onActionSpeedChange: (newSpeed: number) => void;
    actionModeCurrent?: ActionModeType;
    onActionModeChange?: (newMode: ActionModeType) => void;
    isCameraVeilVisible: boolean;
    isCameraVeilVisibleSet: React.Dispatch<React.SetStateAction<boolean>>;
    setHideMap?: React.Dispatch<React.SetStateAction<boolean>>;
}

const FooterControls: React.FC<FooterControlsProps> = ({ actionSpeedCurrent, onActionSpeedChange, actionModeCurrent, onActionModeChange, isCameraVeilVisibleSet, isCameraVeilVisible, setHideMap }) => {

    return (
        <div className="footer-controls_XP">
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
            <button onPointerDown={
                () => setHideMap((hideMap) => (!hideMap))
            }>
                Map
            </button>
        </div>
    )
}

export default FooterControls;
