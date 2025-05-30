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
    isCameraVeilVisibleSet: React.Dispatch<React.SetStateAction<boolean>>;
}

const FooterControls: React.FC<FooterControlsProps> = ({ actionSpeedCurrent, onActionSpeedChange, actionModeCurrent, onActionModeChange, isCameraVeilVisibleSet }) => {

    return (
        <div className="footer-controls_XP">
            <ActionSpeed
                speed={actionSpeedCurrent}
                onChange={onActionSpeedChange}
                setCameraVeilCallback={isCameraVeilVisibleSet}
            />
            <ActionMode
                mode={actionModeCurrent}
                onChange={onActionModeChange}
                setCameraVeilCallback={isCameraVeilVisibleSet}
            />
            <div className="battery-gauge">
                <BatteryGauge />
            </div>
        </div>
    )
}

export default FooterControls;
