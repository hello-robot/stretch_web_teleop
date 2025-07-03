import React from 'react'
import { BatteryGauge } from '../static_components/BatteryGauge';
import { ActionSpeed } from '../static_components/ActionSpeed';
import { ActionModeType } from '../utils/component_definitions';
import { ActionMode } from '../static_components/ActionMode';
import "operator/css/FooterAutoNav.css";

interface FooterControlsProps {
    actionSpeedCurrent?: number;
    onActionSpeedChange: (newSpeed: number) => void;
    actionModeCurrent?: ActionModeType;
    onActionModeChange?: (newMode: ActionModeType) => void;
    isCameraVeilVisible: boolean;
    isCameraVeilVisibleSet: React.Dispatch<React.SetStateAction<boolean>>;
    setHideMap?: React.Dispatch<React.SetStateAction<boolean>>;
}

const FooterAutoNav: React.FC<FooterControlsProps> = ({ actionSpeedCurrent, onActionSpeedChange, actionModeCurrent, onActionModeChange, isCameraVeilVisibleSet, isCameraVeilVisible, setHideMap }) => {

    return (
        <div className="footer-auto-nav_XP">
            <div className="select-location-modal-wrapper">
                <button onPointerDown={
                    () => setHideMap((hideMap) => (!hideMap))
                }>
                    🗺️
                </button>
                <button>📋</button>
            </div>
            <button>Move</button>
            <div className="add-location-modal-wrapper">
                <button>
                    📍+
                </button>
            </div>
        </div>
    )
}

export default FooterAutoNav;
