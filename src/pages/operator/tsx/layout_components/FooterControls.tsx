import React from 'react'

import { BatteryGauge } from '../static_components/BatteryGauge';
import ModalActionMode from './ModalActionMode';
import "operator/css/FooterControls.css";
import { ActionSpeed } from '../static_components/ActionSpeed';

interface FooterControlsProps {
  actionModes: string[];
  actionSpeedCurrent?: number;
  onActionSpeedChange: (newSpeed: number) => void;
  actionModeCurrent?: string;
  isCameraVeilVisibleSet: React.Dispatch<React.SetStateAction<boolean>>;
}

const FooterControls: React.FC<FooterControlsProps> = ({ actionSpeedCurrent, actionModeCurrent, onActionSpeedChange, isCameraVeilVisibleSet }) => {
  const [isOpenModalActionSpeed, isOpenModalActionSpeedSet] = React.useState<boolean>(false);
  const [isOpenModalActionMode, isOpenModalActionModeSet] = React.useState<boolean>(false);

  return (
    <div className="footer-controls_XP">
      <ActionSpeed
        speed={actionSpeedCurrent}
        onChange={onActionSpeedChange}
        setCameraVeilCallback={isCameraVeilVisibleSet}
      />
      {/* START Action Mode */}
      <div className="action-mode">
        <ModalActionMode isOpen={isOpenModalActionMode} handleClose={() => {
          isOpenModalActionModeSet(false);
          isCameraVeilVisibleSet(false);
        }} />
        <button
          onClick={() => {
            isOpenModalActionModeSet(!isOpenModalActionMode);
            isCameraVeilVisibleSet(prev => !prev)
          }}
        >
          <span className="action-mode-icon"></span>
          <div>{actionModeCurrent}</div>
        </button>
      </div>
      {/* END Action Mode */}
      {/* START Battery Gauge */}
      <div className="battery-gauge">
        <BatteryGauge />
      </div>
      {/* END Battery Gauge */}
    </div>
  )
}

export default FooterControls;
