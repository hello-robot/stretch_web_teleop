import React from 'react'

import { BatteryGauge } from '../static_components/BatteryGauge';
import ModalActionMode from './ModalActionMode';
import ModalActionSpeed from './ModalActionSpeed';
import "operator/css/FooterControls.css";

interface FooterControlsProps {
  actionModes: string[];
  actionSpeedCurrent?: string;
  actionModeCurrent?: string;
  isCameraVeilVisibleSet: React.Dispatch<React.SetStateAction<boolean>>;
}

const FooterControls: React.FC<FooterControlsProps> = ({ actionSpeedCurrent, actionModeCurrent, isCameraVeilVisibleSet }) => {
  const [isOpenModalActionSpeed, isOpenModalActionSpeedSet] = React.useState<boolean>(false);
  const [isOpenModalActionMode, isOpenModalActionModeSet] = React.useState<boolean>(false);

  return (
    <div className="footer-controls_XP">
      {/* START Action Speed */}
      <div className="action-speed">
        <ModalActionSpeed isOpen={isOpenModalActionSpeed} handleClose={() => {
          isOpenModalActionSpeedSet(false);
          isCameraVeilVisibleSet(false);
        }} />
        <button
          onClick={() => {
            isOpenModalActionSpeedSet(!isOpenModalActionSpeed);
            isCameraVeilVisibleSet(prev => !prev)
          }}
        >
          <span className={`action-speed-icon ${actionSpeedCurrent}`}></span>
        </button>
      </div>
      {/* END Action Speed */}
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
