import React from 'react'

import { BatteryGauge } from '../static_components/BatteryGauge';
import ModalActionMode from './ModalActionMode';
import ModalActionSpeed from './ModalActionSpeed';

interface FooterControlsProps {
  actionModes: string[];
  actionModeCurrent?: string;
  isCameraVeilVisibleSet: React.Dispatch<React.SetStateAction<boolean>>;
}

const FooterControls: React.FC<FooterControlsProps> = ({ actionModeCurrent, isCameraVeilVisibleSet }) => {
  const [isOpenModalActionSpeed, isOpenModalActionSpeedSet] = React.useState<boolean>(false);
  const [isOpenModalActionMode, isOpenModalActionModeSet] = React.useState<boolean>(false);

  return (
    <div className="footer-controls_XP">
      {/* START Action Speed */}
      <div>
        <ModalActionSpeed isOpen={isOpenModalActionSpeed} handleClose={() => {
          isOpenModalActionSpeedSet(false);
          isCameraVeilVisibleSet(false);
        }} />
        <button
          onClick={() => {
            isOpenModalActionSpeedSet(!isOpenModalActionSpeed);
            isCameraVeilVisibleSet(prev => !prev)
          }}
          className="camera-blur-toggle"
        >
          😹
        </button>
      </div>
      {/* END Action Speed */}
      {/* START Action Mode */}
      <div>
        <ModalActionMode isOpen={isOpenModalActionMode} handleClose={() => {
          isOpenModalActionModeSet(false);
          isCameraVeilVisibleSet(false);
        }} />
        <button
          onClick={() => {
            isOpenModalActionModeSet(!isOpenModalActionMode);
            isCameraVeilVisibleSet(prev => !prev)
          }}
          className="camera-blur-toggle"
        >
          {actionModeCurrent}
        </button>
      </div>
      {/* END Action Mode */}
      {/* START Battery Gauge */}
      <BatteryGauge />
      {/* END Battery Gauge */}
    </div>
  )
}

export default FooterControls