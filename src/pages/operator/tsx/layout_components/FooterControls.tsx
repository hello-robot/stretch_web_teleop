import React from 'react'

import { SpeedControl } from '../static_components/SpeedControl';
import { FunctionProvider } from "../function_providers/FunctionProvider";
import { BatteryGauge } from '../static_components/BatteryGauge';
import ModalActionMode from './ModalActionMode';

interface FooterControlsProps {
  actionModes: string[];
  actionModeCurrent?: string;
  isCameraVeilVisibleSet: React.Dispatch<React.SetStateAction<boolean>>;
}

const FooterControls: React.FC<FooterControlsProps> = ({ actionModeCurrent, isCameraVeilVisibleSet }) => {
  const [velocityScale, setVelocityScale] = React.useState<number>(
    FunctionProvider.velocityScale
  );
  // START –> <ButtonActionModeSelection/>
  const [isOpen, isOpenSet] = React.useState<boolean>(false);
  // END –> <ButtonActionModeSelection/>

  return (
    <div className="footer-controls_XP">
      <SpeedControl
        scale={velocityScale}
        onChange={(newScale: number) => {
          setVelocityScale(newScale);
          FunctionProvider.velocityScale = newScale;
        }}
      />
      <div>
        <ModalActionMode isOpen={isOpen} handleClose={() => {
          isOpenSet(false);
          isCameraVeilVisibleSet(false);
        }} />
        <button
          onClick={() => {
            isOpenSet(!isOpen);
            isCameraVeilVisibleSet(prev => !prev)
          }}
          className="camera-blur-toggle"
        >
          {actionModeCurrent}
        </button>
      </div>
      <BatteryGauge />
    </div>
  )
}

export default FooterControls