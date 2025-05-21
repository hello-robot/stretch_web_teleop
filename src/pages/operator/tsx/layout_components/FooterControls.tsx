import React from 'react'

import { SpeedControl } from '../static_components/SpeedControl';
import { FunctionProvider } from "../function_providers/FunctionProvider";
import { BatteryGuage } from '../static_components/BatteryGauge';

const FooterControls = (props: {
  actionModeCurrent?: string;
}) => {

  const [velocityScale, setVelocityScale] = React.useState<number>(
    FunctionProvider.velocityScale
  );

  return (
    <div className="footer-controls_XP">
      <SpeedControl
        scale={velocityScale}
        onChange={(newScale: number) => {
          setVelocityScale(newScale);
          FunctionProvider.velocityScale = newScale;
        }}
      />
      <button>{props.actionModeCurrent}</button>
      <BatteryGuage />
    </div>
  )
}

export default FooterControls