import React from "react";
import "operator/css/SpeedControl.css";
import { RadioGroup, Radio, FormControlLabel } from '@mui/material';
import { useMediaQuery } from '@mui/material';
import { Theme } from '@mui/material/styles';
import { isBrowser, isTablet } from "react-device-detect";

/** Details of a velocity setting */
type VelocityDetails = {
  /** Name of the setting to display on the button */
  label: string;
  /** The speed of this setting */
  scale: number;
};

/** Props for {@link SpeedControl} */
type SpeedControlProps = {
  /** Initial speed when interface first loaded. */
  scale: number;
  /**
   * Callback function when a new speed is selected.
   * @param newSpeed the new selected speed
   */
  onChange: (newScale: number, label?: string) => void;
};

/** The different velocity settings to display. */
export const VELOCITY_SCALE: VelocityDetails[] = [
  { label: "Slowest", scale: 0.2 },
  { label: "Slow", scale: 0.4 },
  { label: "Medium", scale: 0.8 },
  { label: "Fast", scale: 1.2 },
  { label: "Fastest", scale: 1.6 },
];

/** The speed the interface should initialize with */
export const DEFAULT_VELOCITY_SCALE: number = VELOCITY_SCALE[2].scale;

const getLabelByScale = (scale: number): string => {
  const velocityDetail = VELOCITY_SCALE.find(item => item.scale === scale);
  return velocityDetail ? velocityDetail.label : undefined;
};

/**
 * Set of buttons so the user can control the scaling of the speed for all controls.
 * @param props see {@link SpeedControlProps}
 */
export const SpeedControl = (props: SpeedControlProps) => {
  /** Maps the velocity labels and speeds to radio buttons (for mobile) or buttons (for desktop) */
  const mapFunc = ({ scale, label }: VelocityDetails) => {
    const active = scale === props.scale;
    if (!isBrowser && !isTablet) {
      // Render radio buttons for mobile
      return (
        <FormControlLabel
          key={label}
          value={JSON.stringify({ scale, label })} // Pass both scale and label
          control={<Radio />}
          label={""}
          checked={active}
          style={{ margin: 0 }}
        />
      );
    } else {
      // Render regular buttons for larger screens
      return (
        <button
          key={label}
          className={active ? "btn-blue font-white" : ""}
          onPointerDown={() => props.onChange(scale, label)}
        >
          {label}
        </button>
      );
    }
  };

  return (
    <div id="velocity-control-container">
      {!isBrowser && !isTablet ? (
        <div className="velocity-radio-group">
          <span className="label">Slowest</span>
          <RadioGroup
            style={{ display: 'flex', flexDirection: 'row', justifyContent: 'center' }}
            value={JSON.stringify({ scale: props.scale, label: getLabelByScale(props.scale)})} // Match the format
            onChange={(e) => {
              const selectedItem = JSON.parse(e.target.value); // Parse the selected item
              props.onChange(selectedItem.scale, selectedItem.label); // Call props.onChange
            }}
          >
            {VELOCITY_SCALE.map(mapFunc)}
          </RadioGroup>
          <span className="label">Fastest</span>
        </div>
      ) : (
        VELOCITY_SCALE.map(mapFunc)
      )}
    </div>
  );
};
