import React from "react";
import "../css/operator.css";

/**Details of a velocity setting */
type velocityDetails = {
    /**Name of the setting to display on the button */
    label: string, 
    /**The speed of this setting */
    speed: number
};

/**Props for VelocityControl */
type velocityControlProps = {
    currentSpeed: number;
    onChange: React.ChangeEventHandler<HTMLInputElement>;
}

/**The different velocity settings to display. */
const VELOCITY_SPEEDS: velocityDetails[] = [
    {label: "Slowest", speed: 0.2},
    {label: "Slow",    speed: 0.4},
    {label: "Medium",  speed: 0.5},
    {label: "Fast",    speed: 0.6},
    {label: "Fastest", speed: 0.8}
]

/**The speed the interface should initialize with */
export const DEFAULT_SPEED: number = VELOCITY_SPEEDS[2].speed;

/** The velocity control buttons. */
export const VelocityControl = ({currentSpeed, onChange}: velocityControlProps) => {
    const mapFunc = ({label, speed}: velocityDetails, index: number) => {
        const checked = speed == currentSpeed;
        const id = `speed-${index}`;
        return (
            <>
                <input 
                    type="radio" 
                    name="velocity" 
                    id={id}
                    key={`input-${index}`}
                    value={speed}
                    className="velocity-radio-button"
                    checked={checked}
                    onChange={onChange}
                />
                <label key={`label-${index}`} htmlFor={id} className="velocity-label">{label}</label>
            </>
        )
    }

    return (
        <div id="velocity-control-wrapper">
            {VELOCITY_SPEEDS.map(mapFunc)}
        </div>
    );
}