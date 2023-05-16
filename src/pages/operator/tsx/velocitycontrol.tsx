import React, { useState } from "react";
import "operator/css/velocitycontrol.css";

/**Details of a velocity setting */
type velocityDetails = {
    /**Name of the setting to display on the button */
    label: string, 
    /**The speed of this setting */
    speed: number
};

/**Props for VelocityControl */
type velocityControlProps = {
    /** Initial speed when interface first loaded. */
    initialVelocityScale: number;
    /**
     * Callback function when a new speed is selected.
     * @param newSpeed the new selected speed
     */
    onChange: (newSpeed: number) => void;
}

/**The different velocity settings to display. */
const VELOCITY_SCALE: velocityDetails[] = [
    {label: "Slowest", speed: 0.25},
    {label: "Slow",    speed: 0.5},
    {label: "Medium",  speed: 1.0},
    {label: "Fast",    speed: 1.5},
    {label: "Fastest", speed: 2.0}
]

/**The speed the interface should initialize with */
export const DEFAULT_VELOCITY_SCALE: number = VELOCITY_SCALE[2].speed;

/** The velocity control buttons. */
export const VelocityControl = ({initialVelocityScale, onChange}: velocityControlProps) => {
    const [currentSpeed, setCurrentSpeed] = React.useState(initialVelocityScale);

    /** When a velocity button is clicked */
    const changeFunc = (e: React.ChangeEvent<HTMLInputElement>) => {
        const newSpeed = +e.target.value;
        setCurrentSpeed(newSpeed);
        onChange(newSpeed);
    }

    /** Maps the velocity labels and speeds to radio buttons */
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
                    onChange={changeFunc}
                />
                <label key={`label-${index}`} htmlFor={id} className="velocity-label">{label}</label>
            </>
        )
    }

    return (
        <div id="velocity-control-wrapper">
            {VELOCITY_SCALE.map(mapFunc)}
        </div>
    );
}